/*
 * drivers/net/wireless/bcmdhd/nv_logger.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2016 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <nv_logger.h>

atomic_t list1_val = ATOMIC_INIT(1);
atomic_t list2_val = ATOMIC_INIT(1);
char logbuf[MAX_LOGLIMIT + 128];
bool enable_file_logging;
struct list_head list1;
struct list_head list2;
static int reset_log_size;
struct mutex sysfs_dump_mtx;
struct mutex suspend_lock;

struct workqueue_struct *logger_wqueue;
struct log_buffer {
	char tmstmp[22];
	char *buf;
	char *info;
	int event;
};

struct log_node {
	struct list_head list;
	struct log_buffer *log;
};

struct work_struct enqueue_work;

void write_log_init()
{
	logger_wqueue = create_workqueue("dhd_log");
	if (logger_wqueue == NULL) {
		pr_err("write_log_init: failed to allocate workqueue\n");
		return;
	}
	INIT_WORK(&enqueue_work, write_queue_work);
	INIT_LIST_HEAD(&list1);
	INIT_LIST_HEAD(&list2);

	if (dhd_log_netlink_init())
		goto queue_fail;

	if (dhdlog_sysfs_init())
		goto netlink_fail;

	mutex_init(&sysfs_dump_mtx);
	mutex_init(&suspend_lock);

	dhd_log_netlink_send_msg(0, 0, 0, NULL, 0);
	enable_file_logging = true;

	return;

netlink_fail:
	dhd_log_netlink_deinit();
queue_fail:
	destroy_workqueue(logger_wqueue);
	enable_file_logging = false;
}

void write_log_uninit()
{
	pr_info("write_log_uninit\n");

	flush_workqueue(logger_wqueue);
	destroy_workqueue(logger_wqueue);
	dhd_log_netlink_deinit();
	dhdlog_sysfs_deinit();
}

int write_log(int event, const char *buf, const char *info)
{
	struct log_node *temp;
	int buf_len = 0;
	int info_len = 0;
	struct timespec ts;
	static int list1_size;
	static int list2_size;
	struct timeval now;
	struct tm date_time;

	mutex_lock(&suspend_lock);
	if (!enable_file_logging || logger_wqueue == NULL) {
		mutex_unlock(&suspend_lock);
		return -1;
	}
	mutex_unlock(&suspend_lock);

	if (buf == NULL)
		return -1;

	if (mutex_trylock(&sysfs_dump_mtx)) {
		if (1 == reset_log_size) {
			reset_log_size = 0;
			list1_size = 0;
			list2_size = 0;
		}
		mutex_unlock(&sysfs_dump_mtx);
	}

	switch (event) {

	case WLC_E_ESCAN_RESULT:
		break;
	default:
		temp = kmalloc(sizeof(struct log_node), GFP_ATOMIC);
		if (temp == NULL) {
			pr_err("write_log: temp memory allocation failed");
			return -1;
		}

		temp->log = kmalloc(sizeof(struct log_buffer), GFP_ATOMIC);
		if (temp->log == NULL) {
			pr_err("write_log: log memory allocation failed");
			kfree(temp);
			return -1;
		}

		buf_len = strlen(buf) + 1;
		temp->log->buf = kmalloc(buf_len, GFP_ATOMIC);
		if (temp->log->buf == NULL) {
			pr_err("write_log_buf: log memory allocation failed");
			kfree(temp);
			kfree(temp->log);
			return -1;
		}

		strncpy(temp->log->buf, buf, buf_len);

		do_gettimeofday(&now);
		sprintf(temp->log->tmstmp, "[%.2d-%.2d %.2d:%.2d:%.2d.%u]",
					date_time.tm_mon+1,
					date_time.tm_mday,
					date_time.tm_hour,
					date_time.tm_min,
					date_time.tm_sec ,
					(unsigned int)(now.tv_usec/1000));
		if (info != NULL) {
			info_len = strlen(info) + 1;
			temp->log->info = kmalloc(info_len, GFP_ATOMIC);
			strncpy(temp->log->info, info, info_len);
		}

		temp->log->event = event;

	/* whichever list is not busy, dump data in that list */
		if (1 == atomic_read(&list1_val)) {
			list_add_tail(&(temp->list), &(list1));
			list1_size += buf_len + info_len;
		} else if (1 == atomic_read(&list2_val)) {
			list_add_tail(&(temp->list), &(list2));
			list2_size += buf_len + info_len;
		} else {
		/* send data directly over netlink because both lists are busy*/
			pr_err("Message dropped due to busy queues");
		}

		if (list1_size > MAX_LOGLIMIT) {
			atomic_set(&list1_val, 0);
			queue_work(logger_wqueue, &enqueue_work);
			list1_size = 0;
		} else if (list2_size > MAX_LOGLIMIT) {
			atomic_set(&list2_val, 0);
			queue_work(logger_wqueue, &enqueue_work);
			list2_size = 0;
		}

		break;
	}
	return buf_len + info_len;
}

void write_queue_work(struct work_struct *work)
{
	struct log_node *temp = NULL;
	struct list_head *pos = NULL, *n = NULL;
	char *log = NULL;

	/* queuing in list1 is blocked, so can dequeue list1*/
	if (atomic_read(&list1_val) == 0) {

		list_for_each_safe(pos, n, &(list1)) {
			temp = list_entry(pos, struct log_node, list);
		/* for the correct string of the event */
			strcat(logbuf, temp->log->tmstmp);

			strcat(logbuf, temp->log->buf);
			strcat(logbuf, " ");
			strcat(logbuf, temp->log->info);
			strcat(logbuf, "\n");

			list_del(pos);
			kfree(temp);
		}
		write_log_file(logbuf);
		memset(logbuf, '\0', sizeof(logbuf));
		/* make this list available for writing now */
		atomic_set(&list1_val, 1);

	}

	/* queuing in list1 is blocked, so can dequeue list1*/
	if (atomic_read(&list2_val) == 0) {

		list_for_each_safe(pos, n, &(list2)) {
			temp = list_entry(pos, struct log_node, list);
		/* for the correct string of the event */
			strcat(logbuf, temp->log->tmstmp);

			strcat(logbuf, temp->log->buf);
			strcat(logbuf, " ");
			strcat(logbuf, temp->log->info);
			strcat(logbuf, "\n");

			list_del(pos);
			kfree(temp);
		}
		write_log_file(logbuf);
		memset(logbuf, '\0', sizeof(logbuf));
		/* make this list available for writing now */
		atomic_set(&list2_val, 1);
	}

}

void write_log_file(const char *log)
{
	static int seq;
	dhd_log_netlink_send_msg(0, 0, seq++, log, strlen(log) + 1);
	if (seq % 1024)
		seq = 0;
}

void nvlogger_suspend_work()
{
	mutex_lock(&suspend_lock);
	enable_file_logging = false;
	mutex_unlock(&suspend_lock);
	pr_info("nvlogger_suspend_work\n");
	cancel_work_sync(&enqueue_work);
}

void nvlogger_resume_work()
{
	pr_info("nvlogger_resume_work\n");
	mutex_lock(&suspend_lock);
	enable_file_logging = true;
	mutex_unlock(&suspend_lock);
}

#define NETLINK_CARBON     29

static struct sock *nl_sk;

static int g_pid;
static void dhd_log_netlink_recv(struct sk_buff *skb)
{

	struct nlmsghdr *nlh;
	nlh = (struct nlmsghdr *)skb->data;

	if (nlh == NULL) {
		pr_err("ids received messaged with null data\n");
		return;
	}

	g_pid = nlh->nlmsg_pid;

	if (g_pid > 0)
		dhd_log_netlink_send_msg(0, 0, 0, "Firmware logs\n", 15);
}

static int dhd_log_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input	= dhd_log_netlink_recv,
	};

	pr_info("ids dhd_log_netlink_init\n");
	if (nl_sk != NULL) {
		pr_err("ids nl_sk already assigned\n");
		return 0;
	}

	nl_sk = netlink_kernel_create(&init_net, NETLINK_CARBON, &cfg);

	if (nl_sk == NULL) {
		pr_err("ids netlink create failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void dhd_log_netlink_deinit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

s32
dhd_log_netlink_send_msg(int pid, int type, int seq, void *data, size_t size)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int ret = -1;

	if (nl_sk == NULL)
		goto nlmsg_failure;

	skb = alloc_skb(NLMSG_SPACE(size), GFP_ATOMIC);
	if (skb == NULL)
		goto nlmsg_failure;

	nlh = nlmsg_put(skb, 0, 0, 0, size, 0);
	if (nlh == NULL) {
		dev_kfree_skb(skb);
		goto nlmsg_failure;
	}

	memcpy(nlmsg_data(nlh), data, size);
	nlh->nlmsg_seq = seq;
	nlh->nlmsg_type = type;

	/* netlink_unicast() takes ownership of the skb and frees it itself. */
	ret = netlink_unicast(nl_sk, skb, g_pid, 0);

nlmsg_failure:
	return ret;
}

void dumplogs()
{
	/* try sleeping blocking two queues to avoid blocking both queues */
	pr_info("dumplogs from nv_logger\n");
	atomic_set(&list1_val, 0);
	atomic_set(&list2_val, 0);
	queue_work(logger_wqueue, &enqueue_work);
	mutex_lock(&sysfs_dump_mtx);
	reset_log_size = 1;
	mutex_unlock(&sysfs_dump_mtx);
}

struct kobject *dhdlog_sysfs_kobj;

static ssize_t dhdlog_sysfs_enablelog_store(struct kobject *kobj,
			struct kobj_attribute *attr,
				const char *buf, ssize_t count)
{
	int val;
	pr_info("dhdlog_sysfs_enablelog_store = %s", buf);
	if (strncmp(buf, "0", 1) == 0 || strncmp(buf, "false", 5) == 0
		|| strncmp(buf, "no", 2) == 0) {
		mutex_lock(&suspend_lock);
		enable_file_logging = false;
		mutex_unlock(&suspend_lock);
	} else if (strncmp(buf, "dump", 4) == 0) {
		dumplogs();
	} else if (strncmp(buf, "1", 1) == 0 || strncmp(buf, "true", 4) == 0
		|| strncmp(buf, "yes", 3) == 0) {
		mutex_lock(&suspend_lock);
		enable_file_logging = true;
		mutex_unlock(&suspend_lock);
	}

	return count;
}

static ssize_t dhdlog_sysfs_enablelog_show(struct kobject *kobj,
			struct kobj_attribute *attr,
				char *buf) {

	pr_info("dhdlog_sysfs_enablelog_show");
	return sprintf(buf, "%d\n", enable_file_logging);
}

static struct kobj_attribute dhdlog_sysfs_enablelog_attribute =
	__ATTR(enablelog, 0644, dhdlog_sysfs_enablelog_show,
					dhdlog_sysfs_enablelog_store);

static struct attribute *dhdlog_sysfs_attrs[] = {
	&dhdlog_sysfs_enablelog_attribute,
	NULL,
};

static struct attribute_group dhdlog_sysfs_attr_group = {
	.attrs = dhdlog_sysfs_attrs,
};
int dhdlog_sysfs_deinit(void)
{
	if (dhdlog_sysfs_kobj) {
		kobject_put(dhdlog_sysfs_kobj);
		dhdlog_sysfs_kobj = NULL;
	}

	return 0;
}

int dhdlog_sysfs_init()
{

	int ret = 0;

	dhdlog_sysfs_kobj = kobject_create_and_add(KBUILD_MODNAME, kernel_kobj);
	if (!dhdlog_sysfs_kobj) {
		pr_err("%s: kobject creation_add failed\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_group(dhdlog_sysfs_kobj, &dhdlog_sysfs_attr_group);
	if (ret) {
		pr_err("%s: create_group failed\n", __func__);
		dhdlog_sysfs_deinit();
		return ret;
	}

	return 0;
}
