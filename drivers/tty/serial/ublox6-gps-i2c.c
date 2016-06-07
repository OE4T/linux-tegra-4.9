/* u-blox 6 I2C GPS driver
 *
 * Copyright (C) 2015 Felipe F. Tonello <eu@felipetonello.com>
 *
 * Driver that translates a serial tty GPS device to a i2c GPS device
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/gps_ublox.h>
/*
 * Version Information
 */
#define DRIVER_VERSION "v0.1"
#define DRIVER_DESC "u-blox 6 I2C GPS driver"

#define UBLOX_GPS_MAJOR 0
#define UBLOX_GPS_NUM 1 /* Only support 1 GPS at a time */

/* By default u-blox GPS fill its buffer every 1 second (1000 msecs) */
#define READ_TIME 1000

static struct tty_port *ublox_gps_tty_port;
static struct i2c_client *ublox_gps_i2c_client;
static int ublox_gps_is_open;
static struct file *ublox_gps_filp;

static void ublox_gps_read_worker(struct work_struct *private);

static DECLARE_DELAYED_WORK(ublox_gps_wq, ublox_gps_read_worker);

static void ublox_gps_read_worker(struct work_struct *private)
{
	s32 gps_buf_size, buf_size = 0;
	u8 *buf;

	if (!ublox_gps_is_open)
		return;

	/* check if driver was removed */
	if (!ublox_gps_i2c_client)
		return;

	gps_buf_size = i2c_smbus_read_word_data(ublox_gps_i2c_client, 0xfd);
	if (gps_buf_size < 0) {
		dev_warn(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": couldn't read register(0xfd) from GPS.\n");
		/* try one more time */
		goto end;
	}

	/* 0xfd is the MSB and 0xfe is the LSB */
	gps_buf_size = ((gps_buf_size & 0xf) << 8) | ((gps_buf_size & 0xf0) >> 8);

	if (gps_buf_size > 0) {

		buf = kcalloc(gps_buf_size, sizeof(*buf), GFP_KERNEL);
		if (!buf) {
			dev_warn(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": couldn't allocate memory.\n");
			/* try one more time */
			goto end;
		}

		do {
			buf_size = i2c_master_recv(ublox_gps_i2c_client, (char *)buf, gps_buf_size);
			if (buf_size < 0) {
				dev_warn(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": couldn't read data from GPS.\n");
				kfree(buf);
				/* try one more time */
				goto end;
			}

			tty_insert_flip_string(ublox_gps_tty_port, buf, buf_size);

			gps_buf_size -= buf_size;

			/* There is a small chance that we need to split the data over
			   several buffers. If this is the case we must loop */
		} while (unlikely(gps_buf_size > 0));

		tty_flip_buffer_push(ublox_gps_tty_port);

		kfree(buf);
	}

end:
	/* resubmit the workqueue again */
	schedule_delayed_work(&ublox_gps_wq, msecs_to_jiffies(READ_TIME)); /* 1 sec delay */
}

static int ublox_gps_serial_open(struct tty_struct *tty, struct file *filp)
{
	if (ublox_gps_is_open)
		return -EBUSY;

	ublox_gps_filp = filp;
	ublox_gps_tty_port = tty->port;
	ublox_gps_tty_port->low_latency = true; /* make sure we push data immediately */
	ublox_gps_is_open = true;

	schedule_delayed_work(&ublox_gps_wq, 0);

	return 0;
}

static void ublox_gps_serial_close(struct tty_struct *tty, struct file *filp)
{
	if (!ublox_gps_is_open)
		return;

	/* avoid stop when the denied (in open) file structure closes itself */
	if (ublox_gps_filp != filp)
		return;

	ublox_gps_is_open = false;
	ublox_gps_filp = NULL;
	ublox_gps_tty_port = NULL;
}

static int ublox_gps_serial_write(struct tty_struct *tty, const unsigned char *buf,
	int count)
{
	if (!ublox_gps_is_open)
		return 0;

	/* check if driver was removed */
	if (!ublox_gps_i2c_client)
		return 0;

	/* we don't write back to the GPS so just return same value here */
	return count;
}

static int ublox_gps_write_room(struct tty_struct *tty)
{
	if (!ublox_gps_is_open)
		return 0;

	/* check if driver was removed */
	if (!ublox_gps_i2c_client)
		return 0;

	/* we don't write back to the GPS so just return some value here */
	return 1024;
}

static const struct tty_operations ublox_gps_serial_ops = {
	.open = ublox_gps_serial_open,
	.close = ublox_gps_serial_close,
	.write = ublox_gps_serial_write,
	.write_room = ublox_gps_write_room,
};

static struct tty_driver *ublox_gps_tty_driver;

static int ublox_gps_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int result = 0;

	ublox_gps_tty_driver = alloc_tty_driver(UBLOX_GPS_NUM);
	if (!ublox_gps_tty_driver)
		return -ENOMEM;

	ublox_gps_tty_driver->owner = THIS_MODULE;
	ublox_gps_tty_driver->driver_name = "ublox_gps";
	ublox_gps_tty_driver->name = "ttyS";
	ublox_gps_tty_driver->major = UBLOX_GPS_MAJOR;
	ublox_gps_tty_driver->minor_start = 0;
	ublox_gps_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ublox_gps_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	ublox_gps_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	ublox_gps_tty_driver->init_termios = tty_std_termios;
	ublox_gps_tty_driver->init_termios.c_iflag = IGNCR | IXON;
	ublox_gps_tty_driver->init_termios.c_oflag = OPOST;
	ublox_gps_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
		HUPCL | CLOCAL;
	ublox_gps_tty_driver->init_termios.c_ispeed = 9600;
	ublox_gps_tty_driver->init_termios.c_ospeed = 9600;
	tty_set_operations(ublox_gps_tty_driver, &ublox_gps_serial_ops);
	result = tty_register_driver(ublox_gps_tty_driver);
	if (result) {
		dev_err(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": %s - tty_register_driver failed\n",
			__func__);
		goto err;
	}

	ublox_gps_i2c_client = client;
	ublox_gps_filp = NULL;
	ublox_gps_tty_port = NULL;
	ublox_gps_is_open = false;

	/* i2c_set_clientdata(client, NULL); */

	dev_info(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": " DRIVER_VERSION ": "
		DRIVER_DESC "\n");

	return result;

err:
	dev_err(&ublox_gps_i2c_client->dev, KBUILD_MODNAME ": %s - returning with error %d\n",
		__func__, result);

	put_tty_driver(ublox_gps_tty_driver);

	return result;
}

static int ublox_gps_remove(struct i2c_client *client)
{
	tty_unregister_driver(ublox_gps_tty_driver);
	put_tty_driver(ublox_gps_tty_driver);

	ublox_gps_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id ublox_gps_id[] = {
	{ "ublox_gps", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ublox_gps_id);

static struct i2c_driver ublox_gps_i2c_driver = {
	.driver = {
		.name  = "ublox_gps",
		.owner = THIS_MODULE,
	},
	.id_table  = ublox_gps_id,
	.probe     = ublox_gps_probe,
	.remove    = ublox_gps_remove,
};

module_i2c_driver(ublox_gps_i2c_driver);

MODULE_AUTHOR("Felipe F. Tonello <eu@felipetonello.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
