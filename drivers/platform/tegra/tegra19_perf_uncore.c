 /*
 * Carmel Uncore PMU support
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 #include <linux/version.h>

/*
* perf events refactored include structure starting with 4.4
* This driver is only valid with kernel version 4.4 and greater
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
#include <asm/irq_regs.h>

#include <linux/of.h>
#include <linux/perf/arm_pmu.h>
#include <linux/platform_device.h>
#include <linux/tegra-mce.h>

#include "dmce_perfmon.h"

#define CARMEL_PMU_MAX_HWEVENTS		8

/*
 * Carmel perf uncore supports two counters
 */
#define CARMEL_MAX_UNCORE_CNTS		2

/*
 * PMXEVTYPER: Event selection reg
 */
#define ARMV8_EVTYPE_EVENT	0x3ff		/* Mask for EVENT bits */
#define CARMEL_EVTYPE_MASK	0x00000d00
#define CARMEL_EVTYPE_EVENT_ID	0x0ff

static DEFINE_PER_CPU(struct pmu_hw_events, cpu_hw_events);

#define to_arm_pmu(p) (container_of(p, struct arm_pmu, pmu))

enum carmel_uncore_perf_types {
	CARMEL_PMU_L2D_CACHE = 0x16,
	CARMEL_PMU_L2D_CACHE_REFILL = 0x17,
	CARMEL_PMU_L2D_CACHE_WB = 0x18,

	CARMEL_PMU_BUS_ACCESS = 0x19,
	CARMEL_PMU_BUS_CYCLES = 0x1D,
	CARMEL_PMU_L3D_CACHE_ALLOCATE = 0x29,
	CARMEL_PMU_L3D_CACHE_REFILL = 0x2A,
	CARMEL_PMU_L3D_CACHE = 0x2B,
	CARMEL_PMU_L3D_CACHE_WB = 0x2C,

	CARMEL_PMU_L2D_CACHE_LD = 0x50,
	CARMEL_PMU_L2D_CACHE_ST,
	CARMEL_PMU_L2D_CACHE_REFILL_LD,
	CARMEL_PMU_L2D_CACHE_REFILL_ST,
	CARMEL_PMU_L2D_CACHE_WB_VIC_TIM = 0x56

};

/*
 * Carmel perf tool only support 2 events tracking in one run.
 */
#define FIRST_TRACKING_EVENT	1
#define SECOND_TRACKING_EVENT	2

/*
 * Perf Events' indices
 */
#define ARMV8_IDX_CYCLE_COUNTER	0
#define ARMV8_IDX_COUNTER0	1
#define ARMV8_MAX_COUNTERS	32
#define ARMV8_COUNTER_MASK	(ARMV8_MAX_COUNTERS - 1)

/*
 * Perf Event to low level counters mapping
 */
#define ARMV8_IDX_TO_COUNTER(x) \
	(((x) - ARMV8_IDX_COUNTER0) & ARMV8_COUNTER_MASK)

/*
 * Per-CPU PMCR: config reg
 */
#define ARMV8_PMCR_E		(1 << 0) /* Enable all counters */
#define ARMV8_PMCR_P		(1 << 1) /* Reset all counters */
#define ARMV8_PMCR_C		(1 << 2) /* Cycle counter reset */
#define ARMV8_PMCR_D		(1 << 3) /* CCNT counts every 64th cpu cycle */
#define ARMV8_PMCR_X		(1 << 4) /* Export to ETM */
#define ARMV8_PMCR_DP		(1 << 5) /* Disable CCNT if non-invasive dbg */
#define ARMV8_PMCR_N_SHIFT	11	 /* Number of counters supported */
#define ARMV8_PMCR_N_MASK	0x1f
#define ARMV8_PMCR_MASK		0x3f	 /* Mask for writable bits */

/*
 * PMOVSR: counters overflow flag status reg
 */
#define ARMV8_OVSR_MASK		0xffffffff	/* Mask for writable bits */
#define ARMV8_OVERFLOWED_MASK	ARMV8_OVSR_MASK

/*
 * Event filters for PMUv3
 */
#define ARMV8_EXCLUDE_EL1	(1 << 31)
#define ARMV8_EXCLUDE_EL0	(1 << 30)
#define ARMV8_INCLUDE_EL2	(1 << 27)

static struct dmce_perfmon_cnt_info carmel_uncore_event[CARMEL_MAX_UNCORE_CNTS];

static u32 mce_perfmon_rw(uint8_t command, carmel_pmc_reg_t reg, u32 *data)
{
	u32 status = -1;

	if (command == DMCE_PERFMON_COMMAND_WRITE)
		status = tegra_mce_write_uncore_perfmon(reg, *data);
	else if (command == DMCE_PERFMON_COMMAND_READ)
		status = tegra_mce_read_uncore_perfmon(reg, data);
	else
		pr_err("perfmon command not recognized");

	if (status != DMCE_PERFMON_STATUS_SUCCESS) {
		pr_err("perfmon status error: %u", status);
	}

	return status;
}

static inline int get_ctr_info(u32 idx, struct dmce_perfmon_cnt_info *info)
{
	int i;

	for (i = 0; i < CARMEL_MAX_UNCORE_CNTS; i++) {
		if (carmel_uncore_event[i].index == idx &&
			carmel_uncore_event[i].valid == 1) {
			*info = carmel_uncore_event[i];
			return 0;
		}
	}
	return -1;
}

static inline int alloc_carmel_ctr(u32 idx, u32 group, u32 event)
{
	int i;
	struct dmce_perfmon_cnt_info info;

	if (get_ctr_info(idx, &info) < 0) {
		for (i = 0; i < CARMEL_MAX_UNCORE_CNTS; i++) {
			if (carmel_uncore_event[i].valid == 0) {
				carmel_uncore_event[i].counter = event;
				carmel_uncore_event[i].group = group;
				carmel_uncore_event[i].unit = 0;
				carmel_uncore_event[i].index = idx;
				carmel_uncore_event[i].idx = i;
				carmel_uncore_event[i].valid = 1;
				break;
			}
		}

		if (i == CARMEL_MAX_UNCORE_CNTS) {
			pr_err("Failed to allocate Carmel uncore ctr\n");
			return -1;
		}
	}

	return 0;
}

static inline int clear_carmel_ctr(u32 idx)
{
	int i;

	for (i = 0; i < CARMEL_MAX_UNCORE_CNTS; i++) {
		if (carmel_uncore_event[i].index == idx) {
			carmel_uncore_event[i].valid = 0;
			break;
		}
	}

	return 0;
}

static inline int get_uncore_group(u32 event, int flag_select_group)
{
	u32 group = 0x0;//, pre_group;
	switch (event) {
		case CARMEL_PMU_L2D_CACHE:
		case CARMEL_PMU_L2D_CACHE_REFILL:
		case CARMEL_PMU_L2D_CACHE_WB:
		case CARMEL_PMU_L2D_CACHE_LD:
		case CARMEL_PMU_L2D_CACHE_ST:
		case CARMEL_PMU_L2D_CACHE_REFILL_LD:
		case CARMEL_PMU_L2D_CACHE_REFILL_ST:
		case CARMEL_PMU_L2D_CACHE_WB_VIC_TIM:
		{
			group = 0x0100;
			break;
		}

		case CARMEL_PMU_BUS_ACCESS:
		case CARMEL_PMU_BUS_CYCLES:
		case CARMEL_PMU_L3D_CACHE_ALLOCATE:
		case CARMEL_PMU_L3D_CACHE_REFILL:
		case CARMEL_PMU_L3D_CACHE:
		case CARMEL_PMU_L3D_CACHE_WB:
		{
			group = 0x0000;
			break;
		}

		default:
			return -1;
	}

	pr_err("Perfmon: get_uncore_group The event is 0x%x, the group is 0x%x\n", event, group);
	if(flag_select_group)
	{
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMSELR_EL0, &group);
	}

	return group;
}

static inline int carmel_pmu_counter_valid(u32 idx,
		struct dmce_perfmon_cnt_info *info)
{
	int ret, flag_select_group;

	if (get_ctr_info(idx, info) < 0)
		return 0;

	flag_select_group = 0;
	ret = get_uncore_group(info->counter, flag_select_group);

	return ret >= 0 ? 1 : 0;
}

static int
carmel_pmu_map_raw_event(u32 raw_event_mask, u64 config)
{
	int flag_select_group;
	flag_select_group = 1;
	if (get_uncore_group(config & CARMEL_EVTYPE_EVENT_ID, flag_select_group) < 0)
		return -ENOENT;
	else
		return (int)(config & raw_event_mask);
}

static inline int carmel_pmu_has_overflowed(u32 pmovsr)
{
	return pmovsr & ARMV8_OVERFLOWED_MASK;
}

static inline int carmel_pmu_counter_has_overflowed(u32 pmnc, int idx)
{
	int ret = 0;
	u32 counter;
	struct dmce_perfmon_cnt_info info;

	if (!carmel_pmu_counter_valid(idx, &info)) {
		pr_err("CPU%u checking wrong counter %d overflow status\n",
			smp_processor_id(), idx);
	} else {
		counter = ARMV8_IDX_TO_COUNTER(idx);
		ret = pmnc & BIT(counter);
	}

	return ret;
}

static inline u32 carmel_pmu_read_counter(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	struct dmce_perfmon_cnt_info info;
	u32 value = 0;

	if (carmel_pmu_counter_valid(idx, &info)){
		//pr_err("\nPerfmon: carmel_pmu_read_counter, idx = %d\n", idx);
		//pr_err("Perfmon: carmel_pmu_read_counter: Now it is reading 0x%x\n", idx);
		if(idx == FIRST_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_READ, NV_PMEVCNTR0_EL0, &value);
		else if(idx == SECOND_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_READ, NV_PMEVCNTR1_EL0, &value);
		else {
			pr_err("Perfmon: idx is not equal 1 or 2, please check\n");
		}
	}
	else
		pr_err("CPU%u reading wrong counter %d\n",
			smp_processor_id(), idx);

	return value;
}

static inline void carmel_pmu_write_counter(struct perf_event *event, u32 value)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	struct dmce_perfmon_cnt_info info;

	if (carmel_pmu_counter_valid(idx, &info)){
		pr_err("Perfmon: carmel_pmu_write_counter: Now writing idx = 0x%x, value = 0x%x\n", idx, value);
		if(idx == FIRST_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMEVCNTR0_EL0, &value);
		else if(idx == SECOND_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMEVCNTR1_EL0, &value);
		else {
			pr_err("Perfmon: idx is not equal 1 or 2, please check\n");
		}
	}
	else
		pr_err("CPU%u writing wrong counter %d\n",
			smp_processor_id(), idx);

}

static inline void carmel_pmu_write_evtype(int idx, u32 val)
{
	int group, flag_select_group;
	struct dmce_perfmon_cnt_info info;

	val &= CARMEL_EVTYPE_EVENT_ID;
	if (carmel_pmu_counter_valid(idx, &info)) {
		flag_select_group = 1;
		group = get_uncore_group(val, flag_select_group);
		if(group < 0) {
			pr_err("Perfmon Error: Try to write an invalid group\n");
			return ;
		}

		pr_err("Perfmon: carmel_pmu_write_evtype: Now it is writing idx = 0x%x, val = 0x%x\n", idx, val);
		if(idx == FIRST_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMEVTYPER0_EL0, &val);
		else if(idx == SECOND_TRACKING_EVENT)
			mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMEVTYPER1_EL0, &val);
		else {
			pr_err("Perfmon: idx is not equal 1 or 2, please check\n");
		}
	}

}

static inline int carmel_pmu_enable_counter(int idx)
{
	struct dmce_perfmon_cnt_info info;
	u32 data = 0;

	if (carmel_pmu_counter_valid(idx, &info)) {
		data = BIT(ARMV8_IDX_TO_COUNTER(idx));
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMCNTENSET_EL0, &data);
	} else {
		pr_err("CPU%u enabling wrong PMNC counter %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_disable_counter(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(ARMV8_IDX_TO_COUNTER(idx));
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMCNTENCLR_EL0, &data);
	} else {
		pr_err("CPU%u disabling wrong PMNC counter %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_enable_intens(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(ARMV8_IDX_TO_COUNTER(idx));
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMINTENSET_EL1, &data);
	} else {
		pr_err("CPU%u enabling wrong PMNC counter enable %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_disable_intens(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(ARMV8_IDX_TO_COUNTER(idx));
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMINTENCLR_EL1, &data);
		mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMOVSCLR_EL0, &data);
	} else {
		pr_err("CPU%u enabling wrong PMNC counter disable %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline u32 carmel_pmu_getreset_flags(void)
{
	u32 data = 0;

	mce_perfmon_rw(DMCE_PERFMON_COMMAND_READ, NV_PMINTENCLR_EL1, &data);
	data &= ARMV8_OVSR_MASK;
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMOVSCLR_EL0, &data);

	return data;
}

static void carmel_pmu_enable_event(struct perf_event *event)
{
	unsigned long flags;
	struct hw_perf_event *hwc = &event->hw;
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);
	struct pmu_hw_events *events = this_cpu_ptr(cpu_pmu->hw_events);
	int idx = hwc->idx;

	/*
	 * Enable counter and interrupt, and set the counter to count
	 * the event that we're interested in.
	 */
	raw_spin_lock_irqsave(&events->pmu_lock, flags);

	/*
	 * Disable counter
	 */
	carmel_pmu_disable_counter(idx);

	/*
	 * Set event (if destined for PMNx counters).
	 */
	carmel_pmu_write_evtype(idx, hwc->config_base);

	/*
	 * Enable interrupt for this counter
	 */
	carmel_pmu_enable_intens(idx);

	/*
	 * Enable counter
	 */
	carmel_pmu_enable_counter(idx);

	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static void carmel_pmu_disable_event(struct perf_event *event)
{
	unsigned long flags;
	struct hw_perf_event *hwc = &event->hw;
	struct arm_pmu *uncore_pmu = to_arm_pmu(event->pmu);
	struct pmu_hw_events *events = this_cpu_ptr(uncore_pmu->hw_events);
	int idx = hwc->idx;

	/*
	 * Disable counter and interrupt
	 */
	raw_spin_lock_irqsave(&events->pmu_lock, flags);

	/*
	 * Disable counter
	 */
	carmel_pmu_disable_counter(idx);

	/*
	 * Disable interrupt for this counter
	 */
	carmel_pmu_disable_intens(idx);

	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static irqreturn_t carmel_pmu_handle_irq(int irq_num, void *dev)
{
	u32 pmovsr;
	struct perf_sample_data data;
	struct arm_pmu *uncore_pmu = (struct arm_pmu *)dev;
	struct pmu_hw_events *cpuc = this_cpu_ptr(uncore_pmu->hw_events);
	struct pt_regs *regs;
	int idx;

	/*
	 * Get and reset the IRQ flags
	 */
	pmovsr = carmel_pmu_getreset_flags();

	/*
	 * Did an overflow occur?
	 */
	if (!carmel_pmu_has_overflowed(pmovsr))
		return IRQ_NONE;

	/*
	 * Handle the counter(s) overflow(s)
	 */
	regs = get_irq_regs();

	cpuc = this_cpu_ptr(&cpu_hw_events);
	for (idx = 0; idx < uncore_pmu->num_events; ++idx) {
		struct perf_event *event = cpuc->events[idx];
		struct hw_perf_event *hwc;

		/* Ignore if we don't have an event. */
		if (!event)
			continue;

		/*
		 * We have a single interrupt for all counters. Check that
		 * each counter has overflowed before we process it.
		 */
		if (!carmel_pmu_counter_has_overflowed(pmovsr, idx))
			continue;

		hwc = &event->hw;
		armpmu_event_update(event);
		perf_sample_data_init(&data, 0, hwc->last_period);
		if (!armpmu_event_set_period(event))
			continue;

		if (perf_event_overflow(event, &data, regs))
			uncore_pmu->disable(event);
	}

	/*
	 * Handle the pending perf events.
	 *
	 * Note: this call *must* be run with interrupts disabled. For
	 * platforms that can have the PMU interrupts raised as an NMI, this
	 * will not work.
	 */
	irq_work_run();

	return IRQ_HANDLED;
}

static void carmel_pmu_start(struct arm_pmu *uncore_pmu)
{
	unsigned long flags;
	struct pmu_hw_events *events = this_cpu_ptr(uncore_pmu->hw_events);
	u32 value = 0;


	raw_spin_lock_irqsave(&events->pmu_lock, flags);
	/* Enable all counters */
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_READ, NV_PMCR_EL0, &value);
	value |= ARMV8_PMCR_E;
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMCR_EL0, &value);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static void carmel_pmu_stop(struct arm_pmu *uncore_pmu)
{
	unsigned long flags;
	struct pmu_hw_events *events = this_cpu_ptr(uncore_pmu->hw_events);
	u32 value = 0;

	raw_spin_lock_irqsave(&events->pmu_lock, flags);
	/* Disable all counters */
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_READ, NV_PMCR_EL0, &value);
	value &= ~ARMV8_PMCR_E;
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMCR_EL0, &value);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static int carmel_pmu_get_event_idx(struct pmu_hw_events *cpuc,
				  struct perf_event *event)
{
	int idx;
	int group, flag_select_group;
	struct arm_pmu *uncore_pmu = to_arm_pmu(event->pmu);

	/*
	 * For anything other than a cycle counter, try and use
	 * the events counters
	 */
	for (idx = ARMV8_IDX_COUNTER0; idx < uncore_pmu->num_events; ++idx) {
		if (!test_and_set_bit(idx, cpuc->used_mask)) {
			pr_err("Go to get_uncore_group: carmel_pmu_get_event_idx");
			flag_select_group = 1;
			group = get_uncore_group(event->attr.config & CARMEL_EVTYPE_EVENT_ID, flag_select_group);
                        alloc_carmel_ctr(idx, group, event->attr.config & CARMEL_EVTYPE_EVENT_ID);
			return idx;
		}
	}

	/* The counters are all in use. */
	return -EAGAIN;
}

/*
 * Add an event filter to a given event. This will only work for PMUv2 PMUs.
 */
static int carmel_pmu_set_event_filter(struct hw_perf_event *event,
				     struct perf_event_attr *attr)
{
	unsigned long config_base = 0;

	if (attr->exclude_idle)
		return -EPERM;
	if (attr->exclude_user)
		config_base |= ARMV8_EXCLUDE_EL0;
	if (attr->exclude_kernel)
		config_base |= ARMV8_EXCLUDE_EL1;
	if (!attr->exclude_hv)
		config_base |= ARMV8_INCLUDE_EL2;

	/*
	 * Install the filter into config_base as this is used to
	 * construct the event type.
	 */
	event->config_base = config_base;

	return 0;
}

static void carmel_pmu_reset(void *info)
{
	struct arm_pmu *uncore_pmu = (struct arm_pmu *)info;
	u32 idx, nb_cnt = uncore_pmu->num_events;
	int value = 0;

	/* The counter and interrupt enable registers are unknown at reset. */
	for (idx = ARMV8_IDX_CYCLE_COUNTER; idx < nb_cnt; ++idx)
		clear_carmel_ctr(idx);

	/* Initialize & Reset PMNC: C and P bits. */
	value |= ARMV8_PMCR_P;
	mce_perfmon_rw(DMCE_PERFMON_COMMAND_WRITE, NV_PMCR_EL0, &value);
}

static int carmel_pmu_map_event(struct perf_event *event)
{
	if ((event->attr.config & CARMEL_EVTYPE_MASK) == CARMEL_EVTYPE_MASK)
		return carmel_pmu_map_raw_event(ARMV8_EVTYPE_EVENT,
					event->attr.config);

	return -ENOENT;
}

static int carmel_uncore_pmu_init(struct arm_pmu *uncore_pmu)
{
	uncore_pmu->handle_irq		= carmel_pmu_handle_irq,
	uncore_pmu->enable		= carmel_pmu_enable_event,
	uncore_pmu->disable		= carmel_pmu_disable_event,
	uncore_pmu->read_counter	= carmel_pmu_read_counter,
	uncore_pmu->write_counter	= carmel_pmu_write_counter,
	uncore_pmu->get_event_idx	= carmel_pmu_get_event_idx,
	uncore_pmu->start		= carmel_pmu_start,
	uncore_pmu->stop		= carmel_pmu_stop,
	uncore_pmu->reset		= carmel_pmu_reset,
	uncore_pmu->max_period		= (1LLU << 32) - 1,
	uncore_pmu->set_event_filter	= carmel_pmu_set_event_filter;
	uncore_pmu->name		= "carmel_uncore_pmu";
	uncore_pmu->map_event		= carmel_pmu_map_event;
	uncore_pmu->num_events		= CARMEL_MAX_UNCORE_CNTS + 1;

	return 0;
}

static const struct of_device_id carmel_pmu_of_device_ids[] = {
	{.compatible = "nvidia,carmel-pmu", .data = carmel_uncore_pmu_init},
	{},
};

static int carmel_pmu_device_probe(struct platform_device *pdev)
{
	return arm_pmu_device_probe(pdev, carmel_pmu_of_device_ids, NULL);
}

static struct platform_driver carmel_pmu_driver = {
	.driver = {
		.name = "carmel-pmu",
		.of_match_table = carmel_pmu_of_device_ids,
	},
	.probe = carmel_pmu_device_probe,
};

static int __init register_pmu_driver(void)
{
	return platform_driver_register(&carmel_pmu_driver);
}
device_initcall(register_pmu_driver);
#endif
