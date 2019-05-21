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
#include <asm/sysreg.h>

#include <linux/of.h>
#include <linux/perf/arm_pmu.h>
#include <linux/platform_device.h>

#include <soc/tegra/chip-id.h>

// Global registers
#define SYS_NV_PMSELR_EL0     sys_reg(3, 3, 15, 5, 1)

// Unit registers
#define SYS_NV_PMCNTENSET_EL0 sys_reg(3, 3, 15, 4, 0)
#define SYS_NV_PMCNTENCLR_EL0 sys_reg(3, 3, 15, 4, 1)
#define SYS_NV_PMOVSSET_EL0   sys_reg(3, 3, 15, 4, 2)
#define SYS_NV_PMOVSCLR_EL0   sys_reg(3, 3, 15, 4, 3)
#define SYS_NV_PMCR_EL0       sys_reg(3, 3, 15, 4, 4)
#define SYS_NV_PMINTENSET_EL1 sys_reg(3, 0, 15, 2, 0)
#define SYS_NV_PMINTENCLR_EL1 sys_reg(3, 0, 15, 2, 1)

// Counter registers
#define SYS_NV_PMEVCNTR0_EL0  sys_reg(3, 3, 15, 0, 0)
#define SYS_NV_PMEVCNTR1_EL0  sys_reg(3, 3, 15, 0, 1)
#define SYS_NV_PMEVTYPER0_EL0 sys_reg(3, 3, 15, 2, 0)
#define SYS_NV_PMEVTYPER1_EL0 sys_reg(3, 3, 15, 2, 1)

#ifdef PMCDBG
#define PMCPRINT(msg, ...) pr_err("carmel_pmu %s:%d "msg, __func__, __LINE__, ## __VA_ARGS__)
#else
#define PMCPRINT(msg, ...)
#endif

/*
 * Carmel uncore perfmon supports two counters per unit
 */
#define CARMEL_MAX_UNCORE_CNTS		2

/*
 * NV_PMCR: config reg
 */
#define CARMEL_PMCR_E		(1 << 0) /* Enable all counters */
#define CARMEL_PMCR_P		(1 << 1) /* Reset all counters */

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
	CARMEL_PMU_L2D_CACHE_ST = 0x51,
	CARMEL_PMU_L2D_CACHE_REFILL_LD = 0x52,
	CARMEL_PMU_L2D_CACHE_REFILL_ST = 0x53,
	CARMEL_PMU_L2D_CACHE_WB_VIC_TIM = 0x56,

	CARMEL_PMU_EVENT_NV_INT_START = 0x200,
	CARMEL_PMU_EVENT_NV_INT_END = 0x208,

};

/**
 * Data for each uncore perfmon counter
 */
struct perfmon_cnt_info {
	uint8_t counter;	/* Event id */
	uint8_t group;		/* Group selector */
	uint8_t unit;		/* Unit selector */
	uint8_t index;		/* Virtual Index */
	uint8_t idx;		/* Physical Index */
	uint8_t valid;		/* Valid info */
};

static struct perfmon_cnt_info carmel_uncore_event[CARMEL_MAX_UNCORE_CNTS];


static const u32 nv_pmevcntrs[] = { SYS_NV_PMEVCNTR0_EL0, SYS_NV_PMEVCNTR1_EL0 };
static const u32 nv_pmevtypers[] = { SYS_NV_PMEVTYPER0_EL0, SYS_NV_PMEVTYPER1_EL0 };

static void sys_counter_write(u32 reg, u32 val)
{
	switch(reg) {
		case SYS_NV_PMEVCNTR0_EL0: write_sysreg_s(val, SYS_NV_PMEVCNTR0_EL0); return;
		case SYS_NV_PMEVCNTR1_EL0: write_sysreg_s(val, SYS_NV_PMEVCNTR1_EL0); return;
		case SYS_NV_PMEVTYPER0_EL0: write_sysreg_s(val, SYS_NV_PMEVTYPER0_EL0); return;
		case SYS_NV_PMEVTYPER1_EL0: write_sysreg_s(val, SYS_NV_PMEVTYPER1_EL0); return;
		default:
			WARN(1, "Illegal counter write\n");
			return;
	}
}

static u32 sys_counter_read(u32 reg)
{
	switch(reg) {
		case SYS_NV_PMEVCNTR0_EL0: return read_sysreg_s(SYS_NV_PMEVCNTR0_EL0);
		case SYS_NV_PMEVCNTR1_EL0: return read_sysreg_s(SYS_NV_PMEVCNTR1_EL0);
		case SYS_NV_PMEVTYPER0_EL0: return read_sysreg_s(SYS_NV_PMEVTYPER0_EL0);
		case SYS_NV_PMEVTYPER1_EL0: return read_sysreg_s(SYS_NV_PMEVTYPER1_EL0);
		default:
			WARN(1, "Illegal counter read\n");
			return 0;
	}
}

static inline int get_ctr_info(u32 idx, struct perfmon_cnt_info *info)
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
	struct perfmon_cnt_info info;

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
			pr_err("carmel_pmu: failed to allocate Carmel uncore ctr\n");
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

static inline int get_uncore_group(u32 event, bool set)
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
		case CARMEL_PMU_EVENT_NV_INT_START ... CARMEL_PMU_EVENT_NV_INT_END:
		{
			group = 0x0000;
			break;
		}

		default:
			return -1;
	}

	if (set)
	{
		write_sysreg_s(group, SYS_NV_PMSELR_EL0);
	}

	return group;
}

static inline int carmel_pmu_counter_valid(u32 idx,
		struct perfmon_cnt_info *info)
{
	if (get_ctr_info(idx, info) < 0)
		return 0;
	return 1;
}

static inline int carmel_pmu_counter_has_overflowed(u32 pmnc, int idx)
{
	int ret = 0;
	struct perfmon_cnt_info info;

	if (!carmel_pmu_counter_valid(idx, &info)) {
		pr_err("carmel_pmu: CPU%u checking wrong counter %d overflow status\n",
			smp_processor_id(), idx);
	} else {
		ret = pmnc & BIT(idx);
	}

	return ret;
}

static inline u32 carmel_pmu_read_counter(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	struct perfmon_cnt_info info;
	u32 value = 0;

	PMCPRINT("Read counter %d\n", idx);

	if (carmel_pmu_counter_valid(idx, &info)){
		value = sys_counter_read(nv_pmevcntrs[idx]);
	}
	else
		pr_err("carmel_pmu: CPU%u reading invalid counter %d\n",
			smp_processor_id(), idx);

	return value;
}

static inline void carmel_pmu_write_counter(struct perf_event *event, u32 value)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	struct perfmon_cnt_info info;

	if (carmel_pmu_counter_valid(idx, &info)){
		sys_counter_write(nv_pmevcntrs[idx], value);
	}
	else
		pr_err("carmel_pmu: CPU%u writing invalid counter %d\n",
			smp_processor_id(), idx);

}

static inline void carmel_pmu_write_evtype(int idx, u32 val)
{
	int group;
	struct perfmon_cnt_info info;

	if (carmel_pmu_counter_valid(idx, &info)) {
		group = get_uncore_group(val, true);
		if(group < 0) {
			pr_err("carmel_pmu: Try to write an invalid group\n");
			return ;
		}

		sys_counter_write(nv_pmevtypers[idx], val);
	}

}

static inline int carmel_pmu_enable_counter(int idx)
{
	struct perfmon_cnt_info info;
	u32 data = 0;

	if (carmel_pmu_counter_valid(idx, &info)) {
		data = BIT(idx);
		write_sysreg_s(data, SYS_NV_PMCNTENSET_EL0);
	} else {
		pr_err("carmel_pmu: CPU%u enabling wrong PMNC counter %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_disable_counter(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(idx);
		write_sysreg_s(data, SYS_NV_PMCNTENCLR_EL0);
	} else {
		pr_err("carmel_pmu: CPU%u disabling wrong PMNC counter %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_enable_intens(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(idx);
		write_sysreg_s(data, SYS_NV_PMINTENSET_EL1);
	} else {
		pr_err("carmel_pmu: CPU%u enabling wrong PMNC counter enable %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline int carmel_pmu_disable_intens(int idx)
{
	u32 data = 0;

	if (idx <= CARMEL_MAX_UNCORE_CNTS) {
		data = BIT(idx);
		write_sysreg_s(data, SYS_NV_PMINTENCLR_EL1);
		write_sysreg_s(data, SYS_NV_PMOVSCLR_EL0);
	} else {
		pr_err("carmel_pmu: CPU%u enabling wrong PMNC counter disable %d\n",
			smp_processor_id(), idx);
		return -EINVAL;
	}

	return idx;
}

static inline u32 carmel_pmu_getreset_flags(void)
{
	u32 inten = 0;
	u32 ovf = 0;

	// find counters with interrupts enabled
	inten = read_sysreg_s(SYS_NV_PMINTENCLR_EL1);

	// find counters that have overflowed
	ovf = read_sysreg_s(SYS_NV_PMOVSCLR_EL0);

	// only handle/clear counters with interrupts enable
	ovf &= inten;

	// clear overflows
	write_sysreg_s(ovf, SYS_NV_PMOVSCLR_EL0);

	// toggle interrupt enable
	write_sysreg_s(inten, SYS_NV_PMINTENCLR_EL1);
	write_sysreg_s(inten, SYS_NV_PMINTENSET_EL1);

	return ovf;
}

static void carmel_pmu_enable_event(struct perf_event *event)
{
	unsigned long flags;
	struct hw_perf_event *hwc = &event->hw;
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);
	struct pmu_hw_events *events = this_cpu_ptr(cpu_pmu->hw_events);
	int idx = hwc->idx;

	PMCPRINT("Enable counter %d with event %ld\n", idx, hwc->config_base);
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

	PMCPRINT("Disable counter %d\n", idx);
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

	PMCPRINT("Core %u handling IRQ %d\n",smp_processor_id(), irq_num);

	/*
	 * Get and reset the IRQ flags
	 */
	pmovsr = carmel_pmu_getreset_flags();

	/*
	 * Did an overflow occur?
	 */
	if (!pmovsr) {
		pr_err("carmel_pmu: no ovf detected, not handled\n");
		return IRQ_NONE;
	}

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

	PMCPRINT("Starting Carmel PMU\n");

	raw_spin_lock_irqsave(&events->pmu_lock, flags);
	/* Enable all counters */
	value = read_sysreg_s(SYS_NV_PMCR_EL0);
	value |= CARMEL_PMCR_E;
	write_sysreg_s(value, SYS_NV_PMCR_EL0);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static void carmel_pmu_stop(struct arm_pmu *uncore_pmu)
{
	unsigned long flags;
	struct pmu_hw_events *events = this_cpu_ptr(uncore_pmu->hw_events);
	u32 value = 0;

	PMCPRINT("Stopping Carmel PMU\n");

	raw_spin_lock_irqsave(&events->pmu_lock, flags);
	/* Disable all counters */
	value = read_sysreg_s(SYS_NV_PMCR_EL0);
	value &= ~CARMEL_PMCR_E;
	write_sysreg_s(value, SYS_NV_PMCR_EL0);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static int carmel_pmu_get_event_idx(struct pmu_hw_events *cpuc,
				  struct perf_event *event)
{
	int idx;
	int group;
	struct arm_pmu *uncore_pmu = to_arm_pmu(event->pmu);

	PMCPRINT("Finding counter for event %lld\n", event->attr.config);

	for (idx = 0; idx < uncore_pmu->num_events; ++idx) {
		if (!test_and_set_bit(idx, cpuc->used_mask)) {
			group = get_uncore_group(event->attr.config, true);
			alloc_carmel_ctr(idx, group, event->attr.config);
			return idx;
		}
	}

	/* The counters are all in use. */
	return -EAGAIN;
}

static void carmel_pmu_reset(void *info)
{
	struct arm_pmu *uncore_pmu = (struct arm_pmu *)info;
	u32 idx, nb_cnt = uncore_pmu->num_events;
	int value = 0;

	PMCPRINT("Resetting Carmel PMU\n");

	/* The counter and interrupt enable registers are unknown at reset. */
	for (idx = 0; idx < nb_cnt; ++idx)
		clear_carmel_ctr(idx);

	/* Initialize & Reset PMNC: C and P bits. */
	value |= CARMEL_PMCR_P;
	write_sysreg_s(value, SYS_NV_PMCR_EL0);
}

static int carmel_pmu_map_event(struct perf_event *event)
{
	PMCPRINT("Mapping event %lld\n", event->attr.config);

	if (get_uncore_group(event->attr.config, true) < 0)
		return -ENOENT;
	else
		return (int)(event->attr.config);
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
	uncore_pmu->name		= "carmel_uncore_pmu";
	uncore_pmu->map_event		= carmel_pmu_map_event;
	uncore_pmu->num_events		= CARMEL_MAX_UNCORE_CNTS;

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
	if (tegra_platform_is_silicon())
		return platform_driver_register(&carmel_pmu_driver);
	return 0;
}
device_initcall(register_pmu_driver);
#endif
