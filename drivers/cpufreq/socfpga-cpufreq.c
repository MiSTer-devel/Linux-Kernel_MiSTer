// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq driver for the Intel/Altera Cyclone V SoC FPGA (DE10-Nano).
 *
 * Reprograms the Cyclone V main PLL VCO and the MPU / main / dbg-AT /
 * cfg-s2f-user0 dividers together, so that the CPU can be clocked at
 * 400 / 800 (stock) / 1000 / 1200 MHz while every other clock derived from
 * the main PLL keeps its stock frequency.
 *
 * Copyright (C) 2022 Michael Huang <coolbho3000@gmail.com>
 */

#include <linux/bits.h>
#include <linux/cpufreq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/wait_bit.h>

#define DRIVER_AUTHOR "Michael Huang <coolbho3000@gmail.com>"
#define DRIVER_DESCRIPTION "DE10 Nano cpufreq driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

// Address offsets
#define CLKMGR_GEN5_BYPASS     0x04
#define CLKMGR_STAT            0x14
#define MAINPLL_VCO            0x40
#define MAINPLL_MPUCLK         0x48
#define MAINPLL_CFGS2FUSER0CLK 0x5c
#define ALTR_MPUCLK            0xe0
#define ALTR_MAINCLK           0xe4
#define ALTR_DBGATCLK          0xe8

// Register masks
#define CLKMGR_BYPASS_MAINPLL BIT(0)
#define CLKMGR_STAT_BUSY      BIT(0)

// Offsets for calculating VCO frequency/register value
#define VCO_NUMER_OFFSET 3
#define VCO_DENOM_OFFSET 16

// Mask for setting main PLL VCO
#define MAINPLL_VCO_MASK 0x003ffff8

static DEFINE_MUTEX(socfpga_cpufreq_mutex);

static void __iomem *socfpga_cpufreq_clk_mgr_base_addr;

static u32 socfpga_cpufreq_osc1_hz;

struct socfpga_clock_data {
	u32 vco_numer; // Numerator for calculating VCO register
	u32 vco_denom; // Denominator for calculating VCO register
	u32 alteragrp_mpuclk; // Divides the VCO frequency by the value+1
	u32 alteragrp_mainclk; // Divides the VCO frequency by the value+1
	u32 alteragrp_dbgatclk; // Divides the VCO frequency by the value+1
	u32 mainpll_cfgs2fuser0clk; // Divides the VCO frequency by the value+1
};

// 1200 MHz overclock
static const struct socfpga_clock_data clock_data_1200000 = {
	.vco_numer = 95, // 25 MHz * (95 + 1) / (0 + 1) = 2400 MHz
	.vco_denom = 0,
	.alteragrp_mpuclk = 1, // 2400 MHz / (1 + 1) = 1200 MHz
	.alteragrp_mainclk = 5, // 2400 MHz / (5 + 1) = 400 MHz
	.alteragrp_dbgatclk = 5, // 2400 MHz / (5 + 1) = 400 MHz
	.mainpll_cfgs2fuser0clk = 23, // 2400 MHz / (23 + 1) = 100 MHz
};

// 1000 MHz overclock
static const struct socfpga_clock_data clock_data_1000000 = {
	.vco_numer = 79, // 25 MHz * (79 + 1) / (0 + 1) = 2000 MHz
	.vco_denom = 0,
	.alteragrp_mpuclk = 1, // 2000 MHz / (1 + 1) = 1000 MHz
	.alteragrp_mainclk = 4, // 2000 MHz / (4 + 1) = 400 MHz
	.alteragrp_dbgatclk = 4, // 2000 MHz / (4 + 1) = 400 MHz
	.mainpll_cfgs2fuser0clk = 19, // 2000 MHz / (19 + 1) = 100 MHz
};

// 800 MHz. Default for -I7 and -C7 speed grades
static const struct socfpga_clock_data clock_data_800000 = {
	.vco_numer = 63, // 25 MHz * (63 + 1) / (0 + 1) = 1600 MHz
	.vco_denom = 0,
	.alteragrp_mpuclk = 1, // 1600 MHz / (1 + 1) = 800 MHz
	.alteragrp_mainclk = 3, // 1600 MHz / (3 + 1) = 400 MHz
	.alteragrp_dbgatclk = 3, // 1600 MHz / (3 + 1) = 400 MHz
	.mainpll_cfgs2fuser0clk = 15, // 1600 MHz / (15 + 1) = 100 MHz
};

// 400 MHz underclock
static const struct socfpga_clock_data clock_data_400000 = {
	.vco_numer = 63, // 25 MHz * (63 + 1) / (0 + 1) = 1600 MHz
	.vco_denom = 0,
	.alteragrp_mpuclk = 3, // 1600 MHz / (3 + 1) = 400 MHz
	.alteragrp_mainclk = 3, // 1600 MHz / (3 + 1) = 400 MHz
	.alteragrp_dbgatclk = 3, // 1600 MHz / (3 + 1) = 400 MHz
	.mainpll_cfgs2fuser0clk = 15, // 1600 MHz / (15 + 1) = 100 MHz
};

#define SOCFPGA_CPUFREQ_ROW(freq_khz, f) \
	{ \
		.driver_data = (unsigned int) &clock_data_##freq_khz, \
		.frequency = freq_khz, \
		.flags = f, \
	}

static struct cpufreq_frequency_table freq_table[] = {
	// Mark OC rows as boost freq to prevent cpufreq from setting them on
	// boot. The user should have control of this.
	SOCFPGA_CPUFREQ_ROW(1200000, CPUFREQ_BOOST_FREQ),
	SOCFPGA_CPUFREQ_ROW(1000000, CPUFREQ_BOOST_FREQ),
	SOCFPGA_CPUFREQ_ROW(800000, 0),
	SOCFPGA_CPUFREQ_ROW(400000, 0),
	{
		.driver_data = 0,
		.frequency   = CPUFREQ_TABLE_END,
	},
};

static inline u32 calculate_vco_reg(u32 numer, u32 denom)
{
	u32 vco_reg;

	vco_reg = readl(socfpga_cpufreq_clk_mgr_base_addr + MAINPLL_VCO);
	return (vco_reg & ~MAINPLL_VCO_MASK) | (((denom << VCO_DENOM_OFFSET) |
		(numer << VCO_NUMER_OFFSET)) & MAINPLL_VCO_MASK);
}

static inline u64 calculate_vco_clock_hz(u32 numer, u32 denom)
{
	u64 vco_freq = socfpga_cpufreq_osc1_hz;

	vco_freq *= (numer + 1);
	do_div(vco_freq, (denom + 1));
	return vco_freq;
}

static inline u64 get_vco_clock_hz(void)
{
	u32 numer, denom, vco_reg;

	vco_reg = readl(socfpga_cpufreq_clk_mgr_base_addr + MAINPLL_VCO);
	numer = vco_reg >> VCO_NUMER_OFFSET;
	denom = vco_reg >> VCO_DENOM_OFFSET;
	return calculate_vco_clock_hz(numer, denom);
}

static void wait_for_fsm(void)
{
	wait_on_bit((void *)(socfpga_cpufreq_clk_mgr_base_addr + CLKMGR_STAT),
		CLKMGR_STAT_BUSY, TASK_UNINTERRUPTIBLE);
}

static int socfpga_verify_speed(struct cpufreq_policy_data *policy)
{
	return cpufreq_frequency_table_verify(policy);
}

static unsigned int socfpga_get(unsigned int cpu)
{
	u32 alteragrp_mpuclk_reg, mpuclk_cnt_reg;
	u64 mpuclk_freq;

	mutex_lock(&socfpga_cpufreq_mutex);

	// Get value of alteragrp_mpuclk
	alteragrp_mpuclk_reg = readl(socfpga_cpufreq_clk_mgr_base_addr +
		ALTR_MPUCLK);

	// Get value of mpuclk_cnt
	mpuclk_cnt_reg = readl(socfpga_cpufreq_clk_mgr_base_addr +
		MAINPLL_MPUCLK);

	// Get and calculate VCO clock
	mpuclk_freq = get_vco_clock_hz();

	mutex_unlock(&socfpga_cpufreq_mutex);

	// Divide by value of registers
	do_div(mpuclk_freq, alteragrp_mpuclk_reg + 1);
	do_div(mpuclk_freq, mpuclk_cnt_reg + 1);

	// Convert to KHz
	do_div(mpuclk_freq, 1000);

	return (unsigned int) mpuclk_freq;
}

static inline void set_dividers(const struct socfpga_clock_data *clock_data)
{
	// Put main PLL into bypass
	writel(CLKMGR_BYPASS_MAINPLL, socfpga_cpufreq_clk_mgr_base_addr +
		CLKMGR_GEN5_BYPASS);
	wait_for_fsm();

	// Hardware-managed clocks
	writel(clock_data->alteragrp_mpuclk,
		socfpga_cpufreq_clk_mgr_base_addr + ALTR_MPUCLK);
	writel(clock_data->alteragrp_mainclk,
		socfpga_cpufreq_clk_mgr_base_addr + ALTR_MAINCLK);
	writel(clock_data->alteragrp_dbgatclk,
		socfpga_cpufreq_clk_mgr_base_addr + ALTR_DBGATCLK);

	// Software-managed clocks
	writel(clock_data->mainpll_cfgs2fuser0clk,
		socfpga_cpufreq_clk_mgr_base_addr + MAINPLL_CFGS2FUSER0CLK);

	// Other affected clocks are driven by peripheral PLL on DE10 Nano

	// Put main PLL out of bypass
	writel(0, socfpga_cpufreq_clk_mgr_base_addr + CLKMGR_GEN5_BYPASS);
	wait_for_fsm();
}

static inline void set_vco_freq(const struct socfpga_clock_data *clock_data)
{
	// Put main PLL into bypass
	writel(CLKMGR_BYPASS_MAINPLL, socfpga_cpufreq_clk_mgr_base_addr +
		CLKMGR_GEN5_BYPASS);
	wait_for_fsm();

	// Set VCO register
	writel(calculate_vco_reg(clock_data->vco_numer, clock_data->vco_denom),
		socfpga_cpufreq_clk_mgr_base_addr + MAINPLL_VCO);

	// Put main PLL out of bypass
	writel(0, socfpga_cpufreq_clk_mgr_base_addr + CLKMGR_GEN5_BYPASS);
	wait_for_fsm();
}

static int socfpga_target_index(struct cpufreq_policy *policy,
	unsigned int index)
{
	const struct socfpga_clock_data *clock_data;
	u64 current_vco_clock_hz, target_vco_clock_hz;

	clock_data = (const struct socfpga_clock_data *)
		freq_table[index].driver_data;

	mutex_lock(&socfpga_cpufreq_mutex);

	current_vco_clock_hz = get_vco_clock_hz();
	target_vco_clock_hz = calculate_vco_clock_hz(clock_data->vco_numer,
		clock_data->vco_denom);

	if (target_vco_clock_hz == current_vco_clock_hz) {
		set_dividers(clock_data);
	} else if (target_vco_clock_hz > current_vco_clock_hz) {
		set_dividers(clock_data);
		set_vco_freq(clock_data);
	} else if (target_vco_clock_hz < current_vco_clock_hz) {
		set_vco_freq(clock_data);
		set_dividers(clock_data);
	}

	mutex_unlock(&socfpga_cpufreq_mutex);

	return 0;
}

static int socfpga_cpu_init(struct cpufreq_policy *policy)
{
	policy->cur = socfpga_get(policy->cpu);
	policy->cpuinfo.transition_latency = 1000000;
	/*
	 * Deliberately do NOT set cpuinfo.max_freq here.
	 * cpufreq_frequency_table_cpuinfo() derives it from the table, skipping
	 * the CPUFREQ_BOOST_FREQ rows while boost is off, so the default ceiling
	 * is the stock 800 MHz.  Overclocking to 1.0/1.2 GHz is opt-in via the
	 * standard boost knob (/sys/devices/system/cpu/cpufreq/boost), wired up
	 * by .set_boost below.
	 *
	 * The previous forward-port set cpuinfo.max_freq = 1200000 here; on 6.18
	 * that value leaks into the default policy->max (through the FREQ_QOS_MAX
	 * resolve), so with the performance default governor the board silently
	 * booted at 1.2 GHz -- an unrequested overclock.  Boost support replaces
	 * that hack.
	 */
	policy->freq_table = freq_table;
	cpumask_setall(policy->cpus);
	return 0;
}

static void socfpga_cpu_exit(struct cpufreq_policy *policy)
{
	socfpga_cpufreq_clk_mgr_base_addr = NULL;
}

static struct freq_attr *socfpga_cpufreq_attr[] = {
	/*
	 * Both scaling_available_frequencies and scaling_boost_frequencies are
	 * created by the cpufreq core itself: the former for every policy with a
	 * freq_table, the latter once the driver advertises boost via ->set_boost
	 * (below).  Listing either here would make the core's sysfs_create_file()
	 * fail with -EEXIST and abort policy creation, so this array is empty.
	 */
	NULL,
};

static struct cpufreq_driver socfpga_cpufreq_driver = {
	.verify        = socfpga_verify_speed,
	.target_index  = socfpga_target_index,
	.get           = socfpga_get,
	.init          = socfpga_cpu_init,
	.exit          = socfpga_cpu_exit,
	.name          = "socfpga",
	.attr          = socfpga_cpufreq_attr,
	/*
	 * Overclocking is opt-in, gated behind the standard boost knob
	 * (echo 1 > /sys/devices/system/cpu/cpufreq/boost).  cpufreq_boost_set_sw
	 * toggles the CPUFREQ_BOOST_FREQ rows (1.0/1.2 GHz); with boost off the
	 * usable ceiling is the stock 800 MHz.  boost_enabled = false keeps boost
	 * off by default, so the board does not overclock on its own.
	 */
	.set_boost     = cpufreq_boost_set_sw,
	.boost_enabled = false,
};

static int __init socfpga_cpufreq_init(void)
{
	const __be32 *osc1_hz;
	struct device_node *clkmgr_np, *clocks_np, *osc1_np;
	int ret = -ENODEV;

	clkmgr_np = of_find_compatible_node(NULL, NULL, "altr,clk-mgr");
	if (!clkmgr_np)
		return -ENODEV;

	socfpga_cpufreq_clk_mgr_base_addr = of_iomap(clkmgr_np, 0);
	if (!socfpga_cpufreq_clk_mgr_base_addr)
		goto out_put_clkmgr;

	/*
	 * osc1's rate is not implied by the SoC dtsi -- the board DTS must
	 * carry "clock-frequency = <25000000>" on &osc1 (the stock MiSTer DTB
	 * does).  Without it the VCO maths below is meaningless, so refuse to
	 * register rather than dereference a NULL property.
	 */
	clocks_np = of_get_child_by_name(clkmgr_np, "clocks");
	if (!clocks_np)
		goto out_unmap;

	osc1_np = of_get_child_by_name(clocks_np, "osc1");
	if (!osc1_np)
		goto out_put_clocks;

	osc1_hz = of_get_property(osc1_np, "clock-frequency", NULL);
	if (!osc1_hz) {
		pr_err("socfpga-cpufreq: osc1 has no clock-frequency property\n");
		goto out_put_osc1;
	}

	socfpga_cpufreq_osc1_hz = be32_to_cpup(osc1_hz);

	of_node_put(osc1_np);
	of_node_put(clocks_np);
	of_node_put(clkmgr_np);

	return cpufreq_register_driver(&socfpga_cpufreq_driver);

out_put_osc1:
	of_node_put(osc1_np);
out_put_clocks:
	of_node_put(clocks_np);
out_unmap:
	iounmap(socfpga_cpufreq_clk_mgr_base_addr);
	socfpga_cpufreq_clk_mgr_base_addr = NULL;
out_put_clkmgr:
	of_node_put(clkmgr_np);
	return ret;
}

static void __exit socfpga_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&socfpga_cpufreq_driver);
}

module_init(socfpga_cpufreq_init);
module_exit(socfpga_cpufreq_exit);
