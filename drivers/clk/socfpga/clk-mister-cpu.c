#include <linux/bitfield.h>
#include <linux/genalloc.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <asm/fncpy.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/stop_machine.h>

#include "clk.h"
#include "clk-mister-ocram.h"

#define CM_CTRL		0x00
#define CM_BYPASS	0x04
#define CM_INTER		0x08
#define CM_STAT		0x14
#define CM_VCO		0x40
#define CM_MPU		0x48
#define CM_MAIN		0x4c
#define CM_DBG		0x50
#define CM_USER		0x5c
#define CM_MAIN_EN	0x60
#define CM_L4SRC		0x70
#define CM_PER_EN	0xa0
#define CM_PER_SRC	0xac
#define CM_ALTR_MPU	0xe0
#define CM_ALTR_MAIN	0xe4
#define CM_ALTR_DBG	0xe8

#define CM_SAFE		BIT(0)
#define CM_MAIN_BYPASS	BIT(0)
#define CM_BUSY		BIT(0)
#define CM_MAIN_LOCK	BIT(6)
#define CM_VCO_NUMER	GENMASK(15, 3)
#define CM_VCO_DENOM	GENMASK(21, 16)
#define CM_VCO_ENABLE	BIT(1)
#define CM_VCO_POWERDOWN	(BIT(0) | BIT(2))
#define CM_COUNTER	GENMASK(8, 0)
#define CM_C5_GATES	(BIT(8) | BIT(9))
#define CM_L4_PERIPH	(BIT(0) | BIT(1))
#define CM_TIMEOUT_US	1000

#define MISTER_OCRAM_BASE	0xffff0000ULL
#define MISTER_OCRAM_FLAGS	0xfffff000ULL

struct mister_cpu_rate {
	unsigned long rate;
	u32 numer;
	u32 mpu_ext;
	u32 main_int;
	u32 user;
};

static const struct mister_cpu_rate mister_rates[] = {
	{ 400000000, 63, 1, 3, 15 },
	{ 800000000, 63, 0, 3, 15 },
	{ 1000000000, 79, 0, 4, 19 },
	{ 1200000000, 95, 0, 5, 23 },
};

struct mister_transition {
	const struct mister_cpu_rate *target;
	u32 flash_gates;
};

static bool mister_clock_fault;
static unsigned long mister_osc_rate;

static u32 cm_read(u32 offset)
{
	return readl(clk_mgr_base_addr + offset);
}

static void cm_write(u32 offset, u32 value)
{
	writel(value, clk_mgr_base_addr + offset);
}

static void cm_update(u32 offset, u32 mask, u32 value)
{
	cm_write(offset, (cm_read(offset) & ~mask) | (value & mask));
}

static int cm_wait_idle(void)
{
	u32 val;

	return readl_poll_timeout_atomic(clk_mgr_base_addr + CM_STAT, val,
					!(val & CM_BUSY), 1, CM_TIMEOUT_US);
}

static unsigned long mister_cpu_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	u32 vco = cm_read(CM_VCO);
	u64 rate;

	if ((cm_read(CM_BYPASS) & CM_MAIN_BYPASS) ||
	    (cm_read(CM_CTRL) & CM_SAFE))
		return mister_osc_rate;

	rate = (u64)mister_osc_rate * (FIELD_GET(CM_VCO_NUMER, vco) + 1);
	rate = div_u64(rate, FIELD_GET(CM_VCO_DENOM, vco) + 1);
	rate = div_u64(rate, FIELD_GET(CM_COUNTER, cm_read(CM_ALTR_MPU)) + 1);
	return div_u64(rate, FIELD_GET(CM_COUNTER, cm_read(CM_MPU)) + 1);
}

static const struct mister_cpu_rate *mister_find_rate(unsigned long rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mister_rates); i++)
		if (mister_rates[i].rate == rate)
			return &mister_rates[i];
	return NULL;
}

static long mister_cpu_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *parent_rate)
{
	if (!mister_find_rate(rate))
		return -EINVAL;
	return rate;
}

static bool mister_device_enabled(const char *compatible)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, compatible) {
		if (of_device_is_available(np)) {
			of_node_put(np);
			return true;
		}
	}
	return false;
}

static int mister_flash_gates(u32 *gates)
{
	u32 src = cm_read(CM_PER_SRC);

	*gates = 0;

	if (FIELD_GET(GENMASK(1, 0), src) == 1) {
		if (mister_device_enabled("altr,socfpga-dw-mshc"))
			return -EBUSY;
		*gates |= BIT(8);
	}
	if (FIELD_GET(GENMASK(3, 2), src) == 1) {
		if (mister_device_enabled("altr,socfpga-denali-nand"))
			return -EBUSY;
		*gates |= BIT(9) | BIT(10);
	}
	if (FIELD_GET(GENMASK(5, 4), src) == 1) {
		if (mister_device_enabled("cdns,qspi-nor"))
			return -EBUSY;
		*gates |= BIT(11);
	}
	return 0;
}

static int mister_check_plan(void)
{
	const struct mister_cpu_rate *r;
	u32 vco = cm_read(CM_VCO);

	if (mister_clock_fault || (cm_read(CM_CTRL) & CM_SAFE) ||
	    (cm_read(CM_BYPASS) & CM_MAIN_BYPASS) ||
	    !(cm_read(CM_INTER) & CM_MAIN_LOCK))
		return -EIO;
	if ((vco & GENMASK(30, 24)) || !(vco & CM_VCO_ENABLE) || (vco & CM_VCO_POWERDOWN) ||
	    FIELD_GET(CM_VCO_DENOM, vco) ||
	    (cm_read(CM_L4SRC) & CM_L4_PERIPH) != CM_L4_PERIPH ||
	    (cm_read(CM_ALTR_MPU) & CM_COUNTER) != 1 ||
	    (cm_read(CM_MAIN) & CM_COUNTER) ||
	    (cm_read(CM_DBG) & CM_COUNTER))
		return -EINVAL;
	r = mister_find_rate(mister_cpu_recalc_rate(NULL, 0));
	if (!r || FIELD_GET(CM_VCO_NUMER, vco) != r->numer ||
	    (cm_read(CM_MPU) & CM_COUNTER) != r->mpu_ext ||
	    (cm_read(CM_ALTR_MAIN) & CM_COUNTER) != r->main_int ||
	    (cm_read(CM_ALTR_DBG) & CM_COUNTER) != r->main_int ||
	    (cm_read(CM_USER) & CM_COUNTER) != r->user)
		return -EINVAL;
	return 0;
}

extern int socfpga_mister_ocram(void *base, void *context);
extern const u32 socfpga_mister_ocram_sz;
static int (*mister_ocram_fn)(void *base, void *context);
static void __iomem *mister_ocram_context;

static int __init mister_setup_ocram(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct gen_pool *pool;
	struct genpool_data_align align = { .align = PAGE_SIZE };
	unsigned long allocation;
	phys_addr_t physical;
	void __iomem *mapping;

	if (!IS_ENABLED(CONFIG_ARM_SOCFPGA_CPUFREQ) || !mister_osc_rate)
		return 0;
	if (socfpga_mister_ocram_sz > MISTER_CTX_OFFSET)
		return -E2BIG;
	np = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!np)
		return -ENODEV;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -ENODEV;
	pool = gen_pool_get(&pdev->dev, NULL);
	put_device(&pdev->dev);
	if (!pool)
		return -ENODEV;
	allocation = gen_pool_alloc_algo(pool, PAGE_SIZE,
					 gen_pool_first_fit_align, &align);
	if (!allocation)
		return -ENOMEM;
	physical = gen_pool_virt_to_phys(pool, allocation);
	if (physical < MISTER_OCRAM_BASE ||
	    physical > MISTER_OCRAM_FLAGS - PAGE_SIZE) {
		gen_pool_free(pool, allocation, PAGE_SIZE);
		pr_err("MiSTer CPU: OCRAM allocation outside the usable area\n");
		return -ERANGE;
	}
	mapping = __arm_ioremap_exec(physical, PAGE_SIZE, false);
	if (!mapping) {
		gen_pool_free(pool, allocation, PAGE_SIZE);
		return -ENOMEM;
	}
	mister_ocram_fn = (void *)fncpy(mapping, &socfpga_mister_ocram,
					 socfpga_mister_ocram_sz);
	mister_ocram_context = mapping + MISTER_CTX_OFFSET;
	writel(0, mister_ocram_context + MISTER_STAGE);
	writel(0, mister_ocram_context + MISTER_ERROR);
	pr_info("MiSTer CPU: PLL transitions use OCRAM at %pa (%u bytes)\n",
		&physical, socfpga_mister_ocram_sz);
	return 0;
}
late_initcall(mister_setup_ocram);

static int mister_transition(void *arg)
{
	const struct mister_transition *tr = arg;
	const struct mister_cpu_rate *r = tr->target;
	u32 old_mpu = cm_read(CM_MPU);
	int ret;

	ret = cm_wait_idle();
	if (ret)
		return ret;
	if (FIELD_GET(CM_VCO_NUMER, cm_read(CM_VCO)) == r->numer) {
		cm_update(CM_MPU, CM_COUNTER, r->mpu_ext);
		ret = cm_wait_idle();
		if (ret) {
			cm_write(CM_MPU, old_mpu);
			if (cm_wait_idle())
				mister_clock_fault = true;
		}
		return ret;
	}

	ret = mister_ocram_fn(clk_mgr_base_addr, mister_ocram_context);
	if (ret == -EIO)
		mister_clock_fault = true;
	return ret;
}

static int mister_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct mister_transition tr = { .target = mister_find_rate(rate) };
	int ret;

	if (!tr.target)
		return -EINVAL;
	ret = mister_check_plan();
	if (ret)
		return ret;
	if (mister_cpu_recalc_rate(hw, parent_rate) == rate)
		return 0;
	ret = mister_flash_gates(&tr.flash_gates);
	if (ret)
		return ret;
	if (mister_ocram_context)
		writel(0, mister_ocram_context + MISTER_STAGE);

	if (FIELD_GET(CM_VCO_NUMER, cm_read(CM_VCO)) != tr.target->numer) {
		if (!mister_ocram_fn)
			return -ENODEV;
		writel(tr.target->numer, mister_ocram_context + MISTER_TARGET_NUMER);
		writel(tr.target->mpu_ext, mister_ocram_context + MISTER_TARGET_MPU);
		writel(tr.target->main_int, mister_ocram_context + MISTER_TARGET_MAIN);
		writel(tr.target->user, mister_ocram_context + MISTER_TARGET_USER);
		writel(tr.flash_gates, mister_ocram_context + MISTER_FLASH_GATES);
		writel(0, mister_ocram_context + MISTER_STAGE);
		writel(0, mister_ocram_context + MISTER_ERROR);
		pr_info("MiSTer CPU: OCRAM PLL transition %lu -> %lu Hz\n",
			mister_cpu_recalc_rate(hw, parent_rate), rate);
	}

	ret = stop_machine(mister_transition, &tr, NULL);
	if (ret)
		pr_err("MiSTer CPU clock transition failed: %d%s\n", ret,
		       mister_clock_fault ? "; clock state uncertain, reboot required" : "");
	if (mister_ocram_context && readl(mister_ocram_context + MISTER_STAGE))
		pr_info("MiSTer CPU: OCRAM stage=%u result=%d actual=%lu Hz\n",
			readl(mister_ocram_context + MISTER_STAGE), ret,
			mister_cpu_recalc_rate(hw, parent_rate));
	return ret;
}

const struct clk_ops socfpga_mister_cpu_ops = {
	.recalc_rate = mister_cpu_recalc_rate,
	.round_rate = mister_cpu_round_rate,
	.set_rate = mister_cpu_set_rate,
};

bool __init socfpga_mister_cpu_clock(struct device_node *node)
{
	struct device_node *np;
	u32 reg, rate;

	if (!IS_ENABLED(CONFIG_ARM_SOCFPGA_CPUFREQ) ||
	    !of_machine_is_compatible("terasic,de10-nano") ||
	    of_property_read_u32(node, "reg", &reg) || reg != CM_MPU)
		return false;

	np = of_find_node_by_name(NULL, "osc1");
	if (!np)
		return false;
	reg = of_property_read_u32(np, "clock-frequency", &rate);
	of_node_put(np);
	if (reg || rate != 25000000)
		return false;
	mister_osc_rate = rate;
	return true;
}
