#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>

static struct clk *mpu_clk;

static struct cpufreq_frequency_table socfpga_freq_table[] = {
	{ .frequency = 400000 },
	{ .frequency = 800000 },
	{ .frequency = 1000000, .flags = CPUFREQ_BOOST_FREQ },
	{ .frequency = 1200000, .flags = CPUFREQ_BOOST_FREQ },
	{ .frequency = CPUFREQ_TABLE_END },
};

static int socfpga_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	unsigned long rate = socfpga_freq_table[index].frequency * 1000UL;
	int ret;

	if (rate > 800000000 &&
	    (!cpufreq_boost_enabled() || !policy->boost_enabled))
		return -EINVAL;

	ret = clk_set_rate(policy->clk, rate);
	if (ret)
		return ret;

	return clk_get_rate(policy->clk) == rate ? 0 : -EIO;
}

static int socfpga_cpu_init(struct cpufreq_policy *policy)
{
	policy->clk = mpu_clk;
	policy->freq_table = socfpga_freq_table;
	policy->cur = clk_get_rate(mpu_clk) / 1000;
	policy->cpuinfo.transition_latency = 1000000;
	cpumask_copy(policy->cpus, cpu_possible_mask);

	return 0;
}

static struct cpufreq_driver socfpga_driver = {
	.name = "socfpga",
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = socfpga_target_index,
	.get = cpufreq_generic_get,
	.init = socfpga_cpu_init,

	.set_boost = cpufreq_boost_set_sw,
	.boost_enabled = false,
};

static int __init socfpga_cpufreq_init(void)
{
	struct device_node *np;
	struct of_phandle_args clkspec = {};
	unsigned long rate;
	int ret;

	if (!of_machine_is_compatible("terasic,de10-nano"))
		return -ENODEV;

	for_each_compatible_node(np, NULL, "altr,socfpga-perip-clk") {
		u32 reg;

		if (of_property_read_u32(np, "reg", &reg) || reg != 0x48)
			continue;
		clkspec.np = np;
		mpu_clk = of_clk_get_from_provider(&clkspec);
		of_node_put(np);
		break;
	}
	if (!mpu_clk)
		return -ENODEV;
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if (clk_round_rate(mpu_clk, 400000000) != 400000000 ||
	    clk_round_rate(mpu_clk, 800000000) != 800000000) {
		pr_err("socfpga-cpufreq: compatible MPU clock provider required\n");
		ret = -ENODEV;
		goto put_clk;
	}

	rate = clk_get_rate(mpu_clk);
	if (rate != 400000000 && rate != 800000000) {
		pr_err("socfpga-cpufreq: unsupported boot rate %lu Hz\n", rate);
		ret = -EINVAL;
		goto put_clk;
	}
	ret = cpufreq_register_driver(&socfpga_driver);
	if (!ret)
		return 0;
put_clk:
	clk_put(mpu_clk);
	return ret;
}

static void __exit socfpga_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&socfpga_driver);
	clk_put(mpu_clk);
}

module_init(socfpga_cpufreq_init);
module_exit(socfpga_cpufreq_exit);
MODULE_AUTHOR("Michael Huang <coolbho3000@gmail.com>");
MODULE_DESCRIPTION("MiSTer Cyclone V CPUFreq with explicit overclock opt-in");
MODULE_LICENSE("GPL");
