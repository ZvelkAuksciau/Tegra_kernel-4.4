#
# Memory controller code.
#

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra \
             -I$(srctree)/drivers/platform/tegra/include

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-$(CONFIG_ARCH_TEGRA_210_SOC)        += mcerr-t21.o

# Latency allowance
obj-y                                   += latency_allowance.o
obj-$(CONFIG_ARCH_TEGRA_210_SOC)        += tegra21x_la.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_210_SOC)        += mc-timing-t21x.o
endif
obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr.o pmqos_bwmgr_client.o
ifeq ($(CONFIG_ARCH_TEGRA_210_SOC),y)
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr-t21x.o
endif

ifeq ($(CONFIG_TEGRA_MC),y)

ifneq ($(CONFIG_ARCH_TEGRA_210_SOC),y)
obj-y += tegra_emc.o
obj-y 					+= tegra_emc_timers.o
endif

obj-y                                   += tegra-mc-sid.o

endif
