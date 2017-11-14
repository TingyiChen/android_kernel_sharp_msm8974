ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= shtps_rmi_spi.o shtps_rmi_devctl.o shtps_rmi_debug.o

