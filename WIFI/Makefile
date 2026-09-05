include $(dir $(lastword $(MAKEFILE_LIST)))Kbuild

ifeq ($(COUSTOM_PLATFORM), unisoc)
KO_MODULE_NAME := sprdwl_ng_ott
else
KO_MODULE_NAME := sprdwl_ng
endif
KO_MODULE_OUT := $(BSP_MODULES_OUT)/$(KO_MODULE_NAME)
KO_MODULE_KBUILD := $(CURDIR)/Kbuild

.PHONY: modules modules_install clean

modules:
	@echo build driver
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	$(CROSS_COMPILE)strip -g $(KO_MODULE_NAME).ko

modules_install:
	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) $@

# Remove the out directory wholly
clean:
	@#$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@
	rm -rf $(KO_MODULE_OUT)
