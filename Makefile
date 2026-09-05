KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

ifeq ($(O),)
out_dir := .
else
out_dir := $(O)
endif

DRIVER_KOS := BSP/uwe5621_bsp_sdio.ko WIFI/sprdwl_ng.ko

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/BSP CFG_AML_WIFI_DEVICE_UWE5621=y UNISOC_WCN_CHIP_ID=uwe5621 COUSTOM_PLATFORM=aml UNISOC_WCN_HW_TYPE=sdio $(@)
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/WIFI CFG_AML_WIFI_DEVICE_UWE5621=y UNISOC_WCN_CHIP_ID=uwe5621 COUSTOM_PLATFORM=aml UNISOC_WCN_HW_TYPE=sdio $(@)
	rm -f $(out_dir)/$(M)/Module.symvers
	for symvers in $(out_dir)/$(M)/BSP/Module.symvers $(out_dir)/$(M)/WIFI/Module.symvers; do \
		if [ -f $$symvers ]; then \
			cat $$symvers >> $(out_dir)/$(M)/Module.symvers; \
		fi; \
	done
	touch $(out_dir)/$(M)/Module.symvers
	rm -f $(out_dir)/$(M)/modules.order
	for morder in $(out_dir)/$(M)/BSP/modules.order $(out_dir)/$(M)/WIFI/modules.order; do \
		if [ -f $$morder ]; then \
			cat $$morder >> $(out_dir)/$(M)/modules.order; \
		fi; \
	done
	touch $(out_dir)/$(M)/modules.order
	if [ -n "$$INSTALL_MOD_PATH" ]; then \
		for d in $$(find $$INSTALL_MOD_PATH -name "modules.order.*" -exec dirname {} + 2>/dev/null | sort -u); do \
			set -- $$d/modules.order.*; \
			if [ $$# -gt 1 ]; then \
				rm -f "$$d/modules.order.combined"; \
				for ko in $(DRIVER_KOS); do \
					cat "$$@" | grep "$$(basename $$ko .ko)\." >> "$$d/modules.order.combined" || true; \
				done; \
				if [ -s "$$d/modules.order.combined" ]; then \
					rm -f "$$@"; \
					mv "$$d/modules.order.combined" "$$1"; \
				else \
					cat "$$@" > "$$d/modules.order.combined" && rm -f "$$@" && mv "$$d/modules.order.combined" "$$1"; \
				fi; \
			fi; \
		done; \
	fi
	for ko in $(DRIVER_KOS); do \
		if [ -e $(out_dir)/$(M)/$$ko ]; then \
			ln -sf $(out_dir)/$(M)/$$ko $(out_dir)/$(M)/$$(basename $$ko); \
		fi; \
	done

