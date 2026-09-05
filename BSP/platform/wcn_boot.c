// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Communications Inc.
 *
 * Filename : wcn_boot.c
 * Abstract : This file is a implementation for wcn sdio hal function
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include "marlin_platform.h"
#include "wcn_bus.h"

#include "gnss/gnss_common.h"
#include "rf/rf.h"
#include "../sleep/sdio_int.h"
#include "../sleep/slp_mgr.h"
#include "mem_pd_mgr.h"
#include "wcn_op.h"
#include "wcn_parn_parser.h"
#include "pcie_boot.h"
#include "usb_boot.h"
#include "rdc_debug.h"
#include "wcn_boot.h"
#include "wcn_dump.h"
#include "wcn_glb.h"
#include "wcn_log.h"
#include "wcn_misc.h"
#include "wcn_procfs.h"
#include "wcn_gnss.h"
#include "wcn_txrx.h"
#include "mdbg_type.h"
#include "wcn_glb_reg.h"
#include "wcn_ca_trusty.h"

#ifdef CONFIG_AML_BOARD
extern void extern_wifi_set_enable(int is_on);
extern void set_usb_wifi_power(int is_power);
#endif

#ifdef MODULE_PARAM_PREFIX
#undef MODULE_PARAM_PREFIX
#endif
#define MODULE_PARAM_PREFIX	"marlin."

static int clktype = -1;
module_param(clktype, int, 0444);

#ifndef REG_PMU_APB_XTL_WAIT_CNT0
#define REG_PMU_APB_XTL_WAIT_CNT0 0xe42b00ac
#endif
static char BTWF_FIRMWARE_PATH[255];
static char GNSS_FIRMWARE_PATH[255];

#if (defined(CONFIG_THIRD_PARTY_BOARD)) && (!defined(CONFIG_WCN_PARSE_DTS))
/* path of cp2 firmware. */
#ifdef CONFIG_CUSTOMIZE_UNISOC_FW_PATH
#define UNISOC_FW_PATH_DEFAULT CONFIG_CUSTOMIZE_UNISOC_FW_PATH
#elif defined(CONFIG_AML_BOARD)
#define UNISOC_FW_PATH_DEFAULT "/vendor/lib/firmware/"
#else
#define UNISOC_FW_PATH_DEFAULT "/vendor/firmware/"
#endif
#define WCN_FW_MAX_PATH_NUM	4
static char *wcn_fw_path[WCN_FW_MAX_PATH_NUM] = {
	UNISOC_FW_PATH_DEFAULT,		/* most of projects */
	"/vendor/etc/firmware/",	/* allwinner h6/h616... */
	"/lib/firmware/",		/* allwinner r328... */
	"/vendor/firmware/"
};
#if defined(CONFIG_WCN_SDIO)
#define WCN_FW_NAME	"wcnmodem.bin"
#elif defined(CONFIG_WCN_USB)
#define WCN_FW_NAME	"wcnmodem_usb.bin"
#endif

#define GNSS_FW_NAME	"gnssmodem.bin"

#endif // end of (defined(CONFIG_THIRD_PARTY_BOARD)) && (!defined(CONFIG_WCN_PARSE_DTS))

static struct wifi_calibration wifi_data;
struct completion ge2_completion;
static int first_call_flag = 1;
marlin_reset_callback marlin_reset_func;
void *marlin_callback_para;

static struct marlin_device *marlin_dev;
struct sprdwcn_gnss_ops *gnss_ops;
#ifdef CONFIG_WCN_USB
struct completion wcn_usb_rst_fdl_done;
#endif

unsigned char  flag_reset;
char functionmask[8];
static unsigned int reg_val;
static unsigned int clk_wait_val;
static unsigned int cp_clk_wait_val;
static unsigned int marlin2_clk_wait_reg;
#ifdef COMFIG_FSTAB_AB
static char *fstab_ab;
#endif

#ifdef CONFIG_WCN_USB
static struct raw_notifier_head marlin_reset_notifiers[MARLIN_ALL];
#endif
/* temp for rf pwm mode */
/* static struct regmap *pwm_regmap; */

#define IMG_HEAD_MAGIC		"WCNM"
#define IMG_HEAD_MAGIC_COMBINE	"WCNE"
#define IMG_MARLINAA_TAG	"MLAA"
#define IMG_MARLINAB_TAG	"MLAB"
#define IMG_MARLINAC_TAG	"MLAC"

#define MARLIN_MASK		0x27F
#define GNSS_MASK		0x080
#define AUTO_RUN_MASK		0X100

#define AFC_CALI_FLAG		0x54463031	/* cali flag */
#define AFC_CALI_READ_FINISH	0x12121212
#define WCN_AFC_CALI_PATH	"/productinfo/wcn/tsx_bt_data.txt"

extern int init_wcn_sysfs(void);
extern void exit_wcn_sysfs(void);
#ifdef CONFIG_WCN_GNSS
extern int gnss_common_ctl_init(void);
extern void gnss_common_ctl_exit(void);
extern int gnss_pmnotify_ctl_init(void);
extern void gnss_pmnotify_ctl_cleanup(void);
#endif
/* #define E2S(x) { case x: return #x; } */

struct head {
	char magic[4];
	u32 version;
	u32 img_count;
} __packed;

struct imageinfo {
	char tag[4];
	u32 offset;
	u32 size;
} __packed;

unsigned int marlin_get_wcn_chipid(void)
{
	static unsigned long int chip_id;
	int ret;
#ifdef CONFIG_WCN_USB
	return MARLIN3E_AA_CHIPID;
#endif
	if (likely(chip_id))
		return chip_id;

	WCN_DEBUG("%s enter.\n", __func__);

#ifndef CONFIG_CHECK_DRIVER_BY_CHIPID
	ret = sprdwcn_bus_reg_read(CHIPID_REG, &chip_id, 4);
	if (ret < 0) {
		WCN_ERR("marlin read chip ID fail\n");
		return 0;
	}

#else
	/* At first, read Marlin3E chipid register. */
	ret = sprdwcn_bus_reg_read(CHIPID_REG_M3E, &chip_id, 4);
	if (ret < 0) {
		WCN_ERR("read marlin3E chip id fail, ret=%d\n", ret);
		return 0;
	}

	/* If it is not Marlin3E, then read Marlin3 chipid register. */
	if ((chip_id & WCN_CHIPID_MASK) != MARLIN3E_AA_CHIPID) {
		ret = sprdwcn_bus_reg_read(CHIPID_REG_M3_M3L, &chip_id, 4);
		if (ret < 0) {
			WCN_ERR("read marlin3 chip id fail, ret=%d\n", ret);
			return 0;
		}
	}
#endif
	WCN_INFO("%s: chipid: 0x%lx\n", __func__, chip_id);

	return chip_id;
}
EXPORT_SYMBOL_GPL(marlin_get_wcn_chipid);

enum wcn_chip_id_type wcn_get_chip_type(void)
{
	static enum wcn_chip_id_type chip_type;

	if (likely(chip_type))
		return chip_type;

	switch (marlin_get_wcn_chipid()) {
	case MARLIN_AA_CHIPID:
#ifdef CONFIG_UMW2653
	case MARLIN3E_AA_CHIPID:
#endif
		chip_type = WCN_CHIP_ID_AA;
		break;
	case MARLIN_AB_CHIPID:
		chip_type = WCN_CHIP_ID_AB;
		break;
	case MARLIN_AC_CHIPID:
		chip_type = WCN_CHIP_ID_AC;
		break;
	case MARLIN_AD_CHIPID:
		chip_type = WCN_CHIP_ID_AD;
		break;
	default:
		chip_type = WCN_CHIP_ID_INVALID;
		break;
	}

	return chip_type;
}
EXPORT_SYMBOL_GPL(wcn_get_chip_type);

#if defined(CONFIG_SC2355)
#define WCN_CHIP_NAME_PRE "Marlin3_"
#elif defined(CONFIG_UMW2652)
#define WCN_CHIP_NAME_PRE "Marlin3Lite_"
#else
#define WCN_CHIP_NAME_PRE "ERRO_"
#endif

#define _WCN_STR(a) #a
#define WCN_STR(a) _WCN_STR(a)
#define WCN_CON_STR(a, b, c) (a b WCN_STR(c))

static char * const wcn_chip_name[WCN_CHIP_ID_MAX] = {
	"UNKNOWN",
	WCN_CON_STR(WCN_CHIP_NAME_PRE, "AA_", MARLIN_AA_CHIPID),
	WCN_CON_STR(WCN_CHIP_NAME_PRE, "AB_", MARLIN_AB_CHIPID),
	WCN_CON_STR(WCN_CHIP_NAME_PRE, "AC_", MARLIN_AC_CHIPID),
	WCN_CON_STR(WCN_CHIP_NAME_PRE, "AD_", MARLIN_AD_CHIPID),
};

const char *wcn_get_chip_name(void)
{
	enum wcn_chip_id_type chip_type;
	static char *chip_name;

	if (likely(chip_name))
		return chip_name;

	chip_type = wcn_get_chip_type();
	if (chip_type != WCN_CHIP_ID_INVALID)
		chip_name = wcn_chip_name[chip_type];

	return wcn_chip_name[chip_type];
}
EXPORT_SYMBOL_GPL(wcn_get_chip_name);

#ifndef CONFIG_UMW2653
static char *wcn_get_chip_tag(void)
{
	enum wcn_chip_id_type chip_type;
	static char *wcn_chip_tag;
	static char * const magic_tag[] = {
		"NULL",
		IMG_MARLINAA_TAG,
		IMG_MARLINAB_TAG,
		IMG_MARLINAC_TAG,
		IMG_MARLINAC_TAG,
	};

	if (likely(wcn_chip_tag))
		return wcn_chip_tag;

	chip_type = wcn_get_chip_type();
	if (chip_type != WCN_CHIP_ID_INVALID)
		wcn_chip_tag = magic_tag[chip_type];

	return wcn_chip_tag;
}
#endif

/* get the subsys string */
const char *strno(enum wcn_sub_sys subsys)
{
	switch (subsys) {
	case MARLIN_BLUETOOTH:
		return "MARLIN_BLUETOOTH";
	case MARLIN_FM:
		return "MARLIN_FM";
	case MARLIN_WIFI:
		return "MARLIN_WIFI";
	case MARLIN_WIFI_FLUSH:
		return "MARLIN_WIFI_FLUSH";
	case MARLIN_SDIO_TX:
		return "MARLIN_SDIO_TX";
	case MARLIN_SDIO_RX:
		return "MARLIN_SDIO_RX";
	case MARLIN_MDBG:
		return "MARLIN_MDBG";
	case MARLIN_GNSS:
		return "MARLIN_GNSS";
	case WCN_AUTO:
		return "WCN_AUTO";
	case MARLIN_ALL:
		return "MARLIN_ALL";
	default: return "MARLIN_SUBSYS_UNKNOWN";
	}
/* #undef E2S */
}

unsigned long marlin_get_power_state(void)
{
	return marlin_dev->power_state;
}
EXPORT_SYMBOL_GPL(marlin_get_power_state);


unsigned char marlin_get_bt_wl_wake_host_en(void)
{
	return marlin_dev->bt_wl_wake_host_en;
}
EXPORT_SYMBOL_GPL(marlin_get_bt_wl_wake_host_en);

/* tsx/dac init */
int marlin_tsx_cali_data_read(struct tsx_data *p_tsx_data)
{
	u32 size = 0;
	u32 read_len = 0;
	struct file *file;
	loff_t offset = 0;
	char *pdata;

	file = filp_open(WCN_AFC_CALI_PATH, O_RDONLY, 0);
	if (IS_ERR(file)) {
		WCN_ERR("open file error\n");
		return -1;
	}
	WCN_INFO("open image "WCN_AFC_CALI_PATH" successfully\n");

	/* read file to buffer */
	size = sizeof(struct tsx_data);
	pdata = (char *)p_tsx_data;
	do {
#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
		read_len = kernel_read(file, (void *)pdata, size, &offset);
#else
		read_len = kernel_read(file, offset, pdata, size);
#endif

		if (read_len > 0) {
			size -= read_len;
			pdata += read_len;
		}
	} while ((read_len > 0) && (size > 0));
	fput(file);
	WCN_INFO("After read, data =%p dac value %02x\n", pdata,
			 p_tsx_data->dac);

	return 0;
}

static u16 marlin_tsx_cali_data_get(void)
{
	int ret;
	return 0;

	WCN_INFO("tsx cali init flag %d\n", marlin_dev->tsxcali.init_flag);

	if (marlin_dev->tsxcali.init_flag == AFC_CALI_READ_FINISH)
		return marlin_dev->tsxcali.tsxdata.dac;

	ret = marlin_tsx_cali_data_read(&marlin_dev->tsxcali.tsxdata);
	marlin_dev->tsxcali.init_flag = AFC_CALI_READ_FINISH;
	if (ret != 0) {
		marlin_dev->tsxcali.tsxdata.dac = 0xffff;
		WCN_INFO("tsx cali read fail! default 0xffff\n");
		return marlin_dev->tsxcali.tsxdata.dac;
	}

	if (marlin_dev->tsxcali.tsxdata.flag != AFC_CALI_FLAG) {
		marlin_dev->tsxcali.tsxdata.dac = 0xffff;
		WCN_INFO("tsx cali flag fail! default 0xffff\n");
		return marlin_dev->tsxcali.tsxdata.dac;
	}
	WCN_INFO("dac flag %d value:0x%x\n",
			    marlin_dev->tsxcali.tsxdata.flag,
			    marlin_dev->tsxcali.tsxdata.dac);

	return marlin_dev->tsxcali.tsxdata.dac;
}

#ifndef CONFIG_UMW2653
static int marlin_judge_imagepack(char *buffer)
{
	struct head *imghead;

	if (buffer == NULL)
		return -1;

	imghead = (struct head *)buffer;

	return strncmp(IMG_HEAD_MAGIC, imghead->magic, 4);
}


static struct imageinfo *marlin_judge_images(char *buffer)
{

	struct imageinfo *imginfo = NULL;
	unsigned char *magic_str;

	magic_str = wcn_get_chip_tag();
	if (!magic_str) {
		WCN_ERR("%s chip id erro\n", __func__);
		return NULL;
	}

	imginfo = kzalloc(sizeof(*imginfo), GFP_KERNEL);
	if (!imginfo)
		return NULL;

	memcpy(imginfo, (buffer + sizeof(struct head)),
	       sizeof(*imginfo));

	if (!strncmp(magic_str, imginfo->tag, 4)) {
		WCN_INFO("%s: marlin imginfo1 type is %s\n",
			 __func__, magic_str);
		return imginfo;
	}
	memcpy(imginfo, buffer + sizeof(*imginfo) + sizeof(struct head),
	       sizeof(*imginfo));
	if (!strncmp(magic_str, imginfo->tag, 4)) {
		WCN_INFO("%s: marlin imginfo2 type is %s\n",
			 __func__, magic_str);
		return imginfo;
	}

	WCN_ERR("Marlin can't find marlin chip image!!!\n");
	kfree(imginfo);

	return  NULL;
}
#endif

#define WCN_XPE_EFUSE_DDR 0x40859030

/* get IPD Vendor ID
addr 0x40859030 30~29bit 00-T 01-X
Before read, need write 4 reg
addr 0x4083c32c bit22 enable clock
addr 0x4083c024 bit11 enable clock
addr 0x4083c050 bit18 enable clock
addr 0x40858040 bit0  enable clock */
unsigned int marlin_get_wcn_xpe_efuse_data(void)
{
	static unsigned int ipd_vendor_id;
	static unsigned int efuse_data1;
	static unsigned int efuse_data2;
	static unsigned int efuse_data3;
	static unsigned int efuse_data4;
	int ret;

	if (unlikely(ipd_vendor_id != 0))
		return ipd_vendor_id;

	ret = sprdwcn_bus_reg_read(WCN_XPE_EFUSE_DDR, &ipd_vendor_id, 4);

	if (ret < 0) {
		WCN_ERR("marlin read ipd_vendor_id fail\n");
		return 0;
	}
	WCN_INFO("ipd_vendor_id = %x, %s\n", WCN_XPE_EFUSE_DDR, __func__);

	ret = sprdwcn_bus_reg_read(0x4083c32c, &efuse_data1, 4);
	WCN_INFO("ADDR :0X4083c32c = %x, %s\n", efuse_data1, __func__);
	efuse_data1 |= 0x400000;
	ret = sprdwcn_bus_reg_write(0x4083c32c, &efuse_data1, 4);
	ret = sprdwcn_bus_reg_read(0x4083c32c, &efuse_data1, 4);
	WCN_INFO("ADDR :0X4083c32c = %x, %s\n", efuse_data1, __func__);
	ret = sprdwcn_bus_reg_read(WCN_XPE_EFUSE_DDR, &ipd_vendor_id, 4);
	WCN_INFO("block6 ADDR :0X40859030 = %x, %s\n", ipd_vendor_id, __func__);
	ipd_vendor_id = ipd_vendor_id << 1;
	ipd_vendor_id = ipd_vendor_id >> 30;
	WCN_INFO("ipd_vendor_id = %x, %s\n", ipd_vendor_id, __func__);

	ret = sprdwcn_bus_reg_read(0x4083c024, &efuse_data2, 4);
	WCN_INFO("ADDR :0X4083c024 = %x, %s\n", efuse_data2, __func__);
	efuse_data2 |= 0x8000;
	ret = sprdwcn_bus_reg_write(0x4083c024, &efuse_data2, 4);
	ret = sprdwcn_bus_reg_read(0x4083c024, &efuse_data2, 4);
	WCN_INFO("ADDR :0X4083c024 = %x, %s\n", efuse_data2, __func__);
	ret = sprdwcn_bus_reg_read(WCN_XPE_EFUSE_DDR, &ipd_vendor_id, 4);
	WCN_INFO("block6 ADDR :0X40859030 = %x, %s\n", ipd_vendor_id, __func__);
	ipd_vendor_id = ipd_vendor_id << 1;
	ipd_vendor_id = ipd_vendor_id >> 30;
	WCN_INFO("ipd_vendor_id = %x, %s\n", ipd_vendor_id, __func__);

	ret = sprdwcn_bus_reg_read(0x4083c050, &efuse_data3, 4);
	WCN_INFO("ADDR :0X4083c050 = %x, %s\n", efuse_data3, __func__);
	efuse_data3 |= 0x40000;
	ret = sprdwcn_bus_reg_write(0x4083c050, &efuse_data3, 4);
	ret = sprdwcn_bus_reg_read(0x4083c050, &efuse_data3, 4);
	WCN_INFO("ADDR :0X4083c050 = %x, %s\n", efuse_data3, __func__);
	ret = sprdwcn_bus_reg_read(WCN_XPE_EFUSE_DDR, &ipd_vendor_id, 4);
	WCN_INFO("block6 ADDR :0X40859030 = %x, %s\n", ipd_vendor_id, __func__);
	ipd_vendor_id = ipd_vendor_id << 1;
	ipd_vendor_id = ipd_vendor_id >> 30;
	WCN_INFO("ipd_vendor_id = %x, %s\n", ipd_vendor_id, __func__);

	ret = sprdwcn_bus_reg_read(0x40858040, &efuse_data4, 4);
	WCN_INFO("ADDR :0X40858040 = %x, %s\n", efuse_data4, __func__);
	efuse_data4 |= 0x1;
	ret = sprdwcn_bus_reg_write(0x40858040, &efuse_data4, 4);
	ret = sprdwcn_bus_reg_read(0x40858040, &efuse_data4, 4);
	WCN_INFO("ADDR :0X40858040 = %x, %s\n", efuse_data4, __func__);
	ret = sprdwcn_bus_reg_read(WCN_XPE_EFUSE_DDR, &ipd_vendor_id, 4);
	WCN_INFO("block6 ADDR :0X40859030 = %x, %s\n", ipd_vendor_id, __func__);
	ipd_vendor_id = ipd_vendor_id << 1;
	ipd_vendor_id = ipd_vendor_id >> 30;
	WCN_INFO("ipd_vendor_id = %x, %s\n", ipd_vendor_id, __func__);

        return ipd_vendor_id;
}
EXPORT_SYMBOL_GPL(marlin_get_wcn_xpe_efuse_data);

#define WCN_WFBT_LOAD_FIRMWARE_OFFSET 0x180000
static char *btwf_load_firmware_data(loff_t off, unsigned long int imag_size)
{
	int read_len, size, i, opn_num_max = 15;
	char *buffer = NULL;
	char *data = NULL;
	struct file *file;
	loff_t offset = 0, pos = 0;
	WCN_DEBUG("%s entry ,BTWF_FIRMWARE_PATH = %s \n", __func__ , BTWF_FIRMWARE_PATH );

	file = filp_open(BTWF_FIRMWARE_PATH, O_RDONLY, 0);
	for (i = 1; i <= opn_num_max; i++) {
		if (IS_ERR(file)) {
			WCN_INFO("try open file %s,count_num:%d,%s\n",
				BTWF_FIRMWARE_PATH, i, __func__);
			ssleep(1);
			file = filp_open(BTWF_FIRMWARE_PATH, O_RDONLY, 0);
		} else {
			break;
		}
	}
	if (IS_ERR(file)) {
		WCN_ERR("%s open file %s error\n",
			BTWF_FIRMWARE_PATH, __func__);
		return NULL;
	}
	WCN_DEBUG("marlin %s open image file  successfully\n",
		__func__);
	size = imag_size;
	buffer = vmalloc(size);
	if (!buffer) {
		fput(file);
		WCN_ERR("no memory for image\n");
		return NULL;
	}

#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
	read_len = kernel_read(file, (void *)functionmask, 8, &pos);
#else
	read_len = kernel_read(file, pos, functionmask, 8);
#endif

	if ((functionmask[0] == 0x00) && (functionmask[1] == 0x00))
		offset = offset + 8;
	else
		functionmask[7] = 0;

	data = buffer;
	if ((marlin_get_wcn_xpe_efuse_data() == WCN_XPE_EFUSE_DATA)) {
			off = WCN_WFBT_LOAD_FIRMWARE_OFFSET;
			WCN_INFO("btwf bin --------\r\n");
	}
	offset += off;
	WCN_INFO("WCN_XPE_EFUSE_DATA = %x, marlin_get_wcn_xpe_efuse_data = %x \n",
		 WCN_XPE_EFUSE_DATA, marlin_get_wcn_xpe_efuse_data());
	WCN_INFO("btwf bin off =%lld, offset =%lld \n", off, offset);
	do {
#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
		read_len = kernel_read(file, (void *)buffer, size, &offset);
#else
		read_len = kernel_read(file, offset, buffer, size);
#endif
		if (read_len > 0) {
			size -= read_len;
			buffer += read_len;
		}
	} while ((read_len > 0) && (size > 0));
	fput(file);
	WCN_INFO("%s finish read_Len:%d\n", __func__, read_len);

#if KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE
	/* kernel_read return value changed. */
	if (read_len <= 0) {
		vfree(buffer);
		return NULL;
	}
#endif

	return data;
}

#define WCN_VMAP_RETRY_CNT (20)
static void *wcn_mem_ram_vmap(phys_addr_t start, size_t size,
			      int noncached, unsigned int *count)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;
	phys_addr_t addr;
	int retry = 0;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	*count = page_count;
	if (noncached)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = PAGE_KERNEL;

retry1:
	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		if (retry++ < WCN_VMAP_RETRY_CNT) {
			usleep_range(8000, 10000);
			goto retry1;
		} else {
			WCN_ERR("malloc err\n");
			return NULL;
		}
	}

	for (i = 0; i < page_count; i++) {
		addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
retry2:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	vaddr = vm_map_ram(pages, page_count, -1);
#else
	vaddr = vm_map_ram(pages, page_count, -1, prot);
#endif
	if (!vaddr) {
		if (retry++ < WCN_VMAP_RETRY_CNT) {
			usleep_range(8000, 10000);
			goto retry2;
		} else {
			WCN_ERR("vmap err\n");
			goto out;
		}
	} else {
		vaddr += offset_in_page(start);
	}
out:
	kfree(pages);

	return vaddr;
}

void wcn_mem_ram_unmap(const void *mem, unsigned int count)
{
	vm_unmap_ram(mem - offset_in_page(mem), count);
}

void *wcn_mem_ram_vmap_nocache(phys_addr_t start, size_t size,
			       unsigned int *count)
{
	return wcn_mem_ram_vmap(start, size, 1, count);
}

#ifdef CONFIG_ARM64
static inline void wcn_unalign_memcpy(void *to, const void *from, u32 len)
{
	if (((unsigned long)to & 7) == ((unsigned long)from & 7)) {
		while (((unsigned long)from & 7) && len) {
			*(char *)(to++) = *(char *)(from++);
			len--;
		}
		memcpy(to, from, len);
	} else if (((unsigned long)to & 3) == ((unsigned long)from & 3)) {
		while (((unsigned long)from & 3) && len) {
			*(char *)(to++) = *(char *)(from++);
			len--;
		}
		while (len >= 4) {
			*(u32 *)(to) = *(u32 *)(from);
			to += 4;
			from += 4;
			len -= 4;
		}
		while (len) {
			*(char *)(to++) = *(char *)(from++);
			len--;
		}
	} else {
		while (len) {
			*(char *)(to++) = *(char *)(from++);
			len--;
		}
	}
}
#else
static inline void wcn_unalign_memcpy(void *to, const void *from, u32 len)
{
	memcpy(to, from, len);
}
#endif

int wcn_write_data_to_phy_addr(phys_addr_t phy_addr,
			       void *src_data, u32 size)
{
	char *virt_addr, *src;
	unsigned int cnt;

	src = (char *)src_data;
	virt_addr = (char *)wcn_mem_ram_vmap_nocache(phy_addr, size, &cnt);
	if (virt_addr) {
		wcn_unalign_memcpy((void *)virt_addr, (void *)src, size);
		wcn_mem_ram_unmap(virt_addr, cnt);
		return 0;
	}

	WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
	return -1;
}

int wcn_read_data_from_phy_addr(phys_addr_t phy_addr,
				void *tar_data, u32 size)
{
	char *virt_addr, *tar;
	unsigned int cnt;

	tar = (char *)tar_data;
	virt_addr = wcn_mem_ram_vmap_nocache(phy_addr, size, &cnt);
	if (virt_addr) {
		wcn_unalign_memcpy((void *)tar, (void *)virt_addr, size);
		wcn_mem_ram_unmap(virt_addr, cnt);
		return 0;
	}

	WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
	return -1;
}

#ifndef CONFIG_UMW2653
static int marlin_download_from_partition(void)
{
	int err, len, trans_size, ret;
	unsigned long int img_size;
	char *buffer = NULL;
	char *temp = NULL;
	struct imageinfo *imginfo = NULL;
	uint32_t sec_img_magic;
	struct sys_img_header *imgHeader = NULL;

	if (marlin_dev->maxsz_btwf > 0)
		img_size = marlin_dev->maxsz_btwf;
	else
		img_size = FIRMWARE_MAX_SIZE;

	WCN_INFO("%s entry\n", __func__);
	buffer = btwf_load_firmware_data(0, img_size);
	if (!buffer) {
		WCN_INFO("%s buff is NULL\n", __func__);
		return -1;
	}
	temp = buffer;

	imgHeader = (struct sys_img_header *) buffer;
	sec_img_magic = imgHeader->mMagicNum;
	if (sec_img_magic != SEC_IMAGE_MAGIC) {
		WCN_INFO("%s image magic 0x%x.\n",
			__func__, sec_img_magic);
	} else if (marlin_dev->maxsz_btwf > 0) {
		wcn_write_data_to_phy_addr(marlin_dev->base_addr_btwf, buffer, marlin_dev->maxsz_btwf);
#ifdef CONFIG_WCN_FIRMWARE_VERIFY
		if (wcn_firmware_sec_verify(1, marlin_dev->base_addr_btwf, marlin_dev->maxsz_btwf) < 0) {
			vfree(temp);
			WCN_ERR("%s sec verify fail.\n", __func__);
			return -1;
		} else 
#endif
		{
			img_size = FIRMWARE_MAX_SIZE;
			wcn_read_data_from_phy_addr(marlin_dev->base_addr_btwf + SEC_IMAGE_HDR_SIZE,
				buffer, img_size);
		}
	}

	ret = marlin_judge_imagepack(buffer);
	if (!ret) {
		WCN_INFO("marlin %s imagepack is WCNM type,need parse it\n",
			__func__);
		marlin_get_wcn_chipid();

		imginfo = marlin_judge_images(buffer);
		vfree(temp);
		if (!imginfo) {
			WCN_ERR("marlin:%s imginfo is NULL\n", __func__);
			return -1;
		}
		img_size = imginfo->size;
		if (img_size > FIRMWARE_MAX_SIZE)
			WCN_INFO("%s real size %ld is large than the max:%d\n",
				 __func__, img_size, FIRMWARE_MAX_SIZE);
		buffer = btwf_load_firmware_data(imginfo->offset, img_size);
		if (!buffer) {
			WCN_ERR("marlin:%s buffer is NULL\n", __func__);
			kfree(imginfo);
			return -1;
		}
		temp = buffer;
		kfree(imginfo);
	}

	len = 0;
	while (len < img_size) {
		trans_size = (img_size - len) > PACKET_SIZE ?
				PACKET_SIZE : (img_size - len);
		memcpy(marlin_dev->write_buffer, buffer + len, trans_size);
		err = sprdwcn_bus_direct_write(CP_START_ADDR + len,
			marlin_dev->write_buffer, trans_size);
		if (err < 0) {
			WCN_ERR(" %s: dt write SDIO error:%d\n", __func__, err);
			vfree(temp);
			return -1;
		}
		len += PACKET_SIZE;
	}
	vfree(temp);
	WCN_INFO("%s finish and successful\n", __func__);

	return 0;
}
#endif

int wcn_gnss_ops_register(struct sprdwcn_gnss_ops *ops)
{
	if (gnss_ops) {
		WARN_ON(1);
		return -EBUSY;
	}

	gnss_ops = ops;

	return 0;
}

void wcn_gnss_ops_unregister(void)
{
	gnss_ops = NULL;
}

static char *gnss_load_firmware_data(unsigned long int imag_size)
{
	int read_len, size, i, opn_num_max = 15;
	char *buffer = NULL;
	char *data = NULL;
	struct file *file;
	loff_t pos = 0;

	WCN_DEBUG("%s entry\n", __func__);
	if (gnss_ops && (gnss_ops->set_file_path))
		gnss_ops->set_file_path(&GNSS_FIRMWARE_PATH[0]);
	else
		WCN_ERR("%s gnss_ops set_file_path error\n", __func__);
	file = filp_open(GNSS_FIRMWARE_PATH, O_RDONLY, 0);
	for (i = 1; i <= opn_num_max; i++) {
		if (IS_ERR(file)) {
			WCN_INFO("try open file %s,count_num:%d,errno=%ld,%s\n",
				 GNSS_FIRMWARE_PATH, i,
				 PTR_ERR(file), __func__);
			if (PTR_ERR(file) == -ENOENT)
				WCN_ERR("No such file or directory\n");
			if (PTR_ERR(file) == -EACCES)
				WCN_ERR("Permission denied\n");
			ssleep(1);
			file = filp_open(GNSS_FIRMWARE_PATH, O_RDONLY, 0);
		} else {
			break;
		}
	}

	if (IS_ERR(file)) {
		WCN_ERR("%s marlin3 gnss open file %s error\n",
			GNSS_FIRMWARE_PATH, __func__);
		return NULL;
	}
	WCN_DEBUG("%s open image file  successfully\n", __func__);
	size = imag_size;
	buffer = vmalloc(size);
	if (!buffer) {
		fput(file);
		WCN_ERR("no memory for gnss img\n");
		return NULL;
	}

	data = buffer;
	do {
#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
		read_len = kernel_read(file, (void *)buffer, size, &pos);
#else
		read_len = kernel_read(file, pos, buffer, size);
#endif
		if (read_len > 0) {
			size -= read_len;
			buffer += read_len;
		}
	} while ((read_len > 0) && (size > 0));
	fput(file);
	WCN_INFO("%s finish read_Len:%d\n", __func__, read_len);
	if (read_len <= 0)
		return NULL;

	return data;
}

static int gnss_download_from_partition(void)
{
	int err, len, trans_size;
	unsigned long int imgpack_size, img_size;
	char *buffer = NULL;
	char *temp = NULL;
	uint32_t sec_img_magic;
	struct sys_img_header *imgHeader = NULL;

	img_size = imgpack_size =  GNSS_FIRMWARE_MAX_SIZE;

	if (marlin_dev->maxsz_gnss > 0)
		imgpack_size = marlin_dev->maxsz_gnss;

	WCN_INFO("GNSS %s entry\n", __func__);
	temp = buffer = gnss_load_firmware_data(imgpack_size);
	if (!buffer) {
		WCN_INFO("%s gnss buff is NULL\n", __func__);
		return -1;
	}

	imgHeader = (struct sys_img_header *) buffer;
	sec_img_magic = imgHeader->mMagicNum;
	if (sec_img_magic != SEC_IMAGE_MAGIC) {
		WCN_INFO("%s image magic 0x%x.\n",
			__func__, sec_img_magic);
	} else if (marlin_dev->maxsz_gnss > 0) {
		wcn_write_data_to_phy_addr(marlin_dev->base_addr_gnss, buffer, imgpack_size);
#ifdef CONFIG_WCN_FIRMWARE_VERIFY
		if (wcn_firmware_sec_verify(2, marlin_dev->base_addr_gnss, imgpack_size) < 0) {
			vfree(temp);
			WCN_ERR("%s sec verify fail.\n", __func__);
			return -1;
		} else 
#endif
		{
			wcn_read_data_from_phy_addr(marlin_dev->base_addr_gnss + SEC_IMAGE_HDR_SIZE,
				buffer, img_size);
		}
	}

	len = 0;
	while (len < img_size) {
		trans_size = (img_size - len) > PACKET_SIZE ?
				PACKET_SIZE : (img_size - len);
		memcpy(marlin_dev->write_buffer, buffer + len, trans_size);
		err = sprdwcn_bus_direct_write(GNSS_CP_START_ADDR + len,
			marlin_dev->write_buffer, trans_size);
		if (err < 0) {
			WCN_ERR("gnss dt write %s error:%d\n", __func__, err);
			vfree(temp);
			return -1;
		}
		len += PACKET_SIZE;
	}
	vfree(temp);
	WCN_INFO("%s gnss download firmware finish\n", __func__);

	return 0;
}

static int gnss_download_firmware(void)
{
	const struct firmware *firmware;
	char *buf;
	int err;
	int i, len, count, trans_size;

	if (marlin_dev->is_gnss_in_sysfs) {
		err = gnss_download_from_partition();
		return err;
	}

	WCN_INFO("%s start from /system/etc/firmware/\n", __func__);
	buf = marlin_dev->write_buffer;
	err = request_firmware_direct(&firmware, "gnssmodem.bin", NULL);
	if (err < 0) {
		WCN_ERR("%s no find gnssmodem.bin err:%d(ignore)\n",
			__func__, err);
		marlin_dev->is_gnss_in_sysfs = true;
		err = gnss_download_from_partition();

		return err;
	}
	count = (firmware->size + PACKET_SIZE - 1) / PACKET_SIZE;
	len = 0;
	for (i = 0; i < count; i++) {
		trans_size = (firmware->size - len) > PACKET_SIZE ?
				PACKET_SIZE : (firmware->size - len);
		memcpy(buf, firmware->data + len, trans_size);
		err = sprdwcn_bus_direct_write(GNSS_CP_START_ADDR + len, buf,
				trans_size);
		if (err < 0) {
			WCN_ERR("gnss dt write %s error:%d\n", __func__, err);
			release_firmware(firmware);

			return err;
		}
		len += trans_size;
	}
	release_firmware(firmware);
	WCN_INFO("%s successfully through request_firmware!\n", __func__);

	return 0;
}

#ifdef CONFIG_UMW2653

struct combin_img_info {
	unsigned int addr;			/* image target address */
	unsigned int offset;			/* image combin img offset */
	unsigned int size;			/* image size */
};

struct marlin_firmware {
	const u8 *data;
	size_t size;
	bool is_from_fs;
	const void *priv;
	bool is_from_hex;
};

#define marlin_firmware_get_combin_info(buffer) \
		(struct combin_img_info *)(buffer + sizeof(struct head))

#define bin_magic_is(data, magic_tag) \
	!strncmp(((struct head *)data)->magic, magic_tag, strlen(magic_tag))

#define marlin_fw_get_img_count(img) (((struct head *)img)->img_count)

static const struct imageinfo *marlin_imageinfo_get_from_data(const char *tag,
		const void *data)
{
	const struct imageinfo *imageinfo;
	int imageinfo_count;
	int i;

	imageinfo = (struct imageinfo *)(data + sizeof(struct head));
	imageinfo_count = marlin_fw_get_img_count(data);

	for (i = 0; i < imageinfo_count; i++)
		if (!strncmp(imageinfo[i].tag, tag, 4))
			return &(imageinfo[i]);
	return NULL;
}

static const struct imageinfo *marlin_judge_images(const unsigned char *buffer)
{
	unsigned long chip_id;

	chip_id = marlin_get_wcn_chipid();
	if (buffer == NULL)
		return NULL;

	switch (chip_id) {
/* FIXME AD AC usb same TAG in marlin3, BUT it maybe different in other */
	case MARLIN_AD_CHIPID:
	case MARLIN_AC_CHIPID:
		return marlin_imageinfo_get_from_data(IMG_MARLINAC_TAG, buffer);

	case MARLIN_AB_CHIPID:
		return marlin_imageinfo_get_from_data(IMG_MARLINAB_TAG, buffer);
	case MARLIN_AA_CHIPID:
#ifdef CONFIG_UMW2653
	case MARLIN3E_AA_CHIPID:
#endif
		return marlin_imageinfo_get_from_data(IMG_MARLINAA_TAG, buffer);
	default:
		WCN_ERR("%s can't find img's tag!(chip_id%lu)\n", __func__,
				chip_id);
	}
	return NULL;
}

static int marlin_request_firmware(struct marlin_firmware **mfirmware_p)
{
	struct marlin_firmware *mfirmware;
	const void *buffer;

	*mfirmware_p = NULL;
	mfirmware = kmalloc(sizeof(struct marlin_firmware), GFP_KERNEL);
	if (!mfirmware)
		return -ENOMEM;

	WCN_INFO("%s entry\n", __func__);
	buffer = btwf_load_firmware_data(0, FIRMWARE_MAX_SIZE);
	if (!buffer) {
		WCN_INFO("%s buff is NULL\n", __func__);
		kfree(mfirmware);
		return -1;
	}

	mfirmware->data = buffer;
	mfirmware->size = FIRMWARE_MAX_SIZE;
	mfirmware->is_from_fs = 0;
	mfirmware->priv = buffer;

	*mfirmware_p = mfirmware;

	return 0;
}

static int marlin_firmware_parse_image(struct marlin_firmware *mfirmware)
{
	int offset = 0;
	int size = 0;
	int old_mfirmware_size = mfirmware->size;

	if (bin_magic_is(mfirmware->data, IMG_HEAD_MAGIC)) {
		const struct imageinfo *info;

		WCN_INFO("marlin %s imagepack is WCNM type,need parse it\n",
			__func__);
		info = marlin_judge_images(mfirmware->data);
		if (!info) {
			WCN_ERR("marlin:%s imginfo is NULL\n", __func__);
			return -1;
		}

		mfirmware->size = info->size;
		mfirmware->data += info->offset;
		offset = info->offset;
		size = info->size;
	} else if (bin_magic_is(mfirmware->data, IMG_HEAD_MAGIC_COMBINE)) {
		/* cal the combin size */
		int img_count;
		const struct combin_img_info *img_info;
		int img_real_size = 0;

		img_count = marlin_fw_get_img_count(mfirmware->data);
		img_info = marlin_firmware_get_combin_info(mfirmware->data);

		img_real_size =
		img_info[img_count - 1].size + img_info[img_count - 1].offset;

		mfirmware->size = img_real_size;
		size = img_real_size;
	}

	if (!mfirmware->is_from_fs && (offset + size) > old_mfirmware_size) {
		const void *buffer;

		/* NOTE! We canot guarantee the img is complete when we read it
		 * first! The macro FIRMWARE_MAX_SIZE only guarantee AA(first in
		 * partition) img is complete. So we need read this img two
		 * times (in this)
		 */
		buffer = btwf_load_firmware_data(offset, size);
		if (!buffer) {
			WCN_ERR("marlin:%s buffer is NULL\n", __func__);
			return -1;
		}
		/* struct data "info" is a part of mfirmware->priv,
		 * if we free mfirmware->priv, "info" will be free too!
		 * so we need free priv at here after use "info"
		 */
		vfree(mfirmware->priv);
		mfirmware->data = buffer;
		mfirmware->priv = buffer;
	}
	return 0;
}
#ifndef CONFIG_WCN_USB
static int sprdwcn_bus_direct_write_dispack(unsigned int addr, const void *buf,
		size_t buf_size, size_t packet_max_size)
{
	int ret = 0;
	size_t offset = 0;
	void *kbuf = marlin_dev->write_buffer;

	while (offset < buf_size) {
		size_t temp_size = min(packet_max_size, buf_size - offset);

		memcpy(kbuf, buf + offset, temp_size);
		ret = sprdwcn_bus_direct_write(addr + offset, kbuf, temp_size);
		if (ret < 0)
			goto OUT;

		offset += temp_size;
	}
OUT:
	if (ret < 0)
		WCN_ERR(" %s: dt write SDIO error:%d\n", __func__, ret);
	return ret;
}
#endif

#ifdef CONFIG_WCN_USB
extern void wcn_usb_lock(void);
extern void wcn_usb_unlock(void);
int marlin_firmware_write(struct marlin_firmware *fw)
{
	int i;
	int img_count;
	const struct combin_img_info *img_info;
	int ret;

	if (!bin_magic_is(fw->data, IMG_HEAD_MAGIC_COMBINE)) {
		WCN_ERR("Marlin3 USB image must have maigc WCNE\n");
		ret = marlin_firmware_download_usb(0x40500000, fw->data,
				0x56F00, PACKET_SIZE);
		if (ret) {
			WCN_ERR("%s usb download error\n", __func__);
			return -1;
		}
		wcn_usb_lock();
		return marlin_firmware_download_exec_usb(WCN_USB_FW_ADDR);
	}

	img_count = marlin_fw_get_img_count(fw->data);
	img_info = marlin_firmware_get_combin_info(fw->data);
	/* for every img_info */
	for (i = 0; i < img_count; i++) {
		if (img_info[i].size + img_info[i].offset > fw->size) {
			WCN_ERR("%s memory crossover\n", __func__);
			return -1;
		}
		ret = marlin_firmware_download_usb(img_info[i].addr,
				fw->data + img_info[i].offset,
				img_info[i].size, PACKET_SIZE);
		if (ret) {
			WCN_ERR("%s usb download error\n", __func__);
			return -1;
		}
	}

	wcn_usb_lock();
	marlin_firmware_download_exec_usb(WCN_USB_FW_ADDR);
	return 0;
}
#else
static int marlin_firmware_write(struct marlin_firmware *mfirmware)
{
	int i = 0;
	int combin_img_count;
	const struct combin_img_info *imginfoE;
	int err;

	if (bin_magic_is(mfirmware->data, IMG_HEAD_MAGIC_COMBINE)) {
		WCN_INFO("marlin %s imagepack is WCNE type,need parse it\n",
			__func__);

		combin_img_count = marlin_fw_get_img_count(mfirmware->data);
		imginfoE = marlin_firmware_get_combin_info(mfirmware->data);
		if (!imginfoE) {
			WCN_ERR("marlin:%s imginfo is NULL\n", __func__);
			return -1;
		}

		for (i = 0; i < combin_img_count; i++) {
			if (imginfoE[i].size + imginfoE[i].offset >
					mfirmware->size) {
				WCN_ERR("%s memory crossover\n", __func__);
				return -1;
			}
			err = sprdwcn_bus_direct_write_dispack(imginfoE[i].addr,
					mfirmware->data + imginfoE[i].offset,
					imginfoE[i].size, PACKET_SIZE);
			if (err) {
				WCN_ERR("%s download error\n", __func__);
				return -1;
			}
		}
	} else {
		err = sprdwcn_bus_direct_write_dispack(CP_START_ADDR,
				mfirmware->data, mfirmware->size, PACKET_SIZE);
		if (err) {
			WCN_ERR("%s download error\n", __func__);
			return -1;
		}
	}
	WCN_INFO("combin_img %d %s finish and successful\n", i, __func__);

	return 0;
}

#endif
#endif

#if defined(CONFIG_WCN_USB)
static void marlin_release_firmware(struct marlin_firmware *mfirmware)
{
	if (mfirmware) {
		if (mfirmware->is_from_fs)
			release_firmware(mfirmware->priv);
		else {
			if(mfirmware->is_from_hex == 0) {
				if(mfirmware->data)
					vfree(mfirmware->data);
			}
		}
		kfree(mfirmware);
	}
}
#endif
/* BT WIFI FM download */
static int btwifi_download_firmware(void)
{
	const struct firmware *firmware;
	char *buf;
	int err;
	int i, len, count, trans_size;

#ifdef CONFIG_UMW2653
	struct marlin_firmware *mfirmware;
#else
	if (marlin_dev->is_btwf_in_sysfs) {
		err = marlin_download_from_partition();
		return err;
	}
	WCN_INFO("marlin %s  start!\n", __func__);
	buf = marlin_dev->write_buffer;
	err = request_firmware_direct(&firmware, "wcnmodem.bin", NULL);

	if (err < 0) {
		WCN_ERR("no find wcnmodem.bin errno:(%d)(ignore!!)\n", err);
		marlin_dev->is_btwf_in_sysfs = true;
		err = marlin_download_from_partition();
		return err;
	}
#endif
#ifdef CONFIG_UMW2653
	WCN_INFO("uwe2653 %s from start!\n", __func__);
	marlin_dev->is_btwf_in_sysfs = true;

	err = marlin_request_firmware(&mfirmware);
	if (err) {
		WCN_ERR("%s request firmware error\n", __func__);
		goto OUT;
	}

	err = marlin_firmware_parse_image(mfirmware);
	if (err) {
		WCN_ERR("%s firmware parse AA\\AB error\n", __func__);
		goto OUT;
	}
	err = marlin_firmware_write(mfirmware);
	if (err) {
		WCN_ERR("%s firmware write error\n", __func__);
		goto OUT;
	}
OUT:
	if (mfirmware) {
		if (mfirmware->priv)
			vfree(mfirmware->priv);
		kfree(mfirmware);
	}
	
	return err;
#endif

	count = (firmware->size + PACKET_SIZE - 1) / PACKET_SIZE;
	len = 0;

	for (i = 0; i < count; i++) {
		trans_size = (firmware->size - len) > PACKET_SIZE ?
				PACKET_SIZE : (firmware->size - len);
		memcpy(buf, firmware->data + len, trans_size);
		WCN_INFO("download count=%d,len =%d,trans_size=%d\n", count,
			 len, trans_size);
		err = sprdwcn_bus_direct_write(CP_START_ADDR + len,
					       buf, trans_size);
		if (err < 0) {
			WCN_ERR("marlin dt write %s error:%d\n", __func__, err);
			release_firmware(firmware);
			return err;
		}
		len += trans_size;
	}

	release_firmware(firmware);
	WCN_INFO("marlin %s successfully!\n", __func__);

	return 0;
}

#ifndef CONFIG_THIRD_PARTY_BOARD
#ifndef CONFIG_UMW2653
static int wcn_get_syscon_regmap(void)
{
	struct device_node *regmap_np;
	struct platform_device *regmap_pdev;

	regmap_np = of_find_compatible_node(NULL, NULL, "sprd,sc27xx-syscon");
	if (!regmap_np) {
		WCN_ERR("unable to get syscon node\n");
		return -ENODEV;
	}

	regmap_pdev = of_find_device_by_node(regmap_np);
	if (!regmap_pdev) {
		of_node_put(regmap_np);
		WCN_ERR("unable to get syscon platform device\n");
		return -ENODEV;
	}

	marlin_dev->syscon_pmic = dev_get_regmap(regmap_pdev->dev.parent, NULL);
	if (!marlin_dev->syscon_pmic)
		WCN_ERR("unable to get pmic regmap device\n");

	of_node_put(regmap_np);

	return 0;
}

static void wcn_get_pmic_config(struct device_node *np)
{
	int ret;
	struct wcn_pmic_config *pmic;

	if (wcn_get_syscon_regmap())
		return;

	pmic = &marlin_dev->avdd12_parent_bound_chip;
	strcpy(pmic->name, "avdd12-parent-bound-chip");
	ret = of_property_read_u32_array(np, pmic->name,
					 (u32 *)pmic->config,
					 WCN_BOUND_CONFIG_NUM);
	pmic->enable = !ret;
	WCN_INFO("vddgen1-bound-chip config enable:%d\n", pmic->enable);

	pmic = &marlin_dev->avdd12_bound_wbreq;
	strcpy(pmic->name, "avdd12-bound-wbreq");
	ret = of_property_read_u32_array(np, pmic->name,
					 (u32 *)pmic->config,
					 WCN_BOUND_CONFIG_NUM);
	pmic->enable = !ret;
	WCN_INFO("avdd12-bound-wbreq config status:%d\n", pmic->enable);

	pmic = &marlin_dev->avdd33_bound_wbreq;
	strcpy(pmic->name, "avdd33-bound-wbreq");
	ret = of_property_read_u32_array(np, pmic->name,
					 (u32 *)pmic->config,
					 WCN_BOUND_CONFIG_NUM);
	pmic->enable = !ret;
	WCN_INFO("avdd33-bound-wbreq config status:%d\n", pmic->enable);
}
#endif
#endif

static int wcn_pmic_do_bound(struct wcn_pmic_config *pmic, bool bound)
{
	int ret;
	u32 *chip;

	if (!marlin_dev->syscon_pmic || !pmic->enable)
		return -1;

	chip = pmic->config;

	if (bound) {
		WCN_INFO("%s bound\n", pmic->name);
		ret = regmap_update_bits(marlin_dev->syscon_pmic,
					 chip[0], chip[1], chip[3]);
		if (ret)
			WCN_ERR("%s bound:%d\n", pmic->name, ret);
	} else {
		WCN_INFO("%s unbound\n", pmic->name);
		ret = regmap_update_bits(marlin_dev->syscon_pmic,
					 chip[0], chip[1], chip[2]);
		if (ret)
			WCN_ERR("%s unbound:%d\n", pmic->name, ret);
	}
	usleep_range(1000, 2000);

	return 0;
}

static inline int wcn_avdd12_parent_bound_chip(bool enable)
{
	return wcn_pmic_do_bound(&marlin_dev->avdd12_parent_bound_chip, enable);
}

static inline int wcn_avdd12_bound_xtl(bool enable)
{
	return wcn_pmic_do_bound(&marlin_dev->avdd12_bound_wbreq, enable);
}

/* wifipa bound XTLEN3, gnss not need wifipa bound */
static inline int wcn_wifipa_bound_xtl(bool enable)
{
	return wcn_pmic_do_bound(&marlin_dev->avdd33_bound_wbreq, enable);
}

#ifndef CONFIG_WCN_PARSE_DTS
static int marlin_config_wcn_resource(void)
{
	WCN_INFO("%s ,not config dts!!\n", __func__);
	/**
	 * if customer not use dts to config wcn gpio,
	 * then should config marlin_dev resource in here,
	 * except gpio,other resource default to be null
	 */
	if (!marlin_dev) {
		WCN_ERR("marlin_dev is NULL!!!\n");
		return -1;
	}
	marlin_dev->wakeup_ap = -1;
	WCN_DEBUG("%s wakeup_ap is %d\n", __func__, marlin_dev->wakeup_ap);
	marlin_dev->reset = -1;
	WCN_DEBUG("%s reset is %d\n", __func__, marlin_dev->reset);
	marlin_dev->chip_en = -1;
	WCN_DEBUG("%s chip_en is %d\n", __func__, marlin_dev->chip_en);
	marlin_dev->int_ap = -1;
	WCN_DEBUG("%s int_ap is %d\n", __func__, marlin_dev->int_ap);
#ifdef CONFIG_WCN_BT_HOST_WAKE
	marlin_dev->bt_host_wake = -1;
	WCN_DEBUG("%s bt_host_wake is %d\n", __func__, marlin_dev->bt_host_wake);
#endif
#ifdef CONFIG_WCN_WL_HOST_WAKE
	marlin_dev->wl_host_wake = -1;
	WCN_DEBUG("%s wl_host_wake is %d\n", __func__, marlin_dev->wl_host_wake);
#endif
#ifdef CONFIG_WCN_EXTERNAL_3V3_DCDC
	marlin_dev->wcn_power_en = -1;
	WCN_DEBUG("%s wcn_power_en is %d\n", __func__, marlin_dev->wcn_power_en);
#endif
#ifdef CONFIG_WCN_EXTERNAL_1V2_DCDC
	marlin_dev->wcn_1v2 = -1;
	WCN_DEBUG("%s wcn_1v2 is %d\n", __func__, marlin_dev->wcn_1v2);
#endif
	sprintf(BTWF_FIRMWARE_PATH, "%s%s", wcn_fw_path[0], WCN_FW_NAME);
	WCN_INFO("btwf firmware path is %s\n", BTWF_FIRMWARE_PATH);
	sprintf(GNSS_FIRMWARE_PATH, "%s%s", wcn_fw_path[0], GNSS_FW_NAME);
	WCN_INFO("gnss firmware path is %s\n", GNSS_FIRMWARE_PATH);

	WCN_INFO("%s done\n", __func__);

	return 0;
}
#else
static int marlin_parse_dt(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
#ifndef CONFIG_THIRD_PARTY_BOARD
	struct regmap *pmu_apb_gpr;
#ifndef CONFIG_UMW2653
	char *buf;
	struct wcn_clock_info *clk;
#endif
#endif
	struct resource res;

	if (!marlin_dev)
		return -1;

#ifndef CONFIG_THIRD_PARTY_BOARD
#ifndef CONFIG_UMW2653
	wcn_get_pmic_config(np);
#else
	marlin_dev->coexist = of_get_named_gpio(np,
			"m2-to-ap-coexist-gpios", 0);
	if (!gpio_is_valid(marlin_dev->coexist)) {
		WCN_ERR("get m2-to-ap-coexist-gpios err\n");
	}
#endif
#endif

	marlin_dev->wakeup_ap = of_get_named_gpio(np, "m2-wakeup-ap-gpios", 0);
	if (!gpio_is_valid(marlin_dev->wakeup_ap))
		WCN_ERR("can not get wakeup gpio\n");
	
	marlin_dev->reset = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(marlin_dev->reset))
	{		
		WCN_ERR("reset-gpios not config! disable reset function\n");
	}

	marlin_dev->chip_en = of_get_named_gpio(np, "enable-gpios", 0);
	if (!gpio_is_valid(marlin_dev->chip_en))
		WCN_ERR("enable-gpios not config! disable chip_en!\n");

	marlin_dev->int_ap = of_get_named_gpio(np,
			"m2-to-ap-irq-gpios", 0);
	if (!gpio_is_valid(marlin_dev->int_ap)) {
		WCN_ERR("Get int irq error!\n");
	}
#ifdef CONFIG_THIRD_PARTY_BOARD
	marlin_dev->avdd18 = devm_regulator_get(&pdev->dev, "avdd18");
	if (IS_ERR(marlin_dev->avdd18)) {
		WCN_ERR("avdd18 err =%ld\n", PTR_ERR(marlin_dev->avdd18));
		if (PTR_ERR(marlin_dev->avdd18) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		WCN_ERR("Get regulator of avdd18 error!\n");
	}

	marlin_dev->avdd33 = devm_regulator_get(&pdev->dev, "avdd33");
	if (IS_ERR(marlin_dev->avdd33)) {
		if (PTR_ERR(marlin_dev->avdd33) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		WCN_ERR("Get regulator of avdd33 error!\n");
	}
#endif

#ifndef CONFIG_THIRD_PARTY_BOARD
#ifndef CONFIG_UMW2653
	clk = &marlin_dev->clk_xtal_26m;
	clk->gpio = of_get_named_gpio(np, "xtal-26m-clk-type-gpio", 0);
	if (!gpio_is_valid(clk->gpio))
		WCN_INFO("xtal-26m-clk gpio not config\n");

	/* xtal-26m-clk-type has priority over than xtal-26m-clk-type-gpio */
	ret = of_property_read_string(np, "xtal-26m-clk-type",
				      (const char **)&buf);
	if (!ret) {
		WCN_INFO("force config xtal 26m clk %s\n", buf);
		if (!strncmp(buf, "TCXO", 4))
			clk->type = WCN_CLOCK_TYPE_TCXO;
		else if (!strncmp(buf, "TSX", 3))
			clk->type = WCN_CLOCK_TYPE_TSX;
		else
			WCN_ERR("force config xtal 26m clk %s err!\n", buf);
	} else {
		if (clktype == 0) {
			WCN_INFO("cmd config clk TCXO\n");
			clk->type = WCN_CLOCK_TYPE_TCXO;
		} else if (clktype == 1) {
			WCN_INFO("cmd config clk TSX\n");
			clk->type = WCN_CLOCK_TYPE_TSX;
		} else {
			WCN_INFO("may be not config clktype:%d\n", clktype);
			clk->type = WCN_CLOCK_TYPE_UNKNOWN;
		}
	}
#endif

	marlin_dev->dvdd12 = devm_regulator_get(&pdev->dev, "dvdd12");
	if (IS_ERR(marlin_dev->dvdd12)) {
		WCN_INFO("Get regulator of dvdd12 error!\n");
		WCN_INFO("Maybe share the power with mem\n");
	}

#ifndef CONFIG_UMW2653
	if (of_property_read_bool(np, "bound-avdd12")) {
		WCN_INFO("forbid avdd12 power ctrl\n");
		marlin_dev->bound_avdd12 = true;
	} else {
		WCN_INFO("do avdd12 power ctrl\n");
		marlin_dev->bound_avdd12 = false;
	}
#endif

	marlin_dev->avdd12 = devm_regulator_get(&pdev->dev, "avdd12");
	if (IS_ERR(marlin_dev->avdd12)) {
		WCN_ERR("avdd12 err =%ld\n", PTR_ERR(marlin_dev->avdd12));
		if (PTR_ERR(marlin_dev->avdd12) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		WCN_ERR("Get regulator of avdd12 error!\n");
	}

	marlin_dev->avdd33 = devm_regulator_get(&pdev->dev, "avdd33");
	if (IS_ERR(marlin_dev->avdd33)) {
		if (PTR_ERR(marlin_dev->avdd33) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		WCN_ERR("Get regulator of avdd33 error!\n");
	}

	marlin_dev->dcxo18 = devm_regulator_get(&pdev->dev, "dcxo18");
	if (IS_ERR(marlin_dev->dcxo18)) {
		if (PTR_ERR(marlin_dev->dcxo18) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		WCN_ERR("Get regulator of dcxo18 error!\n");
	}

#ifndef CONFIG_UMW2653
	if (of_property_read_bool(np, "bound-dcxo18")) {
		WCN_INFO("forbid dcxo18 power ctrl\n");
		marlin_dev->bound_dcxo18 = true;
	} else {
		WCN_INFO("do dcxo18 power ctrl\n");
		marlin_dev->bound_dcxo18 = false;
	}
#endif

	marlin_dev->clk_32k = devm_clk_get(&pdev->dev, "clk_32k");
	if (IS_ERR(marlin_dev->clk_32k)) {
		WCN_ERR("can't get wcn clock dts config: clk_32k\n");
	}

	marlin_dev->clk_parent = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(marlin_dev->clk_parent)) {
		WCN_ERR("can't get wcn clock dts config: source\n");
	}
	if (!IS_ERR(marlin_dev->clk_32k)) {
		clk_set_parent(marlin_dev->clk_32k, marlin_dev->clk_parent);
	}
	
	marlin_dev->clk_enable = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(marlin_dev->clk_enable)) {
		WCN_ERR("can't get wcn clock dts config: enable\n");
	}


#endif /* CONFIG_THIRD_PARTY_BOARD */

#ifdef CONFIG_WCN_EXTERNAL_3V3_DCDC
	marlin_dev->wcn_power_en = of_get_named_gpio(np,
				"wcn-power-gpios", 0);
	if (gpio_is_valid(marlin_dev->wcn_power_en)) {
		WCN_INFO("wcn-power-gpios enable!\n");
		ret = gpio_request(marlin_dev->wcn_power_en, "wcn_power_on");
		if (ret) {
			WCN_ERR("gpio wcn_power_on request fail: %d\n",
			marlin_dev->wcn_power_en);
			return -EINVAL;
		}
	}
#endif
#ifdef CONFIG_WCN_EXTERNAL_1V2_DCDC
	marlin_dev->wcn_1v2 = of_get_named_gpio(np,
				"wcn-power-gpios-1v2", 0);
	if (gpio_is_valid(marlin_dev->wcn_1v2)) {
		WCN_INFO("wcn-power-gpios-1v2 enable!\n");
		ret = gpio_request(marlin_dev->wcn_1v2, "wcn_power_on_1v2");
		if (ret) {
			WCN_ERR("gpio wcn_power_on_1v2 request fail: %d\n",
				marlin_dev->wcn_power_en);
			return -EINVAL;
		}
	}
#endif

#ifdef CONFIG_WCN_BT_HOST_WAKE
	marlin_dev->bt_host_wake = of_get_named_gpio(np, "bt-host-wake-gpios", 0);
	if (gpio_is_valid(marlin_dev->bt_host_wake)) {
		WCN_INFO("bt_host_wake[gpio%d] enable!\n", marlin_dev->bt_host_wake);
		ret = gpio_request(marlin_dev->bt_host_wake, "bt_host_wake");
		if (ret) {
			WCN_ERR("gpio bt_host_wake request fail: %d\n",
			marlin_dev->bt_host_wake);
			return -EINVAL;
		}
	}
#endif /*CONFIG_WCN_BT_HOST_WAKE*/
#ifdef CONFIG_WCN_WL_HOST_WAKE
	marlin_dev->wl_host_wake = of_get_named_gpio(np, "wl-host-wake-gpios", 0);
	if (gpio_is_valid(marlin_dev->wl_host_wake)) {
		WCN_INFO("wl_host_wake[gpio%d] enable!\n", marlin_dev->wl_host_wake);
		ret = gpio_request(marlin_dev->wl_host_wake, "wl_host_wake");
		if (ret) {
			WCN_ERR("gpio wl_host_wake request fail: %d\n",
			marlin_dev->wl_host_wake);
			return -EINVAL;
		}
	}
#endif /*CONFIG_WCN_WL_HOST_WAKE*/

	if (gpio_is_valid(marlin_dev->reset))
	{	
		ret = gpio_request(marlin_dev->reset, "wcn-reset");
		if (ret)
			WCN_ERR("gpio reset request err: %d %d\n", marlin_dev->reset, ret);
	}

	if (gpio_is_valid(marlin_dev->chip_en)) {
		gpio_free(marlin_dev->chip_en);
		ret = gpio_request(marlin_dev->chip_en, "chip_en");
		if (ret){
			WCN_ERR("chip_en request err: %d %d\n", marlin_dev->chip_en, ret);
			marlin_dev->chip_en = -1;
		}

// #ifdef CONFIG_THIRD_PARTY_BOARD
// 	    gpio_direction_output(marlin_dev->chip_en, 0);
// #endif
	}

	if (gpio_is_valid(marlin_dev->int_ap)) {
		ret = gpio_request(marlin_dev->int_ap, "int_ap");
		if (ret)
			WCN_ERR("gpio_rst request err: %d %d\n", marlin_dev->int_ap, ret);	
	}

#ifndef CONFIG_THIRD_PARTY_BOARD 
#ifndef CONFIG_UMW2653
	if (gpio_is_valid(clk->gpio)) {
		ret = gpio_request(clk->gpio, "wcn_xtal_26m_type");
		if (ret)
			WCN_ERR("xtal 26m gpio request err: %d\n", ret);
	}
#endif
#endif

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		WCN_INFO("No BTWF mem.\n");
	} else {
		marlin_dev->base_addr_btwf = res.start;
		marlin_dev->maxsz_btwf = resource_size(&res);
		WCN_INFO("cp base = 0x%llx, size = 0x%x\n",
			 (u64)marlin_dev->base_addr_btwf, marlin_dev->maxsz_btwf);
	}

#ifdef CONFGI_WCN_GNSS
	ret = of_address_to_resource(np, 1, &res);
	if (ret) {
		WCN_INFO("No GNSS mem.\n");
	} else {
		marlin_dev->base_addr_gnss = res.start;
		marlin_dev->maxsz_gnss = resource_size(&res);
		WCN_INFO("cp base = 0x%x, size = 0x%x\n",
			 (u64)marlin_dev->base_addr_gnss, marlin_dev->maxsz_gnss);
	}
#endif /* CONFGI_WCN_GNSS */

	WCN_INFO("BTWF_FIRMWARE_PATH len=%ld\n",
		(long)strlen(BTWF_FIRMWARE_PATH));
	ret = of_property_read_string(np, "sprd,btwf-file-name",
				      (const char **)&marlin_dev->btwf_path);
	if (!ret) {
		WCN_INFO("btwf firmware name:%s\n", marlin_dev->btwf_path);
		strcpy(BTWF_FIRMWARE_PATH, marlin_dev->btwf_path);
		WCN_INFO("BTWG path is %s\n", BTWF_FIRMWARE_PATH);
	}

	WCN_INFO("BTWF_FIRMWARE_PATH2 len=%ld\n",
		(long)strlen(BTWF_FIRMWARE_PATH));

#ifdef CONFGI_WCN_GNSS
	ret = of_property_read_string(np, "sprd,gnss-file-name",
				      (const char **)&marlin_dev->gnss_path);
	if (!ret) {
		WCN_INFO("gnss firmware name:%s\n", marlin_dev->gnss_path);
		strcpy(GNSS_FIRMWARE_PATH, marlin_dev->gnss_path);
	}
#endif /* CONFGI_WCN_GNSS */
#ifdef COMFIG_FSTAB_AB
	if (fstab_ab) {
		if (strncmp(fstab_ab + strlen(SUFFIX), "_a", 2) == 0) {
			strcat(BTWF_FIRMWARE_PATH, "_a");
			strcat(GNSS_FIRMWARE_PATH, "_a");
		} else if (strncmp(fstab_ab + strlen(SUFFIX), "_b", 2) == 0) {
			strcat(BTWF_FIRMWARE_PATH, "_b");
			strcat(GNSS_FIRMWARE_PATH, "_b");
		}
		WCN_INFO("BTWG path:%s\n GNSS path:%s\n",
			 BTWF_FIRMWARE_PATH, GNSS_FIRMWARE_PATH);
	}
#endif
	if (of_property_read_bool(np, "keep-power-on")) {
		WCN_INFO("wcn config keep power on\n");
		marlin_dev->keep_power_on = true;
	}
#ifdef CONFIG_WCN_USB
	WCN_INFO("wcn usb interface config keep power on\n");
	marlin_dev->keep_power_on = true;
#endif
	if (of_property_read_bool(np, "wait-ge2")) {
		WCN_INFO("wait-ge2 need wait gps ready\n");
		marlin_dev->wait_ge2 = true;
	}

#ifndef CONFIG_THIRD_PARTY_BOARD 
	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(np,
				"sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr)) {
		WCN_ERR("%s:failed to find pmu_apb_gpr(26M)(ignore)\n",
				__func__);
		return -EINVAL;
	}
	ret = regmap_read(pmu_apb_gpr, REG_PMU_APB_XTL_WAIT_CNT0,
					&clk_wait_val);
	WCN_INFO("marlin2 clk_wait value is 0x%x\n", clk_wait_val);

	ret = of_property_read_u32(np, "sprd,reg-m2-apb-xtl-wait-addr",
			&marlin2_clk_wait_reg);
	if (ret) {
		WCN_ERR("Did not find reg-m2-apb-xtl-wait-addr\n");
		return -EINVAL;
	}
	WCN_INFO("marlin2 clk reg is %d\n", marlin2_clk_wait_reg);
#endif /* CONFIG_THIRD_PARTY_BOARD */

	return 0;
}
#endif

static int marlin_gpio_free(struct platform_device *pdev)
{
	if (!marlin_dev)
		return -1;

	if (marlin_dev->reset > 0)
		gpio_free(marlin_dev->reset);
	
	if (marlin_dev->chip_en > 0)
		gpio_free(marlin_dev->chip_en);
	if (marlin_dev->int_ap > 0)
		gpio_free(marlin_dev->int_ap);
	
	if (!gpio_is_valid(marlin_dev->clk_xtal_26m.gpio))
		gpio_free(marlin_dev->clk_xtal_26m.gpio);

	return 0;
}

static int marlin_clk_enable(bool enable)
{
	int ret = 0;
	if (IS_ERR(marlin_dev->clk_32k))
		return -1;
	if (enable) {
		ret = clk_prepare_enable(marlin_dev->clk_32k);
		ret = clk_prepare_enable(marlin_dev->clk_enable);
		WCN_INFO("marlin %s successfully!\n", __func__);
	} else {
		clk_disable_unprepare(marlin_dev->clk_enable);
		clk_disable_unprepare(marlin_dev->clk_32k);
	}
	return ret;
}

static int marlin_avdd18_dcxo_enable(bool enable)
{
	int ret = 0;

	if (!marlin_dev->dcxo18)
		return 0;

	if (enable) {
#ifndef CONFIG_WCN_PCIE
		if (!marlin_dev->bound_dcxo18 &&
		    regulator_is_enabled(marlin_dev->dcxo18)) {
			WCN_INFO("avdd18_dcxo 1v8 have enable\n");
			return 0;
		}
#endif
		WCN_INFO("avdd18_dcxo set 1v8\n");
		regulator_set_voltage(marlin_dev->dcxo18, 1800000, 1800000);
		if (!marlin_dev->bound_dcxo18) {
			WCN_INFO("avdd18_dcxo power enable\n");
			ret = regulator_enable(marlin_dev->dcxo18);
			if (ret)
				WCN_ERR("fail to enable avdd18_dcxo\n");
		}
	} else {
		if (!marlin_dev->bound_dcxo18 &&
		    regulator_is_enabled(marlin_dev->dcxo18)) {
			WCN_INFO("avdd18_dcxo power disable\n");
			ret = regulator_disable(marlin_dev->dcxo18);
			if (ret)
				WCN_ERR("fail to disable avdd18_dcxo\n");
		}
	}

	return ret;
}

static int marlin_digital_power_enable(bool enable)
{
	int ret = 0;

	if (marlin_dev->dvdd12 == NULL)
		return 0;

	WCN_INFO("%s D1v2 %d\n", __func__, enable);

	if (enable) {
		regulator_set_voltage(marlin_dev->dvdd12, 200000, 1200000);
		ret = regulator_enable(marlin_dev->dvdd12);
	} else {
		if (regulator_is_enabled(marlin_dev->dvdd12))
			ret = regulator_disable(marlin_dev->dvdd12);
	}

	return ret;
}

#ifdef CONFIG_WCN_EXTERNAL_1V2_DCDC
static int marlin_analog_power1v2_enable(bool enable)
{
	int wcn_1v2 = 0;
	if (enable) {
		if (gpio_is_valid(marlin_dev->wcn_1v2))
			gpio_direction_output(marlin_dev->wcn_1v2, 1);
	} else {
		if (gpio_is_valid(marlin_dev->wcn_1v2))
			gpio_direction_output(marlin_dev->wcn_1v2, 0);
	}
	wcn_1v2 = gpio_get_value(marlin_dev->wcn_1v2);
	WCN_INFO("%s marlin 1V2 set COMPLETE\n wcn_1v2 %d", __func__, wcn_1v2);
	return 0;
}
#endif

static int marlin_analog_power_enable(bool enable)
{
	int ret = 0;

	if (marlin_dev->avdd12 != NULL) {
		usleep_range(4000, 5000);
		if (enable) {
#ifdef CONFIG_WCN_PCIE
			WCN_INFO("%s avdd12 set 1.35v\n", __func__);
			regulator_set_voltage(marlin_dev->avdd12,
					      1350000, 1350000);
#else
			WCN_INFO("%s avdd12 set 1.2v\n", __func__);
			regulator_set_voltage(marlin_dev->avdd12,
					      1200000, 1200000);
#endif
#ifndef CONFIG_UMW2653
			if (!marlin_dev->bound_avdd12) {
#endif
				WCN_INFO("%s avdd12 power enable\n", __func__);
				ret = regulator_enable(marlin_dev->avdd12);
				if (ret)
					WCN_ERR("fail to enalbe avdd12\n");
#ifndef CONFIG_UMW2653
			}
#endif
		} else {
			if (
#ifndef CONFIG_UMW2653
				!marlin_dev->bound_avdd12 &&
#endif
			    regulator_is_enabled(marlin_dev->avdd12)) {
				WCN_INFO("%s avdd12 power disable\n", __func__);
				ret = regulator_disable(marlin_dev->avdd12);
				if (ret)
					WCN_ERR("fail to disable avdd12\n");
			}
		}
	}
#ifdef CONFIG_WCN_EXTERNAL_1V2_DCDC
//	usleep_range(4000, 5000);
	if (enable) {
		marlin_analog_power1v2_enable(true);
		usleep_range(42, 44); //VDD1V2---(42US)--->DVDD_CORE---(55.2MS)--->VDDWIFIPA
		mdelay(55);
		usleep_range(200, 300);
	} else {
		marlin_analog_power1v2_enable(false);
	}
#endif
	return ret;
}

/*
 * hold cpu means cpu register is clear
 * different from reset pin gpio
 * reset gpio is all register is clear
 */
void marlin_hold_cpu(void)
{
	int ret = 0;
	unsigned int temp_reg_val = 0;

	ret = sprdwcn_bus_reg_read(CP_RESET_REG, &temp_reg_val, 4);
	if (ret < 0) {
		WCN_ERR("%s read reset reg error:%d\n", __func__, ret);
		return;
	}
	WCN_INFO("%s reset reg val:0x%x\n", __func__, temp_reg_val);
	temp_reg_val |= 1;
	ret = sprdwcn_bus_reg_write(CP_RESET_REG, &temp_reg_val, 4);
	if (ret < 0) {
		WCN_ERR("%s write reset reg error:%d\n", __func__, ret);
		return;
	}
}

void marlin_read_cali_data(void)
{
	int err;

	WCN_INFO("marlin sync entry is_calibrated:%d\n",
		wifi_data.cali_data.cali_config.is_calibrated);

	if (!wifi_data.cali_data.cali_config.is_calibrated) {
		memset(&wifi_data.cali_data, 0x0,
			sizeof(struct wifi_cali_t));
		err = sprdwcn_bus_reg_read(CALI_OFSET_REG,
			&wifi_data.cali_data, sizeof(struct wifi_cali_t));
		if (err < 0) {
			WCN_ERR("marlin read cali data fail:%d\n", err);
			return;
		}
	}

	if ((marlin2_clk_wait_reg > 0) && (clk_wait_val > 0)) {
		sprdwcn_bus_reg_read(marlin2_clk_wait_reg,
					&cp_clk_wait_val, 4);
		WCN_INFO("marlin2 cp_clk_wait_val is 0x%x\n", cp_clk_wait_val);
		clk_wait_val = ((clk_wait_val & 0xFF00) >> 8);
		cp_clk_wait_val =
			((cp_clk_wait_val & 0xFFFFFC00) | clk_wait_val);
		WCN_INFO("marlin2 cp_clk_wait_val is modifyed 0x%x\n",
					cp_clk_wait_val);
		err = sprdwcn_bus_reg_write(marlin2_clk_wait_reg,
					       &cp_clk_wait_val, 4);
		if (err < 0)
			WCN_ERR("marlin2 write 26M error:%d\n", err);
	}

	/* write this flag to notify cp that ap read calibration data */
	reg_val = 0xbbbbbbbb;
	err = sprdwcn_bus_reg_write(CALI_REG, &reg_val, 4);
	if (err < 0) {
		WCN_ERR("marlin write cali finish error:%d\n", err);
		return;
	}

	sprdwcn_bus_runtime_get();

	//complete(&marlin_dev->download_done);
}

#if defined(CONFIG_UMW2652) && defined(CONFIG_THIRD_PARTY_BOARD) && defined(CONFIG_WCN_SDIO)
static void marlin_send_sdio_config_to_cp_vendor(void)
{
	union wcn_sdiohal_config sdio_cfg = {0};
	/* Vendor config */

	/* bit[0]: sdio_config_en:
	 * 0: disable sdio config
	 * 1: enable sdio config
	 */
	sdio_cfg.cfg.sdio_config_en = 1;
	
	/* bit[4]: sdio_rx_mode: 0: adma; 1: sdma */
	if (sprdwcn_bus_get_rx_mode()) {
		sdio_cfg.cfg.sdio_rx_mode = 0;
		WCN_INFO("sdio_config rx mode:[adma]\n");
	} else {
		sdio_cfg.cfg.sdio_rx_mode = 1;
		WCN_INFO("sdio_config rx mode:[sdma]\n");
	}

	/* bit[7:5]: sdio_blk_size: 000: blocksize 840; 001: blocksize 512 */
	if (sprdwcn_bus_get_blk_size() == 512) {
		sdio_cfg.cfg.sdio_blk_size = 1;
		WCN_INFO("sdio_config blksize:[512]\n");
	} else
		WCN_INFO("sdio_config blksize:[840]\n");

		
	/* bit[12:11]: sdio_irq_type:
	 * 00:dedicated irq, gpio1
	 * 01:inband data1 irq
	 * 10:use BT_WAKEUP_HOST(pubint) pin as gpio irq
	 * 11:use WL_WAKEUP_HOST(esmd3) pin as gpio irq
	 */
	sdio_cfg.cfg.sdio_irq_type = sprdwcn_bus_get_irq_type();
		
	if (sdio_cfg.cfg.sdio_irq_type == 0)
		WCN_INFO("sdio_config sdio_irq:[gpio1]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 1)
		WCN_INFO("sdio_config irq:[inband]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 2)
		WCN_INFO("sdio_config sdio_irq:[pubint]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 3)
		WCN_INFO("sdio_config sdio_irq:[esmd3]\n");
	else
		WCN_INFO("sdio_config sdio_irq:[error]\n");

	marlin_dev->sync_f.sdio_config = sdio_cfg.val;
}
#endif

#if defined(CONFIG_UMW2653) && defined(CONFIG_THIRD_PARTY_BOARD) && defined(CONFIG_WCN_SDIO)
static void marlin_send_sdio_config_to_cp_vendor(void)
{
	union wcn_sdiohal_config sdio_cfg = {0};

	/* Vendor config */

	/* bit[0]: sdio_config_en:
	 * 0: disable sdio config
	 * 1: enable sdio config
	 */
	sdio_cfg.cfg.sdio_config_en = 1;

	/* bit[3:1]: vendor_id:
	 * 000: default id, unisoc[0x0]
	 * 001: hisilicon default version, pull chipen after resume
	 * 010: hisilicon version, keep power (NOT pull chipen) and
	 *      reset sdio after resume
	 */
	sdio_cfg.cfg.vendor_id = WCN_VENDOR_DEFAULT;
	WCN_DEBUG("sdio_config vendor:[default]\n");

	/* bit[4]: sdio_rx_mode: 0: adma; 1: sdma */
	if (sprdwcn_bus_get_rx_mode()) {
		sdio_cfg.cfg.sdio_rx_mode = 0;
		WCN_DEBUG("sdio_config rx mode:[adma]\n");
	} else {
		sdio_cfg.cfg.sdio_rx_mode = 1;
		WCN_INFO("sdio_config rx mode:[sdma]\n");
	}

	/* bit[7:5]: sdio_blk_size: 000: blocksize 840; 001: blocksize 512 */
	if (sprdwcn_bus_get_blk_size() == 512) {
		sdio_cfg.cfg.sdio_blk_size = 1;
		WCN_INFO("sdio_config blksize:[512]\n");
	} else
		WCN_INFO("sdio_config blksize:[840]\n");

	/*
	 * bit[8]: bt_wake_host_en: 0: disable, 1: enable
	 *
	 * When bit[8] is 1, bit[10:9] region will be parsed:
	 * bit[10:9]: bt_wake_host_trigger_type:
	 * 00:BT_WAKEUP_HOST  trigger type low
	 * 01:BT_WAKEUP_HOST  trigger type rising
	 * 10:BT_WAKEUP_HOST  trigger type falling
	 * 11:BT_WAKEUP_HOST  trigger type high
	 */
	sdio_cfg.cfg.bt_wake_host_en = 0;
	WCN_INFO("sdio_config bt_wake_host:[en]\n");

	//sdio_cfg.cfg.bt_wake_host_trigger_type = 0;
	//WCN_INFO("sdio_config bt_wake_host trigger:[low]\n");

	//sdio_cfg.cfg.bt_wake_host_trigger_type = 3;
	//WCN_INFO("sdio_config bt_wake_host trigger:[high]\n");

	/* bit[12:11]: sdio_irq_type:
	 * 00:dedicated irq, gpio1
	 * 01:inband data1 irq
	 * 10:use BT_WAKEUP_HOST(pubint) pin as gpio irq
	 * 11:use WL_WAKEUP_HOST(esmd3) pin as gpio irq
	 */
	sdio_cfg.cfg.sdio_irq_type = sprdwcn_bus_get_irq_type();
		
	if (sdio_cfg.cfg.sdio_irq_type == 0)
		WCN_INFO("sdio_config sdio_irq:[gpio1]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 1)
		WCN_INFO("sdio_config irq:[inband]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 2)
		WCN_INFO("sdio_config sdio_irq:[pubint]\n");
	else if (sdio_cfg.cfg.sdio_irq_type == 3)
		WCN_INFO("sdio_config sdio_irq:[esmd3]\n");
	else
		WCN_INFO("sdio_config sdio_irq:[error]\n");
	/*
	 * When bit[12:11] is 10/11, bit[14:13] region will be parsed:
	 * bit[14:13]: sdio_irq_trigger_type:
	 * 00:pubint gpio irq trigger type low
	 * 01:pubint gpio irq trigger type rising [NOT support]
	 * 10:pubint gpio irq trigger type falling [NOT support]
	 * 11:pubint gpio irq trigger type high
	 */
	//sdio_cfg.cfg.sdio_irq_trigger_type = 0;
	//WCN_INFO("sdio_config sdio_irq trigger:[low]\n");
	//sdio_cfg.cfg.sdio_irq_trigger_type = 3;
	//WCN_INFO("sdio_config sdio_irq trigger:[high]\n");

	/*
	 * bit[15]: wl_wake_host_en: 0: disable, 1: enable
	 *
	 * When bit[15] is 1, bit[17:16] region will be parsed:
	 * bit[17:16]: wl_wake_host_trigger_type:
	 * 00:WL_WAKEUP_HOST  trigger type low
	 * 01:WL_WAKEUP_HOST  trigger type rising
	 * 10:WL_WAKEUP_HOST  trigger type falling
	 * 11:WL_WAKEUP_HOST  trigger type high
	 */
	sdio_cfg.cfg.wl_wake_host_en = 0;
	WCN_INFO("sdio_config wl_wake_host:[en]\n");

	//sdio_cfg.cfg.wl_wake_host_trigger_type = 0;
	//WCN_INFO("sdio_config wl_wake_host trigger:[low]\n");

	/*
	 * bit[22:18]: wake_host_level_duration_10s: BT_WAKEUP_HOST or
	 * WL_WAKEUP_HOST level dyration time per 10ms,
	 * example: 0:0ms; 3:30ms; 20:200ms
	 */
	sdio_cfg.cfg.wake_host_level_duration_10ms = 2;
	WCN_INFO("sdio_config wake_host_level_duration_time:[%dms]\n",
		 (sdio_cfg.cfg.wake_host_level_duration_10ms * 10));

	/*
	 * bit[23]: wake_host_data_separation:
	 * 0: if BT_WAKEUP_HOST en or WL_WAKEUP_HOST en,
	 *    wifi and bt packets can wake host;
	 * 1: if BT_WAKEUP_HOST en, ONLY bt packets can wake host;
	 *    if WL_WAKEUP_HOST en, ONLY wifi packets can wake host
	 */
	sdio_cfg.cfg.wake_host_data_separation = 0;
	WCN_INFO("sdio_config wake_host_data_separation:[bt/wifi reuse]\n");

	marlin_dev->sync_f.sdio_config = sdio_cfg.val;

}
#endif

#if defined(CONFIG_THIRD_PARTY_BOARD) && defined(CONFIG_WCN_SDIO)
static int marlin_send_sdio_config_to_cp(void)
{
	int sdio_config_off = 0;

	sdio_config_off = (unsigned long)(&(marlin_dev->sync_f.sdio_config)) -
		(unsigned long)(&(marlin_dev->sync_f));
	WCN_INFO("sdio_config_offset:0x%x\n", sdio_config_off);
#if defined(CONFIG_UMW2652) || defined(CONFIG_UMW2653)
	marlin_send_sdio_config_to_cp_vendor();
#endif
	WCN_INFO("%s sdio_config:0x%x (%sable config)\n",
		 __func__, marlin_dev->sync_f.sdio_config,
		 (marlin_dev->sync_f.sdio_config & BIT(0)) ? "en" : "dis");

	return sprdwcn_bus_reg_write(SYNC_ADDR + sdio_config_off,
				     &(marlin_dev->sync_f.sdio_config), 4);
}
#endif

#if (!defined(CONFIG_WCN_PCIE) && !defined(CONFIG_WCN_USB))
static int marlin_write_cali_data(void)
{
	int i;
	int ret = 0, init_state = 0, cali_data_offset = 0;

	WCN_INFO("tsx_dac_data:%d\n", marlin_dev->tsxcali.tsxdata.dac);
	cali_data_offset = (unsigned long)(&(marlin_dev->sync_f.tsx_dac_data))
		- (unsigned long)(&(marlin_dev->sync_f));
	WCN_INFO("cali_data_offset:0x%x\n", cali_data_offset);
	WCN_INFO("SYNC_ADDR:0x%x\n", SYNC_ADDR);
	for (i = 0; i <= 65; i++) {
		ret = sprdwcn_bus_reg_read(SYNC_ADDR, &init_state, 4);
		if (ret < 0) {
			WCN_ERR("%s marlin3 read SYNC_ADDR error:%d\n",
				__func__, ret);
			return ret;
		}
		WCN_INFO("%s sync init_state:0x%x\n", __func__, init_state);
		if (init_state != SYNC_CALI_WAITING)
			usleep_range(3000, 5000);
		/* wait cp in the state of waiting cali data */
		else {
			/* write cali data to cp */
			marlin_dev->sync_f.tsx_dac_data =
					marlin_dev->tsxcali.tsxdata.dac;
			ret = sprdwcn_bus_direct_write(SYNC_ADDR +
					cali_data_offset,
					&(marlin_dev->sync_f.tsx_dac_data), 2);
			if (ret < 0) {
				WCN_ERR("write cali data error:%d\n", ret);
				return ret;
			}
#if defined(CONFIG_THIRD_PARTY_BOARD) && defined(CONFIG_WCN_SDIO)
			/*write sdio config to cp*/
			ret = marlin_send_sdio_config_to_cp();
			if (ret < 0) {
				WCN_ERR("write sdio_config error:%d\n", ret);
				return ret;
			}
#endif
			/* tell cp2 can handle cali data */
			init_state = SYNC_CALI_WRITE_DONE;
			ret = sprdwcn_bus_reg_write(SYNC_ADDR, &init_state, 4);
			if (ret < 0) {
				WCN_ERR("write cali_done flag error:%d\n", ret);
				return ret;
			}

			WCN_INFO("%s finish\n", __func__);
			return ret;
		}
	}

	WCN_ERR("%s sync init_state:0x%x\n", __func__, init_state);

	return -1;
}
#endif

enum wcn_clock_type wcn_get_xtal_26m_clk_type(void)
{
	return marlin_dev->clk_xtal_26m.type;
}
EXPORT_SYMBOL_GPL(wcn_get_xtal_26m_clk_type);

enum wcn_clock_mode wcn_get_xtal_26m_clk_mode(void)
{
	return marlin_dev->clk_xtal_26m.mode;
}
EXPORT_SYMBOL_GPL(wcn_get_xtal_26m_clk_mode);

#if (!defined(CONFIG_WCN_PCIE)) && (!defined(CONFIG_WCN_USB))
static int spi_read_rf_reg(unsigned int addr, unsigned int *data)
{
	unsigned int reg_data = 0;
	int ret;

	reg_data = ((addr & 0x7fff) << 16) | SPI_BIT31;
	ret = sprdwcn_bus_reg_write(SPI_BASE_ADDR, &reg_data, 4);
	if (ret < 0) {
		WCN_ERR("write SPI RF reg error:%d\n", ret);
		return ret;
	}

	usleep_range(4000, 6000);

	ret = sprdwcn_bus_reg_read(SPI_BASE_ADDR, &reg_data, 4);
	if (ret < 0) {
		WCN_ERR("read SPI RF reg error:%d\n", ret);
		return ret;
	}
	*data = reg_data & 0xffff;

	return 0;
}

#ifndef CONFIG_UMW2653
static void wcn_check_xtal_26m_clk(void)
{
	int ret = 0;
	unsigned int temp_val;
	struct wcn_clock_info *clk;

	clk = &marlin_dev->clk_xtal_26m;
	if (likely(clk->type != WCN_CLOCK_TYPE_UNKNOWN) &&
	    likely(clk->mode != WCN_CLOCK_MODE_UNKNOWN)) {
		WCN_INFO("xtal 26m clk type:%s mode:%s\n",
			 (clk->type == WCN_CLOCK_TYPE_TSX) ? "TSX" : "TCXO",
			 (clk->mode == WCN_CLOCK_MODE_XO) ? "XO" : "BUFFER");
		return;
	}

	if (clk->type == WCN_CLOCK_TYPE_UNKNOWN) {
		if (gpio_is_valid(clk->gpio)) {
			gpio_direction_input(clk->gpio);
			ret = gpio_get_value(clk->gpio);
			clk->type = ret ? WCN_CLOCK_TYPE_TSX :
				    WCN_CLOCK_TYPE_TCXO;
			WCN_INFO("xtal gpio clk type:%d %d\n",
				 clk->type, ret);
		} else {
			WCN_ERR("xtal_26m clk type erro!\n");
		}
	}

	if (clk->mode == WCN_CLOCK_MODE_UNKNOWN) {
		ret = spi_read_rf_reg(AD_DCXO_BONDING_OPT, &temp_val);
		if (ret < 0) {
			WCN_ERR("read AD_DCXO_BONDING_OPT error:%d\n", ret);
			return;
		}
		WCN_INFO("read AD_DCXO_BONDING_OPT val:0x%x\n", temp_val);
		if (temp_val & WCN_BOUND_XO_MODE) {
			WCN_INFO("xtal_26m clock XO mode\n");
			clk->mode = WCN_CLOCK_MODE_XO;
		} else {
			WCN_INFO("xtal_26m clock Buffer mode\n");
			clk->mode = WCN_CLOCK_MODE_BUFFER;
		}
	}
}

static int check_cp_clock_mode(void)
{
	struct wcn_clock_info *clk;

	WCN_INFO("%s\n", __func__);

	clk = &marlin_dev->clk_xtal_26m;
	if (clk->mode == WCN_CLOCK_MODE_BUFFER) {
		WCN_INFO("xtal_26m clock use BUFFER mode\n");
		marlin_avdd18_dcxo_enable(false);
		return 0;
	} else if (clk->mode == WCN_CLOCK_MODE_XO) {
		WCN_INFO("xtal_26m clock use XO mode\n");
		return 0;
	}

	return -1;
}
#elif (!defined(CONFIG_WCN_USB))
static int check_cp_clock_mode(void)
{
	int ret = 0;
	unsigned int temp_val;

	WCN_INFO("%s\n", __func__);

	ret = spi_read_rf_reg(AD_DCXO_BONDING_OPT, &temp_val);
	if (ret < 0) {
		WCN_ERR("read AD_DCXO_BONDING_OPT error:%d\n", ret);
		return ret;
	}
	WCN_INFO("read AD_DCXO_BONDING_OPT val:0x%x\n", temp_val);
	if ((temp_val & tsx_mode) == tsx_mode)
		WCN_INFO("clock mode: TSX\n");
	else {
		WCN_INFO("clock mode: TCXO, outside clock\n");
		//marlin_avdd18_dcxo_enable(false);
	}

	return ret;
}
#endif
#endif

/* release CPU */
static int marlin_start_run(void)
{
	int ret;
	unsigned int ss_val = 0;

	WCN_INFO("%s\n", __func__);

	marlin_tsx_cali_data_get();
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_btwf_en0();
	/* after chip power on, reset sleep status */
	slp_mgr_reset();
#endif

	ret = sprdwcn_bus_reg_read(CP_RESET_REG, &ss_val, 4);
	if (ret < 0) {
		WCN_ERR("%s read reset reg error:%d\n", __func__, ret);
		return ret;
	}
	WCN_INFO("%s read reset reg val:0x%x\n", __func__, ss_val);
#ifndef CONFIG_UMW2653
	ss_val &= (~0) - 1;
#else
	ss_val &= (~(RESET_BIT));
#endif
	WCN_INFO("after do %s reset reg val:0x%x\n", __func__, ss_val);
	ret = sprdwcn_bus_reg_write(CP_RESET_REG, &ss_val, 4);
	if (ret < 0) {
		WCN_ERR("%s write reset reg error:%d\n", __func__, ret);
		return ret;
	}
	/* update the time at once */
	marlin_bootup_time_update();

	ret = sprdwcn_bus_reg_read(CP_RESET_REG, &ss_val, 4);
	if (ret < 0) {
		WCN_ERR("%s read reset reg error:%d\n", __func__, ret);
		return ret;
	}
	WCN_INFO("%s after reset reg val:0x%x\n", __func__, ss_val);

	return ret;
}

/* return 0 is ready, other values is error */
static int check_cp_ready(void)
{
	int i, ret;
#ifdef CONFIG_WCN_USB
	return sprdwcn_check_cp_ready(SYNC_ADDR, 3000);
#endif

	for (i = 0; i <= 25; i++) {
		ret = sprdwcn_bus_direct_read(SYNC_ADDR,
			&(marlin_dev->sync_f), sizeof(struct wcn_sync_info_t));
		if (ret < 0) {
			WCN_ERR("%s marlin3 read SYNC_ADDR error:%d\n",
			       __func__, ret);
			return ret;
		}

		WCN_INFO("%s sync val:0x%x, prj_type val:0x%x\n", __func__,
			marlin_dev->sync_f.init_status,
			marlin_dev->sync_f.prj_type);

		if (marlin_dev->sync_f.init_status == SYNC_IN_PROGRESS)
			usleep_range(3000, 5000);
		if (marlin_dev->sync_f.init_status == SYNC_ALL_FINISHED)
			return 0;
	}

	WCN_ERR("%s sync val:0x%x, prj_type val:0x%x\n",
	       __func__, marlin_dev->sync_f.init_status,
	       marlin_dev->sync_f.prj_type);

	return -1;
}

static int gnss_start_run(void)
{
	int ret = 0;
	unsigned int temp = 0;

	WCN_INFO("gnss start run enter ");
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_gnss_en0();
#endif
	ret = sprdwcn_bus_reg_read(GNSS_CP_RESET_REG, &temp, 4);
	if (ret < 0) {
		WCN_ERR("%s marlin3_gnss read reset reg error:%d\n",
			__func__, ret);
		return ret;
	}
	WCN_INFO("%s reset reg val:0x%x\n", __func__, temp);
	temp &= (~0) - 1;
	ret = sprdwcn_bus_reg_write(GNSS_CP_RESET_REG, &temp, 4);
	if (ret < 0) {
		WCN_ERR("%s marlin3_gnss write reset reg error:%d\n",
				__func__, ret);
		return ret;
	}

	return ret;
}

static int marlin_reset(int val)
{
#ifdef CONFIG_AML_BOARD
	set_usb_wifi_power(0);
	mdelay(RESET_DELAY);
	set_usb_wifi_power(1);
	return 0;
#else
	if (marlin_dev->reset <= 0)
		return 0;

	if (gpio_is_valid(marlin_dev->reset)) {
		gpio_direction_output(marlin_dev->reset, 0);
		mdelay(RESET_DELAY);
		gpio_direction_output(marlin_dev->reset, 1);
	}

	return 0;
#endif
}

static int chip_reset_release(int val)
{
	if (marlin_dev->reset <= 0)
		return 0;
	
	if (!gpio_is_valid(marlin_dev->reset)) {
		WCN_ERR("reset gpio error\n");
		return -1;
	}

	if (val)
		gpio_direction_output(marlin_dev->reset, 1);
	else
		gpio_direction_output(marlin_dev->reset, 0);

	return 0;
}

void marlin_chip_en(bool enable, bool reset)
{
#if defined(CONFIG_AML_BOARD) && defined(CONFIG_WCN_SDIO)
	if (reset) {
		set_usb_wifi_power(0);
		msleep(100);
		set_usb_wifi_power(1);
		WCN_INFO("marlin chip en reset\n");
	} else if (enable) {
		set_usb_wifi_power(1);
		WCN_INFO("marlin chip en enable\n");
	} else {
		set_usb_wifi_power(0);
		WCN_INFO("marlin chip en disable\n");
	}
#else
	if (gpio_is_valid(marlin_dev->chip_en)) {
		if (reset) {
			gpio_direction_output(marlin_dev->chip_en, 0);
			WCN_INFO("marlin chip en reset\n");
			msleep(100);
			gpio_direction_output(marlin_dev->chip_en, 1);
		} else if (enable) {
			if(gpio_get_value(marlin_dev->chip_en)==0){
				gpio_direction_output(marlin_dev->chip_en, 0);
				mdelay(1);
				gpio_direction_output(marlin_dev->chip_en, 1);
				mdelay(1);
				WCN_INFO("marlin chip en pull up\n");
			} else {
				WCN_INFO("marlin chip en already set to high\n");
			}
		} else {
			gpio_direction_output(marlin_dev->chip_en, 0);
			WCN_INFO("marlin chip en pull down\n");
		}
	}
#endif
}
EXPORT_SYMBOL_GPL(marlin_chip_en);

static int set_cp_mem_status(enum wcn_sub_sys subsys, int val)
{
	int ret;
	unsigned int temp_val;

#if defined(CONFIG_UMW2652) || defined(CONFIG_WCN_PCIE) || defined(CONFIG_UMW2653)
	return 0;
#endif
	ret = sprdwcn_bus_reg_read(REG_WIFI_MEM_CFG1, &temp_val, 4);
	if (ret < 0) {
		WCN_ERR("%s read wifimem_cfg1 error:%d\n", __func__, ret);
		return ret;
	}
	WCN_INFO("%s read btram poweron(bit22)val:0x%x\n", __func__, temp_val);

	if ((subsys == MARLIN_BLUETOOTH) && (val == 1)) {
		temp_val = temp_val & (~FORCE_SHUTDOWN_BTRAM);
		WCN_INFO("wr btram poweron(bit22) val:0x%x\n", temp_val);
		ret = sprdwcn_bus_reg_write(REG_WIFI_MEM_CFG1, &temp_val, 4);
		if (ret < 0) {
			WCN_ERR("write wifimem_cfg1 reg error:%d\n", ret);
			return ret;
		}
		return 0;
	} else if (test_bit(MARLIN_BLUETOOTH, &marlin_dev->power_state) &&
		   (subsys != MARLIN_BLUETOOTH))
		return 0;

	temp_val = temp_val | FORCE_SHUTDOWN_BTRAM;
	WCN_INFO(" shut down btram(bit22) val:0x%x\n", temp_val);
	ret = sprdwcn_bus_reg_write(REG_WIFI_MEM_CFG1, &temp_val, 4);
	if (ret < 0) {
		WCN_ERR("write wifimem_cfg1 reg error:%d\n", ret);
		return ret;
	}

	return ret;
}

int enable_spur_remove(void)
{
	int ret;
	unsigned int temp_val;

	temp_val = FM_ENABLE_SPUR_REMOVE_FREQ2_VALUE;
	ret = sprdwcn_bus_reg_write(FM_REG_SPUR_FEQ1_ADDR, &temp_val, 4);
	if (ret < 0) {
		WCN_ERR("write FM_REG_SPUR reg error:%d\n", ret);
		return ret;
	}

	return 0;
}

int disable_spur_remove(void)
{
	int ret;
	unsigned int temp_val;

	temp_val = FM_DISABLE_SPUR_REMOVE_VALUE;
	ret = sprdwcn_bus_reg_write(FM_REG_SPUR_FEQ1_ADDR, &temp_val, 4);
	if (ret < 0) {
		WCN_ERR("write disable FM_REG_SPUR reg error:%d\n", ret);
		return ret;
	}

	return 0;
}

static void set_fm_supe_freq(enum wcn_sub_sys subsys,
			     int val, unsigned long sub_state)
{
	switch (subsys) {
	case MARLIN_FM:
		if (test_bit(MARLIN_GNSS, &sub_state) && (val == 1))
			enable_spur_remove();
		else
			disable_spur_remove();
		break;
	case MARLIN_GNSS:
		if (test_bit(MARLIN_FM, &sub_state) && (val == 1))
			enable_spur_remove();
		else
			disable_spur_remove();
		break;
	default:
		break;
	}
}

/*
 * MARLIN_GNSS no need loopcheck action
 * MARLIN_AUTO no need loopcheck action
 */
static void power_state_notify_or_not(enum wcn_sub_sys subsys, int poweron)
{

	if (poweron == 1) {
		set_cp_mem_status(subsys, poweron);
		set_fm_supe_freq(subsys, poweron, marlin_dev->power_state);
	}

	if ((test_bit(MARLIN_BLUETOOTH, &marlin_dev->power_state) +
		test_bit(MARLIN_FM, &marlin_dev->power_state) +
		test_bit(MARLIN_WIFI, &marlin_dev->power_state) +
		test_bit(MARLIN_MDBG, &marlin_dev->power_state)) == 1) {
		WCN_INFO("only one module open, need to notify loopcheck\n");
		start_loopcheck();

		marlin_dev->loopcheck_status_change = 1;
		wakeup_loopcheck_int();
	}

	if (((marlin_dev->power_state) & MARLIN_MASK) == 0) {
		WCN_INFO("marlin close, need to notify loopcheck\n");
		stop_loopcheck();
		marlin_dev->loopcheck_status_change = 1;
		wakeup_loopcheck_int();
	}
}

void marlin_scan_finish(void)
{
	WCN_INFO("%s!\n", __func__);
	complete(&marlin_dev->carddetect_done);
}
EXPORT_SYMBOL_GPL(marlin_scan_finish);

static int find_firmware_path(void)
{
	int ret;
	int pre_len;

	if (strlen(BTWF_FIRMWARE_PATH) != 0)
		return 0;

	ret = parse_firmware_path(BTWF_FIRMWARE_PATH);
	if (ret != 0) {
		WCN_ERR("can not find wcn partition\n");
		return ret;
	}
	WCN_INFO("BTWF path is %s\n", BTWF_FIRMWARE_PATH);
	pre_len = strlen(BTWF_FIRMWARE_PATH) - strlen("wcnmodem");
	memcpy(GNSS_FIRMWARE_PATH,
		BTWF_FIRMWARE_PATH,
		strlen(BTWF_FIRMWARE_PATH));
	memcpy(&GNSS_FIRMWARE_PATH[pre_len], "gnssmodem",
		strlen("gnssmodem"));
	GNSS_FIRMWARE_PATH[pre_len + strlen("gnssmodem")] = '\0';
	WCN_INFO("GNSS path is %s\n", GNSS_FIRMWARE_PATH);

	return 0;
}

#ifdef CONFIG_WCN_USB
static unsigned char fdl_hex_buf[] = {
#include "../usb/usb_fdl.bin.hex"
};
#define FDL_HEX_SIZE sizeof(fdl_hex_buf)

static int wcn_usb_fdl_download(void)
{
	int ret;
	struct marlin_firmware *firmware;

	firmware = kmalloc(sizeof(struct marlin_firmware), GFP_KERNEL);
	if (!firmware)
		return -ENOMEM;

	WCN_INFO("marlin %s from wcnmodem.bin.hex start!\n", __func__);
	firmware->data = fdl_hex_buf;
	firmware->size = FDL_HEX_SIZE;
	firmware->is_from_fs = 0;
	firmware->priv = fdl_hex_buf;
	firmware->is_from_hex = 1;

	ret = marlin_firmware_parse_image(firmware);
	if (ret) {
		WCN_ERR("%s firmware parse AA\\AB error\n", __func__);
		goto OUT;
	}

	ret = marlin_firmware_write(firmware);
	if (ret) {
		WCN_ERR("%s firmware write error\n", __func__);
		goto OUT;
	}
OUT:
	marlin_release_firmware(firmware);

	return ret;
}

/*
	* Fix Bug 1349945.
	* Because the usb vbus can't be controlled on some platforms,
	* So, BT WIFI can't work after ap sys reboot, the reason is cp state is
	* not rebooted. So, wen need pull reset pin to reset cp.
	* But on cp init, set the reset_hold reg to keep iram for dump mem,
	* it's lead to the chip can't reset cache and power state. So that,
	* the chip can't work normal.
	* To solve this problem, we need a fdl to clear the reset_hold reg
	* before re-reset. After clear the reset_hold reg, then reset chip
	* again and normal boot system.
*/
static void btwifi_download_fdl_firmware(void)
{

	WCN_INFO("%s start!\n", __func__);
	marlin_firmware_download_start_usb();
	wcn_get_chip_name();

	if (wcn_usb_fdl_download()) {
		wcn_usb_unlock();
		WCN_INFO("fdl download err\n");
		return;
	}
	msleep(100);
	wcn_usb_unlock();
}
#endif

static void pre_gnss_download_firmware(struct work_struct *work)
{
	static int cali_flag;
	int ret;

	/* ./fstab.xxx is prevent for user space progress */
	find_firmware_path();

	if (gnss_download_firmware() != 0) {
		WCN_ERR("gnss download firmware fail\n");
		return;
	}

	if (gnss_ops && (gnss_ops->write_data)) {
		if (gnss_ops->write_data() != 0)
			return;
	} else {
		WCN_ERR("%s gnss_ops write_data error\n", __func__);
	}

	if (gnss_start_run() != 0)
		WCN_ERR("gnss start run fail\n");

	if (cali_flag == 0) {
		WCN_INFO("gnss start to backup calidata\n");
		if (gnss_ops && gnss_ops->backup_data) {
			ret = gnss_ops->backup_data();
			if (ret == 0)
				cali_flag = 1;
		} else {
			WCN_ERR("%s gnss_ops backup_data error\n", __func__);
		}
	} else {
		WCN_INFO("gnss wait boot finish\n");
		if (gnss_ops && gnss_ops->wait_gnss_boot)
			gnss_ops->wait_gnss_boot();
		else
			WCN_ERR("%s gnss_ops wait boot error\n", __func__);
	}
	complete(&marlin_dev->gnss_download_done);
}

static void pre_btwifi_download_sdio(struct work_struct *work)
{
#ifdef CONFIG_WCN_USB
	marlin_firmware_download_start_usb();
#endif /*CONFIG_WCN_USB*/

	if (btwifi_download_firmware() == 0 &&
		marlin_start_run() == 0) {
#ifdef CONFIG_WCN_SDIO
		check_cp_clock_mode();
		marlin_write_cali_data();
#endif

#ifdef CONFIG_MEM_PD
		mem_pd_save_bin();
#endif

#ifdef CONFIG_WCN_RDCDBG
		wcn_debug_init();
#endif
		WCN_INFO("%s check_cp_ready start\n", __func__);
		if (check_cp_ready() != 0) {
#ifdef CONFIG_WCN_USB
			wcn_usb_unlock();
#endif
			sprdwcn_bus_set_carddump_status(true);
			return;
		}
		WCN_INFO("%s check_cp_ready\n", __func__);
#ifdef CONFIG_WCN_USB
		wcn_usb_unlock();
#endif

		complete_all(&marlin_dev->download_done);
	}
	/* Runtime PM is useless, mainly to enable sdio_func1 and rx irq */
	sprdwcn_bus_runtime_get();
	wcn_firmware_init();
#ifndef CONFIG_WCN_RDCDBG
	//switch_cp2_log(false);
#endif
}

static int bus_scan_card(void)
{
	init_completion(&marlin_dev->carddetect_done);
	sprdwcn_bus_rescan(marlin_dev);
	if (wait_for_completion_timeout(&marlin_dev->carddetect_done,
		msecs_to_jiffies(CARD_DETECT_WAIT_MS)) == 0) {
		WCN_ERR("wait bus rescan card time out\n");
		return -1;
	}

	return 0;
}

static void wifipa_enable(int enable)
{
	int ret = -1;
#ifndef CONFIG_WCN_PARSE_DTS
	return;
#endif
#ifdef CONFIG_WCN_EXTERNAL_3V3_DCDC
	if (enable) {
		if (gpio_is_valid(marlin_dev->wcn_power_en)) {
			gpio_direction_output(marlin_dev->wcn_power_en, 1);
			WCN_INFO("%s marlin_dev->wcn_power_en output 1\n", __func__);
		}
	}
	else{
		if (gpio_is_valid(marlin_dev->wcn_power_en)) {
			gpio_direction_output(marlin_dev->wcn_power_en, 0);
			WCN_INFO("%s marlin_dev->wcn_power_en output 0\n", __func__);
		}
	}
#endif
//	usleep_range(4000, 5000);
	if (marlin_dev->avdd33) {
		WCN_INFO("wifipa 3v3 %d\n", enable);
		usleep_range(4000, 5000);
		if (enable) {
			if (regulator_is_enabled(marlin_dev->avdd33))
				return;

			regulator_set_voltage(marlin_dev->avdd33,
					      3300000, 3300000);
			ret = regulator_enable(marlin_dev->avdd33);
			if (ret)
				WCN_ERR("fail to enable wifipa\n");
		} else {
			if (regulator_is_enabled(marlin_dev->avdd33)) {
				ret = regulator_disable(marlin_dev->avdd33);
				if (ret)
					WCN_ERR("fail to disable wifipa\n");
			}
		}
	}
}

static void set_wifipa_status(enum wcn_sub_sys subsys, int val)
{
	if (val == 1) {
		if (((subsys == MARLIN_BLUETOOTH) || (subsys == MARLIN_WIFI)) &&
		    ((marlin_dev->power_state & 0x5) == 0)) {
			wifipa_enable(1);
			wcn_wifipa_bound_xtl(true);
		}

		if (((subsys != MARLIN_BLUETOOTH) && (subsys != MARLIN_WIFI)) &&
		    ((marlin_dev->power_state & 0x5) == 0)) {
			wcn_wifipa_bound_xtl(false);
			wifipa_enable(0);
		}
	} else {
		if (((subsys == MARLIN_BLUETOOTH) &&
		     ((marlin_dev->power_state & 0x4) == 0)) ||
		    ((subsys == MARLIN_WIFI) &&
		     ((marlin_dev->power_state & 0x1) == 0))) {
			wcn_wifipa_bound_xtl(false);
			wifipa_enable(0);
		}
	}
}

/*
 * RST_N (LOW)
 * VDDIO -> DVDD12/11 ->CHIP_EN ->DVDD_CORE(inner)
 * ->(>=550uS) RST_N (HIGH)
 * ->(>=100uS) ADVV12
 * ->(>=10uS)  AVDD33
 */
 #ifdef CONFIG_THIRD_PARTY_BOARD
 /* For customer to achieve
   please meke sure the power on/off sequence is correct!!!
 */
 static int chip_power_on(enum wcn_sub_sys subsys)
 {
	WCN_INFO("[%d]:%s, called by thirdpart!\n", subsys, __func__);
	 /* the function below as reference, you can change to adapt main control*/
	marlin_avdd18_dcxo_enable(true);      //1.8v power on
	marlin_clk_enable(true);              //32k lck enable
	marlin_digital_power_enable(true);    //digital 1.2v power on
	marlin_chip_en(true, false);          //chip_power_on enable
	msleep(20);                           //sequence control
	chip_reset_release(1);                //chip reset enable
	marlin_analog_power_enable(true);     //analog 1.2v power on
	wifipa_enable(1);                     //3.3v power on


	/* do not change the follow content*/
	if (bus_scan_card() < 0)
		return -1;
	loopcheck_ready_set();

#ifdef CONFIG_MEM_PD
	mem_pd_poweroff_deinit();
#endif
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_poweron(true);
#endif
#if (!defined(CONFIG_UMW2653)) && (!defined(CONFIG_WCN_PCIE))
	wcn_check_xtal_26m_clk();
#endif
	return 0;
}
 
static int chip_power_off(enum wcn_sub_sys subsys)
{
	WCN_INFO("[%d]:%s, called by thirdpart!\n", subsys, __func__);
#ifdef CONFIG_WCN_PCIE
	sprdwcn_bus_remove_card(marlin_dev);
#endif
	marlin_dev->power_state = 0;
	
	 /* the function below as reference, you can change to adapt main control*/
	wifipa_enable(0);                   //3.3v power off
	marlin_avdd18_dcxo_enable(false);   //1.8v powr off
	marlin_clk_enable(false);           //32k clk disable
	marlin_chip_en(false, false);       //chip_en disable
	marlin_digital_power_enable(false); //digital 1.2v disable
	marlin_analog_power_enable(false);  //analog 1.2v disable
	chip_reset_release(0);              // chip reset disable
	
	/* do not change the follow content*/
	marlin_dev->wifi_need_download_ini_flag = 0;

#ifdef CONFIG_MEM_PD
	mem_pd_poweroff_deinit();
#endif
#ifndef CONFIG_WCN_PCIE
	sprdwcn_bus_remove_card(marlin_dev);
#endif
	loopcheck_ready_clear();

#ifdef CONFIG_WCN_SLP
	sdio_pub_int_poweron(false);
#endif
	return 0;
 }
 #else
static int chip_power_on(enum wcn_sub_sys subsys)
{

	wcn_avdd12_parent_bound_chip(false);
	marlin_avdd18_dcxo_enable(true);
	marlin_clk_enable(true);
	marlin_digital_power_enable(true);
	marlin_chip_en(true, false);
	usleep_range(4000, 5000);
	chip_reset_release(1);
	marlin_analog_power_enable(true);
	wcn_avdd12_bound_xtl(true);
	usleep_range(50, 60);
	wifipa_enable(1);
	wcn_wifipa_bound_xtl(true);
	if (bus_scan_card() < 0)
		return -1;
	loopcheck_ready_set();

#ifdef CONFIG_MEM_PD
	mem_pd_poweroff_deinit();
#endif
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_poweron(true);
#endif
#if (!defined(CONFIG_UMW2653)) && (!defined(CONFIG_WCN_PCIE))
	wcn_check_xtal_26m_clk();
#endif

	return 0;
}

static int chip_power_off(enum wcn_sub_sys subsys)
{
#ifdef CONFIG_WCN_PCIE
	sprdwcn_bus_remove_card(marlin_dev);
#endif
	marlin_dev->power_state = 0;

	wcn_avdd12_bound_xtl(false);
	wcn_wifipa_bound_xtl(false);
	wcn_avdd12_parent_bound_chip(true);

	wifipa_enable(0);
	marlin_avdd18_dcxo_enable(false);
	marlin_clk_enable(false);
	marlin_chip_en(false, false);
	marlin_digital_power_enable(false);
	marlin_analog_power_enable(false);
	chip_reset_release(0);
	marlin_dev->wifi_need_download_ini_flag = 0;
#ifdef CONFIG_MEM_PD
	mem_pd_poweroff_deinit();
#endif
#ifndef CONFIG_WCN_PCIE
	sprdwcn_bus_remove_card(marlin_dev);
#endif
	loopcheck_ready_clear();
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_poweron(false);
#endif

	return 0;
}
#endif

void wcn_chip_power_on(void)
{
	chip_power_on(0);
}
EXPORT_SYMBOL_GPL(wcn_chip_power_on);

void wcn_chip_power_off(void)
{
	mutex_lock(&marlin_dev->power_lock);
	chip_power_off(0);
	mutex_unlock(&marlin_dev->power_lock);
}
EXPORT_SYMBOL_GPL(wcn_chip_power_off);

#ifdef CONFIG_WCN_GNSS
static int gnss_powerdomain_open(void)
{
	/* add by this. */
	int ret = 0, retry_cnt = 0;
	unsigned int temp = 0;

	WCN_INFO("%s\n", __func__);
	ret = sprdwcn_bus_reg_read(PD_GNSS_SS_AON_CFG4, &temp, 4);
	if (ret < 0) {
		WCN_ERR("%s read PD_GNSS_SS_AON_CFG4 err:%d\n", __func__, ret);
		return ret;
	}
	WCN_INFO("%s PD_GNSS_SS_AON_CFG4:0x%x\n", __func__, temp);
	temp = temp & (~(FORCE_DEEP_SLEEP));
	WCN_INFO("%s PD_GNSS_SS_AON_CFG4:0x%x\n", __func__, temp);
	ret = sprdwcn_bus_reg_write(PD_GNSS_SS_AON_CFG4, &temp, 4);
	if (ret < 0) {
		WCN_ERR("write PD_GNSS_SS_AON_CFG4 err:%d\n", ret);
		return ret;
	}

	/* wait gnss sys power on finish */
	do {
		usleep_range(3000, 6000);

		ret = sprdwcn_bus_reg_read(CHIP_SLP_REG, &temp, 4);
		if (ret < 0) {
			WCN_ERR("%s read CHIP_SLP_REG err:%d\n", __func__, ret);
			return ret;
		}

		WCN_INFO("%s CHIP_SLP:0x%x,bit12,13 need 1\n", __func__, temp);
		retry_cnt++;
	} while ((!(temp & GNSS_SS_PWRON_FINISH)) &&
		 (!(temp & GNSS_PWR_FINISH)) && (retry_cnt < 3));

	return 0;
}

/*
 * CGM_GNSS_FAKE_CFG : 0x0: for 26M clock; 0x2: for 266M clock
 * gnss should select 26M clock before powerdomain close
 *
 * PD_GNSS_SS_AON_CFG4: 0x4041308->0x4041300 bit3=0 power on
 */
static int gnss_powerdomain_close(void)
{
	/* add by this. */
	int ret;
	int i = 0;
	unsigned int temp = 0;

	WCN_INFO("%s\n", __func__);

	ret = sprdwcn_bus_reg_read(CGM_GNSS_FAKE_CFG, &temp, 4);
	if (ret < 0) {
		WCN_ERR("%s read CGM_GNSS_FAKE_CFG error:%d\n", __func__, ret);
		return ret;
	}
	WCN_INFO("%s R_CGM_GNSS_FAKE_CFG:0x%x\n", __func__, temp);
	temp = temp & (~(CGM_GNSS_FAKE_SEL));
	ret = sprdwcn_bus_reg_write(CGM_GNSS_FAKE_CFG, &temp, 4);
	if (ret < 0) {
		WCN_ERR("write CGM_GNSS_FAKE_CFG err:%d\n", ret);
		return ret;
	}
retry:
	ret = sprdwcn_bus_reg_read(CGM_GNSS_FAKE_CFG, &temp, 4);
	if (ret < 0) {
		WCN_ERR("%s read CGM_GNSS_FAKE_CFG error:%d\n", __func__, ret);
		return ret;
	}
	i++;
	if ((temp & 0x3) && (i < 3)) {
		WCN_ERR("FAKE_CFG:0x%x, GNSS select clk err\n", temp);
		goto retry;
	}

	ret = sprdwcn_bus_reg_read(PD_GNSS_SS_AON_CFG4, &temp, 4);
	if (ret < 0) {
		WCN_ERR("read PD_GNSS_SS_AON_CFG4 err:%d\n", ret);
		return ret;
	}
	WCN_INFO("%s PD_GNSS_SS_AON_CFG4:0x%x\n", __func__, temp);
	temp = (temp | FORCE_DEEP_SLEEP | PD_AUTO_EN) &
		(~(CHIP_DEEP_SLP_EN));
	WCN_INFO("%s PD_GNSS_SS_AON_CFG4:0x%x\n", __func__, temp);
	ret = sprdwcn_bus_reg_write(PD_GNSS_SS_AON_CFG4, &temp, 4);
	if (ret < 0) {
		WCN_ERR("write PD_GNSS_SS_AON_CFG4 err:%d\n", ret);
		return ret;
	}

	return 0;
}
#endif

int open_power_ctl(void)
{
	marlin_dev->keep_power_on = false;
	clear_bit(WCN_AUTO, &marlin_dev->power_state);

	return 0;
}
EXPORT_SYMBOL_GPL(open_power_ctl);

void marlin_schedule_download_wq(void)
{
	unsigned long timeleft;

	reinit_completion(&marlin_dev->download_done);

	marlin_dev->wifi_need_download_ini_flag = 0;
	schedule_work(&marlin_dev->download_wq);
	timeleft = wait_for_completion_timeout(
		&marlin_dev->download_done,
		msecs_to_jiffies(POWERUP_WAIT_MS));
	if (!timeleft) {
		WCN_ERR("marlin download timeout\n");
	}

}

static int marlin_set_power(enum wcn_sub_sys subsys, int val)
{
	unsigned long timeleft;

	mutex_lock(&marlin_dev->power_lock);

	if (marlin_dev->wait_ge2) {
		if (first_call_flag == 1) {
			WCN_INFO("(marlin2+ge2)waiting ge2 download finish\n");
			timeleft
				= wait_for_completion_timeout(
				&ge2_completion, 12*HZ);
			if (!timeleft)
				WCN_ERR("wait ge2 timeout\n");
			first_call_flag = 2;
		}
	}

	WCN_INFO("marlin power state:0x%lx, subsys: [%s] power %d\n",
		marlin_dev->power_state, strno(subsys), val);
	init_completion(&marlin_dev->download_done);
	init_completion(&marlin_dev->gnss_download_done);

	/*  power on */
	if (val) {
		/*
		 * 1. when the first open:
		 * `- first download gnss, and then download btwifi
		 */
		marlin_dev->first_power_on_flag++;
		if (unlikely(!marlin_dev->first_power_on_ready)) {
			WCN_INFO("the first power on start\n");

			if (chip_power_on(subsys) < 0) {
				mutex_unlock(&marlin_dev->power_lock);
				return -1;
			}

			set_bit(subsys, &marlin_dev->power_state);

#ifdef CONFIG_WCN_GNSS
			WCN_INFO("GNSS start to auto download\n");
			schedule_work(&marlin_dev->gnss_dl_wq);
			timeleft
				= wait_for_completion_timeout(
				&marlin_dev->gnss_download_done, 10 * HZ);
			if (!timeleft) {
				WCN_ERR("GNSS download timeout\n");
				goto out;
			}
			WCN_INFO("gnss auto download finished and run ok\n");

			if (subsys & MARLIN_MASK)
				gnss_powerdomain_close();
#endif
			marlin_dev->first_power_on_ready = 1;

			WCN_INFO("then marlin start to download\n");
#if defined(CONFIG_UNISOC_BOARD) && defined(CONFIG_WCN_USB)
			reinit_completion(&wcn_usb_rst_fdl_done);
			marlin_set_usb_reset_status(1);
			marlin_reset(true);
			if (wait_for_completion_timeout(&wcn_usb_rst_fdl_done,
							msecs_to_jiffies(3000)) == 0) {
				WCN_ERR("reset download fdl timeout\n");
				goto out;
			}
			marlin_reset(true);
#else
			schedule_work(&marlin_dev->download_wq);
#endif
			timeleft = wait_for_completion_timeout(
				&marlin_dev->download_done,
				msecs_to_jiffies(POWERUP_WAIT_MS));
			if (!timeleft) {
				WCN_ERR("marlin download timeout\n");
				goto out;
			}
			atomic_set(&marlin_dev->download_finish_flag, 1);
			WCN_INFO("then marlin download finished and run ok\n");
#ifndef CONFIG_UMW2653
			set_wifipa_status(subsys, val);
#endif
			mutex_unlock(&marlin_dev->power_lock);

			power_state_notify_or_not(subsys, val);
#ifndef CONFIG_UMW2653
			if (subsys == WCN_AUTO) {
				marlin_set_power(WCN_AUTO, false);
				return 0;
			}
#endif
			/*
			 * If first power on is GNSS, must power off it
			 * after cali finish, and then re-power on it.
			 * This is gnss requirement.
			 */
			if (subsys == MARLIN_GNSS) {
				marlin_set_power(MARLIN_GNSS, false);
				marlin_set_power(MARLIN_GNSS, true);
				return 0;
			}

			return 0;
		}
		/* 2. the second time, WCN_AUTO coming */
		else if (subsys == WCN_AUTO) {
			if (marlin_dev->keep_power_on) {
				WCN_INFO("have power on, no action\n");
				set_wifipa_status(subsys, val);
				set_bit(subsys, &marlin_dev->power_state);
			} else {
				WCN_INFO("!1st,not to bkup gnss cal, no act\n");
			}
		}

		/*
		 * 3. when GNSS open,
		 *	  |- GNSS and MARLIN have power on and ready
		 */
		else if ((((marlin_dev->power_state) & AUTO_RUN_MASK) != 0)
			|| (((marlin_dev->power_state) & GNSS_MASK) != 0)) {
			WCN_INFO("GNSS and marlin have ready\n");
			if (((marlin_dev->power_state) & MARLIN_MASK) == 0)
				loopcheck_ready_set();
			set_wifipa_status(subsys, val);
			set_bit(subsys, &marlin_dev->power_state);

			goto check_power_state_notify;
		}
		/* 4. when GNSS close, marlin open.
		 *	  ->  subsys=gps,GNSS download
		 */
		else if (((marlin_dev->power_state) & MARLIN_MASK) != 0) {
			if ((subsys == MARLIN_GNSS) || (subsys == WCN_AUTO)) {
				WCN_INFO("BTWF ready, GPS start to download\n");
				set_wifipa_status(subsys, val);
				set_bit(subsys, &marlin_dev->power_state);
#ifdef CONFIG_WCN_GNSS
				gnss_powerdomain_open();

				schedule_work(&marlin_dev->gnss_dl_wq);
				timeleft = wait_for_completion_timeout(
					&marlin_dev->gnss_download_done, 10*HZ);
				if (!timeleft) {
					WCN_ERR("GNSS download timeout\n");
					goto out;
				}
#endif
				WCN_INFO("GNSS download finished and ok\n");

			} else {
				WCN_INFO("marlin have open, GNSS is closed\n");
				set_wifipa_status(subsys, val);
				set_bit(subsys, &marlin_dev->power_state);

				goto check_power_state_notify;
			}
		}
		/* 5. when GNSS close, marlin close.no module to power on */
		else {
			WCN_INFO("no module to power on, start to power on\n");
			if (chip_power_on(subsys) < 0) {
				mutex_unlock(&marlin_dev->power_lock);
				return -1;
			}
			set_bit(subsys, &marlin_dev->power_state);

			/* 5.1 first download marlin, and then download gnss */
			if ((subsys == WCN_AUTO || subsys == MARLIN_GNSS)) {
				WCN_INFO("marlin start to download\n");
				schedule_work(&marlin_dev->download_wq);
				timeleft = wait_for_completion_timeout(
					&marlin_dev->download_done,
					msecs_to_jiffies(POWERUP_WAIT_MS));
				if (!timeleft) {
					WCN_ERR("marlin download timeout\n");
					goto out;
				}
				atomic_set(&marlin_dev->download_finish_flag,
					   1);
				WCN_INFO("marlin dl finished and run ok\n");
#ifdef CONFIG_WCN_GNSS
				WCN_INFO("GNSS start to download\n");
				schedule_work(&marlin_dev->gnss_dl_wq);
				timeleft = wait_for_completion_timeout(
					&marlin_dev->gnss_download_done, 10*HZ);
				if (!timeleft) {
					WCN_ERR("then GNSS download timeout\n");
					goto out;
				}
				WCN_INFO("then gnss dl finished and ok\n");
#endif
			}
			/*
			 * 5.2 only download marlin, and then
			 * close gnss power domain
			 */
			else {
				WCN_INFO("only marlin start to download\n");
				schedule_work(&marlin_dev->download_wq);
				if (wait_for_completion_timeout(
					&marlin_dev->download_done,
					msecs_to_jiffies(POWERUP_WAIT_MS))
					<= 0) {

					WCN_ERR("marlin download timeout\n");
					goto out;
				}
				atomic_set(&marlin_dev->download_finish_flag,
					   1);
				WCN_INFO("BTWF download finished and run ok\n");
#ifdef CONFIG_WCN_GNSS
				gnss_powerdomain_close();
#endif
			}
			set_wifipa_status(subsys, val);
		}
		/* power on together's Action */
		power_state_notify_or_not(subsys, val);

		WCN_INFO("wcn chip power on and run finish: [%s]\n",
				  strno(subsys));
	/* power off */
	} else {
		if (marlin_dev->power_state == 0) {
			if (flag_reset)
				flag_reset = 0;
			goto check_power_state_notify;
		}

		if (marlin_dev->keep_power_on) {
			if (!flag_reset) {
				if (subsys != WCN_AUTO) {
					/* in order to not download again */
					set_bit(WCN_AUTO,
						&marlin_dev->power_state);
					clear_bit(subsys,
						&marlin_dev->power_state);
				}
				WCN_DEBUG("marlin reset flag_reset:%d\n",
					flag_reset);
				goto check_power_state_notify;
			}
		}

		set_wifipa_status(subsys, val);
		clear_bit(subsys, &marlin_dev->power_state);
		if ((marlin_dev->power_state != 0) && (!flag_reset)) {
			WCN_INFO("can not power off, other module is on\n");
#ifdef CONFIG_WCN_GNSS
			if (subsys == MARLIN_GNSS)
				gnss_powerdomain_close();
#endif
			goto check_power_state_notify;
		}

		set_cp_mem_status(subsys, val);
// fm marco,add from OTT
#ifdef CONFIG_WCN_FM
		set_fm_supe_freq(subsys, val, marlin_dev->power_state);
#endif
		power_state_notify_or_not(subsys, val);

		WCN_INFO("wcn chip start power off!\n");
		sprdwcn_bus_runtime_put();
		chip_power_off(subsys);
		WCN_INFO("marlin power off!\n");
		atomic_set(&marlin_dev->download_finish_flag, 0);
		if (flag_reset) {
			flag_reset = FALSE;
			marlin_dev->power_state = 0;
		}
	} /* power off end */

	/* power on off together's Action */
	mutex_unlock(&marlin_dev->power_lock);

	return 0;

out:
	sprdwcn_bus_runtime_put();
#ifdef CONFIG_MEM_PD
	mem_pd_poweroff_deinit();
#endif
	wifipa_enable(0);
	marlin_avdd18_dcxo_enable(false);
	marlin_clk_enable(false);
	marlin_chip_en(false, false);
	marlin_digital_power_enable(false);
	marlin_analog_power_enable(false);
	chip_reset_release(0);
	marlin_dev->power_state = 0;
	atomic_set(&marlin_dev->download_finish_flag, 0);
	mutex_unlock(&marlin_dev->power_lock);

	return -1;

check_power_state_notify:
	power_state_notify_or_not(subsys, val);
	WCN_DEBUG("mutex_unlock\n");
	mutex_unlock(&marlin_dev->power_lock);

	return 0;

}

void marlin_power_off(enum wcn_sub_sys subsys)
{
	WCN_INFO("%s all\n", __func__);

	marlin_dev->keep_power_on = false;
	set_bit(subsys, &marlin_dev->power_state);
	marlin_set_power(subsys, false);
}

int marlin_get_power(void)
{
	return marlin_dev->power_state;
}
EXPORT_SYMBOL_GPL(marlin_get_power);

int marlin_get_set_power_status(void)
{
	return marlin_dev->first_power_on_flag;
}
EXPORT_SYMBOL_GPL(marlin_get_set_power_status);


bool marlin_get_download_status(void)
{
	return atomic_read(&marlin_dev->download_finish_flag);
}
EXPORT_SYMBOL_GPL(marlin_get_download_status);

void marlin_set_download_status(int f)
{
	atomic_set(&marlin_dev->download_finish_flag, f);
}
EXPORT_SYMBOL_GPL(marlin_set_download_status);

int wcn_get_module_status_changed(void)
{
	return marlin_dev->loopcheck_status_change;
}
EXPORT_SYMBOL_GPL(wcn_get_module_status_changed);

void wcn_set_module_status_changed(bool status)
{
	marlin_dev->loopcheck_status_change = status;
}
EXPORT_SYMBOL_GPL(wcn_set_module_status_changed);

int marlin_get_module_status(void)
{
	if (test_bit(MARLIN_BLUETOOTH, &marlin_dev->power_state) ||
	    test_bit(MARLIN_FM, &marlin_dev->power_state) ||
	    test_bit(MARLIN_WIFI, &marlin_dev->power_state) ||
	    test_bit(MARLIN_MDBG, &marlin_dev->power_state))
		/*
		 * Can't send mdbg cmd before download flag ok
		 * If download flag not ready,loopcheck get poweroff
		 */
		return atomic_read(&marlin_dev->download_finish_flag);
#ifdef CONFIG_UMW2653
	if (marlin_dev->keep_power_on == true &&
		test_bit(WCN_AUTO, &marlin_dev->power_state))
		return 1;
#endif
		return 0;
}
EXPORT_SYMBOL_GPL(marlin_get_module_status);

int is_first_power_on(enum wcn_sub_sys subsys)
{
	if (marlin_dev->wifi_need_download_ini_flag == 1)
		return 1;	/*the first */
	else
		return 0;	/* not the first */
}
EXPORT_SYMBOL_GPL(is_first_power_on);

int cali_ini_need_download(enum wcn_sub_sys subsys)
{
#ifndef CONFIG_WCN_PCIE
	unsigned int pd_wifi_st = 0;

#ifdef CONFIG_MEM_PD
	pd_wifi_st = mem_pd_wifi_state();
#endif
	if ((marlin_dev->wifi_need_download_ini_flag == 1) || pd_wifi_st) {
		WCN_INFO("%s return 1\n", __func__);
		return 1;	/* the first */
	}
#endif
	return 0;	/* not the first */
}
EXPORT_SYMBOL_GPL(cali_ini_need_download);

int marlin_set_wakeup(enum wcn_sub_sys subsys)
{
	int ret = 0;	/* temp */

	return 0;
	if (!atomic_read(&marlin_dev->download_finish_flag))
		return -1;

	return ret;
}
EXPORT_SYMBOL_GPL(marlin_set_wakeup);

int marlin_set_sleep(enum wcn_sub_sys subsys, bool enable)
{
	return 0;	/* temp */

	if (!atomic_read(&marlin_dev->download_finish_flag))
		return -1;

	return 0;
}
EXPORT_SYMBOL_GPL(marlin_set_sleep);

int marlin_reset_reg(void)
{
	init_completion(&marlin_dev->carddetect_done);
	marlin_reset(true);
	mdelay(1);
	sprdwcn_bus_rescan(marlin_dev);
	if (wait_for_completion_timeout(&marlin_dev->carddetect_done,
		msecs_to_jiffies(CARD_DETECT_WAIT_MS))) {
		return 0;
	}
	WCN_ERR("marlin reset reg wait scan error!\n");

	return -1;
}
EXPORT_SYMBOL_GPL(marlin_reset_reg);

int start_marlin(enum wcn_sub_sys subsys)
{

#ifdef CONFIG_WCN_USB
	do {
		mdelay(20);
	} while (marlin_get_usb_hotplug_status());
#endif
	WCN_INFO("%s [%s]\n", __func__, strno(subsys));
	if (sprdwcn_bus_get_carddump_status() != 0) {
		WCN_ERR("%s SDIO card dump\n", __func__);
		return -1;
	}

	if (get_loopcheck_status()) {
		WCN_ERR("%s loopcheck status is fail\n", __func__);
		return -1;
	}

	if (subsys == MARLIN_WIFI) {
		/* not need write cali */
		if (marlin_dev->wifi_need_download_ini_flag == 0)
			/* need write cali */
			marlin_dev->wifi_need_download_ini_flag = 1;
		else
			/* not need write cali */
			marlin_dev->wifi_need_download_ini_flag = 2;
	}
	if (marlin_set_power(subsys, true) < 0)
		return -1;
#ifdef CONFIG_MEM_PD
	return mem_pd_mgr(subsys, true);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(start_marlin);

int stop_marlin(enum wcn_sub_sys subsys)
{
#ifdef CONFIG_WCN_USB
	do {
		mdelay(20);
	} while (marlin_get_usb_hotplug_status());
#endif
	WCN_INFO("%s [%s]\n", __func__, strno(subsys));
	mutex_lock(&marlin_dev->power_lock);
	if (!marlin_get_power()) {
		mutex_unlock(&marlin_dev->power_lock);
		WCN_INFO("%s no module opend\n", __func__);
		return 0;
	}
	mutex_unlock(&marlin_dev->power_lock);

	if (sprdwcn_bus_get_carddump_status() != 0) {
		WCN_ERR("%s SDIO card dump\n", __func__);
		return -1;
	}

	if (get_loopcheck_status()) {
		WCN_ERR("%s loopcheck status is fail\n", __func__);
		return -1;
	}
#ifdef CONFIG_MEM_PD
	mem_pd_mgr(subsys, false);
#endif

	return marlin_set_power(subsys, false);
}
EXPORT_SYMBOL_GPL(stop_marlin);



static void marlin_power_wq(struct work_struct *work)
{
	WCN_INFO("%s start\n", __func__);

	/* WCN_AUTO is for auto backup gnss cali data */
	marlin_set_power(WCN_AUTO, true);

}

#ifdef CONFIG_WCN_USB
static void marlin_usb_hotplug(struct work_struct *work)
{
	if (marlin_get_usb_reset_status()) {
		btwifi_download_fdl_firmware();
		marlin_set_usb_reset_status(0);
		complete(&wcn_usb_rst_fdl_done);
		WCN_INFO("%s reset usb_fdl_download finish\n", __func__);
	} else {
		marlin_set_download_status(0);
		marlin_schedule_download_wq();
		marlin_set_download_status(1);

		marlin_reset_notify_call(MARLIN_CP2_STS_READY);
		marlin_set_usb_hotplug_status(0);
		WCN_INFO("%s hotplug download firmware finish\n", __func__);
	}
}

int marlin_get_usb_hotplug_status(void)
{
	return marlin_dev->usb_hotplug_status;
}

void marlin_set_usb_reset_status(int status)
{
	marlin_dev->usb_reset_status = status;
}
EXPORT_SYMBOL_GPL(marlin_set_usb_reset_status);

int marlin_get_usb_reset_status(void)
{
	return marlin_dev->usb_reset_status;
}
EXPORT_SYMBOL_GPL(marlin_get_usb_reset_status);

void marlin_set_usb_hotplug_status(int status)
{
	marlin_dev->usb_hotplug_status = status;
}
void marlin_schedule_usb_hotplug(void)
{
	queue_work(system_highpri_wq, &marlin_dev->usb_hotplug);
}

int marlin_probe_status(void)
{
	return marlin_dev->marlin_probe_status;
}

static void marlin_reset_notify_init(void);

#endif

static int marlin_probe(struct platform_device *pdev)
{
	int err;
	struct sprdwcn_bus_ops *bus_ops;

	WCN_INFO("%s: unisoc wcn driver build time: %s %s (UTC)", __func__, __DATE__, __TIME__);

	marlin_dev = devm_kzalloc(&pdev->dev, sizeof(struct marlin_device),
				  GFP_KERNEL);
	if (!marlin_dev)
		return -ENOMEM;

	marlin_dev->write_buffer = devm_kzalloc(&pdev->dev,
						PACKET_SIZE, GFP_KERNEL);
	if (marlin_dev->write_buffer == NULL) {
		devm_kfree(&pdev->dev, marlin_dev);
		WCN_ERR("%s write buffer low memory\n", __func__);
		return -ENOMEM;
	}
#ifdef CONFIG_WCN_PARSE_DTS
	marlin_dev->np = pdev->dev.of_node;
	WCN_INFO("%s: device node name: %s\n", __func__, marlin_dev->np->name);
#endif
	mutex_init(&(marlin_dev->power_lock));
	marlin_dev->power_state = 0;
#ifdef CONFIG_WCN_PARSE_DTS
	err = marlin_parse_dt(pdev);
	if (err < 0) {
		WCN_INFO("marlin parse_dt some para not config\n");
		if (err == -EPROBE_DEFER) {
			devm_kfree(&pdev->dev, marlin_dev);
			WCN_ERR("%s: get some resources fail, defer probe it\n",
			       __func__);
			return err;
		}
	}
#else  /* CONFIG_WCN_PARSE_DTS */
	marlin_config_wcn_resource();
#endif  /* CONFIG_WCN_PARSE_DTS */
// #ifndef CONFIG_WCN_USB
// 	if (gpio_is_valid(marlin_dev->reset))
// 		gpio_direction_output(marlin_dev->reset, 0);
// #endif
	init_completion(&ge2_completion);
	init_completion(&marlin_dev->carddetect_done);
#ifdef CONFIG_WCN_USB
	init_completion(&wcn_usb_rst_fdl_done);
#endif

#ifdef CONFIG_WCN_SLP
	slp_mgr_init();
#endif
	/* register ops */
	wcn_bus_init();
	bus_ops = get_wcn_bus_ops();
	bus_ops->start_wcn = start_marlin;
	bus_ops->stop_wcn = stop_marlin;

	/* sdiom_init or pcie_init */
	err = sprdwcn_bus_preinit();
	if (err) {
		WCN_ERR("sprdwcn_bus_preinit error: %d\n", err);
		goto error3;
	}

	sprdwcn_bus_register_rescan_cb(marlin_scan_finish);
#ifdef CONFIG_WCN_SLP
	err = sdio_pub_int_init(marlin_dev->int_ap);
	if (err) {
		WCN_ERR("sdio_pub_int_init error: %d\n", err);
		sprdwcn_bus_deinit(&marlin_dev);
		wcn_bus_deinit();
		slp_mgr_deinit();
		devm_kfree(&pdev->dev, marlin_dev);
		return err;
	}
#endif
#ifdef CONFIG_MEM_PD
	mem_pd_init();
#endif

	err = proc_fs_init();
	if (err) {
		WCN_ERR("proc_fs_init error: %d\n", err);
		goto error2;
	}

	err = log_dev_init();
	if (err) {
		WCN_ERR("log_dev_init error: %d\n", err);
		goto error1;
	}

	err = wcn_op_init();
	if (err) {
		WCN_ERR("wcn_op_init: %d\n", err);
		goto error0;
	}

	flag_reset = 0;
	loopcheck_init();
#ifdef CONFIG_WCN_USB
	/*This func used in 19b driver to notify subsys do reset, when cp2 was dead
	In 20a driver, only usb driver use this.
	TODO: change usb driver adapt 20a reset notify function.
	*/
	marlin_reset_notify_init();
#endif
	reset_test_init();

	INIT_WORK(&marlin_dev->download_wq, pre_btwifi_download_sdio);
	INIT_WORK(&marlin_dev->gnss_dl_wq, pre_gnss_download_firmware);
	INIT_DELAYED_WORK(&marlin_dev->power_wq, marlin_power_wq);
#ifdef CONFIG_WCN_USB
	INIT_WORK(&marlin_dev->usb_hotplug, marlin_usb_hotplug);
#endif
/* PCIe not need power_wq*/
#if 0
	schedule_delayed_work(&marlin_dev->power_wq,
			  msecs_to_jiffies(1500));
#endif

#ifdef CONFIG_WCN_USB
	marlin_dev->marlin_probe_status = 1;
#endif

	WCN_INFO("%s driver match successful!\n", __func__);

	return 0;
error0:
	log_dev_exit();
error1:
	proc_fs_exit();
error2:
#ifdef CONFIG_MEM_PD
	mem_pd_exit();
#endif
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_deinit();
#endif
	sprdwcn_bus_deinit(&marlin_dev);
error3:
	wcn_bus_deinit();
#ifdef CONFIG_WCN_SLP
	slp_mgr_deinit();
#endif
	devm_kfree(&pdev->dev, marlin_dev);
	return err;
}

static int  marlin_remove(struct platform_device *pdev)
{
	cancel_work_sync(&marlin_dev->download_wq);
	cancel_work_sync(&marlin_dev->gnss_dl_wq);
	cancel_delayed_work_sync(&marlin_dev->power_wq);
#ifdef CONFIG_WCN_USB
	cancel_work_sync(&marlin_dev->usb_hotplug);
#endif
	loopcheck_deinit();
	wcn_op_exit();
	log_dev_exit();
	proc_fs_exit();
#ifdef CONFIG_WCN_SLP
	sdio_pub_int_deinit();
#endif
#ifdef CONFIG_MEM_PD
	mem_pd_exit();
#endif
	sprdwcn_bus_deinit(&marlin_dev);
	if (marlin_dev->power_state != 0) {
		WCN_INFO("marlin some subsys power is on, warning!\n");
		wcn_wifipa_bound_xtl(false);
		wifipa_enable(0);
		marlin_chip_en(false, false);
	}
	wcn_bus_deinit();
#ifdef CONFIG_WCN_SLP
	slp_mgr_deinit();
#endif
	marlin_gpio_free(pdev);
	mutex_destroy(&marlin_dev->power_lock);
	devm_kfree(&pdev->dev, marlin_dev->write_buffer);
	devm_kfree(&pdev->dev, marlin_dev);

#ifdef CONFIG_WCN_USB
	marlin_dev->marlin_probe_status = 0;
#endif
	WCN_INFO("remove ok!\n");

	return 0;
}

static void marlin_shutdown(struct platform_device *pdev)
{
	WCN_INFO("%s start, power_state=%ld\n", __func__, marlin_dev->power_state);
	if (marlin_dev->power_state != 0) {
		pr_warn("marlin some subsys power is on, force close\n");
#ifndef CONFIG_WCN_USB
		sprdwcn_bus_set_carddump_status(true);
#endif
		marlin_dev->power_state = 0;
		stop_loopcheck();
#ifdef CONFIG_ASR_BOARD
#ifdef CONFIG_WCN_SLP
		sdio_pub_int_deinit();
#endif
#endif
		wcn_avdd12_bound_xtl(false);
		wcn_wifipa_bound_xtl(false);
		wifipa_enable(0);
		marlin_analog_power_enable(false);
		marlin_chip_en(false, false);
#ifdef CONFIG_WCN_SLP
		sdio_pub_int_poweron(false);
#endif
	}
	wcn_bus_deinit();
	WCN_INFO("%s end\n", __func__);
}

static int marlin_suspend(struct device *dev)
{

	WCN_INFO("[%s]enter\n", __func__);

	return 0;
}

int marlin_reset_register_notify(void *callback_func, void *para)
{
	marlin_reset_func = (marlin_reset_callback)callback_func;
	marlin_callback_para = para;

	return 0;
}
EXPORT_SYMBOL_GPL(marlin_reset_register_notify);

int marlin_reset_unregister_notify(void)
{
	marlin_reset_func = NULL;
	marlin_callback_para = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(marlin_reset_unregister_notify);

#ifdef CONFIG_WCN_USB
int marlin_reset_notify_call(enum marlin_cp2_status sts) {

	int i = 0;
	for(i = 0; i < MARLIN_ALL; i++) {
		if(NULL != marlin_reset_notifiers[i].head)
			raw_notifier_call_chain(&marlin_reset_notifiers[i], \
				sts, (void *)strno(i));
	}
	return 0;
}
EXPORT_SYMBOL_GPL(marlin_reset_notify_call);

static void marlin_reset_notify_init(void) {
	int i = 0;
	for(i = 0; i < MARLIN_ALL; i++)
		 RAW_INIT_NOTIFIER_HEAD(&marlin_reset_notifiers[i]);
}
#endif

static int marlin_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops marlin_pm_ops = {
	.suspend = marlin_suspend,
	.resume	= marlin_resume,
};

static const struct of_device_id marlin_match_table[] = {
	{.compatible = "sprd,marlin3",},
	{ },
};

static struct platform_driver marlin_driver = {
	.driver = {
		.name = "unisoc_bsp",
		.pm = &marlin_pm_ops,
#ifdef CONFIG_WCN_PARSE_DTS
		.of_match_table = marlin_match_table,
#endif
	},
	.probe = marlin_probe,
	.remove = marlin_remove,
	.shutdown = marlin_shutdown,
};

#ifndef CONFIG_WCN_PARSE_DTS
static void marlin_bsp_release(struct device *dev)
{
	WCN_INFO("[%s]enter\n", __func__);
}

static struct platform_device marlin_bsp_device = {
	.name = "unisoc_bsp",
	.dev = {
		.release = marlin_bsp_release,
	}
};
#endif

static int __init marlin_init(void)
{
	int ret;
	WCN_INFO("%s entry!\n", __func__);
#ifndef CONFIG_WCN_PARSE_DTS
	platform_device_register(&marlin_bsp_device);
#endif
#ifdef COMFIG_FSTAB_AB
	fstab_ab = strstr(saved_command_line, SUFFIX);
	if (fstab_ab)
		WCN_INFO("fstab: %s\n", fstab_ab);
#endif
	ret =  platform_driver_register(&marlin_driver);
#ifdef CONFIG_WCN_GNSS
	gnss_common_ctl_init();
	gnss_pmnotify_ctl_init();
#endif
	init_wcn_sysfs();

	return ret;
}

#ifdef CONFIG_WCN_PCIE
device_initcall(marlin_init);
#else
late_initcall(marlin_init);
#endif

static void __exit marlin_exit(void)
{
	WCN_INFO("marlin_exit entry!\n");
#ifndef CONFIG_WCN_PARSE_DTS
	platform_device_unregister(&marlin_bsp_device);
#endif
	exit_wcn_sysfs();
#ifdef CONFIG_WCN_GNSS
	gnss_common_ctl_exit();
	gnss_pmnotify_ctl_cleanup();
#endif
	platform_driver_unregister(&marlin_driver);
}
module_exit(marlin_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum  WCN Marlin Driver");
MODULE_AUTHOR("Yufeng Yang <yufeng.yang@spreadtrum.com>");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif