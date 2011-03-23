/*
 *  arch/arm/include/asm/mach/mmc.h
 */
#ifndef ASMARM_MACH_MMC_H
#define ASMARM_MACH_MMC_H

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>

typedef void (*msm_mmc_callback)(int card_present, void *dev_id);

struct embedded_sdio_data {
	struct sdio_cis cis;
	struct sdio_cccr cccr;
	struct sdio_embedded_func *funcs;
	int num_funcs;
};

struct mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	int built_in;				/* built-in device flag */
	u32 (*translate_vdd)(struct device *, unsigned int);
	unsigned int (*status)(struct device *);
	int sdcc_id;
	struct embedded_sdio_data *embedded_sdio;
	int (*register_status_notify)(msm_mmc_callback, void *dev_id);
};

#endif
