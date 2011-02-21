#ifndef __AMSS_5225_H__
#define __AMSS_5225_H__

#include <mach/smd.h>

static struct amss_value amss_5225_para[] = {
//	{AMSS_AUDMGR_PROG_VERS, AMSS_VAL_STRING, { .string = "rc30000013:00000000"}},
	{AMSS_AUDMGR_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_AUDMGR_PROG_VERS, AMSS_VAL_UINT, {.value = 0x30000013}},
	{AMSS_AUDMGR_CB_PROG_VERS, AMSS_VAL_UINT, {.value = 0x5fa922a9}},
	{AMSS_SND_SET_DEVICE_PROC, AMSS_VAL_UINT, { .value = 1}},
	{AMSS_SND_SET_VOLUME_PROC, AMSS_VAL_UINT, { .value = 2}},
	{AMSS_RPC_SND_VERS, AMSS_VAL_UINT, { .value = 0x0}},
//	{AMSS_AUDMGR_CB_PROG_VERS, AMSS_VAL_STRING, { .string = "rs31000013:5fa922a9"}},
	{AMSS_AUDMGR_CB_VERS, AMSS_VAL_UINT, { .value = 0x5fa922a9}},
	{AMSS_RPC_ADSP_RTOS_ATOM_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_RPC_ADSP_RTOS_MTOA_VERS, AMSS_VAL_UINT, { .value = 0x0}},
};

#endif