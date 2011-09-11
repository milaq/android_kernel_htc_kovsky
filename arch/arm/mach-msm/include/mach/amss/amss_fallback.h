#ifndef __AMSS_FALLBACK_H__
#define __AMSS_FALLBACK_H__

#include <mach/msm_smd.h>

static struct amss_value amss_fallback_params[] = {
	{AMSS_DOG_KEEPALIVE_PROG, .value = 0x30000015},
	{AMSS_DOG_KEEPALIVE_VERS, .value = 0},
	{AMSS_DOG_KEEPALIVE_BEACON, .value = 1},
	{AMSS_TIME_REMOTE_MTOA_VERS, .value = 0},
	{AMSS_TIME_TOD_SET_APPS_BASES, .value = 2},
};

static int n_amss_fallback_params = ARRAY_SIZE(amss_fallback_params);
#endif
