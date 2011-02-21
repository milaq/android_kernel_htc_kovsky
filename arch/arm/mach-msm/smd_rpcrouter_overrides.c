/* linux/arch/arm/mach-msm/smd_rpcrouter_overrides
*
* Implements functions to lookup amss-specific rpc servers and versions
*
* Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <mach/smd.h>

static struct smd_overrides *pdata = NULL;

bool amss_get_num_value(enum amss_id id, uint32_t * out)
{
	int i;
	if (!pdata) {
		//we should not ever get here.
		//but leave it here until it is tested
		printk(KERN_ERR "%s: adsp_info is NULL\n", __func__);
		return false;
	}
	
	for (i = 0; i < pdata->n_amss_values; i++) {
		if (pdata->amss_values[i].id == id
		    && pdata->amss_values[i].type == AMSS_VAL_UINT) {
			*out = pdata->amss_values[i].value;
			return true;
		}
	}
	return false;
}

const char *amss_get_str_value(enum amss_id id)
{
	int i;
	if (!pdata) {
		//we should not ever get here.
		//but leave it here until it is tested
		printk(KERN_ERR "%s: adsp_info is NULL\n", __func__);
		return NULL;
	}
	for (i = 0; i < pdata->n_amss_values; i++) {
		if (pdata->amss_values[i].id == id
		    && pdata->amss_values[i].type == AMSS_VAL_STRING)
			return pdata->amss_values[i].string;
	}
	return NULL;
}

extern void amss_set_overrides(struct smd_overrides* new_pdata) {
	pdata = new_pdata;
}