#ifndef _DS2746_BATTERY_H
#define _DS2746_BATTERY_H

struct ds2746_platform_data {
	unsigned short resistance;
	unsigned short capacity;
	unsigned short high_voltage;
	unsigned short low_voltage;
	void (*set_charge)(int flags);
};

#endif
