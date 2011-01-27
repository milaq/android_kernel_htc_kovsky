#ifndef _MICROP_KEYPAD_H
#define _MICROP_KEYPAD_H

#include <linux/platform_device.h>

enum microp_key_state {
	KEY_RELEASED,
};

struct microp_keypad_platform_data {
	int irq_keypress;
	int irq_clamshell;
	int (*get_scancode)(int *out_code, enum microp_key_state flags);
	int (*is_keypad_disabled)(void);
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
	int *keypad_scancodes;
	int keypad_scancodes_size;	
};

#endif
