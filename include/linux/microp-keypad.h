#ifndef _MICROP_KEYPAD_H
#define _MICROP_KEYPAD_H

#include <linux/platform_device.h>

#define MICROP_KSC_RELEASED_BIT 0x80
#define MICROP_KSC_SCANCODE_MASK (MICROP_KSC_RELEASED_BIT - 1)

#define MICROP_KSC_ID_SCANCODE	0x10
#define MICROP_KSC_ID_MODIFIER	0x11

enum microp_key_state {
	KEY_RELEASED,
};

struct microp_keypad_platform_data {
	int irq_keypress;
	int irq_clamshell;
	bool read_modifiers;
	int (*is_keypad_disabled)(void);
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
	int *keypad_scancodes;
	int keypad_scancodes_size;
};

#endif
