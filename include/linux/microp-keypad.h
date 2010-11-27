#ifndef _MICROP_KEYPAD_H
#define _MICROP_KEYPAD_H


#define MICROP_KBD_MAX_KEYS 0x48

struct microp_keypad_clamshell {
        int gpio;
        int irq;
};

struct microp_keypad_platform_data {
        struct microp_keypad_clamshell clamshell;
	int backlight_gpio;
};



#endif
