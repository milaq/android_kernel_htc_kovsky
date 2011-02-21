#ifndef _MICROP_NG_H
#define _MICROP_NG_H

struct microp_platform_data {
	bool (*is_supported)(void);
	struct platform_device** clients;
	int nclients;
};

extern int microp_ng_write(uint8_t* sendbuf, int len);
extern int microp_ng_read(uint8_t id, uint8_t *buf, int len);

#endif
