#ifndef _BLUETOOTH_H
#define _BLUETOOTH_H

#include "stdint.h"

#define FRAME_HEADER  0x5A
#define FRAME_TAIL    0x11
#define FRAME_LENGTH  14

typedef struct {
	uint8_t header;
	uint8_t data[12];
	uint8_t tail;
}struct_frame;

#endif

