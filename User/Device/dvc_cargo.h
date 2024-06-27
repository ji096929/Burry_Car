#ifndef _DVC_CARGO_H
#define _DVC_CARGO_H

#include "main.h"


struct Struct_Cargo
{
    uint8_t Position_X;
    uint8_t Position_Y;
    uint8_t Phone_Number[11];
    uint8_t Code[4];
} __attribute__((packed));


#endif
