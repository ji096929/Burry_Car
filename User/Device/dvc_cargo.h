#ifndef _DVC_CARGO_H
#define _DVC_CARGO_H

#include "main.h"
#include <string.h>

struct Struct_Cargo
{
    uint8_t Position_X;
    uint8_t Position_Y;
    uint8_t Phone_Number[11];
    uint8_t Code[4];
    Struct_Cargo* Next_Cargo;
} __attribute__((packed));


class Class_Cargo_List
{
public:

    Struct_Cargo* First_Cargo;

    void Init();
    void Add_Cargo(uint8_t Position_X, uint8_t Position_Y, uint8_t* Phone_Number, uint8_t* Code);
    void Delete_Cargo(uint8_t Code[4]);
    Struct_Cargo* Exist_Cargo(uint8_t Code[4]);
};

#endif
