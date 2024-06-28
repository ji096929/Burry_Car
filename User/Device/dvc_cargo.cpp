#include "dvc_cargo.h"


void Class_Cargo_List::Init()
{
    First_Cargo = NULL;
}


void Class_Cargo_List::Add_Cargo(uint8_t Position_X, uint8_t Position_Y, uint8_t* Phone_Number, uint8_t* Code)
{
    //头插法插入新节点
    Struct_Cargo* New_Cargo = new Struct_Cargo;
    New_Cargo->Position_X = Position_X;
    New_Cargo->Position_Y = Position_Y;

    memcpy(New_Cargo->Phone_Number, Phone_Number, 11 );
    memcpy(New_Cargo->Code, Code, 4 * sizeof(uint8_t));

    New_Cargo->Next_Cargo = First_Cargo;
    First_Cargo = New_Cargo;
}

void Class_Cargo_List::Delete_Cargo(uint8_t Code[4])
{
    Struct_Cargo* p = First_Cargo;
    Struct_Cargo* q = First_Cargo;
    while (p != NULL)
    {
        if (p->Code[0] == Code[0] && p->Code[1] == Code[1] && p->Code[2] == Code[2] && p->Code[3] == Code[3])
        {
            q->Next_Cargo = p->Next_Cargo;
            delete p;
            break;
        }
        q = p;
        p = p->Next_Cargo;
    }

}

Struct_Cargo* Class_Cargo_List::Exist_Cargo(uint8_t Code[4])
{
    Struct_Cargo* p = First_Cargo;
    while (p != NULL)
    {
        if (p->Code[0] == Code[0] && p->Code[1] == Code[1] && p->Code[2] == Code[2] && p->Code[3] == Code[3])
        {
            return p;
        }
        p = p->Next_Cargo;
    }
    return NULL;
}





