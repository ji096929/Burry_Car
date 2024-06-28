#include "dvc_cargo.h"


void Class_Cargo_List::Init()
{
    First_Cargo = NULL;
}


void Class_Cargo_List::Add_Cargo(uint8_t Position_X, uint8_t Position_Y, uint8_t Phone_Number[11], uint8_t Code[4])
{
    //头插法插入新节点
    Struct_Cargo* New_Cargo = new Struct_Cargo;
    New_Cargo->Position_X = Position_X;
    New_Cargo->Position_Y = Position_Y;
    for (int i = 0; i < 11; i++)
    {
        New_Cargo->Phone_Number[i] = Phone_Number[i];
    }
    for (int i = 0; i < 4; i++)
    {
        New_Cargo->Code[i] = Code[i];
    }
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

uint8_t Class_Cargo_List::Exist_Cargo(uint8_t Code[4])
{
    Struct_Cargo* p = First_Cargo;
    while (p != NULL)
    {
        if (p->Code[0] == Code[0] && p->Code[1] == Code[1] && p->Code[2] == Code[2] && p->Code[3] == Code[3])
        {
            return 1;
        }
        p = p->Next_Cargo;
    }
    return 0;
}





