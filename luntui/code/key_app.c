#include "key_app.h"

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)


void my_key_init  ()
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入 
}


uint8_t key_read()
{
    uint8_t keu_val=0;
    
    if (gpio_get_level(KEY1)==0)
    {
        keu_val =1;
    }
    if (gpio_get_level(KEY2)==0)
    {
        keu_val =2;
    }
    if (gpio_get_level(KEY3)==0)
    {
        keu_val =3;
    }
    if (gpio_get_level(KEY4)==0)
    {
        keu_val =4;//清空
    }
    return keu_val;
}
extern uint8_t start_flag;

void key_task()
{
	static uint32_t key_num=0;

	static uint8_t key_val = 0, key_old = 0,key_up,key_down,num=0;
	key_val = key_read();
	key_down= key_val & (key_val^key_old);
	key_up = ~key_val & (key_val^key_old);
	key_old = key_val;
        if (key_down==4)
        {
          start_flag=!start_flag;
          g_System_State=start_flag;
        }
        if (key_down==1)
        {
          g_Debug_Target_Speed+=5;
        }
        if (key_down==2)
        {
          g_Debug_Target_Speed-=5;
        }
}

