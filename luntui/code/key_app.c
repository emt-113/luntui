#include "key_app.h"
#include "auto_menu.h"          // 引入菜单系统

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
        keu_val =4;
    }
    return keu_val;
}
uint8_t g_menu_enabled = 1;
// 菜单系统全局开关 (可根据需要开启/关闭菜单功能)
extern uint8_t g_menu_enabled;  // 在 main_cm7_0.c 中定义并初始化

void key_task()
{
	static uint32_t key_num=0;

	static uint8_t key_val = 0, key_old = 0,key_up,key_down,num=0;
	key_val = key_read();
	key_down= key_val & (key_val^key_old);
	key_up = ~key_val & (key_val^key_old);
	key_old = key_val;

        // 菜单系统集成：检测到按键按下时直接调用 Menu_Key_Handler
        if (g_menu_enabled)
        {
            if (key_down != 0)
            {
                Menu_Key_Handler(key_down);  // 直接传递按键值到菜单系统
            }
        }


}

