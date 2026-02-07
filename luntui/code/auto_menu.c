/*********************************************************************************************************************
 * 文件名称          auto_menu
 * 功能描述          通用多级菜单框架实现 - 支持 int/uint/float/ReadOnly 类型
 * 平台              CYT4BB + IAR + TFT180 (128x160)
 *
 * 实现说明：
 * 1. 状态机驱动：NAV_MODE 和 EDIT_MODE 双状态
 * 2. 数据回滚：编辑模式下退出时自动恢复 backup_val
 * 3. 高效渲染：仅重绘当前行，避免全屏闪烁
 * 4. 类型安全：使用联合体和枚举确保类型正确访问
 *
 * 修改记录
 * 作者              时间                备注
 * 2025-02-06       AutoMenu           first version
********************************************************************************************************************/

#include "auto_menu.h"
#include "filt.h"  // 引入 icm_output_t 定义
#include <string.h>

//================================================= 全局变量 ==================================================
static menu_system_t g_menu = {0};


 static int32_t speed_kp = 100;
  static int32_t speed_ki = 50;
  static float    angle_kp = 121.134f;
  static uint32_t max_speed = 5000;

  // 外部变量声明（实时数据显示）
  extern icm_output_t icm_output;
  extern int8_t g_System_State;         // 系统状态变量 (int8_t类型,非int32_t!)
  extern pid_struct_t balance_gyro_pid; // 陀螺仪环PID控制器

  // 定义确认回调函数 (保存参数到 Flash 等)
  void save_pid_params(void)
  {
      // TODO: 在这里添加保存逻辑，如写入 EEPROM
      // uart_printf("PID params saved!\r\n");
  }

  // 定义菜单条目
//  static menu_item_t pid_items[] = {
//      {"Speed K", &speed_kp, MENU_TYPE_INT32, .step.i32=10, .max.i32=500, .min.i32=0, .display_num=3, .float_point_num=0, save_pid_params},
//      {"Speed K", &speed_ki, MENU_TYPE_INT32, .step.i32=5,  .max.i32=200, .min.i32=0, .display_num=3, .float_point_num=0, save_pid_params},
//      {"Angle Kp", &angle_kp, MENU_TYPE_FLOAT, .step.f=0.1f, .max.f=10.0f, .min.f=0.0f, .display_num=3, .float_point_num=1, save_pid_params},
//      {"Max Spd", &max_speed, MENU_TYPE_UINT32, .step.u32=100, .max.u32=10000, .min.u32=0, .display_num=4, .float_point_num=0, save_pid_params},
//  };

  static menu_item_t sensor_items[] = {
      {"yaw",    &icm_output.yaw,  MENU_TYPE_READONLY, MENU_TYPE_FLOAT, .step.f=0,   .max.f=0,      .min.f=0,      .display_num=3, .float_point_num=1, NULL},
      {"pitch",     &icm_output.pitch,   MENU_TYPE_READONLY, MENU_TYPE_FLOAT, .step.f=0,   .max.f=0,      .min.f=0,      .display_num=4, .float_point_num=1, NULL},
      {"roll",     &icm_output.roll,    MENU_TYPE_READONLY, MENU_TYPE_FLOAT, .step.f=0,   .max.f=0,      .min.f=0,      .display_num=3, .float_point_num=1, NULL},
      {"gyro_y",    &icm_output.gyro_y,  MENU_TYPE_READONLY, MENU_TYPE_FLOAT, .step.f=0,   .max.f=0,      .min.f=0,      .display_num=3, .float_point_num=1, NULL},
      {"gyro_x",     &icm_output.gyro_x,   MENU_TYPE_READONLY, MENU_TYPE_FLOAT, .step.f=0,   .max.f=0,      .min.f=0,      .display_num=4, .float_point_num=1, NULL},
      {"State", &g_System_State, MENU_TYPE_INT8, .step.i32=1, .max.i32=1, .min.i32=0, .display_num=1, .float_point_num=0, save_pid_params}, // ✅ 改为 MENU_TYPE_INT8
      {"0_jixi", &mECHANICAL_ZERO, MENU_TYPE_FLOAT, .step.f=0.1, .max.f=30, .min.f=-30, .display_num=2, .float_point_num=2, NULL},
  };
  static menu_item_t balance_gyro_pid_items[] = {
      {"kp", &balance_gyro_pid.kp, MENU_TYPE_FLOAT, .step.f=2, .max.f=500, .min.f=0, .display_num=3, .float_point_num=1, NULL},
      {"Kd", &balance_gyro_pid.kd, MENU_TYPE_FLOAT, .step.f=0.01,  .max.f=10, .min.f=0, .display_num=2, .float_point_num=3, NULL},
      {"ki", &balance_gyro_pid.ki, MENU_TYPE_FLOAT, .step.f=0.01f, .max.f=10.0f, .min.f=0.0f, .display_num=2, .float_point_num=3, NULL},
      {"DEAD_0", &DEAD_ZONE, MENU_TYPE_UINT8, .step.u32=5, .max.u32=200, .min.u32=0, .display_num=3, .float_point_num=0, NULL},
      {"PWM_te",     &g_Debug_PWM_temp,   MENU_TYPE_READONLY, MENU_TYPE_INT16, .step.i32=0,   .max.i32=0,      .min.i32=0,      .display_num=4, .float_point_num=1, NULL},
      {"PWM",     &g_Debug_PWM,    MENU_TYPE_READONLY, MENU_TYPE_INT16, .step.i32=0,   .max.i32=0,      .min.i32=0,      .display_num=4, .float_point_num=1, NULL},      
      {"State", &g_System_State, MENU_TYPE_INT8, .step.i32=1, .max.i32=1, .min.i32=0, .display_num=1, .float_point_num=0, save_pid_params},
      {"0_jixi", &mECHANICAL_ZERO, MENU_TYPE_FLOAT, .step.f=0.05, .max.f=30, .min.f=-30, .display_num=2, .float_point_num=2, NULL},
  };
   static menu_item_t balance_angle_pid_items[] = {
      {"kp", &balance_angle_pid.kp, MENU_TYPE_FLOAT, .step.f=0.1, .max.f=200, .min.f=0, .display_num=1, .float_point_num=3, NULL},
      {"Kd", &balance_angle_pid.kd, MENU_TYPE_FLOAT, .step.f=0.01,  .max.f=10, .min.f=0, .display_num=1, .float_point_num=3, NULL},  // ✅ 改为 2 位小数
      {"ki", &balance_angle_pid.ki, MENU_TYPE_FLOAT, .step.f=0.01f, .max.f=10, .min.f=0, .display_num=1, .float_point_num=3, NULL},
      {"outgyr", &g_Debug_Target_Gyro, MENU_TYPE_READONLY,MENU_TYPE_FLOAT, .step.f=0, .max.f=0, .min.f=0, .display_num=3, .float_point_num=1, NULL},
      {"sp_flg", &speed_flag, MENU_TYPE_INT8, .step.i32=1, .max.i32=1, .min.i32=0, .display_num=1, .float_point_num=0, NULL},
      {"State", &g_System_State, MENU_TYPE_INT8, .step.i32=1, .max.i32=1, .min.i32=0, .display_num=1, .float_point_num=0, save_pid_params},
      {"djSTEP", &MAX_PWM_STEP, MENU_TYPE_INT8, .step.i32=5, .max.i32=60, .min.i32=0, .display_num=2, .float_point_num=0, save_pid_params},
  };
   static menu_item_t speed_pid_items[] = {
      {"kp", &speed_pid.kp, MENU_TYPE_FLOAT, .step.f=0.01f, .max.f=10, .min.f=0, .display_num=1, .float_point_num=4, NULL},
      {"Kd", &speed_pid.kd, MENU_TYPE_FLOAT, .step.f=0.01f,  .max.f=10, .min.f=0, .display_num=1, .float_point_num=4, NULL},
      {"ki", &speed_pid.ki, MENU_TYPE_FLOAT, .step.f=0.01f, .max.f=10, .min.f=0, .display_num=1, .float_point_num=4, NULL},
      {"Sp_Out", &g_Debug_Speed_Output, MENU_TYPE_READONLY,MENU_TYPE_FLOAT, .step.f=0, .max.f=0, .min.f=0, .display_num=2, .float_point_num=2, NULL},
      {"sp_tar", &g_Debug_Target_Speed, MENU_TYPE_FLOAT, .step.f=5, .max.f=1000, .min.f=0, .display_num=3, .float_point_num=1, NULL},
  };

  // 定义菜单页面
  static menu_page_t menu_pages[] = {
      {"Sensors",     sensor_items, sizeof(sensor_items)/sizeof(sensor_items[0])},
      {"gyro_pid",     balance_gyro_pid_items, sizeof(balance_gyro_pid_items)/sizeof(balance_gyro_pid_items[0])},
      {"angle_pid",     balance_angle_pid_items, sizeof(balance_angle_pid_items)/sizeof(balance_angle_pid_items[0])},
      {"spd_pid",     speed_pid_items, sizeof(speed_pid_items)/sizeof(speed_pid_items[0])},
  };
void my_menu_init()
{
  tft180_init();


    // 2. 清屏为白色背景
    tft180_full(RGB565_WHITE);

    // 3. 设置默认字体为 8x16（清晰易读）
    tft180_set_font(TFT180_8X16_FONT);

    // 4. 设置默认前景色和背景色（黑字白底）
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);

    // 5. 初始化菜单系统
    menu_init(menu_pages, 4);  // 2 个页面：PID Control 和 Sensors
}

//================================================= 内部辅助函数声明 ==================================================
static void menu_backup_value(menu_item_t *item);
static void menu_restore_value(menu_item_t *item);
static void menu_adjust_value(menu_item_t *item, int8_t direction);
static void menu_draw_item(uint8_t item_index, uint8_t is_current);
static void menu_draw_page_header(void);

//================================================= 菜单系统初始化 ==================================================
void menu_init(menu_page_t *pages, uint8_t page_count)
{
    if (pages == NULL || page_count == 0)
    {
        return; // 防御性编程：空指针检查
    }

    g_menu.pages = pages;
    g_menu.page_count = page_count;
    g_menu.current_page_index = 0;
    g_menu.current_item_index = 0;
    g_menu.state = MENU_NAV_MODE;
    g_menu.refresh_flag = 1; // 标记全屏刷新
    g_menu.last_item_index = 0;

    // 清屏并显示初始页面
    tft180_clear();
}

//================================================= 按键处理函数 (不含屏幕刷新) ==================================================
void Menu_Key_Handler(uint8_t key_val)
{
    menu_page_t *current_page = &g_menu.pages[g_menu.current_page_index];
    menu_item_t *current_item = &current_page->items[g_menu.current_item_index];

    switch (key_val)
    {
        case 4: // 上键
            if (g_menu.state == MENU_NAV_MODE)
            {
                // 浏览模式：向上移动光标
                if (g_menu.current_item_index > 0)
                {
                    g_menu.last_item_index = g_menu.current_item_index;
                    g_menu.current_item_index--;
                    g_menu.refresh_flag = 2; // 标记刷新两行
                }
            }
            else if (g_menu.state == MENU_EDIT_MODE)
            {
                // 编辑模式：增加数值
                if (current_item->type != MENU_TYPE_READONLY)
                {
                    menu_adjust_value(current_item, 1); // direction=1 表示增加
                    g_menu.refresh_flag = 2; // 刷新当前行
                }
            }
            break;

        case 3: // 下键
            if (g_menu.state == MENU_NAV_MODE)
            {
                // 浏览模式：向下移动光标
                if (g_menu.current_item_index < current_page->item_count - 1)
                {
                    g_menu.last_item_index = g_menu.current_item_index;
                    g_menu.current_item_index++;
                    g_menu.refresh_flag = 2; // 标记刷新两行
                }
            }
            else if (g_menu.state == MENU_EDIT_MODE)
            {
                // 编辑模式：减少数值
                if (current_item->type != MENU_TYPE_READONLY)
                {
                    menu_adjust_value(current_item, -1); // direction=-1 表示减少
                    g_menu.refresh_flag = 2; // 刷新当前行
                }
            }
            break;

        case 1: // 确认键
            if (g_menu.state == MENU_NAV_MODE)
            {
                // 浏览 -> 编辑：备份当前值并进入编辑模式
                if (current_item->type != MENU_TYPE_READONLY)
                {
                    menu_backup_value(current_item);
                    g_menu.state = MENU_EDIT_MODE;
                    g_menu.refresh_flag = 2; // 刷新当前行（光标从 > 变为 *）
                }
            }
            else if (g_menu.state == MENU_EDIT_MODE)
            {
                // 编辑 -> 浏览：✅ 将临时值写入原变量，执行回调并保存
                switch (current_item->type)
                {
                    case MENU_TYPE_INT8:
                        *(int8_t*)(current_item->ptr) = g_menu.edit_val.i8;
                        break;
                    case MENU_TYPE_INT16:
                        *(int16_t*)(current_item->ptr) = g_menu.edit_val.i16;
                        break;
                    case MENU_TYPE_INT32:
                        *(int32_t*)(current_item->ptr) = g_menu.edit_val.i32;
                        break;
                    case MENU_TYPE_UINT8:
                        *(uint8_t*)(current_item->ptr) = g_menu.edit_val.u8;
                        break;
                    case MENU_TYPE_UINT16:
                        *(uint16_t*)(current_item->ptr) = g_menu.edit_val.u16;
                        break;
                    case MENU_TYPE_UINT32:
                        *(uint32_t*)(current_item->ptr) = g_menu.edit_val.u32;
                        break;
                    case MENU_TYPE_FLOAT:
                        *(float*)(current_item->ptr) = g_menu.edit_val.f;
                        break;
                    default:
                        break;
                }

                // 执行确认回调（如保存参数）
                if (current_item->callback != NULL)
                {
                    current_item->callback();
                }

                g_menu.state = MENU_NAV_MODE;
                g_menu.refresh_flag = 2; // 刷新当前行
            }
            break;

        case 2: // 退出键
            if (g_menu.state == MENU_EDIT_MODE)
            {
                // 编辑模式 -> 浏览模式：数据回滚（不保存）
                menu_restore_value(current_item);
                g_menu.state = MENU_NAV_MODE;
                g_menu.refresh_flag = 2; // 刷新当前行
            }
            else if (g_menu.state == MENU_NAV_MODE)
            {
                // 页面切换：退出键用于在不同页面之间切换
                if (g_menu.current_page_index < g_menu.page_count - 1)
                {
                    // 进入下一页
                    g_menu.current_page_index++;
                }
                else
                {
                    // 已经在最后一页，返回第一页（循环切换）
                    g_menu.current_page_index = 0;
                }

                // 重置光标位置
                g_menu.current_item_index = 0;
                g_menu.last_item_index = 0;
                g_menu.refresh_flag = 1; // 标记全屏刷新
            }
            break;

        default:
            // 未知按键，忽略
            break;
    }
}

//================================================= 菜单刷新任务 (由调度器调用) ==================================================
void Menu_Refresh_Task(void)
{
    menu_page_t *current_page = &g_menu.pages[g_menu.current_page_index];

    // 全屏刷新
    if (g_menu.refresh_flag == 1)
    {
        tft180_clear();
        menu_draw_page_header();

        // 绘制所有条目
        for (uint8_t i = 0; i < current_page->item_count; i++)
        {
            menu_draw_item(i, (i == g_menu.current_item_index));
        }

        g_menu.refresh_flag = 0; // 清除刷新标志
    }
    // 局部刷新（两行）
    else if (g_menu.refresh_flag == 2)
    {
        // 重绘上一次选中的行（清除光标）
        menu_draw_item(g_menu.last_item_index, 0);

        // 重绘当前选中的行（显示光标）
        menu_draw_item(g_menu.current_item_index, 1);

        g_menu.refresh_flag = 0; // 清除刷新标志
    }
    // 性能优化：降低自动刷新频率 (避免频繁重绘)
    // 只有在非编辑模式下，且间隔一定周期才刷新只读数据
    else if (g_menu.state == MENU_NAV_MODE)
    {
        // 建议：在调度器中以较低频率调用 (如每100ms调用一次)
        // 不要在此处添加时间判断，而是通过调整调度器调用周期来控制刷新频率
        static uint8_t refresh_skip_counter = 0;

        // 跳帧刷新：每5次调用才真正刷新一次 (降低50%刷新率)
        refresh_skip_counter++;
        if (refresh_skip_counter >= 5)
        {
            refresh_skip_counter = 0;

            // 遍历当前页面的所有条目，仅刷新只读类型的数据
            for (uint8_t i = 0; i < current_page->item_count; i++)
            {
                menu_item_t *item = &current_page->items[i];
                if (item->type == MENU_TYPE_READONLY)
                {
                    // 只刷新只读条目的数值部分（不影响光标和选中状态）
                    menu_draw_item(i, (i == g_menu.current_item_index));
                }
            }
        }
    }
}

//================================================= 获取当前菜单状态 ==================================================
menu_state_enum menu_get_current_state(void)
{
    return g_menu.state;
}

//================================================= 内部辅助函数实现 ==================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     menu_backup_value
// 功能描述     备份当前条目的值到 backup_val 和 edit_val (进入编辑模式时调用)
// 输入参数     item: 当前条目指针
//-------------------------------------------------------------------------------------------------------------------
static void menu_backup_value(menu_item_t *item)
{
    if (item->ptr == NULL) return;

    switch (item->type)
    {
        case MENU_TYPE_INT8:
            g_menu.backup_val.i8 = *(int8_t*)(item->ptr);
            g_menu.edit_val.i8 = *(int8_t*)(item->ptr);
            break;
        case MENU_TYPE_INT16:
            g_menu.backup_val.i16 = *(int16_t*)(item->ptr);
            g_menu.edit_val.i16 = *(int16_t*)(item->ptr);
            break;
        case MENU_TYPE_INT32:
            g_menu.backup_val.i32 = *(int32_t*)(item->ptr);
            g_menu.edit_val.i32 = *(int32_t*)(item->ptr);
            break;
        case MENU_TYPE_UINT8:
            g_menu.backup_val.u8 = *(uint8_t*)(item->ptr);
            g_menu.edit_val.u8 = *(uint8_t*)(item->ptr);
            break;
        case MENU_TYPE_UINT16:
            g_menu.backup_val.u16 = *(uint16_t*)(item->ptr);
            g_menu.edit_val.u16 = *(uint16_t*)(item->ptr);
            break;
        case MENU_TYPE_UINT32:
            g_menu.backup_val.u32 = *(uint32_t*)(item->ptr);
            g_menu.edit_val.u32 = *(uint32_t*)(item->ptr);
            break;
        case MENU_TYPE_FLOAT:
            g_menu.backup_val.f = *(float*)(item->ptr);
            g_menu.edit_val.f = *(float*)(item->ptr);
            break;
        case MENU_TYPE_READONLY:
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     menu_restore_value
// 功能描述     从 backup_val 恢复值 (编辑模式退出时调用，回滚数据)
// 输入参数     item: 当前条目指针
//-------------------------------------------------------------------------------------------------------------------
static void menu_restore_value(menu_item_t *item)
{
    if (item->ptr == NULL) return;

    switch (item->type)
    {
        case MENU_TYPE_INT8:
            *(int8_t*)(item->ptr) = g_menu.backup_val.i8;
            break;
        case MENU_TYPE_INT16:
            *(int16_t*)(item->ptr) = g_menu.backup_val.i16;
            break;
        case MENU_TYPE_INT32:
            *(int32_t*)(item->ptr) = g_menu.backup_val.i32;
            break;
        case MENU_TYPE_UINT8:
            *(uint8_t*)(item->ptr) = g_menu.backup_val.u8;
            break;
        case MENU_TYPE_UINT16:
            *(uint16_t*)(item->ptr) = g_menu.backup_val.u16;
            break;
        case MENU_TYPE_UINT32:
            *(uint32_t*)(item->ptr) = g_menu.backup_val.u32;
            break;
        case MENU_TYPE_FLOAT:
            *(float*)(item->ptr) = g_menu.backup_val.f;
            break;
        case MENU_TYPE_READONLY:
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     menu_adjust_value
// 功能描述     调整当前条目的值 (编辑模式上下键调用，修改 edit_val 而非原变量)
// 输入参数     item: 当前条目指针
// 输入参数     direction: 方向 (1=增加, -1=减少)
//-------------------------------------------------------------------------------------------------------------------
static void menu_adjust_value(menu_item_t *item, int8_t direction)
{
    if (item->ptr == NULL) return;

    switch (item->type)
    {
        case MENU_TYPE_INT8:
        {
            int8_t step = (int8_t)item->step.i32;
            int8_t max = (int8_t)item->max.i32;
            int8_t min = (int8_t)item->min.i32;

            g_menu.edit_val.i8 += (direction * step);
            if (g_menu.edit_val.i8 > max) g_menu.edit_val.i8 = max;
            if (g_menu.edit_val.i8 < min) g_menu.edit_val.i8 = min;
            break;
        }
        case MENU_TYPE_INT16:
        {
            int16_t step = (int16_t)item->step.i32;
            int16_t max = (int16_t)item->max.i32;
            int16_t min = (int16_t)item->min.i32;

            g_menu.edit_val.i16 += (direction * step);
            if (g_menu.edit_val.i16 > max) g_menu.edit_val.i16 = max;
            if (g_menu.edit_val.i16 < min) g_menu.edit_val.i16 = min;
            break;
        }
        case MENU_TYPE_INT32:
        {
            int32_t step = item->step.i32;
            int32_t max = item->max.i32;
            int32_t min = item->min.i32;

            g_menu.edit_val.i32 += (direction * step);
            if (g_menu.edit_val.i32 > max) g_menu.edit_val.i32 = max;
            if (g_menu.edit_val.i32 < min) g_menu.edit_val.i32 = min;
            break;
        }
        case MENU_TYPE_UINT8:
        {
            uint8_t step = (uint8_t)item->step.u32;
            uint8_t max = (uint8_t)item->max.u32;
            uint8_t min = (uint8_t)item->min.u32;

            if (direction == 1)
            {
                if ((uint16_t)g_menu.edit_val.u8 + step <= max)
                    g_menu.edit_val.u8 += step;
                else
                    g_menu.edit_val.u8 = max;
            }
            else
            {
                if (g_menu.edit_val.u8 >= min + step)
                    g_menu.edit_val.u8 -= step;
                else
                    g_menu.edit_val.u8 = min;
            }
            break;
        }
        case MENU_TYPE_UINT16:
        {
            uint16_t step = (uint16_t)item->step.u32;
            uint16_t max = (uint16_t)item->max.u32;
            uint16_t min = (uint16_t)item->min.u32;

            if (direction == 1)
            {
                if ((uint32_t)g_menu.edit_val.u16 + step <= max)
                    g_menu.edit_val.u16 += step;
                else
                    g_menu.edit_val.u16 = max;
            }
            else
            {
                if (g_menu.edit_val.u16 >= min + step)
                    g_menu.edit_val.u16 -= step;
                else
                    g_menu.edit_val.u16 = min;
            }
            break;
        }
        case MENU_TYPE_UINT32:
        {
            uint32_t step = item->step.u32;
            uint32_t max = item->max.u32;
            uint32_t min = item->min.u32;

            if (direction == 1)
            {
                g_menu.edit_val.u32 += step;
                if (g_menu.edit_val.u32 > max) g_menu.edit_val.u32 = max;
            }
            else
            {
                if (g_menu.edit_val.u32 >= min + step)
                    g_menu.edit_val.u32 -= step;
                else
                    g_menu.edit_val.u32 = min;
            }
            break;
        }
        case MENU_TYPE_FLOAT:
        {
            float step = item->step.f;
            float max = item->max.f;
            float min = item->min.f;

            g_menu.edit_val.f += (direction * step);
            if (g_menu.edit_val.f > max) g_menu.edit_val.f = max;
            if (g_menu.edit_val.f < min) g_menu.edit_val.f = min;
            break;
        }
        case MENU_TYPE_READONLY:
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     menu_draw_item
// 功能描述     绘制单个菜单条目
// 输入参数     item_index: 条目索引
// 输入参数     is_current: 是否为当前选中条目 (1=是, 0=否)
//-------------------------------------------------------------------------------------------------------------------
static void menu_draw_item(uint8_t item_index, uint8_t is_current)
{
    menu_page_t *current_page = &g_menu.pages[g_menu.current_page_index];
    menu_item_t *item = &current_page->items[item_index];

    uint16_t y = MENU_LINE_HEIGHT * item_index + 16; // 计算Y坐标，+16 为标题栏留出空间
    char cursor = is_current ? ((g_menu.state == MENU_EDIT_MODE) ? '*' : '>') : ' ';

    // 清除当前行 (用背景色填充一行，清除整行避免残影)
    tft180_set_color(RGB565_WHITE, RGB565_WHITE);
    for (uint8_t j = 0; j < 16; j++)  // 填充整个行高
    {
        for (uint8_t i = 0; i < 128; i++)
        {
            tft180_draw_point(i, y + j, RGB565_WHITE);
        }
    }

    // 设置颜色
    if (g_menu.state == MENU_EDIT_MODE && is_current)
    {
        tft180_set_color(RGB565_GREEN, RGB565_WHITE); // 编辑模式：绿色
    }
    else if (is_current)
    {
        tft180_set_color(RGB565_BLUE, RGB565_WHITE);   // 浏览模式选中：蓝色
    }
    else
    {
        tft180_set_color(RGB565_BLACK, RGB565_WHITE);  // 未选中：黑色
    }

    // 绘制光标
    char cursor_str[2] = {cursor, '\0'};
    tft180_show_string(MENU_CURSOR_X, y, cursor_str);

    // 绘制标签
    tft180_show_string(MENU_LABEL_START_X, y, (char*)item->label);

    // 绘制数值
    if (item->ptr != NULL)
    {
        if (item->type == MENU_TYPE_READONLY)
        {
            // 只读类型：根据 actual_type 显示实际数值（青色）
            tft180_set_color(RGB565_CYAN, RGB565_WHITE);
            switch (item->actual_type)
            {
                case MENU_TYPE_INT8:
                    tft180_show_int(MENU_VALUE_START_X, y, (int32_t)(*(int8_t*)(item->ptr)), item->display_num);
                    break;
                case MENU_TYPE_INT16:
                    tft180_show_int(MENU_VALUE_START_X, y, (int32_t)(*(int16_t*)(item->ptr)), item->display_num);
                    break;
                case MENU_TYPE_INT32:
                    tft180_show_int(MENU_VALUE_START_X, y, *(int32_t*)(item->ptr), item->display_num);
                    break;
                case MENU_TYPE_UINT8:
                    tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)(*(uint8_t*)(item->ptr)), item->display_num);
                    break;
                case MENU_TYPE_UINT16:
                    tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)(*(uint16_t*)(item->ptr)), item->display_num);
                    break;
                case MENU_TYPE_UINT32:
                    tft180_show_uint(MENU_VALUE_START_X, y, *(uint32_t*)(item->ptr), item->display_num);
                    break;
                case MENU_TYPE_FLOAT:
                    tft180_show_float(MENU_VALUE_START_X, y, *(float*)(item->ptr), item->display_num, item->float_point_num);
                    break;
                default:
                    break;
            }
        }
        else
        {
            // 可编辑类型：根据 type 显示数值（正常颜色）
            // ✅ 编辑模式显示临时值，浏览模式显示原变量值
            switch (item->type)
            {
                case MENU_TYPE_INT8:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_int(MENU_VALUE_START_X, y, (int32_t)g_menu.edit_val.i8, item->display_num);  // 显示临时值
                    else
                        tft180_show_int(MENU_VALUE_START_X, y, (int32_t)(*(int8_t*)(item->ptr)), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_INT16:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_int(MENU_VALUE_START_X, y, (int32_t)g_menu.edit_val.i16, item->display_num);  // 显示临时值
                    else
                        tft180_show_int(MENU_VALUE_START_X, y, (int32_t)(*(int16_t*)(item->ptr)), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_INT32:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_int(MENU_VALUE_START_X, y, g_menu.edit_val.i32, item->display_num);  // 显示临时值
                    else
                        tft180_show_int(MENU_VALUE_START_X, y, *(int32_t*)(item->ptr), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_UINT8:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)g_menu.edit_val.u8, item->display_num);  // 显示临时值
                    else
                        tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)(*(uint8_t*)(item->ptr)), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_UINT16:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)g_menu.edit_val.u16, item->display_num);  // 显示临时值
                    else
                        tft180_show_uint(MENU_VALUE_START_X, y, (uint32_t)(*(uint16_t*)(item->ptr)), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_UINT32:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_uint(MENU_VALUE_START_X, y, g_menu.edit_val.u32, item->display_num);  // 显示临时值
                    else
                        tft180_show_uint(MENU_VALUE_START_X, y, *(uint32_t*)(item->ptr), item->display_num);  // 显示原值
                    break;
                case MENU_TYPE_FLOAT:
                    if (g_menu.state == MENU_EDIT_MODE && is_current)
                        tft180_show_float(MENU_VALUE_START_X, y, g_menu.edit_val.f, item->display_num, item->float_point_num);  // 显示临时值
                    else
                        tft180_show_float(MENU_VALUE_START_X, y, *(float*)(item->ptr), item->display_num, item->float_point_num);  // 显示原值
                    break;
                default:
                    break;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     menu_draw_page_header
// 功能描述     绘制页面标题头部
//-------------------------------------------------------------------------------------------------------------------
static void menu_draw_page_header(void)
{
    menu_page_t *current_page = &g_menu.pages[g_menu.current_page_index];

    // 绘制标题背景（蓝色背景）
    for (uint8_t j = 0; j < 16; j++)
    {
        for (uint8_t i = 0; i < 128; i++)
        {
            tft180_draw_point(i, j, RGB565_BLUE);
        }
    }

    // 绘制白色标题文字
    tft180_set_color(RGB565_WHITE, RGB565_BLUE);
    tft180_show_string(2, 0, (char*)current_page->name);

    // 绘制分隔线（底部白色线条）
    for (uint8_t i = 0; i < 128; i++)
    {
        tft180_draw_point(i, 15, RGB565_WHITE);
    }
}
