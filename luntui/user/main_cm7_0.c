/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "bsp_app.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������


// *************************** ����Ӳ������˵�� ***************************
// ʹ����ɿƼ� CMSIS-DAP ��������������
//      ֱ�ӽ���������ȷ�����ں��İ�ĵ������ؽӿڼ���
// ʹ�� USB-TTL ģ������
//      ģ��ܽ�            ��Ƭ���ܽ�
//      USB-TTL-RX          �鿴 zf_common_debug.h �ļ��� DEBUG_UART_TX_PIN �궨������� Ĭ�� P14_0
//      USB-TTL-TX          �鿴 zf_common_debug.h �ļ��� DEBUG_UART_RX_PIN �궨������� Ĭ�� P14_1
//      USB-TTL-GND         ���İ��Դ�� GND
//      USB-TTL-3V3         ���İ� 3V3 ��Դ

//================================�ر�ע��================================
// ���ڽ���ʱһ��Ҫ��GND �����޷�����ͨѶ
//================================�ر�ע��================================
//================================�ر�ע��================================
// ���ڽ���ʱһ��Ҫ��GND �����޷�����ͨѶ
//================================�ر�ע��================================
//================================�ر�ע��================================
// ���ڽ���ʱһ��Ҫ��GND �����޷�����ͨѶ
//================================�ر�ע��================================

// ***************************** ���̲���˵�� *****************************
// 1.���İ���¼��ɱ����̣�����ʹ�ú��İ���������������� USB-TTL ģ�飬�ڶϵ�������������
// 2.���������������� USB-TTL ģ�����ӵ��ԣ�����ϵ�
// 3.������ʹ�ô������ִ򿪶�Ӧ�Ĵ��ڣ����ڲ�����Ϊ DEBUG_UART_BAUDRATE �궨�� Ĭ�� 115200�����İ尴�¸�λ����
// 4.�����ڴ��������Ͽ������´�����Ϣ��
//      UART Text.
// 5.ͨ���������ַ������ݣ����յ���ͬ�ķ�������
//      UART get data:.......
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�

// ******************************* �������� *******************************
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // Ĭ�� UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // Ĭ�� 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // Ĭ�� UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // Ĭ�� UART0_RX_P00_0

extern uint8 uart_get_data[64];                                                        // ���ڽ������ݻ�����
extern uint8 fifo_get_data[64];                                                        // fifo �������������

extern uint8  get_data ;                                                            // �������ݱ���
extern uint32 fifo_data_count0;                                                     // fifo ���ݸ���

extern fifo_struct uart_data_fifo;



int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                   // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
//    interrupt_set_priority(CPUIntIdx2_IRQn, 1);                                    // ���� PIT �����жϵ��ж����ȼ�Ϊ 1�����ڶ��ص��жϲ�ѯ��ʽ������PIT�ж϶����õ�һ���ж�������
      while(1)
      {
          if(imu660rc_init(IMU660RC_QUARTERNION_480HZ   ))
          {
             printf("\r\n imu660rc init error.");                                 // imu660rc ��ʼ��ʧ��
          }
          else
          {
             break;
          }
      }
      // IMU 零偏校准 - 需保持静止
      printf("\r\n ===== GYRO CALIBRATION =====");
      printf("\r\n PLEASE KEEP THE ROBOT STILL!");
      gyro_calibrate();
      printf("\r\n CALIBRATION DONE!");
      printf("\r\n OFFSET: x=%.3f, y=%.3f, z=%.3f",
             icm_output.gyro_x_offset,
             icm_output.gyro_y_offset,
             icm_output.gyro_z_offset);
      printf("\r\n ============================");
    beep_init();
    my_key_init();
    my_menu_init();

    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // ��ʼ������
    uart_rx_interrupt(UART_INDEX, 1);                                           // ���� UART_INDEX �Ľ����ж�
    
    Servo_Init();
    small_driver_uart_init();//��ˢ������ʼ��
    small_driver_get_speed();//���ڷ��������ȡ��ˢ�ٶ�
//    small_driver_set_duty(600,-600);//����
    control_init();
     seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
     
    timer_init(TC_TIME2_CH0, TIMER_MS);
    timer_start(TC_TIME2_CH0);
    pit_ms_init(PIT_CH0, 1); 
    interrupt_set_priority(CPUIntIdx3_IRQn, 2);
    scheduler_init();

    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
      scheduler_run();
        // �� printf һ���������
//        uart_printf(UART_0, "System Run: %d \r\n", timer_get(TC_TIME2_CH0));
        
//      printf("Timer count is %d ms.\r\n", timer_get(TC_TIME2_CH0));                                           // �������
//      system_delay_ms(10);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       UART_INDEX �Ľ����жϴ������� ����������� UART_INDEX ��Ӧ���жϵ���
// ����˵��       void
// ���ز���       void
// ʹ��ʾ��       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // �������� while �ȴ�ʽ ���������ж�ʹ��
//    if(uart_query_byte(UART_INDEX, &get_data))                                  // �������� ��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE
//    {
//        fifo_write_buffer(&uart_data_fifo, &get_data, 1);                       // ������д�� fifo ��
//
//    }
}

// **************************** �������� ****************************
// **************************** ���̳�������˵�� ****************************
// ��������ʱ�밴�������������б����
// ����1������û������
//      �鿴�������ִ򿪵��Ƿ�����ȷ�Ĵ��ڣ����򿪵� COM ���Ƿ��Ӧ���ǵ������������� USB-TTL ģ��� COM ��
//      �����ʹ����ɿƼ� CMSIS-DAP �������������ӣ���ô������������Ƿ��ɶ��������İ崮�������Ƿ��Ѿ����ӣ��������߲鿴���İ�ԭ��ͼ�����ҵ�
//      �����ʹ�� USB-TTL ģ�����ӣ���ô��������Ƿ������Ƿ��ɶ���ģ�� TX �Ƿ����ӵĺ��İ�� RX��ģ�� RX �Ƿ����ӵĺ��İ�� TX
// ����2��������������
//      �鿴�����������õĲ������Ƿ����������һ�£������� zf_common_debug.h �ļ��� DEBUG_UART_BAUDRATE �궨��Ϊ debug uart ʹ�õĴ��ڲ�����




