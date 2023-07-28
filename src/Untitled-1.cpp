#include "AllHeader.h"
#include "Delay.h"
#include "Key.h"
#include "OLED.h"
#include "PWM.h"
#include "Serial.h"
#include "ctype.h"
#include "dht11.h"
#include "stdlib.h"
#include "stm32f10x.h" // Device header
#include "string.h"

uint8_t temperature, humidity, temp, humi;
uint8_t i;
u8 buffer[5];
int main(void) {
    Motor_Init();
    Serial_Init();
    OLED_Init();
    PWM_Init();
    DHT11_Init();
    OLED_ShowString(4, 1, "hello");

    u16 iseed;
    SystemInit();
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
    Adc_Init();
    iseed = Get_Adc_Average(ADC_Channel_14, 3); // 通过模拟输入获取悬空引脚的电平产生随机种子
    srand(iseed);
    ws2812_Init();
    while (1) {

        //		DHT11_Read_Data(&temperature,&temp,&humidity,&humi);
        //		Delay_ms(500);
        //		Serial_Printf("T$%d.%d,%d.%d\r\n",humidity,humi,temperature,temp);
        DHT11_Read_Data_Buffer(buffer, 5);
        Serial_Printf(buffer);
        Delay_ms(1000);
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        if (Serial_RxFlag == 1) {

            if (Serial_RxPacket[0] == 'F') {
                // 角度值
                int angle = Serial_RxPacket[1];
                // 速度值
                int speed = Serial_RxPacket[2];

                turn_angle(1, angle, speed);
                //				PWM_SetCompare1(1365);
                //				Delay_ms(num);
                //				PWM_SetCompare1(1500);
                Serial_SendString("M$F DONE\r\n");
            } else if (Serial_RxPacket[0] == 'R') {
                int angle = Serial_RxPacket[1];
                int speed = Serial_RxPacket[2];
                turn_angle(0, angle, speed);
                //				uint32_t num=atoi(number);
                //				PWM_SetCompare1(1600);
                //				Delay_ms(num);
                //				PWM_SetCompare1(1500);
                Serial_SendString("M$R DONE\r\n");
            } else if (Serial_RxPacket[0] == 'E') { // red
                Serial_SendString("red DONE\r\n");
                //			 Running_water_lamp(255,0,0,200);//流水灯 RGB,间隔时间
                ws2812_AllOpen(255, 0, 0);          // 流水灯 RGB,间隔时间
            } else if (Serial_RxPacket[0] == 'G') { // green
                Serial_SendString("green DONE\r\n");
                ws2812_AllOpen(0, 255, 0);          // 流水灯 RGB,间隔时间
            }
        }
        Serial_RxFlag = 0;

        //			if (Serial_GetRxFlag() == 1)
        //		{
        //			RxData = Serial_GetRxData();
        //			Serial_SendByte(RxData);
        //			SPIN();
        //			SPIN();
        //			OLED_ShowString(1, 1, "DHT11_OK");
        //			OLED_ShowString(2, 1, "SPIN_OK");

        //		}
    }
}

// OLED_ShowString(2, 1, "SPIN_OK");
// SPIN();

//}