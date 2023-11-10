/*
 * @Date: 2022-08-29 09:38:51
 * @LastEditors: Cath pjgao@youealcorp.com.cn
 * @LastEditTime: 2022-09-06 14:15:28
 * @FilePath: \MDK-ARMc:\Users\Administrator\Desktop\snake\snake\Drivers\OLED\oled.h
 */
#ifndef __OLED_H
#define __OLED_H 

#include "main.h"
#include "spi.h"

#define OLED_port hspi1
#define GT20L16_CS_clr HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0)
#define GT20L16_CS_set HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1)
#define DATA_COMMAND_clr HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0)
#define DATA_COMMAND_set HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1)
#define SSD1306_CS_clr HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0)
#define SSD1306_CS_set HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1)
//SCL=SCLK  
//SDA=MOSI
//DC=DC
//CS=CS1
//FS0=MOSI
//CS2=CS2
//-----------------OLED�˿ڶ���---------------- 

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Clear(void);
void OLED_address(uint8_t x,uint8_t y);
void OLED_Display_128x64(uint8_t *dp);
void OLED_Display_16x16(uint8_t x,uint8_t y,uint8_t *dp);
void OLED_Display_8x16(uint8_t x,uint8_t y,uint8_t *dp);
void OLED_Display_5x7(uint8_t x, uint8_t y, uint8_t *dp);
void OLED_Display_xy(uint8_t pos_x, uint8_t pos_y, uint8_t size_x, uint8_t size_y, uint8_t *dp);
void Send_Command_to_ROM(uint8_t dat);
uint8_t Get_data_from_ROM(void);
void Get_data_from_ROM_point(uint8_t *data);
void OLED_get_data_from_ROM(uint8_t addrHigh,uint8_t addrMid,uint8_t addrLow,uint8_t *pbuff,uint8_t DataLen);
void OLED_Display_GB2312_string(uint8_t x,uint8_t y,uint8_t *text,uint8_t rev_color);
void OLED_Display_string_5x7(uint8_t x,uint8_t y,uint8_t *text);
void OLED_Display_block(uint8_t x, uint8_t y, uint8_t blockdata);
void OLED_ShowNum(uint8_t x,uint8_t y,float num,uint8_t len);
void OLED_Init(void);
#endif

