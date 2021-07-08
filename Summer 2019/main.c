#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f2xx.h"
#include "stm322xg_eval.h"
#include "stm322xg_eval_lcd.h"
#include "stdio.h"

#define LCD_PORT P2

sbit rs=P3^5;
sbit rw=P3^6;
sbit en=P3^7;
sbit D7=P3^7;

RCC ->AHBENR

void CMD_WRT(unsigned char);
void DATA_WRT(unsigned char);
void busy;

volatile uint32_t* MODER_B = (volatile uint32_t*) 0x40020400;
PA1 = 1;


 int main{

 }