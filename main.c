#include "stm32f10x.h"
#include<math.h>
#include "delay.h"	 	   		 	 	 	 
#include "lcd.h"
#include <stdio.h>
/* ----------------------- RC Channel Definition---------------------------- */ 
#define RC_MIN  ((uint16_t)364 ) 
#define RC_OFFSET  ((uint16_t)1024) 
#define RC_MAX  ((uint16_t)1684)
#define MAXSPEED  100

/* ----------------------- RC Switch Definition----------------------------- */ 
#define RC_SW_UP   ((uint16_t)1) 
#define RC_SW_MID  ((uint16_t)3) 
#define RC_SW_DOWN ((uint16_t)2) 

/* ----------------------- PC Key Definition-------------------------------- */ 
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0) 
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_SHIFT  ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_CTRL   ((uint16_t)0x01<<7) 

/* ----------------------- Data Struct ------------------------------------- */ 
typedef struct 
{ 

struct 
{
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;
 uint8_t  s1;
 uint8_t  s2; 

}rc; 
struct 
{
 int16_t x;
 int16_t y;
 int16_t z;
 uint8_t press_l;
 uint8_t press_r; 

}mouse; 
struct 
{ 
	uint16_t v; 
}key; 
}RC_Ctl_t; 


/* ----------------------- Internal Data ----------------------------------- */ 
volatile unsigned char sbus_rx_buffer[25]; 
static RC_Ctl_t RC_Ctl; 
u8 speed;
float angle;
u8 s2;
u8 D[5];
//char *str;




 void RC_Init(void) { 
/* -------------- Enable Module Clock Source ----------------------------*/ 
   
   //RCC_PCLK1Config(RCC_HCLK_Div2);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
   //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);	 	 
	 

{ NVIC_InitTypeDef  nvic; 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
  nvic.NVIC_IRQChannel = DMA1_Channel6_IRQn; 
  nvic.NVIC_IRQChannelSubPriority = 0; 
  nvic.NVIC_IRQChannelCmd   = ENABLE; 
  NVIC_Init(&nvic);  
}  
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 /* -------------- Configure GPIO ---------------------------------------*/ 
{
 GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//??????10MHz
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????
GPIO_Init(GPIOA, &GPIO_InitStructure);//?????GPIOx???




}



{
 GPIO_InitTypeDef  gpio; 
 USART_InitTypeDef usart2;

 gpio.GPIO_Pin = GPIO_Pin_2 ;
 
 gpio.GPIO_Mode=GPIO_Mode_AF_PP;
 gpio.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &gpio);

 USART_DeInit(USART2);
 usart2.USART_BaudRate = 9600;
 usart2.USART_WordLength   = USART_WordLength_8b;
 usart2.USART_StopBits = USART_StopBits_1;
 usart2.USART_Parity = USART_Parity_No; 
 usart2.USART_Mode = USART_Mode_Rx|USART_Mode_Tx; 
 usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
 USART_Init(USART2,&usart2); 
 USART_Cmd(USART2,ENABLE); 
 USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	

} 
/* -------------- Configure spi1  ---------------------------------------*/
//{
// 
//GPIO_InitTypeDef GPIO_InitStructure;
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//GPIO_Init(GPIOA, &GPIO_InitStructure);
//GPIO_SetBits(GPIOA,GPIO_Pin_4);

//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//GPIO_Init(GPIOA, &GPIO_InitStructure);



//}






//{
//SPI_InitTypeDef SPI_InitStructure;
//SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;+63;
//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
//SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
//SPI_InitStructure.SPI_CRCPolynomial = 7;
//SPI_Init(SPI1,&SPI_InitStructure);

//SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);
//SPI_Cmd(SPI1,ENABLE);
//}




/* -------------- Configure uart1  ---------------------------------------*/  
{
 GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//??????10MHz
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????
GPIO_Init(GPIOA, &GPIO_InitStructure);//?????GPIOx???
}

{
 GPIO_InitTypeDef  gpio; 
 USART_InitTypeDef usart1;

 gpio.GPIO_Pin = GPIO_Pin_9 ;
 gpio.GPIO_Mode=GPIO_Mode_AF_PP;
 gpio.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &gpio);

 USART_DeInit(USART1);
 usart1.USART_BaudRate = 9600;
 usart1.USART_WordLength   = USART_WordLength_8b;
 usart1.USART_StopBits = USART_StopBits_1;
 usart1.USART_Parity = USART_Parity_No; 
 usart1.USART_Mode = USART_Mode_Rx|USART_Mode_Tx; 
 usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
 USART_Init(USART1,&usart1); 
 USART_Cmd(USART1,ENABLE); 
}  

/* -------------- Configure uart3  ---------------------------------------*/ 
{
 
 GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//??????10MHz
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????
GPIO_Init(GPIOC, &GPIO_InitStructure);//?????GPIOx???
}

{
 GPIO_InitTypeDef  gpio; 
 USART_InitTypeDef usart3;

 gpio.GPIO_Pin = GPIO_Pin_10 ;
 gpio.GPIO_Mode=GPIO_Mode_AF_PP;
 gpio.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOC, &gpio);

 USART_DeInit(USART3);
 usart3.USART_BaudRate = 9600;
 usart3.USART_WordLength   = USART_WordLength_8b;
 usart3.USART_StopBits = USART_StopBits_1;
 usart3.USART_Parity = USART_Parity_No; 
 usart3.USART_Mode = USART_Mode_Rx|USART_Mode_Tx; 
 usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
 USART_Init(USART3,&usart3); 
 USART_Cmd(USART3,ENABLE); 
}  




/* -------------- Configure DMA -----------------------------------------*/  
{ 

	DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel6); 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sbus_rx_buffer;
  DMA_InitStructure.DMA_DIR =  DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 18;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel6,ENABLE);
	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC, ENABLE);
}  
}


void RCC_Configuration(void) 
{
	
  	RCC_DeInit();				
  	RCC_HSEConfig(RCC_HSE_ON);  
  	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);  

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	
    FLASH_SetLatency(FLASH_Latency_2);	  					 
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 					
    RCC_PCLK2Config(RCC_HCLK_Div1);							
    RCC_PCLK1Config(RCC_HCLK_Div2);	  						
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	
    RCC_PLLCmd(ENABLE);	  									
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);		

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	  			 
    while(RCC_GetSYSCLKSource() != 0x08);					


		}

		
void dipandata()
{
	//RC_Ctl.rc.ch2 RC_Ctl.rc.ch3进行处理
	//uint8_t *p=&speed;
	  uint8_t *p1=(uint8_t*)&angle;
	  s2=(u8)RC_Ctl.rc.s2;
	  
	  speed=sqrt(((abs(RC_Ctl.rc.ch2)-RC_OFFSET)/660.0*MAXSPEED*(abs(RC_Ctl.rc.ch2)-RC_OFFSET)/660.0*MAXSPEED)+((abs(RC_Ctl.rc.ch3)-RC_OFFSET)/660.0*MAXSPEED*(abs(RC_Ctl.rc.ch3)-RC_OFFSET)/660.0*MAXSPEED));
		angle=(float)(atan2((double)(RC_Ctl.rc.ch3-RC_OFFSET),(double)(RC_Ctl.rc.ch2-RC_OFFSET)));
		
	  //uint16_t temp=(uint16_t)*p;
		USART_SendData(USART1,speed);
		//SPI_I2S_SendData(SPI1,temp);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
	
	for(uint8_t i=0;i<4;++i)
	{
	  uint16_t temp=(uint16_t)*p1;
		USART_SendData(USART1,temp);
		//SPI_I2S_SendData(SPI1,temp);
		p1++;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
	}
	USART_SendData(USART1,s2);
	//while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
	//SPI_I2S_SendData(SPI1,s2);
	
}

void yuntaidata()
{
	
	
}

void LCD_convert(float count1)
{
	if(count1<0) count1+=2*3.141592653;
	u32 count=(u32)(count1*100);
  D[5]=0;
  D[4]=count%10+48;              
  D[3]=(count%100)/10+48;        
  D[1]=(count%1000)/100+48;      
  D[0]=(count%10000)/1000+48;
  D[2]='.';
}




int main(void)
{
	 RCC_Configuration();
	 RC_Init();
	 delay_init(72);	   	  
 	 LCD_Init();
	 USART_SendData(USART2, 0x44);  	
	while(1)
	{
		POINT_COLOR=BLACK;	  
		LCD_ShowString(30,50,200,16,16,"serial info");	
		LCD_convert(speed);
		//sprintf(str,"%f",speed);
		LCD_ShowString(30,70,200,16,16,"speed:");
		LCD_ShowString(90,70,200,16,16,D);
		//LCD_ShowString(90,70,200,16,16,str);
		LCD_convert(angle);
		//sprintf(str,"%f",angle);
		LCD_ShowString(30,90,200,16,16,"angle:"); 
		LCD_ShowString(90,90,200,16,16,D);
    //LCD_ShowString(90,90,200,16,16,	str);   		
		LCD_ShowString(30,110,200,16,16,"2015-03-16");
                LCD_ShowString(30,130,200,16,16,"Build:0016");		
		delay_ms(1000);	
		
	}
}	 
	
	
 
void DMA1_Channel6_IRQHandler(void)
{  
  
	
	if(DMA_GetITStatus(DMA1_IT_TC6))  
{ 
 DMA_ClearFlag(DMA1_FLAG_TC6); 
 DMA_ClearITPendingBit(DMA1_IT_TC6);     
 RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;  //!< Channel 0 
 RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;  //!< Channel 1 
 RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) |  //!< Channel 2  
(sbus_rx_buffer[4] << 10)) & 0x07ff;  
 RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;  //!< Channel 3 
 RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;  //!< Switch left  
 RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);  //!< Switch right  

 RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);  //!< Mouse X axis 
 RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);  //!< Mouse Y axis 
 RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);  //!< Mouse Z axis 
 RC_Ctl.mouse.press_l = sbus_rx_buffer[12];   //!< Mouse Left Is Press ? 
 RC_Ctl.mouse.press_r = sbus_rx_buffer[13];   //!< Mouse Right Is Press ? 
 RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);  //!< KeyBoard value */

	dipandata();
	yuntaidata();
  
} 
 
}    
