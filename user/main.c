 /**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.3.0
  * @date    16-June-2017
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define RW_len 30
#define Tx_485 GPIOD->ODR|=(uint8_t)GPIO_PIN_4          //485 control
#define Rx_485 GPIOD->ODR&=!((uint8_t)GPIO_PIN_4)       //485 control

#define debugmode 1 //dedicated smlee. after completed develop, should be remove .

#define BLOCK_OPERATION    0    /* block 0 in data eeprom memory: address is 0x4000 */

//char RWBuf[RW_len]={0x01,0x02,0x03};
uint8_t RWBuf[RW_len]={0x00,};
uint8_t RWCnt = 0;
uint8_t rx_flag = 0xff;

uint8_t val = 0x00, val_comp = 0x00;
uint32_t add = 0x00;

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private variables ---------------------------------------------------------*/
__IO TestStatus OperationStatus;


int testtick=0;
int rxflag;

//uint8_t GBuffer[FLASH_BLOCK_SIZE];
uint8_t GBuffer[128];//128=eeprom size

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#if debugmode
void debugmsg(volatile uint8_t * debugbuf);
void debugchar(uint8_t debugchar);

#endif  


void Delay(uint32_t nCount);
static void CLK_Config(void);
void BufInit(uint8_t * buf);
void Write_data(uint8_t * buf);
uint8_t parser_data(void);

void main(void)
{
  volatile uint8_t testarr[5]={0,};
  
  CLK_Config();
  
  //GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);//test gpio
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);//485 CONTROL
  
    //UART1_DeInit();
  /* UART1 configuration ------------------------------------------------------*/
  /* UART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Receive and transmit enabled
        - UART1 Clock disabled
  */
  UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  

/* Infinite loop */
  //TIM1_CtrlPWMOutputs(ENABLE);
  enableInterrupts();
  
  ////////////////eeprom write test smlee///////////////

      /* Define FLASH programming time */
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

    /* Unlock Data memory */
    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    /* Read a byte at a specified address */

    rx_flag = FLASH_ReadByte(0x4000);
  
  //////////////////////////////////////////////////////


  while (1)
  {
    
      Delay(500000);
      
      GPIOA->ODR ^= (uint8_t)GPIO_PIN_3;

      parser_data();    //!!!!parsing!!!!

  }//end while
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


#if debugmode
void debugmsg(volatile uint8_t * debugbuf)
{
  Tx_485;
    while(*debugbuf) 
    {
      UART1_SendData8(*debugbuf++);
      while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    }
    
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  while ((UART1->SR & UART1_FLAG_TC) == RESET);
  Rx_485;
}

void debugchar(uint8_t debugchar)
{
  Tx_485;

  UART1_SendData8(debugchar);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  while ((UART1->SR & UART1_FLAG_TC) == RESET);
  Rx_485;
}

#endif  


void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

static void CLK_Config(void)
{
    ErrorStatus status = FALSE;

    CLK_DeInit();

    /* Configure the Fcpu to DIV1*/
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    
    /* Configure the HSI prescaler to the optimal value */
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);

    status = CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

}
void BufInit(uint8_t * buf)
{
  int j = 0;

  for(j = 0;j<RW_len;j++){buf[j]=0;} 
    RWCnt = 0;
    while ((UART1->SR & UART1_FLAG_TC) == RESET);
  
}


void Write_data(uint8_t * buf)
{
    int i=0;
    Tx_485;
    for(i=0;i<RWCnt;i++){
      UART1_SendData8(buf[i]);
      while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    }
    UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
    
    BufInit(buf);
    //for(int j = 0;j<RW_len;j++){buf[j]=0;}
    while ((UART1->SR & UART1_FLAG_TC) == RESET);
    Rx_485;
}


uint8_t parser_data(void)
{
    uint8_t buf[RW_len]={0x00,};
    unsigned int length = 0;
    uint8_t checksum = 0;
    uint8_t temp = 0;
    uint8_t tmp = 0;
    int i =0;
    if(RWBuf[0] == 0x02)
    {
      memcpy(buf, RWBuf, RW_len);
      
      length = buf[1];
      
      for(i=1;i<=length-2;i++)//xor 
      {
        checksum ^= buf[i];
      }
      
      if(buf[length] != checksum)//checksum error and init
      {
        UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
        BufInit(buf);
        BufInit(RWBuf);
        return 0;
      }
      

      //A8=동작체크, A9=구동 값 확인, 10,11,12,20,21,22,30,31,50,51= 모션 확인
        temp = buf[3];

        switch(temp){ //parsing
          case 0xA4 : //fw update
                      ////////////////eeprom write test smlee///////////////

              /* Define FLASH programming time */
            FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

            /* Unlock Data memory */
            FLASH_Unlock(FLASH_MEMTYPE_DATA);

            /* Read a byte at a specified address */
            add = 0x4000;
            val = 0x01;

            /* Program complement value (of previous read byte) at previous address + 1 */
            FLASH_EraseByte(add);
            FLASH_ProgramByte(add, val);
            //Reset process must be added.!!!!!!!!!
                return tmp;
                
            case 0xA8 : //motor status
                Write_data(RWBuf); 
                return 2;

            case 0xA9 : //motor status
                Write_data(RWBuf); 
                return 3;
                  
            case 0xA1 : //motor control
                //ID=buf[4];
                //Position_H=buf[5];
                //Position_L=buf[6];
                //Velocity_H=buf[7];
                //Velocity_L=buf[8];
                Write_data(RWBuf); 
                return 5;
 
        }

    }
    BufInit(RWBuf);

    return 0;    // other error
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/