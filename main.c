/**
  *
  * Brandon Mouser & Tyler Evans
  * U0962682, u1313811
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);
// use this value for writeToSADD
volatile int prevAddr = 0;
volatile int x = 0;
volatile int y = 0;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
//

/*
 KEY Commands for terminal
 */
void sendData(int loc, int addr, int data)
{
    //GPIOC->ODR |= (1<<8);
      //  HAL_Delay(300);
        /*
         I set this code so it would be done in the while loop
         since it is constantly being written
         I think a function would be better here.
         */
        //setting debug LEDs off
        //TODO here
        //while loop flag varibles.
        int isSent = 0;
        
        /*
         CR2 portion of setup since this is critical
         */
        //slave address in loc in SADD 7:1 at bits 7 to 1
        I2C2->CR2 |= (loc<<1);
        // number byte in NBYTES 7:0 at bits 23:16 and we want a value of 1
        I2C2->CR2 |=  (0x2<<16);
        
        
        //configures read/write bits in I2C
        //RD_WRN it is bit 10
        //0 sets it to be write
        //1 sets it to be read
        I2C2->CR2 &= ~(1<<10);
        
        //to get going one needs to start ha ha
        // start bit is in CR2 at bit 13
        I2C2->CR2 |= (1<<13);
        
        //waiting while NACKF and TXIS are empty
        // this is in the ISR of the I2C
        // NACKF is bit 4
        // TXIS is bit 1
        while(!isSent){
          
            //this will keep it busy hopefully.
            //this checks if NACKF is set
            //this condtional means that the address is not being sent.
            if(((I2C2->ISR & (1<<4)))){
                //if there is an issue it will send RED
                // I will use red whenever there is an issue going forward.
                GPIOC->ODR ^= (1<<6);
                HAL_Delay(100);
            }
            //this is to check TXIS
            if(I2C2->ISR & (1<<1)){
                //if there is an not an issue it will send green
                // green means go.
                isSent = 1;
                //GPIOC->ODR |= (1<<8);
                //HAL_Delay(100);
            }
            //GPIOC->ODR ^= (1<<8);
        }
        //CHECK 1
        //GPIOC->ODR |= (1<<9);
        //Writing Who am I into SADD Address
        //first clearing what was in there the device address
        //I2C2->CR2 &= ~(0x69<<1);
        //now setting it to be the address
        //sending to the WHO AM I
        I2C2->TXDR = addr;
        //to get going one needs to start ha ha
        // start bit is in CR2 at bit 13
        //I2C2->CR2 |= (1<<13);
        int isSent2 = 0;
        
        /*
         CR2 portion of setup since this is critical
         */
       
        //waiting while NACKF and TXIS are empty
        // this is in the ISR of the I2C
        // NACKF is bit 4
        // TXIS is bit 1
        //GPIOC->ODR |= (1<<8);
        while(!isSent2){
          
            //this will keep it busy hopefully.
            //this checks if NACKF is set
            //this condtional means that the address is not being sent.
            if(((I2C2->ISR & (1<<4)))){
                //if there is an issue it will send RED
                // I will use red whenever there is an issue going forward.
                GPIOC->ODR ^= (1<<6);
                HAL_Delay(100);
            }
            //this is to check TXIS
            if(I2C2->ISR & (1<<1)){
                //if there is an not an issue it will send green
                // green means go.
                isSent2 = 1;
                //GPIOC->ODR ^= (1<<9);
            }
            //GPIOC->ODR ^= (1<<8);
        }
        //Writing Who am I into SADD Address
        //first clearing what was in there the device address
        //I2C2->CR2 &= ~(0x69<<1);
        //now setting it to be the address
        //sending data to the addresss
        I2C2->TXDR = data;
        //wating while the transfer complete
        //This should be bit 6 in the ISR
        while(!(I2C2->ISR & (1<<6))){
            //waiting here
            //using Orange LED to see if happnening possible debug
        }
        //set the stop bit 14 in CR2
        // 1 sets the stop bit
        // there is no need to set to 0
        //becasue when start occurs it is 0.
        I2C2->CR2 |= (1<<14);
        
    }

int readData(int loc, int addr){
    /*
     I set this code so it would be done in the while loop
     since it is constantly being written
     I think a function would be better here.
     */
    //setting debug LEDs off
    //TODO here
    //while loop flag varibles.
    int isSent = 0;
    
    /*
     CR2 portion of setup since this is critical
     */
    //slave address in 0x69 in SADD 7:1 at bits 7 to 1
    I2C2->CR2 |= (loc<<1);
    // number byte in NBYTES 7:0 at bits 23:16 and we want a value of 1
    I2C2->CR2 |=  (0x1<<16);
    
    
    //configures read/write bits in I2C
    //RD_WRN it is bit 10
    //0 sets it to be write
    //1 sets it to be read
    I2C2->CR2 &= ~(1<<10);
    
    //to get going one needs to start ha ha
    // start bit is in CR2 at bit 13
    I2C2->CR2 |= (1<<13);
    
    //waiting while NACKF and TXIS are empty
    // this is in the ISR of the I2C
    // NACKF is bit 4
    // TXIS is bit 1
    //GPIOC->ODR ^= (1<<6);
    while(!isSent){
      
        //this will keep it busy hopefully.
        //this checks if NACKF is set
        //this condtional means that the address is not being sent.
        if(((I2C2->ISR & (1<<4)))){
            //if there is an issue it will send RED
            // I will use red whenever there is an issue going forward.
            GPIOC->ODR ^= (1<<6);
            HAL_Delay(100);
        }
        //this is to check TXIS
        if(I2C2->ISR & (1<<1)){
            //if there is an not an issue it will send green
            // green means go.
            isSent = 1;
            //HAL_Delay(100);
            //GPIOC->ODR ^= (1<<9);
            //HAL_Delay(100);
        }
        //GPIOC->ODR ^= (1<<8);
        //HAL_Delay(100);
    }
    //Writing Who am I into SADD Address
    //first clearing what was in there the device address
    //I2C2->CR2 &= ~(0x69<<1);
    //now setting it to be the address
    //sending to the WHO AM I
    I2C2->TXDR = addr;
    
    //wating while the transfer complete
    //This should be bit 6 in the ISR
    while(!(I2C2->ISR & (1<<6))){
        //waiting here
        //using Orange LED to see if happnening possible debug
        
    }
    
    //turning on the blue
    
    /*
     NEXT part 5 reading instead of writting.
     */
    
    // 1<<3 | (1<<1) (1<<0)
    
    //setting LEDS off.
    //TODO here
    int isRev = 0;
    //slave address in 0x69 in SADD 7:1 at bits 7 to 1
    I2C2->CR2 |= (0x69<<1);
    // number byte in NBYTES 7:0 at bits 23:16 and we want a value of 1
    I2C2->CR2 |=  (0x1<<16);
    
    //configures read/write bits in I2C
    //RD_WRN it is bit 10
    //1 sets it to be read
    //0 sets it to be write
    I2C2->CR2 |= (1<<10);
    
    //to get going one needs to start ha ha
    // start bit is in CR2 at bit 13
    I2C2->CR2 |= (1<<13);
    
    //waiting while NACKF and TXIS are empty
    // this is in the ISR of the I2C
    // NACKF is bit 4
    // RXNE is bit 2
 
    while(!isRev){
        //this will keep it busy hopefully.
    
        //this checks if NACKF is set
        //this condtional means that the address is not being sent.
        if((I2C2->ISR & (1<<4))){
            //if there is an issue it will send RED
            // I will use red whenever there is an issue going forward.
            GPIOC->ODR ^= (1<<6);
            HAL_Delay(100);
        }
        //this is to check RXNE
        //RXNE
        if(I2C2->ISR & (1<<2)){
            //if there is an not an issue it will send green
            // green means go.
            isRev = 1;
        }
        
    }
    
    //Writing Who am I into SADD Address
    //first clearing what was in there the device address
    //now setting it to be the address
    //I2C2->CR2 |= (0xD3 <<1);
    
    //check what is contained in RXDR
    //RXDR is a unique address which can be read directly.
    int data = I2C2->RXDR;
 
    //wating while the transfer complete TC
    //TC (transfer complete) should be bit 6 in the ISR
    while(!(I2C2->ISR & (1<<6))){
        //waiting here
        //using Orange LED to see if happnening possible debug
        //GPIOC->ODR |= (1<<7);
    }
    
    //GPIOC->ODR ^= (1<<8);
    //set the stop bit 14 in CR2
    // 1 sets the stop bit
    // there is no need to set to 0
    //becasue when start occurs it is 0.
    I2C2->CR2 |= (1<<14);
    //HAL_Delay(200);
    //GPIOC->ODR ^= (1<<8);
    //HAL_Delay(200);
    return data;
}

//does one interaction.
void readWhoAmI(){
        /*
         I set this code so it would be done in the while loop
         since it is constantly being written
         I think a function would be better here.
         */
        //setting debug LEDs off
        //TODO here
        //while loop flag varibles.
        int isSent = 0;
        
        /*
         CR2 portion of setup since this is critical
         */
        //slave address in 0x69 in SADD 7:1 at bits 7 to 1
        I2C2->CR2 |= (0x69<<1);
        // number byte in NBYTES 7:0 at bits 23:16 and we want a value of 1
        I2C2->CR2 |=  (0x1<<16);
        
        
        //configures read/write bits in I2C
        //RD_WRN it is bit 10
        //0 sets it to be write
        //1 sets it to be read
        I2C2->CR2 &= ~(1<<10);
        
        //to get going one needs to start ha ha
        // start bit is in CR2 at bit 13
        I2C2->CR2 |= (1<<13);
        
        //waiting while NACKF and TXIS are empty
        // this is in the ISR of the I2C
        // NACKF is bit 4
        // TXIS is bit 1
    
        while(!isSent){
          
            //this will keep it busy hopefully.
            //this checks if NACKF is set
            //this condtional means that the address is not being sent.
//            if(((I2C2->ISR & (1<<4)))){
//                //if there is an issue it will send RED
//                // I will use red whenever there is an issue going forward.
//                GPIOC->ODR ^= (1<<6);
//                HAL_Delay(100);
//            }
            //this is to check TXIS
            if(I2C2->ISR & (1<<1)){
                //if there is an not an issue it will send green
                // green means go.
                isSent = 1;
                //HAL_Delay(100);
                //GPIOC->ODR ^= (1<<9);
                //HAL_Delay(100);
            }
            //GPIOC->ODR ^= (1<<8);
            //HAL_Delay(300);
        }
        //Writing Who am I into SADD Address
        //first clearing what was in there the device address
        //I2C2->CR2 &= ~(0x69<<1);
        //now setting it to be the address
        //sending to the WHO AM I
        I2C2->TXDR = 0x0F;
    //GPIOC->ODR |= (1<<6);
        //wating while the transfer complete
        //This should be bit 6 in the ISR
        while(!(I2C2->ISR & (1<<6))){
            //waiting here
            //using Orange LED to see if happnening possible debug
            //GPIOC->ODR ^= (1<<8);
            //HAL_Delay(100);
        }
        
        //turning on the blue
        
        /*
         NEXT part 5 reading instead of writting.
         */
        
        // 1<<3 | (1<<1) (1<<0)
        
        //setting LEDS off.
        //TODO here
        int isRev = 0;
        //slave address in 0x69 in SADD 7:1 at bits 7 to 1
        I2C2->CR2 |= (0x69<<1);
        // number byte in NBYTES 7:0 at bits 23:16 and we want a value of 1
        I2C2->CR2 |=  (0x1<<16);
        
        //configures read/write bits in I2C
        //RD_WRN it is bit 10
        //1 sets it to be read
        //0 sets it to be write
        I2C2->CR2 |= (1<<10);
        
        //to get going one needs to start ha ha
        // start bit is in CR2 at bit 13
        I2C2->CR2 |= (1<<13);
        
        //waiting while NACKF and TXIS are empty
        // this is in the ISR of the I2C
        // NACKF is bit 4
        // RXNE is bit 2
     
        while(!isRev){
            //this will keep it busy hopefully.
        
            //this checks if NACKF is set
            //this condtional means that the address is not being sent.
            if((I2C2->ISR & (1<<4))){
                //if there is an issue it will send RED
                // I will use red whenever there is an issue going forward.
                GPIOC->ODR ^= (1<<6);
                HAL_Delay(100);
            }
            //this is to check RXNE
            //RXNE
            if(I2C2->ISR & (1<<2)){
                //if there is an not an issue it will send green
                // green means go.
                GPIOC->ODR ^= (1<<9);
                HAL_Delay(100);
                isRev = 1;
            }
        }
        
        //Writing Who am I into SADD Address
        //first clearing what was in there the device address
        //now setting it to be the address
        //I2C2->CR2 |= (0xD3 <<1);
        
     
        //wating while the transfer complete TC
        //TC (transfer complete) should be bit 6 in the ISR
        while(!(I2C2->ISR & (1<<6))){
            //waiting here
            //using Orange LED to see if happnening possible debug
            //GPIOC->ODR |= (1<<7);
        }
        //check what is contained in RXDR
        //RXDR is a unique address which can be read directly.
        if(((I2C2->RXDR) == 0xD3)){
            //sets the Blue light if working
            GPIOC->ODR ^= (1<<7);

        }
        
        //GPIOC->ODR ^= (1<<8);
        //set the stop bit 14 in CR2
        // 1 sets the stop bit
        // there is no need to set to 0
        //becasue when start occurs it is 0.
        I2C2->CR2 |= (1<<14);
    //GPIOC->ODR ^= (1<<8);
 }
    



/*
 useful things
 bit follows for the LED
 RED = 6
 BLUE = 7
 ORANGE = 8
 GREEN = 9
 */

int main(void)
{
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    
    
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // done to find the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enabling clock on gpio b
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // enabling clock on I2C2
    
    //sets everything to zero in the pins
    // GPIOC is the GPIO_x where the pin is located.
    GPIOC->MODER &= 0; // sets the mode
    GPIOC->OTYPER &= 0; // sets what type
    GPIOC->OSPEEDR &= 0; // sets the speed
    GPIOC->PUPDR &= 0; // sets the pulldown/pullup resitor
    
    //sets all the values in modder to the correct pin into input mode.
    // the value is 01 for output mode.
    //               PC6        PC7       PC8       PC9
    GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
    
    
    /*
     Below here is the setup section of the I2C
     */
    // enable the gpio pins for alternate function
    //the GPIO ports will be PB11, PB13
    //alternate function mode is 10 in each part
    //               PB11      PB13
    GPIOB->MODER |= (1<<23) | (1<<27);
    // set the open drain configration in OTYPER 1 is open drain config.
    GPIOB->OTYPER |= (1<<11) | (1<<13);
    //set the alternate function in AFR for I2C
    
    /*
     FOR AFR to access low [0]
     FOR AFR to access high [1]
     */
    
    //for PB11 it will be I2C_SDA on AF1 0010
    GPIOB->AFR[1] |= (1<<12);
    //for PB13 it will be IC2_SCL on AF5 1010
    GPIOB->AFR[1] |= (1<<22) | (1<<20);
    //PB14 shold be output mode and push pull output type.
    GPIOB->MODER |= (1<<28);
    //otyper is that way by default so nada steps here
    
    //sets PC0 to output mode
    GPIOC->MODER |= (1<<0);
    
    //OTYPER defaults to the mode. so no need
    
    
    //both pins want to be high
    //PC0 High
    GPIOC->ODR |= (1<<0);
    //PB14 high
    GPIOB->ODR |= (1<<14);
    //don't foget to set the clock. DID IT
    
    
    /*
     The next section is setting up the
     Transaction
     */
    //configure the bus timer for TIMINGR
    //enable the bits in the CR1 regiester that your are using.
    //I am going to set it up from the table column dec.
    //sets 1 in the Prescaler
    // THIS MUST BE CONFIGURED WHILE PE = 0!
    I2C2->TIMINGR |= (1<<28);
    //sets 0x13 to the SCLL in bits 0:7
    I2C2->TIMINGR |= (0x13<<0);
    //SCHL is set to 0xF bits 15:8 so shifted 8
    I2C2->TIMINGR |= (0xF<<8);
    //SDADEL needs 0x2 in bits 19:16
    I2C2->TIMINGR |= (0x2 << 16);
    //SCLDEL needs 0x4 in bits 23:20
    I2C2->TIMINGR |= (0x4<<20);
    // now enabling the bit after Key word after.
    // PE must be 1
    I2C2->CR1 |= (1<<0);
    
    //here this code is to check the who am I
    readWhoAmI();
    HAL_Delay(1000);
    //clears all of the LEDS
    GPIOC->ODR &= ~(1<<7);
    GPIOC->ODR &= ~(1<<9);
    GPIOC->ODR &= ~(1<<8);
    GPIOC->ODR &= ~(1<<6);
    // init contorol regeisters
    // device reg 0x69, reg cr1 0x20, and 3 1 and 0 need to be enabled
    sendData(0x69, 0x20, 0x0B);
    //GPIOC->ODR |= (1<<8);
    //HAL_Delay(100);
    //sendData(0x69, 0x20, 0xB0);
    //HAL_Delay(1000);
    //GPIOC->ODR |= (1<<9);
    while(1) {
        //checking if a delay is all that is needed for 5.1 to do at the same time.
        //readWhoAmI();
                //here I am reading the values in x and y
                HAL_Delay(100);
                x = ((readData(0x69, 0x28) << 5) | readData(0x69, 0x29));
                //GPIOC->ODR |= (1<<7);
                //HAL_Delay(10);
                //y = ((readData(0x69, 0x28) << 5) | readData(0x69, 0x29));
                //GPIOC->ODR ^= (1<<8);
                // Q1 checks if in that range
                if((x >3000)){
                    //turns on green LED
                    //GPIOC->ODR |= (1<<9);
                    GPIOC->ODR &= ~(1<<8);
                    HAL_Delay(25);
                }
                //Q2
                if((x <3000)){
                    //turns on orange LED
                    GPIOC->ODR |= (1<<8);
                    GPIOC->ODR &= ~(1<<9);
                    HAL_Delay(25);
                }
                //Q3
                if((y <3000)){
                    //turns on red LED
                    GPIOC->ODR |= (1<<6);
                    GPIOC->ODR &= ~(1<<7);
                    HAL_Delay(25);
                }
                //Q4
                if ((y > 3000)){
                    //turns on blue LED
                    GPIOC->ODR |= (1<<7);
                    GPIOC->ODR &= ~(1<<6);
                    HAL_Delay(25);
                }
        
            }
        
    }

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
