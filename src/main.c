#include "stm32f30x.h"
#include "stm32f3_discovery.h"

/* Private variables ---------------------------------------------------------*/
  RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;
__IO uint32_t UserButtonPressed = 0;
__IO uint32_t i =0;
__IO uint16_t Capture;

uint32_t CaptureCounter = 0;
uint16_t IC4Value1 = 0;
uint16_t IC4Value2 = 0;

/* Private function prototypes -----------------------------------------------*/
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

static void DAC_Config(void);
static void COMP_Config(void);
static void TIM_Config(void);

#pragma mark - IRQ handlers

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

void EXTI0_IRQHandler(void)
{
    if ((EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)&&(STM_EVAL_PBGetState(BUTTON_USER) != RESET))
    {
        /* Delay */
        for(i=0; i<0x7FFFF; i++);
        
        /* Wait for SEL button to be pressed  */
        while(STM_EVAL_PBGetState(BUTTON_USER) != RESET);
        /* Delay */
        for(i=0; i<0x7FFFF; i++);
        UserButtonPressed++;
        
        if (UserButtonPressed > 0x2) {
            UserButtonPressed = 0x0;
        }
        
        /* Clear the EXTI line pending bit */
        EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
    }
}

/**
 * @brief  This function handles TIM2 global interrupt.
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {
        if (0 == CaptureCounter)  {
            IC4Value1 = TIM_GetCapture4(TIM2);
            CaptureCounter = 1;
        }
        else if (1 == CaptureCounter) {
            CaptureCounter = 0;
            IC4Value2 = TIM_GetCapture4(TIM2);
            
            if (IC4Value2 > IC4Value1) {
                Capture = (IC4Value2 - IC4Value1) - 1;
            }
            else {
                Capture = ((0xFFFF - IC4Value1) + IC4Value2) - 1;
            }
//            DisplayActive = 1;
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    }
}



#pragma mark - Configure Peripherals


/**
 * @brief  Configures the DAC channel 1 with output buffer enabled.
 * @param  None
 * @retval None
 */
static void DAC_Config(void)
{
    /* Init Structure definition */
    DAC_InitTypeDef  dacInit;
    
    /* DAC clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    
    /* Fill DAC InitStructure */
    dacInit.DAC_Trigger = DAC_Trigger_None;
    dacInit.DAC_WaveGeneration = DAC_WaveGeneration_None;
    dacInit.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    dacInit.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &dacInit);
    
    /* Enable DAC Channel1 */
    DAC_Cmd(DAC_Channel_1, ENABLE);
    
    /* Set DAC Channel1 DHR register: DAC_OUT1 = (3.3 * 2000) / 4095 ~ 1.61 V */
    DAC_SetChannel1Data(DAC_Align_12b_R, 2000);
}


/**
 * @brief  Configures COMP1: DAC channel 1 to COMP1 inverting input
 *                           and COMP1 output to TIM2 IC4.
 * @param  None
 * @retval None
 */
static void COMP_Config(void)
{
    /* Init Structure definition */
    COMP_InitTypeDef compInit;
    GPIO_InitTypeDef gpioInit;
    
    /* GPIOA Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    
    /* Init GPIO Init Structure */
    GPIO_StructInit(&gpioInit);
    
    /* Configure PA1: PA1 is used as COMP1 non-inverting input */
    gpioInit.GPIO_Pin = GPIO_Pin_1;
    gpioInit.GPIO_Mode = GPIO_Mode_AN; /*!< GPIO Analog Mode */
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioInit);//assign to PA1
    
    /* COMP Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Init COMP init struct */
    COMP_StructInit(&compInit);
    
    /* COMP1 Init: PA1 is used as COMP1 non-inverting input */
    compInit.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;
    /* DAC1 output is as used COMP1 inverting input */
    compInit.COMP_InvertingInput = COMP_InvertingInput_DAC1;
    /* Redirect COMP1 output to TIM2 Input capture 4 */
    compInit.COMP_Output = COMP_Output_TIM2IC4;
    compInit.COMP_OutputPol = COMP_OutputPol_NonInverted;
    compInit.COMP_BlankingSrce = COMP_BlankingSrce_None;
    compInit.COMP_Hysteresis = COMP_Hysteresis_High;
    compInit.COMP_Mode = COMP_Mode_UltraLowPower;
    COMP_Init(COMP_Selection_COMP1, &compInit);
    
    /* Enable COMP1 */
    COMP_Cmd(COMP_Selection_COMP1, ENABLE);
}

/**
 * @brief  Configures TIM2 channel 4 in input capture (IC) mode (TIM2IC4)
 * @param  None
 * @retval None
 */
static void TIM_Config(void)
{
    /* Init Structure definition */
    TIM_ICInitTypeDef tim_ic_init;
    TIM_TimeBaseInitTypeDef tim_timebase;
    NVIC_InitTypeDef nvicInit;
    
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /* TIM2 Time base configuration */
    TIM_TimeBaseStructInit(&tim_timebase);
    tim_timebase.TIM_Prescaler = 0;
    tim_timebase.TIM_CounterMode = TIM_CounterMode_Up;
    tim_timebase.TIM_Period = 65535;
    tim_timebase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim_timebase);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    /* TIM2 Channel4 Input capture Mode configuration */
    TIM_ICStructInit(&tim_ic_init);
    tim_ic_init.TIM_Channel = TIM_Channel_4;
    /* TIM2 counter is captured at each transition detection: rising or falling edges (both edges) */
    tim_ic_init.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    tim_ic_init.TIM_ICSelection = TIM_ICSelection_DirectTI;
    tim_ic_init.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    tim_ic_init.TIM_ICFilter = 0;
    TIM_ICInit(TIM2, &tim_ic_init);/* assign TIM IC init to TIM2 */
    
    /* TIM2 IRQChannel enable */
    nvicInit.NVIC_IRQChannel = TIM2_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
    
    /* Enable capture interrupt */
    TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
    
    /* Enable the TIM2 counter */
    TIM_Cmd(TIM2, ENABLE);
    
    /* Reset the flags */
    TIM2->SR = 0;
}



/**
 * @brief  Configure the TIM IRQ Handler.
 * @param  None
 * @retval None
 */
//static void TIM_Config(void)
//{
//    GPIO_InitTypeDef gpioInit;
//
//    /* TIM2 clock enable */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//
//    /* GPIOA clock enable */
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//    /* TIM2_CH1 pin (PA.00) and TIM2_CH2 pin (PA.01) configuration */
//    gpioInit.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit); //assign to PA00 + PA01
//
//    /* Connect TIM pins to AF1 */
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);
//}

/* Private functions ---------------------------------------------------------*/
#pragma mark - Private functions

void clear_leds()
{
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED6);
    STM_EVAL_LEDOff(LED7);
    STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOff(LED10);
    STM_EVAL_LEDOff(LED8);
    STM_EVAL_LEDOff(LED9);
    STM_EVAL_LEDOff(LED5);
}


void leds_on()
{
    STM_EVAL_LEDOn(LED3);
    STM_EVAL_LEDOn(LED6);
    STM_EVAL_LEDOn(LED7);
    STM_EVAL_LEDOn(LED4);
    STM_EVAL_LEDOn(LED10);
    STM_EVAL_LEDOn(LED8);
    STM_EVAL_LEDOn(LED9);
    STM_EVAL_LEDOn(LED5);
}

void spin_leds(int gap)
{
    /* Toggle LD3 */
    STM_EVAL_LEDToggle(LED3);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD5 */
    STM_EVAL_LEDToggle(LED5);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD7 */
    STM_EVAL_LEDToggle(LED7);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD9 */
    STM_EVAL_LEDToggle(LED9);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD10 */
    STM_EVAL_LEDToggle(LED10);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD8 */
    STM_EVAL_LEDToggle(LED8);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD6 */
    STM_EVAL_LEDToggle(LED6);
    /* Insert 50 ms delay */
    Delay(gap);
    /* Toggle LD4 */
    STM_EVAL_LEDToggle(LED4);
    /* Insert 50 ms delay */
    Delay(gap);
}

void flash_leds(int gap)
{
    clear_leds();
    Delay(gap); /*500ms - half second*/
    
    leds_on();
    Delay(gap); /*500ms - half second*/
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{  
    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 200); // div 100 normally?

    /* Initialize LEDs and User Button available on STM32F3-Discovery board */
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    STM_EVAL_LEDInit(LED7);
    STM_EVAL_LEDInit(LED8);
    STM_EVAL_LEDInit(LED9);
    STM_EVAL_LEDInit(LED10);
    
    
    /* DAC Ch1 configuration */
    DAC_Config();
    
    /* COMP1 Configuration */
    COMP_Config();
    
    /* TIM2 Configuration in input capture mode */
    TIM_Config();
    
    


    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Configure the USB */
    //    Demo_USB();

    while (1) {
        /* Reset UserButton_Pressed variable */
        UserButtonPressed = 0x00;
        
        /* Waiting User Button is pressed */
        while (UserButtonPressed == 0x00) {
            spin_leds(5);
        }
        
        /* Infinite loop */
        while (UserButtonPressed == 0x01) {
            flash_leds(5);
        }
    }
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
