
#include "stm32f30x_comp.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"

#include "stm32f3_discovery.h"


//sum of TIM_IT_CC1 .. TIM_IT_CC4
#define ALL_TIM_CHANNELS  ((uint16_t)(TIM_IT_CC1 + TIM_IT_CC2 + TIM_IT_CC3 + TIM_IT_CC4))

#define NUM_CHANNELS    4
#define PULSES_PER_PACKET    24
#define CROSSINGS_PER_PACKET (2*PULSES_PER_PACKET)

//microseconds to disallow further input on a particular channel
#define BLANKING_INTERVAL   2500

#define SPIN_DELAY(x) { for(i=0; i<(x); i++) {} }

#define TIME_TO_USEC(x)  ( (uint32_t)(((uint64_t) (x) * 1000000) / ((uint32_t)SystemCoreClock)) )
#define TIME_BY_ADDING_USEC(x,t) ( (uint32_t) ((x) + (uint64_t)((t) * (uint32_t)SystemCoreClock ) / 1000000 ) )

#define METERS_FROM_TICKS(x) ((float) (4.7*(x)/100000) )

#define ALL_CHANNELS_HAVE_DATA  (TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4)

#define MAP_LED_NW      0x80
#define MAP_LED_W       0x40
#define MAP_LED_SW      0x20
#define MAP_LED_S       0x10
#define MAP_LED_SE      0x08
#define MAP_LED_E       0x04
#define MAP_LED_NE      0x02
#define MAP_LED_N       0x01

#pragma mark - Private variables 

RCC_ClocksTypeDef RCC_Clocks;



typedef struct ChannelData {
    //the next index at which a capture time should be stored
    uint16_t writeIndex;
    //rising and falling edge times
    uint32_t times[CROSSINGS_PER_PACKET];
    uint32_t averagePacketCenter;
    uint32_t embargoExitTime;
} ChannelData_t;


__IO uint32_t __timingDelay = 0;
__IO uint32_t __userButtonPressed = 0;
__IO uint32_t i = 0;
__IO uint32_t __dataAvailable = 0;
__IO uint32_t __renderCount = 0;


__IO ChannelData_t __chanData[NUM_CHANNELS];

#pragma mark - Private function prototypes

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

static void DAC_Config(void);
static void COMP_Config(void);
static void TIM_Config(void);

static void resetCompCaptures(void);
static void clear_leds(void);
static void spin_leds(int gap);
static void flash_leds(int gap);
static void illuminate_led_count(int count);
static void illuminate_four_way_leds(int index);
static void illuminate_compass_leds(uint16_t bitmap);

static void watch_input_captures(void);
static void config_one_comparator(GPIO_TypeDef* gpioPort, uint32_t gpioPin, uint32_t compSelection, uint32_t compOutput);
static void config_one_timer_channel(TIM_TypeDef* timer, uint16_t channel);
static void handle_one_capture_channel(TIM_TypeDef* timer, uint16_t channel);

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
        SPIN_DELAY(0x7FFFF);
        
        /* Wait for SEL button to be pressed  */
        while(STM_EVAL_PBGetState(BUTTON_USER) != RESET) {}
        
        SPIN_DELAY(0x7FFFF);
        
        __userButtonPressed++;
        
        if (__userButtonPressed > 0x02) {
            __userButtonPressed = 0x00;
        }
        
        /* Clear the EXTI line pending bit */
        EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
    }
}


/**
 If the interrupt status flag is set for this timer and channel,
 record the value of the channel
 
 @param timer eg TIM2
 @param channel TIM_IT pending bit ie TIM_IT_CC1 - TIM_IT_CC4
 */
void handle_one_capture_channel(TIM_TypeDef* timer, uint16_t channel)
{
    uint32_t captureVal = 0;
    uint32_t pulseWidth = 0;
    uint16_t idx = 0;
    
    if (SET != TIM_GetITStatus(timer, channel) ) {
        //the interrupt for this channel hasn't happened yet
        return;
    }
    
    switch(channel) {
        case TIM_IT_CC4:
            captureVal = TIM_GetCapture4(timer);
            idx = 3;
            break;
            
        case TIM_IT_CC3:
            captureVal = TIM_GetCapture3(timer);
            idx = 2;
            break;
            
        case TIM_IT_CC2:
            captureVal = TIM_GetCapture2(timer);
            idx = 1;
            break;
            
        case TIM_IT_CC1:
            captureVal = TIM_GetCapture1(timer);
            idx = 0;
            break;
    }
    


    ChannelData_t* pChanData = (ChannelData_t*)(&__chanData[idx]);

    if ((pChanData->writeIndex < CROSSINGS_PER_PACKET) &&
        (pChanData->embargoExitTime < captureVal))  {
        
        pChanData->times[pChanData->writeIndex] = captureVal;
        if ((pChanData->writeIndex % 2) == 1) {
            //check the pulse width
            pulseWidth = pChanData->times[pChanData->writeIndex] - pChanData->times[pChanData->writeIndex - 1];
            if (pulseWidth < 0) {
                illuminate_compass_leds(MAP_LED_W);
                //reset the clock...the timer has wrapped
                resetCompCaptures();
                TIM_ClearITPendingBit(timer, channel);
                return;
            }
        }
        pChanData->writeIndex++;

        if (pChanData->writeIndex >= CROSSINGS_PER_PACKET) {
            pChanData->averagePacketCenter = 0;
            for (int k = 0; k < CROSSINGS_PER_PACKET; k++) {
                pChanData->averagePacketCenter += pChanData->times[k];
            }
            pChanData->averagePacketCenter = pChanData->averagePacketCenter / CROSSINGS_PER_PACKET;
            pChanData->embargoExitTime = TIME_BY_ADDING_USEC(pChanData->averagePacketCenter, BLANKING_INTERVAL);

            //this channel now has a whole packet of pulses read
            __dataAvailable |= channel;
        }
    }

    
    
    TIM_ClearITPendingBit(timer, channel);

}

/**
 * @brief  This function handles TIM2 global interrupt.
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler(void)
{
    handle_one_capture_channel(TIM2, TIM_IT_CC1);
    handle_one_capture_channel(TIM2, TIM_IT_CC2);
    handle_one_capture_channel(TIM2, TIM_IT_CC3);
    handle_one_capture_channel(TIM2, TIM_IT_CC4);
}



#pragma mark - Configure Peripherals


/*
 Configure a comparator to take input from a pin and output to a specific TIM channel
 
 @param gpioPin The pin number to assign, ie GPIO_Pin_1 - n
 @param compSelection The comparator to select, ie COMP_Selection_COMP1 - COMP_Selection_COMP7
 @param compOutput The redirect ie COMP_Output_TIM2IC1 - COMP_Output_TIM2IC4
 */
void config_one_comparator(GPIO_TypeDef* gpioPort, uint32_t gpioPin, uint32_t compSelection, uint32_t compOutput)
{
    COMP_InitTypeDef compInit;
    GPIO_InitTypeDef gpioInit;
    
    /* Init GPIO Init Structure */
    GPIO_StructInit(&gpioInit);
    /* Configure GPIO pin to be used as comparator non-inverting input */
    gpioInit.GPIO_Pin = gpioPin;
    gpioInit.GPIO_Mode = GPIO_Mode_AN; /*!< GPIO Analog Mode */
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioInit.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(gpioPort, &gpioInit);
    
    /* Clear compInit struct */
    COMP_StructInit(&compInit);
    /* GPIO pin is used as selected comparator non-inverting input */
    switch (compSelection) {
        case COMP_Selection_COMP1:
            compInit.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;
            break;
        case COMP_Selection_COMP5:
        case COMP_Selection_COMP6:
        case COMP_Selection_COMP7:
            compInit.COMP_NonInvertingInput = COMP_NonInvertingInput_IO2;
            break;
    }

    
    /* DAC1 output is as used COMP1 inverting input, providing threshold */
    compInit.COMP_InvertingInput = COMP_InvertingInput_DAC1;
    /* Redirect selected comparator output */
    compInit.COMP_Output = compOutput;
    compInit.COMP_OutputPol = COMP_OutputPol_NonInverted;
    compInit.COMP_BlankingSrce = COMP_BlankingSrce_None;
    compInit.COMP_Hysteresis = COMP_Hysteresis_High;
    compInit.COMP_Mode = COMP_Mode_HighSpeed; //COMP_Mode_UltraLowPower;
    COMP_Init(compSelection, &compInit);
    
    /* Enable selected comparator */
    COMP_Cmd(compSelection, ENABLE);

}

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
    
    /* Fill DAC init struct */
    dacInit.DAC_Trigger = DAC_Trigger_None;
    dacInit.DAC_WaveGeneration = DAC_WaveGeneration_None;
    dacInit.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    dacInit.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &dacInit);
    
    /* Enable DAC Channel1 */
    DAC_Cmd(DAC_Channel_1, ENABLE);
    
    /*
    Set DAC Channel1 DHR register: DAC_OUT1
    n = (Vref / 3.3 V) * 4095
    eg   n = (2 V  / 3.3 V) * 4095 = 2482
     n = (1.65 V / 3.3 V) * 4095 = 2048
     n = (1.0 V / 3.3 V) * 4095 = 1241
    */
    DAC_SetChannel1Data(DAC_Align_12b_R, 1241);
}


/**
 * @brief  Configures COMP1: DAC channel 1 to COMP1 inverting input
 *                           and COMP1 output to TIM2 IC4.
 * @param  None
 * @retval None
 */
static void COMP_Config(void)
{
    /* GPIOA Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /* GPIOB Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    /* GPIOC Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    /* COMP Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    //configure pins to route captures to TIM2IC1-TIM2IC4
    //Need to follow rules specified in "Comparator input/outputs summary", Table 52 
    
    
    //Only COMP5 can output to TIM2_IC1, gets noninverting input from PB13
    config_one_comparator(GPIOB, GPIO_Pin_13, COMP_Selection_COMP5, COMP_Output_TIM2IC1);// PB13 --> TIM2_IC1
    
    //Only COMP6 can output to TIM2_IC2, gets noninverting input from PB11
    config_one_comparator(GPIOB, GPIO_Pin_11, COMP_Selection_COMP6, COMP_Output_TIM2IC2);// PB11 --> TIM2_IC2
    
    //Only COMP7 can output to TIM2_IC3, gets noninverting input from PC1
    config_one_comparator(GPIOC, GPIO_Pin_1,  COMP_Selection_COMP7, COMP_Output_TIM2IC3);// PC1 --> TIM2_IC3

    //Only COMP1 or COMP2 can output to TIM2_IC4, COMP1 gets noninverting input from PA1
    config_one_comparator(GPIOA, GPIO_Pin_1,  COMP_Selection_COMP1, COMP_Output_TIM2IC4);// PA1 --> TIM2_IC4 (TIM_IT_CC4)
    
    


}


/**
 @param timer TIM1-n
 @param channel eg TIM_Channel_4
 */
void config_one_timer_channel(TIM_TypeDef* timer, uint16_t channel)
{
    TIM_ICInitTypeDef tim_ic_init;

    /* timer channel Input capture Mode configuration */
    TIM_ICStructInit(&tim_ic_init);
    tim_ic_init.TIM_Channel = channel;
    /* timer counter is captured at each transition detection: rising or falling edges (both edges) */
    tim_ic_init.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    tim_ic_init.TIM_ICSelection = TIM_ICSelection_DirectTI;
    tim_ic_init.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    tim_ic_init.TIM_ICFilter = 0;
    TIM_ICInit(timer, &tim_ic_init);
}

/**
 * @brief  Configures TIM2 channel 4 in input capture (IC) mode (TIM2IC4)
 * @param  None
 * @retval None
 */
static void TIM_Config(void)
{
    /* Init Structure definition */
    TIM_TimeBaseInitTypeDef tim_timebase;
    NVIC_InitTypeDef nvicInit;
    
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* TIM2 Time base configuration */
    TIM_TimeBaseStructInit(&tim_timebase);
    tim_timebase.TIM_Prescaler = 0;
    tim_timebase.TIM_CounterMode = TIM_CounterMode_Up;
    tim_timebase.TIM_Period = 0xFFFFFFFF; //TODO double-check this...Should be OK to reload with 32-bit value?
    tim_timebase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim_timebase);
    
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    //config_one_timer_channel(uint32_t timer, uint32_t channel)
    config_one_timer_channel(TIM2, TIM_Channel_1);
    config_one_timer_channel(TIM2, TIM_Channel_2);
    config_one_timer_channel(TIM2, TIM_Channel_3);
    config_one_timer_channel(TIM2, TIM_Channel_4);
    
    /* TIM2 IRQChannel enable */
    nvicInit.NVIC_IRQChannel = TIM2_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
    
    /* Enable capture interrupt */
    TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
    
    /* Enable the TIM2 counter */
    TIM_Cmd(TIM2, ENABLE);
    
    /* Reset the flags */
    TIM2->SR = 0;
}


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
    illuminate_led_count(8);
}


void illuminate_led_count(int count)
{
    clear_leds();
    
    switch (count) {
        case 8:
            STM_EVAL_LEDOn(LED4);
        case 7:
            STM_EVAL_LEDOn(LED6);
        case 6:
            STM_EVAL_LEDOn(LED8);
        case 5:
            STM_EVAL_LEDOn(LED10);
        case 4:
            STM_EVAL_LEDOn(LED9);
        case 3:
            STM_EVAL_LEDOn(LED7);
        case 2:
            STM_EVAL_LEDOn(LED5);
        case 1:
            STM_EVAL_LEDOn(LED3);
            break;
    }
}

void illuminate_compass_leds(uint16_t bitmap)
{
    clear_leds();
    
    if (bitmap & MAP_LED_NW) {
        STM_EVAL_LEDOn(LED4);
    }
    if (bitmap & MAP_LED_W) {
        STM_EVAL_LEDOn(LED6);
    }
    if (bitmap & MAP_LED_SW) {
        STM_EVAL_LEDOn(LED8);
    }
    if (bitmap & MAP_LED_S) {
        STM_EVAL_LEDOn(LED10);
    }
    
    if (bitmap & MAP_LED_SE) {
        STM_EVAL_LEDOn(LED9);
    }
    if (bitmap & MAP_LED_E) {
        STM_EVAL_LEDOn(LED7);
    }
    if (bitmap & MAP_LED_NE) {
        STM_EVAL_LEDOn(LED5);
    }
    if (bitmap & MAP_LED_N) {
        STM_EVAL_LEDOn(LED3);
    }
    
}

void illuminate_four_way_leds(int index)
{
    clear_leds();

    switch (index) {
        case 8:
            STM_EVAL_LEDOn(LED4);
            STM_EVAL_LEDOn(LED8);
            STM_EVAL_LEDOn(LED9);
            STM_EVAL_LEDOn(LED5);
            break;
            
        case 3:
            STM_EVAL_LEDOn(LED6);
            break;
        case 2:
            STM_EVAL_LEDOn(LED10);
            break;
        case 1:
            STM_EVAL_LEDOn(LED7);
            break;
        case 0:
            STM_EVAL_LEDOn(LED3);
            break;
    }
    
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


void renderDirectionForRaceResults(const uint16_t* sortedIndices)
{
    uint16_t delayTime = 5;
    uint16_t bitmap = 0;
    uint32_t firstIdx = sortedIndices[0];
    uint16_t secondIdx = sortedIndices[1];
    uint32_t firstTime = __chanData[firstIdx].averagePacketCenter;
    uint32_t secondTime = __chanData[secondIdx].averagePacketCenter;
    uint32_t winnerGap = (secondTime - firstTime); //raw ticks difference
    
    //convert to microseconds
//    uint32_t winnerGap = TIME_TO_USEC( (secondTime - firstTime) );
    
//    if (firstTime > 65536) {
//        bitmap |= MAP_LED_N;
//    }
//    else {
//        bitmap |= MAP_LED_S;
//    }
    
    switch (firstIdx) {
        case 0:
            bitmap |= MAP_LED_NW;
            break;
        case 1:
            bitmap |= MAP_LED_NE;
            break;
        case 2:
            bitmap |= MAP_LED_SE;
            break;
        case 3:
            bitmap |= MAP_LED_SW;
            break;
    }
    
    //1800 is about one wavelength diff @ 72MHz ?
    //so if the difference is less than this threshold, that
    //means we can barely tell which sensor is closer
//    if (winnerGap < 112) {
//        
//        switch (secondIdx) {
//            case 0:
//                bitmap |= MAP_LED_NW;
//                break;
//            case 1:
//                bitmap |= MAP_LED_NE;
//                break;
//            case 2:
//                bitmap |= MAP_LED_SE;
//                break;
//            case 3:
//                bitmap |= MAP_LED_SW;
//                break;
//        }
//        
////        delayTime = 10;
//    }

    
    
    illuminate_compass_leds(bitmap);
    Delay(delayTime);
}


void push_value_onto_stack(uint16_t val,  uint16_t* sortedIndices)
{
    for (int i = (NUM_CHANNELS -1); i > 0; i--) {
        //shuffle values to the next slot
        sortedIndices[i] = sortedIndices[i-1];
    }
    sortedIndices[0] = val;
}


void getArrivalOrder(uint16_t* sortedIndices)
{
    uint32_t minValue = 0x0FFFFFFF;

    for (int idx  = 0; idx < NUM_CHANNELS; idx++) {
        uint32_t chanAvgPulse = __chanData[idx].averagePacketCenter;
        if (chanAvgPulse > 0 && (chanAvgPulse < minValue)) {
            minValue = chanAvgPulse;
            push_value_onto_stack(idx,sortedIndices);
        }
    }
}

void watch_input_captures()
{

    //wait for data available on all channels
    if (0 != __dataAvailable) {
        uint16_t bitmap = 0;
        uint16_t activeDetectors = 0;
        __renderCount++;

        /* Compute the pulse width in us */
//        __measuredPulse = (uint32_t)(((uint64_t) minPulse * 1000000) / ((uint32_t)SystemCoreClock));

        if (__dataAvailable & TIM_IT_CC1) {
            bitmap += MAP_LED_NW;
            activeDetectors++;
        }
        if (__dataAvailable & TIM_IT_CC2) {
            bitmap += MAP_LED_NE;
            activeDetectors++;
        }
        if (__dataAvailable & TIM_IT_CC3) {
            bitmap += MAP_LED_SE;
            activeDetectors++;
        }
        if (__dataAvailable & TIM_IT_CC4) {
            bitmap += MAP_LED_SW;
            activeDetectors++;
        }
        
        illuminate_compass_leds(bitmap);
        
        //periodically display the directionality
        
        if (__renderCount > 5) {
            if (activeDetectors  > 1) {
                uint16_t orderOfArrivals[NUM_CHANNELS];
                getArrivalOrder(orderOfArrivals);
                renderDirectionForRaceResults(orderOfArrivals);
            }
            
            resetCompCaptures();
            __renderCount = 0;
        }

    }
    else {
        clear_leds();
        __renderCount = 0;
//        illuminate_four_way_leds(8);
//        Delay(5);
    }
    
    
}

/**
 Sure would be nice to have memset here!
 */
void resetCompCaptures(void)
{
    __dataAvailable = 0;

    for (int idx  = 0; idx < NUM_CHANNELS; idx++) {
        __chanData[idx].writeIndex = 0;
        __chanData[idx].averagePacketCenter = 0;
        __chanData[idx].embargoExitTime = 0;
    }
    
}

void setup(void)
{

    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 800); // div 100 normally?
    
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
    
    /* COMP Configuration */
    COMP_Config();
    
    /* TIM2 Configuration in input capture mode */
    TIM_Config();
    
    
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
    
    /* Configure the USB */
    //    Demo_USB();
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{  

    setup();
    
    //user button selects one of multiple modes that run forever
    while (1) {
        __userButtonPressed = 0x00;
        
        while (0x00 == __userButtonPressed) {
            watch_input_captures();
        }

        while (0x01 == __userButtonPressed) {
            leds_on();
//            spin_leds(5);
        }
        
        while (0x02 == __userButtonPressed) {
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
  __timingDelay = nTime;

  while(__timingDelay != 0);
}

/**
  * @brief  Decrements the __timingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (__timingDelay != 0x00)
  { 
    __timingDelay--;
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
      illuminate_compass_leds(MAP_LED_W);
  }
}
#endif
