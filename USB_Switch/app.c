/*
*	Header need to be done
*/

#include <stdio.h>

#include "main.h"
#include "Board_LED.h"                  /* ::Board Support:LED */
#include "Board_Buttons.h"              /* ::Board Support:Buttons */
#include "Board_ADC.h"                  /* ::Board Support:A/D Converter */

#include "RTE_Components.h"             /* Component selection */
#include "stm32l4xx_hal.h"              /* Keil::Device:STM32Cube HAL:Common */

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

// Main stack size must be multiple of 8 Bytes
#define APP_MAIN_STK_SZ (1024U)
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

static volatile uint32_t delay_val = 500U;

static osThreadId_t tid_thrADC;                /* Thread id of thread: ADC */
static osThreadId_t tid_thrLED;                /* Thread id of thread: LED */
static osThreadId_t tid_thrBUT;                /* Thread id of thread: BUT */

/*----------------------------------------------------------------------------
  thrADC: get ADC value and set delay
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrADC(void *argument) {

  (void)argument;

  for (;;) {
    ADC_StartConversion();
    osDelay(20U);                                              /* Wait */

    if (ADC_ConversionDone() == 0) {
      delay_val = (uint32_t)(ADC_GetValue() >> 4U) + 10U;      /* scale delay value */
    }
  }
}

/*----------------------------------------------------------------------------
  thrLED: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrLED(void *argument) {
  uint32_t led_max    = LED_GetCount();
  uint32_t led_num    = 0U;

  (void)argument;

  for (;;) {
    osThreadFlagsWait(0x0001U, osFlagsWaitAny ,osWaitForever);
    LED_On(led_num);                                           /* Turn specified LED on */
    osThreadFlagsWait(0x0001U, osFlagsWaitAny ,osWaitForever);
    LED_Off(led_num);                                          /* Turn specified LED off */

    led_num++;                                                 /* Change LED number */
    if (led_num >= led_max) {
      led_num = 0U;                                            /* Restart with first LED */
    }
  }

}

/*----------------------------------------------------------------------------
  thrBUT: check button state
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrBUT(void *argument) {
  uint32_t button_msk = (1U << Buttons_GetCount()) - 1U;

  (void)argument;

  for (;;) {
    osDelay(delay_val);                                        /* Wait */
    while (Buttons_GetState() & (button_msk));                 /* Wait while holding USER button */
    osThreadFlagsSet(tid_thrLED, 0x0001U);
  }

}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *argument) {

  (void)argument;

#ifdef RTE_Compiler_EventRecorder
  EventRecorderInitialize(0U, 1U);
  EventRecorderEnable (EventRecordError, 0xF0U, 0xF8U);     /* RTOS Events */
  EventRecorderEnable (EventRecordAll, 0xF2U, 0xF2U);       /* Thread Events */
#endif

  LED_Initialize();                                         /* initalize LEDs */
  Buttons_Initialize();                                     /* initalize Buttons */
  ADC_Initialize();                                         /* initialize ADC */

  tid_thrBUT = osThreadNew (thrBUT, NULL, NULL);            /* create BUT thread */
  tid_thrLED = osThreadNew (thrLED, NULL, NULL);            /* create LED thread */
  tid_thrADC = osThreadNew (thrADC, NULL, NULL);            /* create ADC thread */

  for (;;) {}
}
