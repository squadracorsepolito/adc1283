/**
 * @file      adc1283.h
 * @prefix    ADC1283
 * @author    Simone Ruffini [simone.ruffini@squadracorse.com 
 *                            simone.ruffini.work@gmail.com]
 * @author    Federico Carb0ne [federico.carbone@squadracorse.com]
 * @date      Mon Jul  3 03:13:44 PM CEST 2023
 *
 * @brief     ST adc1283 comunication interface
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 (see LICENSE)
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADC1283_H_
#define _ADC1283_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

enum ADC1283_CHNL_t {
    ADC1283_CHNL0 = 0b000,
    ADC1283_CHNL1 = 0b001,
    ADC1283_CHNL2 = 0b010,
    ADC1283_CHNL3 = 0b011,
    ADC1283_CHNL4 = 0b100,
    ADC1283_CHNL5 = 0b101,
    ADC1283_CHNL6 = 0b110,
    ADC1283_CHNL7 = 0b111
};

struct ADC1283_Handle_t {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_gpio_port;
    uint16_t cs_gpio_pin;
};

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

// Standard operative values
#ifndef ADC1283_AVCC
#define ADC1283_AVCC (3.3f)  // 3.3V Analog Voltage Reference in volts
#endif

#ifndef ADC1283_SPI_TIMOUT_MS
#define ADC1283_SPI_TIMOUT_MS (50U)
#endif
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/

HAL_StatusTypeDef ADC1283_conv_channel_raw(struct ADC1283_Handle_t *hadc1283,
                                           enum ADC1283_CHNL_t channel,
                                           uint16_t *pConv);

HAL_StatusTypeDef ADC1283_conv_channel(struct ADC1283_Handle_t *hadc1283, 
                                       enum ADC1283_CHNL_t channel, 
                                       float *pConv);
#endif

