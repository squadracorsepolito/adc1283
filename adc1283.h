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
#define ADC1283_MAX_SAMPLE_RATE_KSPS        (200U)
#define ADC1283_MIN_SAMPLE_RATE_KSPS        (50U)
#define ADC1283_CONV_HOLD_TIME_SCLK_CYCLES  (13U)
#define ADC1283_ACQUSITION_TIME_SCLK_CYCLES (3U)
#define ADC1283_THRGHPT_TIME_SCLK_CYCLES    (_ADC1283_CONV_HOLD_TIME_SCLK_CYCLES + _ADC1283_ACQUSITION_TIME_SCLK_CYCLES)

#ifndef ADC1283_AVCC
#define ADC1283_AVCC (3.3f)  // 3.3V Analog Voltage Reference in volts
#endif

#ifndef ADC1283_SPI_TIMOUT_MS
#define ADC1283_SPI_TIMOUT_MS (50U) // Timeout of a signle conversion (depends on spi bus speed)
#endif

#ifndef ADC1283_MAX_CONSEQ_CONVERSIONS
#define ADC1283_MAX_CONSEQ_CONVERSIONS (8U) // Maximum number of conversions for ADC1283_conv_channels and ADC1283_conv_cannels_raw
#endif
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

HAL_StatusTypeDef ADC1283_conv_channel_raw(struct ADC1283_Handle_t *hadc1283,
                                           enum ADC1283_CHNL_t channel,
                                           uint16_t *pConv);

HAL_StatusTypeDef ADC1283_conv_channel(struct ADC1283_Handle_t *hadc1283, 
                                       enum ADC1283_CHNL_t channel, 
                                       float *pConv);

HAL_StatusTypeDef ADC1283_conv_channels_raw(struct ADC1283_Handle_t *hadc1283,
                                            uint8_t num_channels,
                                            enum ADC1283_CHNL_t channels[num_channels],
                                            uint16_t conv_arr[num_channels]);

HAL_StatusTypeDef ADC1283_conv_channels(struct ADC1283_Handle_t *hadc1283,
                                        uint8_t num_channels,
                                        enum ADC1283_CHNL_t channels[num_channels],
                                        float conv_arr[num_channels]);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif

