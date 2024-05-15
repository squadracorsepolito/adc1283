/**
 * @file      adc1283.h
 * @prefix    ADC1283
 * @author    Simone Ruffini [simone.ruffini@squadracorse.com 
 *                            simone.ruffini.work@gmail.com]
 * @author    Federico Carb0ne [federico.carbone@squadracorse.com]
 * @date      Mon Jul  3 03:13:44 PM CEST 2023
 *
 * @brief     ST adc1283 comunication interface
 * @note      CPOL=high, CLKPHA = 2 edge
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 (see LICENSE)
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADC1283_H_
#define _ADC1283_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

enum ADC1283_Chnl {
    ADC1283_Chnl0 = 0b000,
    ADC1283_Chnl1 = 0b001,
    ADC1283_Chnl2 = 0b010,
    ADC1283_Chnl3 = 0b011,
    ADC1283_Chnl4 = 0b100,
    ADC1283_Chnl5 = 0b101,
    ADC1283_Chnl6 = 0b110,
    ADC1283_Chnl7 = 0b111
};

struct ADC1283_Handle {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_gpio_port;
    uint16_t cs_gpio_pin;
};

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
#define ADC1283_RES_BITS (12U) /*!< ADC1283 Resolution in bits */

#define ADC1283_CHANNELS_NUM (8U) /*!< ADC1283 number of channels */

#define ADC1283_MAX_SAMPLE_RATE_KSPS        (200U)
#define ADC1283_MIN_SAMPLE_RATE_KSPS        (50U)
#define ADC1283_CONV_HOLD_TIME_SCLK_CYCLES  (13U)
#define ADC1283_ACQUSITION_TIME_SCLK_CYCLES (3U)
#define ADC1283_THRGHPT_TIME_SCLK_CYCLES    (ADC1283_CONV_HOLD_TIME_SCLK_CYCLES + ADC1283_ACQUSITION_TIME_SCLK_CYCLES)

#define ADC1283_RES_STEPS (1U << (ADC1283_RES_BITS)) /*!< ADC1283 Resolution in LSB steps */

#define ADC1283_SCLK_MAX_FREQ_HZ (3200000U) /*!< ADC max cloxk frequency 3.2MHz */

#ifndef ADC1283_AVCC_V
#define ADC1283_AVCC_V (3.3f) /*!< Analog Voltage Reference in volts. Default is 3.3V */
#endif

#ifndef ADC1283_DYN_RANGE_V
#define ADC1283_DYN_RANGE_V (ADC1283_AVCC_V) /*!< Dynamic range of the ADC in volts. Default = AVCC-0V */
#endif

#ifndef ADC1283_SPI_TIMEOUT_mS /*!< Timeout in milliseconds for a signle conversion (depends on spi bus speed) */
#define ADC1283_SPI_TIMEOUT_mS (50U)
#endif

#ifndef ADC1283_LSB_V
#define ADC1283_LSB_V ((float)ADC1283_DYN_RANGE_V / ADC1283_RES_STEPS) /*!< ADC1283 Least significant bit voltage */
#endif

#ifndef ADC1283_MAX_CONSEQ_CONVERSIONS
#define ADC1283_MAX_CONSEQ_CONVERSIONS (ADC1283_CHANNELS_NUM) /*!< Maximum number of conversions for ADC1283_conv_channels()
                                                                   Default: the maximum number of channels of the adc (8)
                                                                   this value can be increased to do more conversions */
#endif

/* Exported macros -----------------------------------------------------------*/
#ifndef assert_param
#include <assert.h>
#define assert_param(expr) assert(expr)
#endif

/*
 * @brief Convert raw value of ADC1283 to a physical voltage using ADC1283_LSB_V
 * @note  This function is used by ADC1283_conv_channel() and ADC1283_conv_channels()
 * @param _ADC_RAW_VAL_ raw value in steps converted from the ADC
 * @return Floating point value in millivolts corresponding to _ADC_RAW_VAL_
 */
#define ADC1283_ADC_RAW_TO_PHYS(_ADC_RAW_VAL_) (_ADC_RAW_VAL_ * ADC1283_LSB_V)

/* Exported functions --------------------------------------------------------*/

/*
 * @brief Start a conversion for a single channel and get the result as a raw
 *        ADC value in LSB steps.
 * @note  This is a blocking function with a timeout of max @ref ADC1283_SPI_TIMEOUT_mS
 * @param hadc1283 pointer to @ref struct ADC1283_Handle 
 * @param channel channel to convert of type @ref enum ADC1283_Chnl
 * @param pConv pointer where to save the result of conversion
 * @return HAL_OK in case of correct conversion otherwise returns HAL_SPI_TransmitReceive status code
 */
HAL_StatusTypeDef ADC1283_conv_channel_raw(struct ADC1283_Handle *hadc1283, enum ADC1283_Chnl channel, uint16_t *pConv);

/*
 * @brief Start a conversion for a single channel and get the converted voltage level
 * @note  This is a blocking function with a timeout of max @ref ADC1283_SPI_TIMEOUT_mS
 * @note  The real voltage level is converted by using @ref ADC1283_ADC_RAW_TO_PHYS macro
 * @param hadc1283 pointer to @ref struct ADC1283_Handle 
 * @param channel channel to convert of type @ref enum ADC1283_Chnl
 * @param pConv pointer where to save the result of conversion
 * @return HAL_OK in case of correct conversion otherwise returns HAL_SPI_TransmitReceive status code
 */
HAL_StatusTypeDef ADC1283_conv_channel(struct ADC1283_Handle *hadc1283, enum ADC1283_Chnl channel, float *pConv);

/*
 * @brief Start a conversion for multiple channels and get the converted results 
 *        as raw ADC values in LSB steps.
 * @note  This is a blocking function with a timeout of max num_channels*@ref ADC1283_SPI_TIMEOUT_mS
 * @param hadc1283 pointer to @ref struct ADC1283_Handle 
 * @param num_channels number of channels to convert (limited by @ref ADC1283_MAX_CONSEQ_CONVERSIONS)
 * @param channels an array of length num_channels that contains the channels to be converted consecutively
 *                 The order of channels defines the order of conversions
 * @param conv_arr an array of length num_channels that will contain the result of conversion for each channel
 * @return HAL_OK in case of correct conversion otherwise returns HAL_SPI_TransmitReceive status code
 */
HAL_StatusTypeDef ADC1283_conv_channels_raw(struct ADC1283_Handle *hadc1283,
                                            uint8_t num_channels,
                                            enum ADC1283_Chnl channels[num_channels],
                                            uint16_t conv_arr[num_channels]);

/*
 * @brief Start a conversion for multiple channels and get the converted voltage levels
 * @note  This is a blocking function with a timeout of max num_channels*@ref ADC1283_SPI_TIMEOUT_mS
 * @note  The real voltage level is converted by using @ref ADC1283_ADC_RAW_TO_PHYS macro
 * @param hadc1283 pointer to @ref struct ADC1283_Handle 
 * @param num_channels number of channels to convert (limited by @ref ADC1283_MAX_CONSEQ_CONVERSIONS)
 * @param channels an array of length num_channels that contains the channels to be converted consecutively
 *                 The order of channels defines the order of conversions
 * @param conv_arr an array of length num_channels that will contain the result of conversion for each channel
 * @return HAL_OK in case of correct conversion otherwise returns HAL_SPI_TransmitReceive status code
 */
HAL_StatusTypeDef ADC1283_conv_channels(struct ADC1283_Handle *hadc1283,
                                        uint8_t num_channels,
                                        enum ADC1283_Chnl channels[num_channels],
                                        float conv_arr[num_channels]);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif
