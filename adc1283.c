/**
 * @file      adc1283.c
 * @prefix    ADC1283
 * @author    Simone Ruffini [simone.ruffini@squadracorse.com 
 *                            simone.ruffini.work@gmail.com]
 * @author    Federico Carbone [federico.carbone@squadracorse.com]
 * @date      Mon Jul  3 03:13:44 PM CEST 2023
 *
 * @brief     ST adc1283 comunication interface source file
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 (see LICENSE)
 */

/* Includes ------------------------------------------------------------------*/
#include "adc1283.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ADC1283_MAX_SAMPLE_RATE_KSPS        (200U)
#define ADC1283_MIN_SAMPLE_RATE_KSPS        (50U)
#define ADC1283_CONV_HOLD_TIME_SCLK_CYCLES  (13U)
#define ADC1283_ACQUSITION_TIME_SCLK_CYCLES (3U)
#define ADC1283_THRGHPT_TIME_SCLK_CYCLES    (ADC1283_CONV_HOLD_TIME_SCLK_CYCLES + ADC1283_ACQUSITION_TIME_SCLK_CYCLES)

#define ADC1283_DYN_RANGE_V (ADC1283_AVCC) /*!< ADC1283 dynamic range in volts */
#define ADC1283_RES_STEPS   (4096U)        /*!< ADC1283 Resolution steps (12bit) */

#define ADC1283_LSB \
    ((float)ADC1283_DYN_RANGE_V / ADC1283_RES_STEPS) /*!< ADC1283 Least significant bit voltage over step */

/* Private types -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#define ADC_RAW_TO_PHYS(_ADC_RAW_VAL_, _ADC_LSB_) (_ADC_RAW_VAL_ * _ADC_LSB_)

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void _ADC1238_serialize_tx_payload(enum ADC1283_CHNL_t channel, uint16_t *pCtrl_reg);
void _ADC1238_deserialize_rx_payload(uint16_t tx_payload, uint16_t *pConv_data);
void _ADC1283_cs_enable(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin);
void _ADC1283_cs_disable(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

HAL_StatusTypeDef ADC1283_conv_channel_raw(struct ADC1283_Handle_t *hadc1283,
                                        enum ADC1283_CHNL_t channel,
                                        uint16_t *pConv) {
    HAL_StatusTypeDef ret_code = HAL_OK;

    uint16_t tx_payload = (0U);
    uint16_t rx_payload = (0U);

    _ADC1238_serialize_tx_payload(channel, &tx_payload);

    _ADC1283_cs_enable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    // HAL_SPI_TransmitReceive sends payloads that have a Size multiple of a byte
    ret_code = HAL_SPI_TransmitReceive(hadc1283->hspi,
                                       (uint8_t *)&tx_payload,
                                       (uint8_t *)&rx_payload,
                                       (ADC1283_THRGHPT_TIME_SCLK_CYCLES / 8U),
                                       ADC1283_SPI_TIMOUT_MS);

    _ADC1283_cs_disable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    _ADC1238_deserialize_rx_payload(rx_payload, pConv);

    return ret_code;
}

HAL_StatusTypeDef ADC1283_conv_channel(struct ADC1283_Handle_t *hadc1283, enum ADC1283_CHNL_t channel, float *pConv) {
    HAL_StatusTypeDef ret_code = HAL_OK;
    uint16_t conv_raw          = (0U);
    ret_code                   = ADC1283_conv_channel_raw(hadc1283, channel, &conv_raw);
    if (ret_code != HAL_OK)
        return ret_code;
    *pConv = ADC_RAW_TO_PHYS(conv_raw, ADC1283_LSB);
    return ret_code;
}

/* Private functions ---------------------------------------------------------*/

void _ADC1238_serialize_tx_payload(enum ADC1283_CHNL_t channel, uint16_t *pCtrl_reg){
    assert_param(pCtrl_reg);
    // Address stored from bit 3 to 6 (counting from 0)
    *pCtrl_reg = (0U | (0b111U & (channel << 3)));
}
void _ADC1238_deserialize_rx_payload(uint16_t tx_payload, uint16_t *pConv_data) {
    assert_param(pConv_data);
    // reverse for endianness/MSB first and use first 12 bits
    *pConv_data = (0U | ((tx_payload & 0xFFU) << 8U) | ((tx_payload >> 8U) & 0x0FU));
}

// chip select active low
void _ADC1283_cs_enable(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void _adc1283_cs_disable(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}
