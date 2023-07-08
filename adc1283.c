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

#include "logger_wrapper.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define _ADC1283_DYN_RANGE_V (ADC1283_AVCC) /*!< ADC1283 dynamic range in volts */
#define _ADC1283_RES_STEPS   (4096U)        /*!< ADC1283 Resolution steps (12bit) */

#define _ADC1283_LSB \
    ((float)_ADC1283_DYN_RANGE_V / _ADC1283_RES_STEPS) /*!< ADC1283 Least significant bit voltage over step */

/* Private types -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#define ADC_RAW_TO_PHYS(_ADC_RAW_VAL_, _ADC_LSB_) (_ADC_RAW_VAL_ * _ADC_LSB_)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void _ADC1283_cs_enable(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin);
void _ADC1283_cs_disable(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void _ADC1283_tx_serialize(uint8_t num_channels,
                           enum ADC1283_CHNL_t channels[num_channels],
                           uint16_t tx_payload[num_channels + 1]);
void _ADC1283_rx_deserialize(uint8_t num_conv, uint16_t rx_payload[num_conv + 1], uint16_t conv_data[num_conv]);

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef ADC1283_conv_channels_raw(struct ADC1283_Handle_t *hadc1283,
                                            uint8_t num_channels,
                                            enum ADC1283_CHNL_t channels[num_channels],
                                            uint16_t conv_arr[num_channels]) {
    assert_param(num_channels <= ADC1283_MAX_CONSEQ_CONVERSIONS);

    HAL_StatusTypeDef ret_code = HAL_OK;
    uint16_t tx_rx_buf[ADC1283_MAX_CONSEQ_CONVERSIONS + 1];

    _ADC1283_tx_serialize(num_channels, channels, tx_rx_buf);

    _ADC1283_cs_enable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    // HAL_SPI_TransmitReceive sends payloads that have a Size multiple of a byte
    ret_code = HAL_SPI_TransmitReceive(hadc1283->hspi,
                                       (uint8_t *)&tx_rx_buf,
                                       (uint8_t *)&tx_rx_buf,
                                       sizeof(tx_rx_buf),
                                       ADC1283_SPI_TIMOUT_MS * num_channels);

    _ADC1283_cs_disable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    if (ret_code != HAL_OK)
        return ret_code;

    _ADC1283_rx_deserialize(num_channels, tx_rx_buf, conv_arr);

    return HAL_OK;
}

HAL_StatusTypeDef ADC1283_conv_channels(struct ADC1283_Handle_t *hadc1283,
                                        uint8_t num_channels,
                                        enum ADC1283_CHNL_t channels[num_channels],
                                        float conv_arr[num_channels]) {
    assert_param(num_channels <= ADC1283_MAX_CONSEQ_CONVERSIONS);

    HAL_StatusTypeDef ret_code = HAL_OK;
    uint16_t conv_raw[ADC1283_MAX_CONSEQ_CONVERSIONS];
    ret_code = ADC1283_conv_channels_raw(hadc1283, num_channels, channels, conv_raw);
    if (ret_code != HAL_OK)
        return ret_code;
    for (uint8_t i = 0; i < num_channels; i++) {
        conv_arr[i] = ADC_RAW_TO_PHYS(conv_raw[i], _ADC1283_LSB);
    }
    return HAL_OK;
}
HAL_StatusTypeDef ADC1283_conv_channel_raw(struct ADC1283_Handle_t *hadc1283,
                                           enum ADC1283_CHNL_t channel,
                                           uint16_t *pConv) {
    return ADC1283_conv_channels_raw(hadc1283, 1U, &channel, pConv);
}
HAL_StatusTypeDef ADC1283_conv_channel(struct ADC1283_Handle_t *hadc1283, enum ADC1283_CHNL_t channel, float *pConv) {
    return ADC1283_conv_channels(hadc1283, 1U, &channel, pConv);
}


/* Private functions ---------------------------------------------------------*/
void _ADC1283_cs_enable(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void _ADC1283_cs_disable(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}

void _ADC1283_tx_serialize(uint8_t num_channels,
                           enum ADC1283_CHNL_t channels[num_channels],
                           uint16_t tx_payload[num_channels + 1]) {
    assert_param(num_channels > 0);
    assert_param(channels);
    assert_param(tx_payload);

    for (int i = 0; i < num_channels; i++) {
        tx_payload[i * 2] = 0U | ((0b111 & channels[i]) << 3U);  // serialize as per datasheet
    }
    // initialize padding
    tx_payload[num_channels] = (0U);
}

void _ADC1283_rx_deserialize(uint8_t num_conv, uint16_t rx_payload[num_conv + 1], uint16_t conv_data[num_conv]) {
    assert_param(num_conv > 0);
    assert_param(rx_payload);
    assert_param(conv_data);

    for (int i = 0; i < num_conv; i++) {
        // rx_payload[i+1] because rx_payload[0] will always containt a non
        // requested (default) conversion value
        conv_data[i] = (0U | ((rx_payload[i + 1] & 0x0FU) << 8U) | ((rx_payload[i + 1] >> 8U) & 0xFFU));
    }
}








void _ADC1283_serialize_tx_payload(enum ADC1283_CHNL_t channel, uint16_t *pCtrl_reg) {
    assert_param(pCtrl_reg);
    // Address stored from bit 3 to 6 (counting from 0)
    *pCtrl_reg = (0U | ((0b111U & channel) << 3U));
}

void _ADC1283_padd_tx_payload(uint16_t pTx_payload, uint32_t *pTx_padded_payload) {
    *pTx_padded_payload = (0U) | pTx_payload;
}

void _ADC1283_deserialize_rx_payload(uint16_t rx_payload, uint16_t *pConv_data) {
    assert_param(pConv_data);
    // reverse for endianness/MSB first and use first 12 bits
    *pConv_data = (0U | ((rx_payload & 0x0FU) << 8U) | ((rx_payload >> 8U) & 0xFFU));
}
void _ADC1283_unpadd_rx_payload(uint32_t pRx_padded_payload, uint16_t *pRx_payload) {
    *pRx_payload = (uint16_t)(pRx_padded_payload >> 16U);
}
HAL_StatusTypeDef ADC1283_conv_channel_raw_old(struct ADC1283_Handle_t *hadc1283,
                                               enum ADC1283_CHNL_t channel,
                                               uint16_t *pConv) {
    HAL_StatusTypeDef ret_code = HAL_OK;

    uint16_t tx_payload        = (0U);
    uint32_t tx_padded_payload = (0U);
    uint16_t rx_payload        = (0U);
    uint32_t rx_padded_payload = (0U);

    _ADC1283_serialize_tx_payload(channel, &tx_payload);

    _ADC1283_padd_tx_payload(tx_payload, &tx_padded_payload);
    logger_log(LOGGER_ERROR,
               "tx_padded: %u [%x %x %x %x]",
               tx_padded_payload,
               (tx_padded_payload >> 24) & 0xFF,
               (tx_padded_payload >> 16) & 0xFF,
               (tx_padded_payload >> 8) & 0xFF,
               (tx_padded_payload)&0xFF);
    _ADC1283_cs_enable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    // HAL_SPI_TransmitReceive sends payloads that have a Size multiple of a byte
    ret_code = HAL_SPI_TransmitReceive(hadc1283->hspi,
                                       (uint8_t *)&tx_padded_payload,
                                       (uint8_t *)&rx_padded_payload,
                                       //(ADC1283_THRGHPT_TIME_SCLK_CYCLES / 8U),
                                       4U,
                                       ADC1283_SPI_TIMOUT_MS * 4);

    _ADC1283_cs_disable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    logger_log(LOGGER_ERROR,
               "rx_padded: %u [%x %x %x %x]",
               rx_padded_payload,
               (rx_padded_payload >> 24) & 0xFF,
               (rx_padded_payload >> 16) & 0xFF,
               (rx_padded_payload >> 8) & 0xFF,
               (rx_padded_payload)&0xFF);

    _ADC1283_unpadd_rx_payload(rx_padded_payload, &rx_payload);
    logger_log(LOGGER_ERROR, "rx_payload: %u [%x %x]", rx_payload, (rx_payload >> 8) & 0xFF, (rx_payload)&0xFF);

    _ADC1283_deserialize_rx_payload(rx_payload, pConv);

#if 0
    uint8_t cmd[4];
    uint16_t rcv = 0;
    cmd[1] = cmd[2] = cmd[3] = 0xFF;
    cmd[0]                   = channel << 3;

    _ADC1283_cs_enable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);
     HAL_SPI_TransmitReceive(hadc1283->hspi, cmd, cmd, 4, 100);
    _ADC1283_cs_disable(hadc1283->hspi, hadc1283->cs_gpio_port, hadc1283->cs_gpio_pin);

    rcv |= (uint16_t)cmd[2] << 8;
    rcv |= cmd[3];

    *pConv = rcv;
#endif

    return ret_code;
}

HAL_StatusTypeDef ADC1283_conv_channel_old(struct ADC1283_Handle_t *hadc1283,
                                           enum ADC1283_CHNL_t channel,
                                           float *pConv) {
    HAL_StatusTypeDef ret_code = HAL_OK;
    uint16_t conv_raw          = (0U);
    ret_code                   = ADC1283_conv_channel_raw(hadc1283, channel, &conv_raw);
    if (ret_code != HAL_OK)
        return ret_code;
    *pConv = ADC_RAW_TO_PHYS(conv_raw, _ADC1283_LSB);
    return ret_code;
}

