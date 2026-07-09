/*****************************************************************************
 * Copyright (c) 2024 Renesas Electronics Corporation
 * All Rights Reserved.
 * 
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *****************************************************************************/

/**
 * @file    zmod4510_config_ulp_o3.h
 * @brief   This is the configuration for ZMOD4510 module - ulp_o3 library
 * @version 5.0.0
 * @author  Renesas Electronics Corporation
 */

#ifndef _ZMOD4510_CONFIG_ULP_O3_H_
#define _ZMOD4510_CONFIG_ULP_O3_H_

#include <stdio.h>
#include "zmod4xxx_types.h"

#define INIT        0
#define MEASUREMENT 1

/**********************************/
/* < Define product ID > */
#define ZMOD4510_PID 0x6320

/**********************************/
/* < Define I2C slase address > */
#define ZMOD4510_I2C_ADDR 0x33

/**********************************/
/* < Define product data length > */
#define ZMOD4510_PROD_DATA_LEN 10

/*************************************/
/* < Define ADC result data length > */
#define ZMOD4510_ADC_DATA_LEN 18

// time between samples
#define ZMOD4510_ULP_O3_SAMPLE_TIME (2000U)

#define ZMOD4XXX_H_ADDR 0x40
#define ZMOD4XXX_D_ADDR 0x50
#define ZMOD4XXX_M_ADDR 0x60
#define ZMOD4XXX_S_ADDR 0x68

uint8_t data_set_4510_init[] = {
                                0x00, 0x50,
                                0x00, 0x28, 0xC3, 0xE3,
                                0x00, 0x00, 0x80, 0x40};

uint8_t data_set_4510_ulp_o3[] = {
                                0x00, 0x50, 0xFE, 0x70,
                                0x00, 0x10,
                                0x23, 0x03,
                                0x00, 0x00, 0x06, 0x41,
                                0x06, 0x41, 0x06, 0x41,
                                0x06, 0x41, 0x06, 0x41,
                                0x06, 0x41, 0x06, 0x41,
                                0x86, 0x41
                                };

zmod4xxx_conf zmod_ulp_o3_sensor_cfg[] = {
    [INIT] = {
        .start = 0x80,
        .h = { .addr = ZMOD4XXX_H_ADDR, .len = 2, .data_buf = &data_set_4510_init[0]},
        .d = { .addr = ZMOD4XXX_D_ADDR, .len = 2, .data_buf = &data_set_4510_init[2]},
        .m = { .addr = ZMOD4XXX_M_ADDR, .len = 2, .data_buf = &data_set_4510_init[4]},
        .s = { .addr = ZMOD4XXX_S_ADDR, .len = 4, .data_buf = &data_set_4510_init[6]},
        .r = { .addr = 0x97, .len = 4},
    },

    [MEASUREMENT] = {
        .start = 0x80,
        .h = {.addr = ZMOD4XXX_H_ADDR, .len = 4, .data_buf = &data_set_4510_ulp_o3[0]},
        .d = {.addr = ZMOD4XXX_D_ADDR, .len = 2, .data_buf = &data_set_4510_ulp_o3[4]},
        .m = {.addr = ZMOD4XXX_M_ADDR, .len = 2, .data_buf = &data_set_4510_ulp_o3[6]},
        .s = {.addr = ZMOD4XXX_S_ADDR, .len = 18, .data_buf = &data_set_4510_ulp_o3[8]},
        .r = {.addr = 0x97, .len = 18},
        .prod_data_len = ZMOD4510_PROD_DATA_LEN,
    },
};

#endif //_ZMOD4510_CONFIG_ULP_O3_H_
