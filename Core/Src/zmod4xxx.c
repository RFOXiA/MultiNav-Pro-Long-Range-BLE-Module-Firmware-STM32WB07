/*****************************************************************************
 * Copyright (c) 2024 Renesas Electronics Corporation
 * All Rights Reserved.
 * 
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *****************************************************************************/

/**
 * @file   zmod4xxx.c
 * @brief  zmod4xxx-API functions
 * @version 2.7.1
 * @author Renesas Electronics Corporation
 */

#include "zmod4xxx.h"

zmod4xxx_err zmod4xxx_read_status(zmod4xxx_dev_t *dev, uint8_t *status)
{
    int8_t ret;
    uint8_t st;

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_STATUS, &st, 1);
    if (0 != ret) {
        return ERROR_I2C;
    }
    *status = st;
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_check_error_event(zmod4xxx_dev_t *dev)
{
    int8_t ret;
    uint8_t data_buf;

    ret = dev->read(dev->i2c_addr, 0xB7, &data_buf, 1);
    if (ret) {
        return ERROR_I2C;
    }

    if (0 != data_buf) {
        if (STATUS_POR_EVENT_MASK & data_buf) {
            return ERROR_POR_EVENT;
        } else if (STATUS_ACCESS_CONFLICT_MASK & data_buf) {
            return ERROR_ACCESS_CONFLICT;
        }
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_null_ptr_check(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret;

    if ((dev->read == NULL) || (dev->write == NULL) ||
        (dev->delay_ms == NULL)) {
        ret = ERROR_NULL_PTR;
    } else {
        ret = ZMOD4XXX_OK;
    }
    return ret;
}

zmod4xxx_err zmod4xxx_read_sensor_info(zmod4xxx_dev_t *dev)
{
    int8_t i2c_ret;
    zmod4xxx_err api_ret;
    uint8_t status = 0;
    uint8_t data_buf[ZMOD4XXX_LEN_PID];
    uint16_t product_id;
    uint8_t cmd = 0;
    uint16_t i = 0;

    api_ret = zmod4xxx_null_ptr_check(dev);
    if (api_ret) {
        return api_ret;
    }

    do {
        i2c_ret = dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &cmd, 1);
        if (i2c_ret) {
            return ERROR_I2C;
        }
        api_ret = zmod4xxx_read_status(dev, &status);
        if (api_ret) {
            return api_ret;
        }
        i++;
        dev->delay_ms(200);
    } while ((0x00 != (status & 0x80)) && (i < 1000));

    if (1000 <= i) {
        return ERROR_GAS_TIMEOUT;
    }

    i2c_ret =
        dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_PID, data_buf, ZMOD4XXX_LEN_PID);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    product_id = ((data_buf[0] * 256) + data_buf[1]);

    if (dev->pid != product_id) {
        return ERROR_SENSOR_UNSUPPORTED;
    }

    i2c_ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_CONF, dev->config,
                        ZMOD4XXX_LEN_CONF);
    if (i2c_ret) {
        return ERROR_I2C;
    }

    i2c_ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_PROD_DATA, dev->prod_data,
                        dev->meas_conf->prod_data_len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_tracking_number(zmod4xxx_dev_t *dev,
                                           uint8_t *track_num)
{
    int8_t ret;

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_TRACKING, track_num,
                    ZMOD4XXX_LEN_TRACKING);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_calc_factor(zmod4xxx_conf *conf, uint8_t *hsp,
                                  uint8_t *config)
{
    int16_t hsp_temp[HSP_MAX];
    float hspf;
    uint8_t i;

    for (i = 0; i < conf->h.len; i = i + 2) {
        hsp_temp[i / 2] =
            ((conf->h.data_buf[i] << 8) + conf->h.data_buf[i + 1]);
        hspf = (-((float)config[2] * 256.0F + config[3]) *
                ((config[4] + 640.0F) * (config[5] + hsp_temp[i / 2]) -
                 512000.0F)) /
               12288000.0F;

        hsp[i] = (uint8_t)((uint16_t)hspf >> 8);
        hsp[i + 1] = (uint8_t)((uint16_t)hspf & 0x00FF);
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_init_sensor(zmod4xxx_dev_t *dev)
{
    int8_t i2c_ret;
    zmod4xxx_err api_ret;
    uint8_t hsp[HSP_MAX * 2];
    uint8_t data_r[RSLT_MAX];
    uint8_t zmod4xxx_status;

    i2c_ret = dev->read(dev->i2c_addr, 0xB7, data_r, 1);
    if (i2c_ret) {
        return ERROR_I2C;
    }

    api_ret = zmod4xxx_calc_factor(dev->init_conf, hsp, dev->config);
    if (api_ret) {
        return api_ret;
    }

    i2c_ret = dev->write(dev->i2c_addr, dev->init_conf->h.addr, hsp,
                         dev->init_conf->h.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->init_conf->d.addr,
                         dev->init_conf->d.data_buf, dev->init_conf->d.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->init_conf->m.addr,
                         dev->init_conf->m.data_buf, dev->init_conf->m.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->init_conf->s.addr,
                         dev->init_conf->s.data_buf, dev->init_conf->s.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }

    i2c_ret =
        dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &dev->init_conf->start, 1);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    do {
        api_ret = zmod4xxx_read_status(dev, &zmod4xxx_status);
        if (api_ret) {
            return api_ret;
        }
        dev->delay_ms(50);
    } while (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK);

    i2c_ret = dev->read(dev->i2c_addr, dev->init_conf->r.addr, data_r,
                        dev->init_conf->r.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }

    dev->mox_lr = (uint16_t)(data_r[0] << 8) | data_r[1];
    dev->mox_er = (uint16_t)(data_r[2] << 8) | data_r[3];
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_init_measurement(zmod4xxx_dev_t *dev)
{
    int8_t i2c_ret;
    zmod4xxx_err api_ret;
    uint8_t hsp[HSP_MAX * 2];

    api_ret = zmod4xxx_calc_factor(dev->meas_conf, hsp, dev->config);
    if (api_ret) {
        return api_ret;
    }

    i2c_ret = dev->write(dev->i2c_addr, dev->meas_conf->h.addr, hsp,
                         dev->meas_conf->h.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->meas_conf->d.addr,
                         dev->meas_conf->d.data_buf, dev->meas_conf->d.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->meas_conf->m.addr,
                         dev->meas_conf->m.data_buf, dev->meas_conf->m.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    i2c_ret = dev->write(dev->i2c_addr, dev->meas_conf->s.addr,
                         dev->meas_conf->s.data_buf, dev->meas_conf->s.len);
    if (i2c_ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_start_measurement_at(zmod4xxx_dev_t *dev, uint8_t  step)
{
    int8_t ret;

    ret =
        dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &step, 1);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_start_measurement(zmod4xxx_dev_t *dev)
{
    return zmod4xxx_start_measurement_at ( dev, dev->meas_conf->start );
}

zmod4xxx_err zmod4xxx_read_adc_result(zmod4xxx_dev_t *dev, uint8_t *adc_result)
{
    int8_t ret;

    ret = dev->read(dev->i2c_addr, dev->meas_conf->r.addr, adc_result,
                    dev->meas_conf->r.len);
    if (ret) {
        return ERROR_I2C;
    }

    return ZMOD4XXX_OK;
}

float zmod4xxx_calc_single_rmox (zmod4xxx_dev_t *dev, uint8_t *adc_result ) {
    uint16_t adc_value;
    float rmox;

    adc_value =
        (uint16_t)(((uint16_t)(*adc_result)) << 8) | (*(adc_result + 1));
    if (adc_value <= dev->mox_lr) {
        rmox = 1e2F;
    } else if (dev->mox_er <= adc_value) {
        rmox = 1e12F;
    } else {
        rmox = dev->config[0] * 1e3F * (adc_value - dev->mox_lr) /
               (dev->mox_er - adc_value);
    }
    // calculated rmox can still be out of range!
    if (1e12F < rmox) {
        rmox = 1e12F;
    } else if (1e2F > rmox) {
        rmox = 1e2F;
    }
    return rmox;
}

zmod4xxx_err zmod4xxx_calc_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox)
{

    uint8_t i;
    float *p = rmox;

    for (i = 0; i < dev->meas_conf->r.len; i = i + 2) {
        *p++ = zmod4xxx_calc_single_rmox ( dev, adc_result + i );
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_prepare_sensor(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret;

    ret = zmod4xxx_init_sensor(dev);
    if (ret) {
        return ret;
    }
    dev->delay_ms(50);
    ret = zmod4xxx_init_measurement(dev);
    if (ret) {
        return ret;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox)
{
    zmod4xxx_err ret;
    ret = zmod4xxx_read_adc_result(dev, adc_result);
    if (ret) {
        return ret;
    }
    dev->delay_ms(50);
    ret = zmod4xxx_calc_rmox(dev, adc_result, rmox);
    if (ret) {
        return ret;
    }
    return ZMOD4XXX_OK;
}
