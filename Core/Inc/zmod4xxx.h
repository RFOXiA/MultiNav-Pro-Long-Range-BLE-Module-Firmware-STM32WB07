/*****************************************************************************
 * Copyright (c) 2024 Renesas Electronics Corporation
 * All Rights Reserved.
 * 
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *****************************************************************************/

/**
 * @addtogroup zmod_api
 * @{
 * @file   zmod4xxx.h
 * @brief  zmod4xxx-API functions
 * @version 2.7.1
 * @author Renesas Electronics Corporation
 */

#ifndef _ZMOD4XXX_H
#define _ZMOD4XXX_H

#include "zmod4xxx_types.h"

#define ZMOD4XXX_ADDR_PID       (0x00)
#define ZMOD4XXX_ADDR_CONF      (0x20)
#define ZMOD4XXX_ADDR_PROD_DATA (0x26)
#define ZMOD4XXX_ADDR_CMD       (0x93)
#define ZMOD4XXX_ADDR_STATUS    (0x94)
#define ZMOD4XXX_ADDR_TRACKING  (0x3A)

#define ZMOD4XXX_LEN_PID      (2)
#define ZMOD4XXX_LEN_CONF     (6)
#define ZMOD4XXX_LEN_TRACKING (6)

#define HSP_MAX  (8)
#define RSLT_MAX (32)

#define STATUS_SEQUENCER_RUNNING_MASK   (0x80) /**< Sequencer is running */
#define STATUS_SLEEP_TIMER_ENABLED_MASK (0x40) /**< SleepTimer_enabled */
#define STATUS_ALARM_MASK               (0x20) /**< Alarm */
#define STATUS_LAST_SEQ_STEP_MASK       (0x1F) /**< Last executed sequencer step */
#define STATUS_POR_EVENT_MASK           (0x80) /**< POR_event */
#define STATUS_ACCESS_CONFLICT_MASK     (0x40) /**< AccessConflict */

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Calculate measurement settings
 * @param [in] conf measurement configuration data
 * @param [in] hsp heater set point pointer
 * @param [in] config sensor configuration data pointer
 * @return error code
 * @retval 0 success
 */
zmod4xxx_err zmod4xxx_calc_factor(zmod4xxx_conf *conf, uint8_t *hsp,
                                  uint8_t *config);

/**
 * @brief   Calculate mox resistance from ADC raw data
 * @note    This is not a generic function.
 *          Only use it if indicated in your example program flow.
 * @param   [in] dev pointer to the device
 * @param   [in,out] adc_result pointer to the adc results
 * @return  computed MOX resistance
 */
float zmod4xxx_calc_single_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result);

/**
 * @brief   Calculate mox resistance on array of results
 * @note    This is not a generic function.
 *          Only use it if indicated in your example program flow.
 * @note    This function uses zmod4xxx_calc_single_rmox
 * @param   [in] dev pointer to the device
 * @param   [in,out] adc_result pointer to the adc results
 * @param   [in,out] rmox pointer to the rmox values
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_calc_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox);

/**
 * @brief   Check the error event of the device.
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_check_error_event(zmod4xxx_dev_t *dev);

/**
 * @brief   Initialize the sensor for corresponding measurement.
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 * @note Before calling function, measurement data set has to be passed the
 * dev->meas_conf
 */
zmod4xxx_err zmod4xxx_init_measurement(zmod4xxx_dev_t *dev);

/**
 * @brief   Initialize the sensor after power on.
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 * @note    Before calling function, initialization data set has to be passed
 * the dev->init_conf
 */
zmod4xxx_err zmod4xxx_init_sensor(zmod4xxx_dev_t *dev);

/**
 * @brief   Check if all function pointers are assigned
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_null_ptr_check(zmod4xxx_dev_t *dev);

/**
 * @brief High-level function to prepare sensor
 * @param [in] dev pointer to the device
 * @return error code
 * @retval 0 success
 * @retval "!=0" error
 */
zmod4xxx_err zmod4xxx_prepare_sensor(zmod4xxx_dev_t *dev);

/**
 * @brief   Read adc values from the sensor
 * @param   [in] dev pointer to the device
 * @param   [in,out] adc_result pointer to the adc results
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_read_adc_result(zmod4xxx_dev_t *dev, uint8_t *adc_result);

/**
 * @brief High-level function to read rmox
 * @note    This is not a generic function.
 *          Only use it if indicated in your example program flow.
 * @param [in] dev pointer to the device
 * @param [in,out] adc_result pointer to the adc results
 * @param [in,out] rmox pointer to the rmox values
 * @return error code
 * @retval 0 success
 * @retval "!= 0" error
 */
zmod4xxx_err zmod4xxx_read_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox);

/**
 * @brief   Read sensor parameter.
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 * @note    This function must be called once before running other sensor
 *          functions.
 */
zmod4xxx_err zmod4xxx_read_sensor_info(zmod4xxx_dev_t *dev);

/**
 * @brief   Read the status of the device.
 * @param   [in] dev pointer to the device
 * @param   [in,out] status pointer to the status variable
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_read_status(zmod4xxx_dev_t *dev, uint8_t *status);

/**
 * @brief Read tracking number of sensor
 * @note  The buffer pointed to by track_num must be at least 6 bytes long
 * @param [in] dev pointer to the device
 * @param [in, out] track_num pointer to buffer to store the tracking number
 * @return error code
 * @retval 0 success
 * @retval "!= 0" error

 */
zmod4xxx_err zmod4xxx_read_tracking_number(zmod4xxx_dev_t *dev,
                                           uint8_t *track_num);

/**
 * @brief   Start the measurement.
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_start_measurement(zmod4xxx_dev_t *dev);

/**
 * @brief   Start the measurement at an user-defined sequencer step.
 * @param   [in] dev pointer to the device
 * @param   [in] step sequencer step to start at
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err zmod4xxx_start_measurement_at(zmod4xxx_dev_t *dev, uint8_t step );

#ifdef __cplusplus
}
#endif


/** @} */

#endif /* _ZMOD4XXX_H */
