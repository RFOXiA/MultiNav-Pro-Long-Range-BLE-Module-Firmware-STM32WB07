/*
 * BMI270ROF.C
 *
 *  Created on: May 21, 2025
 *      Author: mario GAMAL FAYEK
 */

//includes

#include "bmi270rof.h"
#include "bmi270.h"
#include "common.h"
#include "i2c.h"
#include "bmi2_defs.h"
#include <math.h>

//defines
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)
/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)


//global vars
struct bmi2_dev bmi;
//Private function prototypes
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/* Public function  -----------------------------------------------*/

HAL_StatusTypeDef BMI270ROF_Init(I2C_HandleTypeDef* i2ch)
{
	  int8_t rslt;



	      /* Assign accel and gyro sensor to variable. */
	      uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

	      /* Sensor initialization configuration. */




	      struct bmi2_sens_config config;

	      /* Interface reference is given as a parameter
	       * For I2C : BMI2_I2C_INTF
	       * For SPI : BMI2_SPI_INTF
	       */
	      rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF,i2ch);
	      bmi2_error_codes_print_result(rslt);

	      /* Initialize bmi270. */
	      rslt = bmi270_init(&bmi);
	      bmi2_error_codes_print_result(rslt);

	      if (rslt == BMI2_OK)
	      {
	          /* Accel and gyro configuration settings. */
	          rslt = set_accel_gyro_config(&bmi);
	          bmi2_error_codes_print_result(rslt);

	          if (rslt == BMI2_OK)
	          {
	              /* NOTE:
	               * Accel and Gyro enable must be done after setting configurations
	               */
	              rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
	              bmi2_error_codes_print_result(rslt);

	              if (rslt == BMI2_OK)
	              {
	                  config.type = BMI2_ACCEL;

	                  /* Get the accel configurations. */
	                  rslt = bmi2_get_sensor_config(&config, 1, &bmi);
	                  bmi2_error_codes_print_result(rslt);

	                  printf(
	                      "\nData set, Accel Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyro_DPS_X, Gyro_DPS_Y, Gyro_DPS_Z\n\n");
	                  return HAL_OK;

	              }
	          }
	      }
	      return HAL_ERROR;
}

HAL_StatusTypeDef BMI270ROF_ReadRawData(struct bmi2_sens_data* sensor_data)
{
	int8_t rslt = bmi2_get_sensor_data(sensor_data, &bmi);
		return rslt;
}

HAL_StatusTypeDef BMI270ROF_ReadRealAcc( struct bmi2_sens_data* sensor_data,float* acc_x ,float* acc_y ,float* acc_z)
{
	if(sensor_data->status & BMI2_DRDY_ACC)
		  {
			  *acc_x = lsb_to_mps2(sensor_data->acc.x, (float)2, bmi.resolution);
			  *acc_y = lsb_to_mps2(sensor_data->acc.y, (float)2, bmi.resolution);
			  *acc_z = lsb_to_mps2(sensor_data->acc.z, (float)2, bmi.resolution);
			  // printf removed to prevent blocking during BLE initialization
			  return HAL_OK;
		  }
	return HAL_ERROR;
}
HAL_StatusTypeDef BMI270ROF_ReadRealGyr(struct bmi2_sens_data* sensor_data,float* gyr_x ,float* gyr_y ,float* gyr_z)
{
	 if(sensor_data->status & BMI2_DRDY_GYR)
		 	  {
			  	  *gyr_x = lsb_to_dps(sensor_data->gyr.x, (float)2000, bmi.resolution);
				  *gyr_y = lsb_to_dps(sensor_data->gyr.y, (float)2000, bmi.resolution);
				  *gyr_z = lsb_to_dps(sensor_data->gyr.z, (float)2000, bmi.resolution);
		 		  // printf removed to prevent blocking during BLE initialization
		 		  return HAL_OK;
		 	  }
	 return HAL_ERROR;
}

/* Static (private) helper functions---------------------------------------------------------*/

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Interrupt pin configuration */
    struct bmi2_int_pin_config pin_config = { 0 };

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configuration for hardware Interrupt */
    rslt = bmi2_get_int_pin_config(&pin_config, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT , BMI2_INT1, bmi);
   // rslt = bmi2_map_data_int(BMI2_ACC_DRDY_INT_MASK, BMI2_INT2, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Interrupt pin configuration */
        pin_config.pin_type = BMI2_INT1;
        pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
        pin_config.int_latch = BMI2_INT_NON_LATCH;

        /* Set Hardware interrupt pin configuration */
        rslt = bmi2_set_int_pin_config(&pin_config, bmi);
        bmi2_error_codes_print_result(rslt);

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
