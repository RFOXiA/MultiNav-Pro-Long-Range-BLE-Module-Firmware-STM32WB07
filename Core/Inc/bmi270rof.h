/*
 * bmi270rof.h
 *
 *  Created on: May 21, 2025
 *      Author: mario gamal fayek
 */

#ifndef INC_BMI270ROF_H_
#define INC_BMI270ROF_H_

#include "i2c.h"
#include "bmi2_defs.h"




//public functions
HAL_StatusTypeDef BMI270ROF_Init(I2C_HandleTypeDef*);

HAL_StatusTypeDef BMI270ROF_ReadRawData(struct bmi2_sens_data* sesor_data);

HAL_StatusTypeDef BMI270ROF_ReadRealAcc(struct bmi2_sens_data* sesor_data,float* acc_x ,float* acc_y ,float* acc_z);
HAL_StatusTypeDef BMI270ROF_ReadRealGyr(struct bmi2_sens_data* sesor_data,float* gyr_x ,float* gyr_y ,float* gyr_z);





#endif /* INC_BMI270ROF_H_ */
