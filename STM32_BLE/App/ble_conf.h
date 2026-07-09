/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_conf.h
  * @author  MCD Application Team
  * @brief   Configuration file for BLE Middleware.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_CONF_H
#define BLE_CONF_H

#include "app_conf.h"

/******************************************************************************
 *
 * BLE Event Handler configuration
 *
 ******************************************************************************/

/**
 * There is one handler per BLE service
 */
#define BLE_CFG_MAX_NBR_GATT_EVT_HANDLERS                       (6)  /* Increased from 4 to accommodate Chat Server + GATT Client */

/* USER CODE BEGIN ble_conf_1 */
/* CRITICAL: Enable 2 simultaneous connections (phone + peer) for dual-role operation */
#define CFG_BLE_NUM_LINK                                        (2)
/* USER CODE END ble_conf_1 */

#endif /*BLE_CONF_H */
