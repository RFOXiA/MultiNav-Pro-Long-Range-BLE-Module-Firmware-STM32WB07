/*
 * gnss_mia_m10q.c
 *
 *  Created on: December 22, 2025
 *  Author: RFOXiA AI Assistant
 *  
 *  MIA-M10Q GNSS Module Driver (I2C Interface)
 */

#include "gnss_mia_m10q.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
static I2C_HandleTypeDef* gnss_i2c = NULL;

/* DMA State Management */
static volatile uint8_t gnss_dma_buffer[192]; /* UBX-NAV-PVT frame is 100 bytes (8 framing + 92 payload);
                                               * extra room consumes any stale/leading bytes in the stream */
static volatile uint8_t gnss_dma_busy = 0;    // 0=idle, 1=poll_sent, 2=reading, 3=complete, 4=error
static volatile uint32_t gnss_dma_start_time = 0;
static volatile uint16_t gnss_dma_read_len = 0;
static volatile uint16_t gnss_available_bytes = 0;

/* GPIO Pins for GNSS Control */
#define GNSS_RESET_PIN     GPIO_PIN_9   // PA9
#define GNSS_RESET_PORT    GPIOA
#define GNSS_VOL_SEL_PIN   GPIO_PIN_9   // PB9
#define GNSS_VOL_SEL_PORT  GPIOB

/* I2C Timeout */
#define GNSS_I2C_TIMEOUT   1000

/* UBX Message Buffer */
#define UBX_BUFFER_SIZE    128
static uint8_t ubx_buffer[UBX_BUFFER_SIZE];

/* Private function prototypes */
static HAL_StatusTypeDef GNSS_SendUBX(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len);
static HAL_StatusTypeDef GNSS_ReadI2CData(uint8_t* buffer, uint16_t length);
static HAL_StatusTypeDef GNSS_WaitForUBX(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t* payload_len, uint32_t timeout);
static void GNSS_CalculateChecksum(uint8_t* data, uint16_t length, uint8_t* ck_a, uint8_t* ck_b);

/**
 * @brief Initialize MIA-M10Q GNSS module via I2C
 */
HAL_StatusTypeDef GNSS_Init(I2C_HandleTypeDef* i2ch)
{
    gnss_i2c = i2ch;
    
    printf("GNSS_Init: Initializing MIA-M10Q module...\r\n");
    
    // Set VOL_SEL HIGH for 3.3V operation (or leave floating)
    HAL_GPIO_WritePin(GNSS_VOL_SEL_PORT, GNSS_VOL_SEL_PIN, GPIO_PIN_SET);
    printf("GNSS_Init: VOL_SEL set to 3.3V\r\n");
    
    /* CRITICAL: Release RESET_N (active-low). MX_GPIO_Init drives PA9 LOW at
     * boot, which holds the M10Q in permanent reset - the module never ACKs
     * on I2C and init always failed with "Device not responding at 0x42".
     * Perform a clean reset pulse and release the pin. */
    GNSS_HardwareReset();
    
    // Check if GNSS module is ready
    if(HAL_I2C_IsDeviceReady(gnss_i2c, GNSS_I2C_ADDR << 1, 3, GNSS_I2C_TIMEOUT) != HAL_OK) {
        /* One retry after extra boot time - module may still be starting up */
        HAL_Delay(500);
        if(HAL_I2C_IsDeviceReady(gnss_i2c, GNSS_I2C_ADDR << 1, 3, GNSS_I2C_TIMEOUT) != HAL_OK) {
            printf("GNSS_Init: Device not responding at I2C address 0x%02X\r\n", GNSS_I2C_ADDR);
            return HAL_ERROR;
        }
    }
    printf("GNSS_Init: Device detected at I2C address 0x%02X\r\n", GNSS_I2C_ADDR);
    
    /* CRITICAL: Disable NMEA output on the I2C port.
     * The M10 outputs NMEA sentences on I2C by DEFAULT, flooding the stream
     * buffer with text. The DMA read path (GNSS_GetDMAResult) reads the FRONT
     * of that buffer and searches it for the UBX-NAV-PVT poll response - with
     * NMEA enabled the response is queued behind hundreds of NMEA bytes and is
     * NEVER found, so no GNSS data ever reached BLE. */
    if(GNSS_DisableNMEAOnI2C() != HAL_OK) {
        printf("GNSS_Init: WARNING - failed to disable NMEA on I2C\r\n");
        /* Not fatal: flush below + larger reads still give UBX a chance */
    } else {
        printf("GNSS_Init: NMEA output on I2C disabled (UBX only)\r\n");
    }
    
    /* Flush any pending (NMEA) bytes already queued in the module buffer */
    GNSS_FlushStream();
    
    // Configure update rate to 1Hz (1000ms)
    if(GNSS_SetUpdateRate(1000) != HAL_OK) {
        printf("GNSS_Init: Failed to set update rate\r\n");
        return HAL_ERROR;
    }
    printf("GNSS_Init: Update rate set to 1Hz\r\n");
    
    printf("GNSS_Init: Initialization complete\r\n");
    return HAL_OK;
}

/**
 * @brief Check if GNSS module is responding
 */
HAL_StatusTypeDef GNSS_IsReady(void)
{
    if(gnss_i2c == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_IsDeviceReady(gnss_i2c, GNSS_I2C_ADDR << 1, 1, GNSS_I2C_TIMEOUT);
}

/**
 * @brief Perform hardware reset of GNSS module
 */
HAL_StatusTypeDef GNSS_HardwareReset(void)
{
    printf("GNSS_HardwareReset: Resetting module...\r\n");
    
    // Pull RESET low
    HAL_GPIO_WritePin(GNSS_RESET_PORT, GNSS_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    
    // Release RESET
    HAL_GPIO_WritePin(GNSS_RESET_PORT, GNSS_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(500);  // Wait for module to boot
    
    printf("GNSS_HardwareReset: Reset complete\r\n");
    return HAL_OK;
}

/**
 * @brief Get number of available bytes in GNSS buffer
 */
HAL_StatusTypeDef GNSS_GetAvailableBytes(uint16_t* num_bytes)
{
    uint8_t length_bytes[2];
    
    // Read the two-byte length register (0xFD and 0xFE)
    if(HAL_I2C_Mem_Read(gnss_i2c, GNSS_I2C_ADDR << 1, GNSS_REG_DATA_LENGTH_HIGH, 1, length_bytes, 2, GNSS_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    
    *num_bytes = (length_bytes[0] << 8) | length_bytes[1];
    return HAL_OK;
}

/**
 * @brief Read data from GNSS I2C data stream register
 */
static HAL_StatusTypeDef GNSS_ReadI2CData(uint8_t* buffer, uint16_t length)
{
    // Read from data stream register (0xFF)
    return HAL_I2C_Mem_Read(gnss_i2c, GNSS_I2C_ADDR << 1, GNSS_REG_DATA_STREAM, 1, buffer, length, GNSS_I2C_TIMEOUT);
}

/**
 * @brief Calculate UBX message checksum
 */
static void GNSS_CalculateChecksum(uint8_t* data, uint16_t length, uint8_t* ck_a, uint8_t* ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    
    for(uint16_t i = 0; i < length; i++) {
        *ck_a = *ck_a + data[i];
        *ck_b = *ck_b + *ck_a;
    }
}

/**
 * @brief Send UBX protocol message to GNSS
 */
static HAL_StatusTypeDef GNSS_SendUBX(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len)
{
    uint8_t ck_a, ck_b;
    uint8_t header[6];
    
    // Build UBX header
    header[0] = UBX_SYNC_CHAR_1;      // Sync char 1
    header[1] = UBX_SYNC_CHAR_2;      // Sync char 2
    header[2] = msg_class;             // Message class
    header[3] = msg_id;                // Message ID
    header[4] = payload_len & 0xFF;    // Length LSB
    header[5] = (payload_len >> 8) & 0xFF;  // Length MSB
    
    // Calculate checksum (over class, ID, length, and payload)
    GNSS_CalculateChecksum(&header[2], 4, &ck_a, &ck_b);
    if(payload_len > 0 && payload != NULL) {
        for(uint16_t i = 0; i < payload_len; i++) {
            ck_a = ck_a + payload[i];
            ck_b = ck_b + ck_a;
        }
    }
    
    // Send header
    if(HAL_I2C_Master_Transmit(gnss_i2c, GNSS_I2C_ADDR << 1, header, 6, GNSS_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Send payload if present
    if(payload_len > 0 && payload != NULL) {
        if(HAL_I2C_Master_Transmit(gnss_i2c, GNSS_I2C_ADDR << 1, payload, payload_len, GNSS_I2C_TIMEOUT) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    // Send checksum
    uint8_t checksum[2] = {ck_a, ck_b};
    if(HAL_I2C_Master_Transmit(gnss_i2c, GNSS_I2C_ADDR << 1, checksum, 2, GNSS_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Configure GNSS update rate
 */
HAL_StatusTypeDef GNSS_SetUpdateRate(uint16_t rate_ms)
{
    // UBX-CFG-RATE payload: measRate(2), navRate(2), timeRef(2)
    uint8_t payload[6];
    payload[0] = rate_ms & 0xFF;        // measRate LSB
    payload[1] = (rate_ms >> 8) & 0xFF; // measRate MSB
    payload[2] = 0x01;                  // navRate LSB (1 = every measurement)
    payload[3] = 0x00;                  // navRate MSB
    payload[4] = 0x00;                  // timeRef LSB (0 = UTC time)
    payload[5] = 0x00;                  // timeRef MSB
    
    return GNSS_SendUBX(UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6);
}

/**
 * @brief Disable NMEA protocol output on the I2C (DDC) port via UBX-CFG-VALSET
 * @note  Sets CFG-I2COUTPROT-NMEA (key 0x10720002) = 0 in RAM + BBR layers
 */
HAL_StatusTypeDef GNSS_DisableNMEAOnI2C(void)
{
    uint8_t payload[9];
    payload[0] = 0x00;  /* version */
    payload[1] = 0x03;  /* layers: RAM (bit0) + BBR (bit1) */
    payload[2] = 0x00;  /* reserved */
    payload[3] = 0x00;  /* reserved */
    /* CFG-I2COUTPROT-NMEA key = 0x10720002 (little-endian) */
    payload[4] = 0x02;
    payload[5] = 0x00;
    payload[6] = 0x72;
    payload[7] = 0x10;
    payload[8] = 0x00;  /* value: 0 = NMEA output disabled */
    
    return GNSS_SendUBX(UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
}

/**
 * @brief Drain any bytes pending in the module's I2C stream buffer
 */
void GNSS_FlushStream(void)
{
    uint16_t available = 0;
    uint16_t total_flushed = 0;
    
    /* Bounded loop: discard up to 2KB of stale data */
    while (total_flushed < 2048) {
        if (GNSS_GetAvailableBytes(&available) != HAL_OK || available == 0) {
            break;
        }
        uint16_t chunk = (available > UBX_BUFFER_SIZE) ? UBX_BUFFER_SIZE : available;
        if (GNSS_ReadI2CData(ubx_buffer, chunk) != HAL_OK) {
            break;
        }
        total_flushed += chunk;
    }
    if (total_flushed > 0) {
        printf("GNSS_Init: Flushed %u stale bytes from stream buffer\r\n", total_flushed);
    }
}

/**
 * @brief Wait for specific UBX message response
 */
static HAL_StatusTypeDef GNSS_WaitForUBX(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t* payload_len, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    uint16_t available_bytes;
    
    while((HAL_GetTick() - start_time) < timeout) {
        // Check how many bytes are available
        if(GNSS_GetAvailableBytes(&available_bytes) != HAL_OK) {
            continue;
        }
        
        if(available_bytes < 8) {  // Minimum UBX message size
            HAL_Delay(10);
            continue;
        }
        
        // Read available data (up to buffer size)
        uint16_t read_len = (available_bytes > UBX_BUFFER_SIZE) ? UBX_BUFFER_SIZE : available_bytes;
        if(GNSS_ReadI2CData(ubx_buffer, read_len) != HAL_OK) {
            continue;
        }
        
        // Search for UBX header
        for(uint16_t i = 0; i < read_len - 8; i++) {
            if(ubx_buffer[i] == UBX_SYNC_CHAR_1 && ubx_buffer[i+1] == UBX_SYNC_CHAR_2) {
                uint8_t rx_class = ubx_buffer[i+2];
                uint8_t rx_id = ubx_buffer[i+3];
                uint16_t rx_len = ubx_buffer[i+4] | (ubx_buffer[i+5] << 8);
                
                // Check if this is the message we're looking for
                if(rx_class == msg_class && rx_id == msg_id) {
                    // Copy payload
                    if(rx_len > 0 && payload != NULL) {
                        uint16_t copy_len = (rx_len > *payload_len) ? *payload_len : rx_len;
                        memcpy(payload, &ubx_buffer[i+6], copy_len);
                        *payload_len = copy_len;
                    }
                    return HAL_OK;
                }
            }
        }
        
        HAL_Delay(10);
    }
    
    return HAL_TIMEOUT;
}

/**
 * @brief Read GNSS position, velocity, time data (UBX-NAV-PVT)
 */
HAL_StatusTypeDef GNSS_ReadPVT(GNSS_Data_t* gnss_data)
{
    // Poll UBX-NAV-PVT message
    if(GNSS_SendUBX(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Wait for response (UBX-NAV-PVT is 92 bytes)
    uint8_t payload[92];
    uint16_t payload_len = 92;
    
    if(GNSS_WaitForUBX(UBX_CLASS_NAV, UBX_NAV_PVT, payload, &payload_len, 1000) != HAL_OK) {
        return HAL_TIMEOUT;
    }
    
    // Parse UBX-NAV-PVT payload (see MIA-M10Q interface description)
    // Offset 0-3: iTOW (ignore)
    // Offset 4-5: year
    gnss_data->year = payload[4] | (payload[5] << 8);
    // Offset 6: month
    gnss_data->month = payload[6];
    // Offset 7: day
    gnss_data->day = payload[7];
    // Offset 8: hour
    gnss_data->hour = payload[8];
    // Offset 9: min
    gnss_data->minute = payload[9];
    // Offset 10: sec
    gnss_data->second = payload[10];
    // Offset 11: valid flags
    gnss_data->valid_flags = payload[11];
    // Offset 20: fixType
    gnss_data->fix_type = payload[20];
    // Offset 23: numSV
    gnss_data->num_satellites = payload[23];
    // Offset 24-27: lon (degrees * 1e-7)
    gnss_data->longitude = (int32_t)(payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24));
    // Offset 28-31: lat (degrees * 1e-7)
    gnss_data->latitude = (int32_t)(payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24));
    // Offset 36-39: hMSL (height above MSL in mm)
    gnss_data->altitude_msl = (int32_t)(payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24));
    // Offset 40-43: hAcc (horizontal accuracy in mm)
    gnss_data->horizontal_acc = (uint32_t)(payload[40] | (payload[41] << 8) | (payload[42] << 16) | (payload[43] << 24));
    // Offset 44-47: vAcc (vertical accuracy in mm)
    gnss_data->vertical_acc = (uint32_t)(payload[44] | (payload[45] << 8) | (payload[46] << 16) | (payload[47] << 24));
    // Offset 60-63: gSpeed (ground speed in mm/s)
    gnss_data->ground_speed = (int32_t)(payload[60] | (payload[61] << 8) | (payload[62] << 16) | (payload[63] << 24));
    // Offset 64-67: headMot (heading of motion in degrees * 1e-5)
    gnss_data->heading = (int32_t)(payload[64] | (payload[65] << 8) | (payload[66] << 16) | (payload[67] << 24));
    
    return HAL_OK;
}

/**
 * @brief Start non-blocking DMA read for GNSS PVT data
 * @note Sends UBX-NAV-PVT poll request only (non-blocking)
 * @retval HAL_OK if started, HAL_BUSY if previous transfer ongoing, HAL_ERROR on failure
 */
HAL_StatusTypeDef GNSS_StartDMARead(void)
{
    if (gnss_i2c == NULL) {
        return HAL_ERROR;
    }
    
    // Check if previous transfer ongoing
    if (gnss_dma_busy >= 1 && gnss_dma_busy <= 2) {
        // Timeout check
        if ((HAL_GetTick() - gnss_dma_start_time) > 1000) {
            gnss_dma_busy = 4;  // Timeout error
            return HAL_ERROR;
        }
        return HAL_BUSY;
    }
    
    gnss_dma_busy = 1;  // Poll sent
    gnss_dma_start_time = HAL_GetTick();
    
    // Send UBX-NAV-PVT poll request (non-blocking)
    if(GNSS_SendUBX(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0) != HAL_OK) {
        gnss_dma_busy = 4;
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Check if GNSS DMA is currently busy
 * @retval 1 if busy (transfer in progress), 0 if idle
 */
uint8_t GNSS_IsBusy(void)
{
    return (gnss_dma_busy >= 1 && gnss_dma_busy <= 2) ? 1 : 0;
}

/**
 * @brief Check if GNSS DMA read is complete and get parsed data
 * @param gnss_data Pointer to GNSS_Data_t structure to fill
 * @retval HAL_OK if data ready, HAL_BUSY if still reading/waiting, HAL_ERROR on error
 */
HAL_StatusTypeDef GNSS_GetDMAResult(GNSS_Data_t* gnss_data)
{
    // If poll just sent, wait minimum 200ms before checking (GNSS needs time to prepare data)
    if (gnss_dma_busy == 1) {
        uint32_t elapsed = HAL_GetTick() - gnss_dma_start_time;
        
        // Wait at least 200ms for GNSS to process and buffer data
        if (elapsed < 200) {
            return HAL_BUSY;  // Not ready yet
        }
        
        // After 200ms, check ONCE for available bytes
        uint16_t available = 0;
        if(GNSS_GetAvailableBytes(&available) == HAL_OK && available >= 100) {
            // Data available - start DMA read
            gnss_dma_busy = 2;  // Reading
            /* Read as much as fits: the full UBX-NAV-PVT frame is 100 bytes
             * (6 header + 92 payload + 2 checksum). Reading LESS than the full
             * frame leaves stale bytes queued in the module buffer, shifting
             * every subsequent frame until parsing fails permanently. */
            uint16_t read_len = (available > sizeof(gnss_dma_buffer)) ? sizeof(gnss_dma_buffer) : available;
            gnss_dma_read_len = read_len;
            
            if(HAL_I2C_Mem_Read_DMA(gnss_i2c, GNSS_I2C_ADDR << 1, GNSS_REG_DATA_STREAM,
                                    I2C_MEMADD_SIZE_8BIT, (uint8_t*)gnss_dma_buffer, read_len) != HAL_OK) {
                gnss_dma_busy = 4;  // Error
                return HAL_ERROR;
            }
        } else if (elapsed > 1000) {
            // Timeout - no data after 1 second
            gnss_dma_busy = 4;
            return HAL_ERROR;
        }
        
        return HAL_BUSY;  // Still waiting
    }
    
    // If currently reading via DMA
    if (gnss_dma_busy == 2) {
        return HAL_BUSY;
    }
    
    // If error occurred
    if (gnss_dma_busy == 4) {
        gnss_dma_busy = 0;  // Reset
        return HAL_ERROR;
    }
    
    // If complete
    if (gnss_dma_busy == 3) {
        // Parse UBX-NAV-PVT from DMA buffer
        // Search for UBX header in buffer.
        // Parse accesses payload[67] = buffer[i+73]; bound the search so the
        // full parsed region stays inside the bytes actually read.
        uint16_t search_end = (gnss_dma_read_len >= 74) ? (uint16_t)(gnss_dma_read_len - 74) : 0;
        for(uint16_t i = 0; i <= search_end; i++) {
            if(gnss_dma_buffer[i] == UBX_SYNC_CHAR_1 && gnss_dma_buffer[i+1] == UBX_SYNC_CHAR_2) {
                uint8_t rx_class = gnss_dma_buffer[i+2];
                uint8_t rx_id = gnss_dma_buffer[i+3];
                
                if(rx_class == UBX_CLASS_NAV && rx_id == UBX_NAV_PVT) {
                    // Found it - parse payload starting at i+6
                    uint8_t* payload = (uint8_t*)&gnss_dma_buffer[i+6];
                    
                    gnss_data->year = payload[4] | (payload[5] << 8);
                    gnss_data->month = payload[6];
                    gnss_data->day = payload[7];
                    gnss_data->hour = payload[8];
                    gnss_data->minute = payload[9];
                    gnss_data->second = payload[10];
                    gnss_data->valid_flags = payload[11];
                    gnss_data->fix_type = payload[20];
                    gnss_data->num_satellites = payload[23];
                    gnss_data->longitude = (int32_t)(payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24));
                    gnss_data->latitude = (int32_t)(payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24));
                    gnss_data->altitude_msl = (int32_t)(payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24));
                    gnss_data->horizontal_acc = (uint32_t)(payload[40] | (payload[41] << 8) | (payload[42] << 16) | (payload[43] << 24));
                    gnss_data->vertical_acc = (uint32_t)(payload[44] | (payload[45] << 8) | (payload[46] << 16) | (payload[47] << 24));
                    gnss_data->ground_speed = (int32_t)(payload[60] | (payload[61] << 8) | (payload[62] << 16) | (payload[63] << 24));
                    gnss_data->heading = (int32_t)(payload[64] | (payload[65] << 8) | (payload[66] << 16) | (payload[67] << 24));
                    
                    gnss_dma_busy = 0;  // Reset for next read
                    return HAL_OK;
                }
            }
        }
        
        // UBX message not found in buffer
        {
            static uint16_t ubx_miss_count = 0;
            ubx_miss_count++;
            if (ubx_miss_count == 1 || (ubx_miss_count % 30) == 0) {
                printf("GNSS: UBX-NAV-PVT not found in %u bytes read (miss #%u), first bytes: %02X %02X %02X %02X\r\n",
                       gnss_dma_read_len, ubx_miss_count,
                       gnss_dma_buffer[0], gnss_dma_buffer[1], gnss_dma_buffer[2], gnss_dma_buffer[3]);
            }
        }
        gnss_dma_busy = 0;
        return HAL_ERROR;
    }
    
    // Not started yet
    return HAL_ERROR;
}

/**
 * @brief DMA RX Complete Callback for GNSS
 * @note Call this from HAL_I2C_MemRxCpltCallback in main.c
 */
void GNSS_DMA_RxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == gnss_i2c && gnss_dma_busy == 2) {
        gnss_dma_busy = 3;  // Mark as complete
    }
}

/**
 * @brief DMA Error Callback for GNSS
 * @note Call this from HAL_I2C_ErrorCallback in main.c
 */
void GNSS_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == gnss_i2c && gnss_dma_busy == 2) {
        gnss_dma_busy = 4;  // Mark as error
    }
}

/**
 * @brief Start DMA read for GNSS (Autonomous DMA system) - DEPRECATED
 * @note For autonomous DMA system - reads 20 bytes of binary GPS data
 * @param buffer Pointer to buffer for 20 bytes
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef GNSS_StartDMARead_OLD(uint8_t* buffer)
{
    if (gnss_i2c == NULL || buffer == NULL) {
        return HAL_ERROR;
    }
    
    /* Read 20 bytes of binary GPS data from GNSS module
     * This is the pre-formatted binary data for BLE characteristic */
    return HAL_I2C_Mem_Read_DMA(gnss_i2c, GNSS_I2C_ADDR << 1, GNSS_REG_DATA_STREAM,
                                I2C_MEMADD_SIZE_8BIT, buffer, 20);
}
