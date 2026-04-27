#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// STM32F103C8T6 Flash: 64KB, page size: 1KB
// Use the last flash page to store calibration data.
#define FLASH_STORAGE_PAGE_ADDR 0x0800FC00U

#define CALIBRATION_VALID_FLAG 0xA5A5U

// 0.00, 0.50, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00 mA
#define CALIB_POINTS_NUM 9U

typedef struct {
    uint16_t full_scale_A;
    uint16_t full_scale_V;
    uint16_t zero_offset_A;
    uint16_t zero_offset_V;
} ChannelCalibration_t;

typedef struct {
    uint16_t measured[CALIB_POINTS_NUM];
    uint8_t enabled;
    uint8_t reserved;
} MultiPointCalib_t;

typedef struct {
    uint16_t valid_flag;
    ChannelCalibration_t channels[4];
    MultiPointCalib_t multi_calib[4];
    uint8_t device_id;
    uint8_t reserved;
    uint16_t checksum;
} CalibrationData_t;

HAL_StatusTypeDef Flash_SaveCalibration(CalibrationData_t *data);
HAL_StatusTypeDef Flash_LoadCalibration(CalibrationData_t *data);
uint16_t Flash_CalculateChecksum(const CalibrationData_t *data);

#endif /* __FLASH_STORAGE_H */
