/**
 * @file flash_storage.h
 * @brief Flash storage for calibration parameters
 */

#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// STM32F103C8T6 Flash: 64KB, Page size: 1KB
// 使用最后一页存储校准数据 (地址: 0x0800FC00)
#define FLASH_STORAGE_PAGE_ADDR   0x0800FC00
// FLASH_PAGE_SIZE 已在 HAL 库中定义为 0x400 (1KB)

// 数据有效性标志
#define CALIBRATION_VALID_FLAG    0xA5A5

// 多点校准点数 (0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0 mA)
#define CALIB_POINTS_NUM  9

// 校准数据结构体 (每个通道)
typedef struct {
    uint16_t full_scale_A;    // 电流满量程
    uint16_t full_scale_V;    // 电压满量程
    uint16_t zero_offset_A;   // 电流零点偏移
    uint16_t zero_offset_V;   // 电压零点偏移
} ChannelCalibration_t;

// 多点校准数据结构体 (每个通道的电流校准)
typedef struct {
    uint16_t measured[CALIB_POINTS_NUM];  // 9个校准点的实际测量值 (以100为基数)
    uint8_t enabled;                       // 是否启用多点校准
    uint8_t reserved;                      // 保留字节(对齐)
} MultiPointCalib_t;

// 完整校准数据结构体
typedef struct {
    uint16_t valid_flag;                  // 数据有效标志
    ChannelCalibration_t channels[4];     // 4个通道的校准数据
    MultiPointCalib_t multi_calib[4];     // 4个通道的多点校准数据
    uint8_t device_id;                    // 设备ID
    uint8_t reserved;                     // 保留字节(对齐)
    uint16_t checksum;                    // 校验和
} CalibrationData_t;

// 函数声明
HAL_StatusTypeDef Flash_SaveCalibration(CalibrationData_t *data);
HAL_StatusTypeDef Flash_LoadCalibration(CalibrationData_t *data);
uint16_t Flash_CalculateChecksum(CalibrationData_t *data);

#endif /* __FLASH_STORAGE_H */
