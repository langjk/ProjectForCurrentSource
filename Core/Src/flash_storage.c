/**
 * @file flash_storage.c
 * @brief Flash storage implementation for calibration parameters
 */

#include "flash_storage.h"
#include <string.h>

/**
 * @brief 计算校验和
 */
uint16_t Flash_CalculateChecksum(CalibrationData_t *data) {
    uint16_t sum = 0;
    uint8_t *ptr = (uint8_t *)data;
    // 计算除checksum字段外的所有字节
    for (uint16_t i = 0; i < sizeof(CalibrationData_t) - sizeof(uint16_t); i++) {
        sum += ptr[i];
    }
    return sum;
}

/**
 * @brief 保存校准数据到Flash
 */
HAL_StatusTypeDef Flash_SaveCalibration(CalibrationData_t *data) {
    HAL_StatusTypeDef status;
    uint32_t address = FLASH_STORAGE_PAGE_ADDR;
    uint16_t *pData = (uint16_t *)data;
    uint16_t dataSize = sizeof(CalibrationData_t) / 2;  // 半字数量

    // 设置有效标志
    data->valid_flag = CALIBRATION_VALID_FLAG;
    // 计算校验和
    data->checksum = Flash_CalculateChecksum(data);

    // 解锁Flash
    HAL_FLASH_Unlock();

    // 擦除页
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_STORAGE_PAGE_ADDR;
    eraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // 写入数据 (每次写入半字 = 16位)
    for (uint16_t i = 0; i < dataSize; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, pData[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 2;
    }

    // 锁定Flash
    HAL_FLASH_Lock();

    return HAL_OK;
}

/**
 * @brief 从Flash加载校准数据
 */
HAL_StatusTypeDef Flash_LoadCalibration(CalibrationData_t *data) {
    // 直接从Flash地址读取数据
    memcpy(data, (void *)FLASH_STORAGE_PAGE_ADDR, sizeof(CalibrationData_t));

    // 检查有效标志
    if (data->valid_flag != CALIBRATION_VALID_FLAG) {
        return HAL_ERROR;  // 数据无效
    }

    // 验证校验和
    uint16_t checksum = Flash_CalculateChecksum(data);
    if (checksum != data->checksum) {
        return HAL_ERROR;  // 校验和错误
    }

    return HAL_OK;
}
