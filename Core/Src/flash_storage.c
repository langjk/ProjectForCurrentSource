#include "flash_storage.h"

#include <string.h>

uint16_t Flash_CalculateChecksum(const CalibrationData_t *data) {
    uint16_t sum = 0;
    const uint8_t *ptr = (const uint8_t *)data;

    for (uint16_t i = 0; i < sizeof(CalibrationData_t) - sizeof(uint16_t); i++) {
        sum = (uint16_t)(sum + ptr[i]);
    }

    return sum;
}

HAL_StatusTypeDef Flash_SaveCalibration(CalibrationData_t *data) {
    HAL_StatusTypeDef status;
    uint32_t address = FLASH_STORAGE_PAGE_ADDR;
    uint16_t *pData = (uint16_t *)data;
    uint16_t dataSize = (uint16_t)(sizeof(CalibrationData_t) / 2U);
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t pageError = 0;

    data->valid_flag = CALIBRATION_VALID_FLAG;
    data->checksum = 0;
    data->checksum = Flash_CalculateChecksum(data);

    HAL_FLASH_Unlock();

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_STORAGE_PAGE_ADDR;
    eraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    for (uint16_t i = 0; i < dataSize; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, pData[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 2U;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

HAL_StatusTypeDef Flash_LoadCalibration(CalibrationData_t *data) {
    uint16_t checksum;

    memcpy(data, (const void *)FLASH_STORAGE_PAGE_ADDR, sizeof(CalibrationData_t));

    if (data->valid_flag != CALIBRATION_VALID_FLAG) {
        return HAL_ERROR;
    }

    checksum = data->checksum;
    data->checksum = 0;
    if (Flash_CalculateChecksum(data) != checksum) {
        data->checksum = checksum;
        return HAL_ERROR;
    }
    data->checksum = checksum;

    return HAL_OK;
}
