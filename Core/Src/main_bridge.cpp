/*
 * main_bridge.cpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#include "main_bridge.h"
#include "CoreMath.hpp"
#include "cmsis_os.h"
#include "imu_icm20602.hpp"
#include "usbd_cdc_if.h"
#include <cstdio>
#include <cstring>

extern SPI_HandleTypeDef hspi1;

void flight_controller_run(void) {
    auto usb_print = [](const char *msg) {
        static char timestamped_msg[224];
        uint32_t uptime_ms = osKernelGetTickCount();
        int len = snprintf(timestamped_msg, sizeof(timestamped_msg), "[%010lu ms] %s", static_cast<unsigned long>(uptime_ms), msg);

        if (len < 0) {
            return;
        }

        size_t tx_len = static_cast<size_t>(len);
        if (tx_len >= sizeof(timestamped_msg)) {
            tx_len = sizeof(timestamped_msg) - 1;
        }

        while (CDC_Transmit_FS((uint8_t *)timestamped_msg, static_cast<uint16_t>(tx_len)) == USBD_BUSY) {
            osDelay(1);
        }
        osDelay(2);
    };

    osDelay(5);
    usb_print("\r\n[     SYS     ] ACTIVE\r\n");

    ICM20602_IMU imu(&hspi1, SPI1_IMU_CS_GPIO_Port, SPI1_IMU_CS_Pin);
    bool imu_ok = imu.begin();

    if (imu_ok) {
        usb_print("[     IMU     ] OK\r\n");
    } else {
        usb_print("[     IMU     ] ERROR\r\n");
    }

    while (1) {
        if (imu_ok) {
            imu.update_accel();
            imu.update_gyro();

            char msg[160];
            snprintf(msg,
                     sizeof(msg),
                     "[     IMU     ] A %.3f %.3f %.3f | G %.3f %.3f %.3f\r\n",
                     static_cast<double>(imu.accel.x),
                     static_cast<double>(imu.accel.y),
                     static_cast<double>(imu.accel.z),
                     static_cast<double>(imu.gyro.x),
                     static_cast<double>(imu.gyro.y),
                     static_cast<double>(imu.gyro.z));
            usb_print(msg);
        } else {
            usb_print("[     SYS     ] TICK\r\n");
        }

        osDelay(5);
    }
}

// [     IMU     ]
// [     SYS     ]
