#include <stmpe1600_class.h>
#include <vl53l0x_x_nucleo_53l0a1_class.h>

/**
 ******************************************************************************
 * @file    X_NUCLEO_53L0A1_HelloWorld.ino
 * @author  AST
 * @version V1.0.0
 * @date    24 March 2016
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L0A1
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l0x_x_nucleo_53l0a1_class.h>
#include <stmpe1600_class.h>
#include <stmpe1600_digit_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_top;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_left;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_right;

STMPE1600Digit *digit0, *digit1, *digit2, *digit3;

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(13, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  SerialPort.println("Starting the demo...");

  // Create VL53L0X top component.
  xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
  sensor_vl53l0x_top = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_top, A2);

  // Switch off VL53L0X top component.
  sensor_vl53l0x_top->VL53L0X_Off();

  // // Create (if present) VL53L0X left component.
  // xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
  // sensor_vl53l0x_left = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_left, 8);
  //
  // // Switch off (if present) VL53L0X left component.
  // sensor_vl53l0x_left->VL53L0X_Off();
  //
  // // Create (if present) VL53L0X right component.
  // xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
  // sensor_vl53l0x_right = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_right, 2);
  //
  // // Switch off (if present) VL53L0X right component.
  // sensor_vl53l0x_right->VL53L0X_Off();

  // Initialize VL53L0X top component.
  status = sensor_vl53l0x_top->InitSensor(0x10);
  if(status)
  {
    SerialPort.println("Init sensor_vl53l0x_top failed...");
  }

  // // Initialize VL53L0X left component.
  // status = sensor_vl53l0x_left->InitSensor(0x12);
  // if(status)
  // {
  //   SerialPort.println("Init sensor_vl53l0x_left failed...");
  // }
  //
  // // Initialize VL53L0X right component.
  // status = sensor_vl53l0x_right->InitSensor(0x14);
  // if(status)
  // {
  //   SerialPort.println("Init sensor_vl53l0x_right failed...");
  // }

  static const ExpGpioPinName digit0_map[7] = {GPIO_10, GPIO_12, GPIO_13, GPIO_11, GPIO_7, GPIO_8, GPIO_9};
  digit0 = new STMPE1600Digit(&DEV_I2C, digit0_map, (0x42 * 2));

  static const ExpGpioPinName digit1_map[7] = {GPIO_3, GPIO_5, GPIO_6, GPIO_4, GPIO_0, GPIO_1, GPIO_2};
  digit1 = new STMPE1600Digit(&DEV_I2C, digit1_map, (0x42 * 2));

  static const ExpGpioPinName digit2_map[7] = {GPIO_10, GPIO_12, GPIO_13, GPIO_11, GPIO_7, GPIO_8, GPIO_9};
  digit2 = new STMPE1600Digit(&DEV_I2C, digit2_map, (0x43 * 2));

  static const ExpGpioPinName digit3_map[7] = {GPIO_3, GPIO_5, GPIO_6, GPIO_4, GPIO_0, GPIO_1, GPIO_2};
  digit3 = new STMPE1600Digit(&DEV_I2C, digit3_map, (0x43 * 2));

  SerialPort.println("Init done.");
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  // Led blinking.
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);

  // Read Range.
  uint32_t distance;
  int status;
  status = sensor_vl53l0x_top->GetDistance(&distance);

  if (status == VL53L0X_ERROR_NONE)
  {
    // Output data.
    char report[64];
    snprintf(report, sizeof(report), "| Distance top [mm]: %ld |", distance);
    SerialPort.println(report);

    snprintf(report, sizeof(report), "%04d", distance);
    digit3->write(report[0] - '0');
    digit2->write(report[1] - '0');
    digit1->write(report[2] - '0');
    digit0->write(report[3] - '0');
  }
  else
  {
    SerialPort.println("Error top!");
  }

  // status = sensor_vl53l0x_left->GetDistance(&distance);
  //
  // if (status == VL53L0X_ERROR_NONE)
  // {
  //   // Output data.
  //   char report[64];
  //   snprintf(report, sizeof(report), "| Distance left [mm]: %ld |", distance);
  //   SerialPort.println(report);
  // }
  // else
  // {
  //   SerialPort.println("Error left!");
  // }
  //
  // status = sensor_vl53l0x_right->GetDistance(&distance);
  //
  // if (status == VL53L0X_ERROR_NONE)
  // {
  //   // Output data.
  //   char report[64];
  //   snprintf(report, sizeof(report), "| Distance right [mm]: %ld |", distance);
  //   SerialPort.println(report);
  // }
  // else
  // {
  //   SerialPort.println("Error right!");
  // }

}
