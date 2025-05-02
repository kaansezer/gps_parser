/*
 * BN220.h — Interface definitions for the BN-220 GPS receiver
 *
 * Copyright (c) 2025  Kaan Sezer  <kaansezer0594@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  • The above copyright notice and this permission notice shall be included
 *    in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ---------------------------------------------------------------------------
 * @file    BN220.h
 * @author  Kaan Sezer
 * @date    27 April 2025
 * @brief   Minimal, platform-agnostic driver interface for the Beitian BN-220
 *          GPS module.  Implementation files should provide parsing routines,
 *          platform I/O hooks and any required utility helpers.
 * ---------------------------------------------------------------------------
 */

#ifndef INC_BN220_H_
#define INC_BN220_H_

#include <string.h>
#include <stdlib.h>
#include <stm32h523xx.h> // that changes with STM model
#include <stm32h5xx_hal.h>

typedef struct NMEA_SENTENCES {
    double lat; //latitude in degrees with decimal places
    char NS;  // N or S
    double lon; //longitude in degrees with decimal places
    char EW; // E or W
    float altitude; //altitude in meters
    float hdop; //horizontal dilution of precision
    int satelliteCount; //number of satellites used in measurement
    int fix; // 1 = fix, 0 = no fix
    char lastMeasure[10]; // hhmmss.ss UTC of last successful measurement; time read from the GPS module
} BN220_GPS;

/**
 * @brief  Parse raw BN-220 data and populate @p gps_data.
 *
 * @param[out] gps_data  Structure that will receive parsed GPS fields.
 * @param[in]  buffer    Pointer to the raw NMEA/UBX byte stream.
 *
 * Fills fix status, latitude, longitude, altitude, velocity and time after
 * checksum verification. No return value.
 */
void gpsParse(BN220_GPS *gps_data, uint8_t *buffer);
#endif /* INC_BN220_H_ */
