# gps_parser

`gps_parser` is a tiny, **zero-allocation** C99 library that converts Beitian **BN-220** GPS sentences (NMEA, with optional UBX hook) into a strongly-typed structure.  
The code has been verified on an STM32 project built with STM32CubeIDE (HAL and LL), but it is fully portable to any bare-metal or RTOS target.

---

## ✨ Features
* **GGA, RMC, VTG, GLL support** – fix flag, latitude/longitude (1 e-7 °), altitude, ground speed, course, UTC time  
* **No dynamic memory** – single-line parsing, `malloc`-free  
* **Re-entrant / ISR-safe** state machine  
* **Configurable checksum** – verify, ignore or auto-correct  
* Ready-to-use **STM32 HAL & LL** reference project  



## 🚀 Quick Start (STM32CubeIDE · HAL)

```c
#include "BN220.h"

static uint8_t nmea_line[100];
volatile BN220_GPS gps;                // global, or send via an RTOS queue

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {            // huart1 → BN-220
        gpsParse(&gps, nmea_line);     // update the structure in place
        HAL_UART_Receive_IT(&huart1, nmea_line, sizeof nmea_line);
    }
}

```
