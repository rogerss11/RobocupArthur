/** @file AS5X47.h
 *
 * @brief A library for Arduino boards that reads angles from AS5047 and AS5147 sensors.
 * 		  Also support configuration of the sensor parameters.
 *
 * @par
 * COPYRIGHT NOTICE: MIT License
 *
 * 	Copyright (c) 2020 Adrien Legrand <contact@adrien-legrand.com>
 *
 * 	Permission is hereby granted, free of charge, to any person obtaining a copy
 * 	of this software and associated documentation files (the "Software"), to deal
 * 	in the Software without restriction, including without limitation the rights
 * 	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * 	copies of the Software, and to permit persons to whom the Software is
 * 	furnished to do so, subject to the following conditions:
 *
 * 	The above copyright notice and this permission notice shall be included in all
 * 	copies or substantial portions of the Software.
 *
 * 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * 	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 	SOFTWARE.
 *
*/



#ifndef AS5X47_h
#define AS5X47_h

#include "AS5X47Spi.h"
#include <stdint.h>

#define AS5X47U

#ifdef AS5X47U
// Volatile Registers Addresses
#define NOP_REG			    0x0000
#define ERRFL_REG 		  0x0001
#define PROG_REG		    0x0003
#define DIAG_REG 	      0x3FF5
#define AGC_REG 	      0x3FF9
#define ANGLE_VEL       0x3FFC
#define MAG_REG 		    0x3FFD
#define ANGLE_REG 		  0x3FFE
#define ANGLE_COMPSATED 0x3FFF
#define ANGLECOM_REG 	  0x3FFF

#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019
#define SETTINGS3_REG 	0x001A

#else
// Volatile Registers Addresses
#define NOP_REG			0x0000
#define ERRFL_REG 		0x0001
#define PROG_REG		0x0003
#define DIAGAGC_REG 	0x3FF9
#define ANGLE_VEL     0x3FFC
#define MAG_REG 		0x3FFD
#define ANGLE_REG 		0x3FFE
#define ANGLECOM_REG 	0x3FFF
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019

#endif
// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017

#define WRITE			0
#define READ			1

#ifdef AS5X47U
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t AGC_warn:1; // 0
    uint16_t MagHalf:1;  // 1
    uint16_t parerr:1;   // 2
    uint16_t p2ram_warning:1;// 3
    uint16_t invcomm:1;       // 4
    uint16_t spi_command_err:1;// 5
    uint16_t crc_error:1;  // 6
    uint16_t watchdog_err:1; // 7
    uint16_t hal_dev_err:1; // 8
    uint16_t comp_not_finished:1; // 9
    uint16_t overflow:1; // 10
    uint16_t unused:5;
  } values;
} Errfl;
#else
// ERRFL Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t frerr:1;
        uint16_t invcomm:1;
        uint16_t parerr:1;
        uint16_t unused:13;
    } values;
} Errfl;
#endif

// PROG Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
    	uint16_t progen:1;
    	uint16_t unused:1;
    	uint16_t otpref:1;
    	uint16_t progotp:1;
        uint16_t unused1:2;
        uint16_t progver:1;
        uint16_t unused2:9;
    } values;
} Prog;

// DIAAGC Register Definition
#ifdef AS5X47U
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t mode_5V:1; // bit 0
    uint16_t lf:1; // loop finished 1
    uint16_t cof:1; // overflow 2
    uint16_t magl:1; // AGC 3
    uint16_t magh:1; // AGC 4
    uint16_t mag_half:1; // mag amplitude half or less 5
    uint16_t cosf:1; // 6
    uint16_t sinf:1; // 7
    uint16_t compf:1; // 8
    uint16_t agcf:1; // 9
    uint16_t Fusa_err:1; // HAL error 10
    uint16_t spi_frame_cnt:2; //  12:11
    uint16_t unused:3; // MSB
  } values;
} Diaagc;
#else
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t agc:8; // lsbit
        uint16_t lf:1;
        uint16_t cof:1;
        uint16_t magh:1;
        uint16_t magl:1;
        uint16_t unused:4; // MSB
    } values;
} Diaagc;
#endif

// MAG Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cmag:14;
        uint16_t unused:2;
    } values;
} Mag;

// ANGLE Register Definition // also velocity
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cordicang:14;
        uint16_t unused:2;
    } values;
} Angle;

// ANGLECOM Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t daecang:14;
        uint16_t unused:2;
    } values;
} Anglecom;


// ZPOSM Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposm;
    } values;
} Zposm;

// ZPOSL Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposl:6;
        uint8_t compLerrorEn:1;
        uint8_t compHerrorEn:1;
    } values;
} Zposl;

#ifdef AS5X47U
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t uvw_off:1;
    uint8_t ABI_off:1;
    uint8_t BRKHALL_Set:4;
    uint8_t Filter_disable:1;
  } values;
} Disable;

// SETTINGS2 Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t k_max:3;
    uint8_t k_min:3;
    uint8_t Dia3_en:1;
  } values;
} Settings1;

// SETTINGS2 Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t IWidth:1;    // 0
    uint8_t NoiseSet:1;  // 1
    uint8_t Direction:1; // 2
    uint8_t UVW_ABI:1;   // 3
    uint8_t DAECDIS:1;   // 4
    uint8_t ABI_DEC:1;   // 5
    uint8_t Data_select:1;   // 6
    uint8_t PWMon:1;   // 7
  } values;
} Settings2;

typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t UVWPP:3;    // 2:0 pole pairs
    uint8_t HYS:2;      // 4:3
    uint8_t ABIRES:3;   // 7:5
  } values;
} Settings3;

#else
// SETTINGS1 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t factorySetting:1;
        uint8_t noiseset:1;
        uint8_t dir:1;
        uint8_t uvw_abi:1;
        uint8_t daecdis:1;
        uint8_t abibin:1;
        uint8_t dataselect:1;
        uint8_t pwmon:1;
    } values;
} Settings1;

// SETTINGS2 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t uvwpp:3;
    	uint8_t hys:2;
    	uint8_t abires:3;
    } values;
} Settings2;

#endif
// Command Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t commandFrame:14;
        uint16_t rw:1;
        uint16_t parc:1;
    } values;
} CommandFrame;

// ReadData Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14; // bit 13:0
        uint16_t ef:1;    // bit 14
        uint16_t pard:1;  // bit 15
    } values;
} ReadDataFrame;

// WriteData Frame  Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14;
        uint16_t low:1;
        uint16_t pard:1;
    } values;
} WriteDataFrame;

class AS5X47 {
  public:
    AS5X47(uint8_t chipSelectPin);
    ReadDataFrame readRegister(uint16_t registerAddress);
    void writeRegister(uint16_t registerAddress, uint16_t registerValue);
    float readAngle(); // in deg
    float readVel(); // velocity in deg/sec
    Errfl readErr();
    Diaagc readDiag();
    void writeSettings1(Settings1 values);
    void writeSettings2(Settings2 values);
    void writeZeroPosition(Zposm zposm, Zposl zposl);
    void printDebugString();

  private:
    bool isEven(uint16_t data);
    AS5X47Spi spi;
};

#endif // #AS5X47_h

