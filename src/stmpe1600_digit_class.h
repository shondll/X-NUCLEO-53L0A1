/**
 ******************************************************************************
 * @file    stmpe1600_digit_class.h
 * @author  shondll
 * @version V0.0.1
 * @date    05-March-2018
 * @brief   Header file for component stmpe1600
 ******************************************************************************
 */
#ifndef     __STMPE1600_DIGIT_CLASS
#define     __STMPE1600_DIGIT_CLASS
/* Includes ------------------------------------------------------------------*/
#include "stmpe1600_class.h"

/* Classes -------------------------------------------------------------------*/
/** Class representing a single stmpe1600 GPIO expander output pin
 */
class STMPE1600Digit : public STMPE1600DigiOut {

 public:
    /** Constructor
     * @param[in] &i2c device I2C to be used for communication
     * @param[in] outpinname the desired out pin name to be created
     * @param[in] DevAddr the stmpe1600 I2C device addres (deft STMPE1600_DEF_DEVICE_ADDRESS)
     * @param[in] lvl the default ot pin level
     */
    STMPE1600Digit (TwoWire *i2c, ExpGpioPinName *segmentMap_, uint8_t DevAddr=STMPE1600_DEF_DEVICE_ADDRESS, int num=0):
        STMPE1600DigiOut(i2c, NOT_CON, DevAddr, STMPE1600_DEF_DIGIOUT_LVL), segmentMap(segmentMap_)
    {
       int i;
       uint8_t data[2];
       /* set the exppinname as output */

       STMPE1600DigiOut_I2CRead(data, expdevaddr, GPDR_0_7, 1);
       STMPE1600DigiOut_I2CRead(&data[1], expdevaddr, GPDR_8_15, 1);

       for (i = 0; i < 7; ++i) {
           exppinname = segmentMap[i];

           *(uint16_t*)data = *(uint16_t*)data | (1<<(uint16_t)exppinname);  // set gpio as out
       }

       STMPE1600DigiOut_I2CWrite(data, expdevaddr, GPDR_0_7, 1);
       STMPE1600DigiOut_I2CWrite(&data[1], expdevaddr, GPDR_8_15, 1);

       write(num);
    }

	/**
	 * @brief       Write on the out pins
	 * @param[in]   number [0-F] to write
	 * @return      0 on Success
	 */
    void write (int num)
    {
       int i;
       uint8_t data[2];

       if (num > 0xF || num < 0) return;

       /* set the exppinname state to lvl */
       STMPE1600DigiOut_I2CRead(data, expdevaddr, GPSR_0_7, 2);

       for (i = 0; i < 7; ++i) {
           exppinname = segmentMap[i];

           *(uint16_t*)data = *(uint16_t*)data | (uint16_t)(1<<(uint16_t)exppinname);
           if (gfedcba_map[num] & (1 << i)) {
               *(uint16_t*)data = *(uint16_t*)data & (uint16_t)(~(1<<(uint16_t)exppinname));  // light it with 0
           }
       }

       STMPE1600DigiOut_I2CWrite(data, expdevaddr, GPSR_0_7, 2);
    }

 private:
    ExpGpioPinName* segmentMap;

    static uint8_t gfedcba_map[16];
};

static uint8_t STMPE1600Digit::gfedcba_map[16] = {
    0x3F,
    0x06,
    0x5B,
    0x4F,
    0x66,
    0x6D,
    0x7D,
    0x07,
    0x7F,
    0x6F,
    0x77, // A
    0x7C,
    0x39,
    0x5E,
    0x79,
    0x71
};

#endif // __STMPE1600_DIGIT_CLASS
