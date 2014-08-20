/*
 * Freescale's MMA8652 driver
 */
#include "acc.h"
#include "hal_i2c.h"
#include "hal_board_cfg.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */
// Sensor I2C address
#define MMA8652_I2C_ADDRESS             0x1D

// MMA8652 register addresses
#define ACC_REG_ADDR_STATUS                 0x00
#define ACC_REG_ADDR_XOUT_H                 0x01
#define ACC_REG_ADDR_XOUT_L                 0x02
#define ACC_REG_ADDR_YOUT_H                 0x03
#define ACC_REG_ADDR_YOUT_L                 0x04
#define ACC_REG_ADDR_ZOUT_H                 0x05
#define ACC_REG_ADDR_ZOUT_L                 0x06
#define ACC_REG_ADDR_SYSMOD                 0x0B
#define ACC_REG_ADDR_WHO_AM_I               0x0D
#define ACC_REG_ADDR_XYZ_DATA_CFG           0x0E
#define ACC_REG_ADDR_CTRL_REG1_DRATE_MODE   0x2A
#define ACC_REG_ADDR_CTRL_REG4_INTRPT_CFG   0x2D

// Select register valies
#define REG_VAL_WHO_AM_I                0x4A

#define ST_HAL_DELAY(n) st( { volatile uint32 i; for (i=0; i<(n); i++) { }; } )

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
static void HalAccSelect(void);
static bool HalAccTest(void);

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalAccInit
 *
 * @brief       This function initializes the HAL Accelerometer abstraction layer.
 *
 * @return      None.
 */
bool HalAccInit(void)
{
    bool ret = FALSE;

    // ACC INT1/2
    P1SEL &= ~(1<<0);
    P1DIR &= ~(1<<0);
    
    P0SEL &= ~(1<<6);
    P0DIR &= ~(1<<6);  
    
    HalAccSelect();
    
    ret = HalAccTest();
    if (ret == TRUE) {
        // Disable interrupt
        HalI2CWriteSingle(ACC_REG_ADDR_CTRL_REG4_INTRPT_CFG, 0x00);
        
        // Turn off acc
        HalI2CWriteSingle(ACC_REG_ADDR_CTRL_REG1_DRATE_MODE, 0x00);
    }
    return ret; 
}

/**************************************************************************************************
 * @fn          HalAccRead
 *
 * @brief       Read data from the accelerometer - X, Y, Z - 3 bytes
 *
 * @return      TRUE if valid data, FALSE if not
 */
bool HalAccRead(int16 *pBuf)
{
    uint16 x;
    uint16 y;
    uint16 z;
    bool success;
    uint8 cmd_tmp;

    // Select this sensor
    HalAccSelect();

    cmd_tmp = HalI2CReadSingle(ACC_REG_ADDR_CTRL_REG1_DRATE_MODE);

    // Turn on sensor
    success = (bool)HalI2CWriteSingle(ACC_REG_ADDR_CTRL_REG1_DRATE_MODE, cmd_tmp | 0x01);

    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);

    // Read the three registers
    x = ((uint16)HalI2CReadSingle(ACC_REG_ADDR_XOUT_H) << 8) | HalI2CReadSingle(ACC_REG_ADDR_XOUT_L);
    y = ((uint16)HalI2CReadSingle(ACC_REG_ADDR_YOUT_H) << 8) | HalI2CReadSingle(ACC_REG_ADDR_YOUT_L);
    z = ((uint16)HalI2CReadSingle(ACC_REG_ADDR_ZOUT_H) << 8) | HalI2CReadSingle(ACC_REG_ADDR_ZOUT_L);

    pBuf[0] = (int16)x;
    pBuf[0] >>= 4;
    pBuf[1] = (int16)y;
    pBuf[1] >>= 4;
    pBuf[2] = (int16)z;
    pBuf[2] >>= 4;

    // Turn off sensor 
    success = (bool)HalI2CWriteSingle(ACC_REG_ADDR_CTRL_REG1_DRATE_MODE, (cmd_tmp & 0xFE));

    return success;
}


/**************************************************************************************************
 * @fn          HalAccTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
static bool HalAccTest(void)
{
    uint8 val;

    // Select this sensor on the I2C bus
    HalAccSelect();

    // Check the WHO AM I register
    val = HalI2CReadSingle(ACC_REG_ADDR_WHO_AM_I);
    if (val == REG_VAL_WHO_AM_I) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/* ------------------------------------------------------------------------------------------------
 *                                           Private functions
 * -------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalAccSelect
 *
 * @brief       Select the accelerometer on the I2C-bus
 *
 * @return
 */
static void HalAccSelect(void)
{
    //Set up I2C that is used to communicate with SHT21
    HalI2CInit(MMA8652_I2C_ADDRESS, i2cClock_267KHZ);
}

/*  Conversion algorithm for X, Y, Z
 *  ================================
 *
 float calcAccel(int8 rawX, uint8 range)
 {
 float v;

 switch (range)
 {
 case HAL_ACC_RANGE_2G:
//-- calculate acceleration, unit G, range -2, +2
v = (rawX * 1.0) / (256/4);
break;

case HAL_ACC_RANGE_4G:
//-- calculate acceleration, unit G, range -4, +4
v = (rawX * 1.0) / (256/8);
break;

case HAL_ACC_RANGE_4G:
//-- calculate acceleration, unit G, range -8, +8
v = (rawX * 1.0) / (256/16);
break;    
}
return v;
}
*/

/*********************************************************************
 *********************************************************************/
