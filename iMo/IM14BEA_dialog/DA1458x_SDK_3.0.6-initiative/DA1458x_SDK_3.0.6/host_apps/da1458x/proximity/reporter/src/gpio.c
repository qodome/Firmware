/**
 ****************************************************************************************
 *
 * @file gpio.c
 *
 * @brief Hardware GPIO abstruction layer.
 *
 * Attribution: Niels Thomsen  <div xmlns:cc="http://creativecommons.org/ns#" 
 * xmlns:dct="http://purl.org/dc/terms/" 
 * about="http://www.embeddedrelated.com/showcode/330.php"><span 
 * property="dct:title">GPIO library</span> (<a rel="cc:attributionURL" property="cc:attributionName" 
 * href="http://www.embeddedrelated.com/code.php?submittedby=63051">Niels Thomsen</a>) / <a rel="license" 
 * href="http://creativecommons.org/licenses/by/3.0/">CC BY 3.0</a></div>
 *
 ****************************************************************************************
 */

#include "arch.h"
#include "rwip_config.h"
#include "gpio.h"


#if DEVELOPMENT__NO_OTP

#define NO_OF_PORTS 4   // cannot be bigger than 4
#define NO_OF_MAX_PINS_PER_PORT 10  // cannot be bigger than 16

static int GPIO[NO_OF_PORTS][NO_OF_MAX_PINS_PER_PORT];

static uint16_t p_mask[4] = { 0xFF, 0xF, 0x3FF, 0x1FF };

volatile uint64_t GPIO_status __attribute__((section("exchange_mem_case1")));
#endif


/*
 * Local Functions
 ****************************************************************************************
 */

/*
 * Name         : GPIO_reservations - Globally reserved GPIOs 
 *
 * Scope        : LOCAL
 *
 * Arguments    : none
 *
 * Description  : Each application reserves its own GPIOs here.
 *                If there are GPIOs that have to be globally reserved (i.e. UART)
 *                then their reservation MUST be done BEFORE any application reservations.
 *
 * Returns      : void
 *
 */
//

static void GPIO_reservations(void)
{
    /*
     * Globally reserved GPIOs reservation
     */

    // UART GPIOs
#ifdef PROGRAM_ENABLE_UART
# ifdef PROGRAM_ALTERNATE_UART_PINS
    RESERVE_GPIO( UART1_TX, GPIO_PORT_0, GPIO_PIN_0, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_0, GPIO_PIN_5, PID_UART1_RX);
# else
    RESERVE_GPIO( UART1_TX, GPIO_PORT_0,  GPIO_PIN_0, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_0,  GPIO_PIN_1, PID_UART1_RX);
# endif // PROGRAM_ALTERNATE_UART_PINS
    RESERVE_GPIO( UART1_TX, GPIO_PORT_0, GPIO_PIN_3, PID_UART1_RTSN);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_0, GPIO_PIN_2, PID_UART1_CTSN);   
#endif // PROGRAM_ENABLE_UART

    /*
     * Application specific GPIOs reservation
     */    
#if (BLE_APP_PRESENT)
#if BLE_PROX_REPORTER
    RESERVE_GPIO( PUSH_BUTTON, GPIO_PORT_0, GPIO_PIN_6, PID_GPIO);   
    RESERVE_GPIO( GREEN_LED, GPIO_PORT_0, GPIO_PIN_7, PID_GPIO);
#endif
#endif

#if BLE_BATT_SERVER
#if !BLE_HID_DEVICE   
	//Setup LED GPIO for battery alert
    RESERVE_GPIO( RED_LED, GPIO_PORT_1, GPIO_PIN_0, PID_GPIO);
#endif	
#endif

#if BLE_ACCEL
    RESERVE_GPIO( SPI_EN, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_EN);
    RESERVE_GPIO( SPI_CLK, GPIO_PORT_0, GPIO_PIN_0, PID_SPI_CLK);
    RESERVE_GPIO( SPI_DO, GPIO_PORT_0, GPIO_PIN_3, PID_SPI_DO);	
    RESERVE_GPIO( SPI_DO, GPIO_PORT_0, GPIO_PIN_5, PID_SPI_DI);
 	//Set P1_5 to ACCEL's INT1
    RESERVE_GPIO( ACCEL_INT1, GPIO_PORT_1, GPIO_PIN_5, PID_GPIO);
#endif // BLE_ACCEL

     
#if BLE_HID_DEVICE

    // Inputs (columns)
    RESERVE_GPIO( INPUT_COL_0,              2,  0,  PID_GPIO);
    RESERVE_GPIO( INPUT_COL_1,              2,  1,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_2,              2,  2,  PID_GPIO);

    RESERVE_GPIO( INPUT_COL_3,              1,  4,  PID_GPIO); 

    RESERVE_GPIO( INPUT_COL_4,              2,  3,  PID_GPIO); 
    //RESERVE_GPIO( INPUT_COL_5,              X,  X,  PID_GPIO);
    RESERVE_GPIO( INPUT_COL_6,              2,  4,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_7,              2,  5,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_8,              2,  6,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_9,              2,  7,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_10,             1,  0,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_11,             1,  1,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_12,             1,  2,  PID_GPIO); 
    RESERVE_GPIO( INPUT_COL_13,             1,  3,  PID_GPIO); 

    // Outputs (rows)
    RESERVE_GPIO( OUTPUT_ROW_0,             0,  0,  PID_GPIO); 
    RESERVE_GPIO( OUTPUT_ROW_1,             2,  8,  PID_GPIO); 
    RESERVE_GPIO( OUTPUT_ROW_2,             2,  9,  PID_GPIO); 
    RESERVE_GPIO( OUTPUT_ROW_3,             0,  3,  PID_GPIO); 
    RESERVE_GPIO( OUTPUT_ROW_4,             0,  4,  PID_GPIO); 
    RESERVE_GPIO( OUTPUT_ROW_5,             0,  5,  PID_GPIO); 

#ifdef MY_PRINTF 
    RESERVE_GPIO( PRINTF_OUTPUT,            0,  6,  PID_UART2_TX); 
#else
    RESERVE_GPIO( OUTPUT_ROW_6,             0,  6,  PID_GPIO); 
#endif

    RESERVE_GPIO( OUTPUT_ROW_7,             0,  7,  PID_GPIO); 

#endif //BLE_HID_DEVICE 
}


/*
 * Global Functions
 ****************************************************************************************
 */

/*
 * Name         : GPIO_init - Initialize gpio assignemnt check 
 *
 * Scope        : PUBLIC
 *
 * Arguments    : none
 *
 * Description  : Initialize the GPIO assignemnt check variables
 *
 * Returns      : void
 *
 */
void GPIO_init( void )
{
#if DEVELOPMENT__NO_OTP
#warning "GPIO assignment checking is active! Deactivate before burning OTP..."
    
    int i, j;

    for (i = 0; i < NO_OF_PORTS; i++)
        for (j = 0; j < NO_OF_MAX_PINS_PER_PORT; j++)
            GPIO[i][j] = 0;
    
    GPIO_reservations();
    
    GPIO_status = 0;
    
    for (i = 0; i < NO_OF_PORTS; i++)
        for (j = 0; j < NO_OF_MAX_PINS_PER_PORT; j++) {
            uint16_t bitmask = (1 << j);
            
            if ( !(p_mask[i] & bitmask) ) // port pin does not exist! continue to next port...
                break;
            
            if (GPIO[i][j] == -1)
                __asm("BKPT #0\n"); // this pin has been previously reserved!
            
            if (GPIO[i][j] == 0)
                continue;
            
            GPIO_status |= ((uint64_t)GPIO[i][j] << j) << (i * 16);
        }
#endif    
}

/*
 * Name         : GPIO_SetPinFunction - Set the pin type and mode
 *
 * Scope        : PUBLIC
 *
 * Arguments    : port - GPIO port
 *                pin  - pin
 *                mode - pin mode (input, output...)
 *                function - pin usage (GPIO, UART, SPI...)
 *
 * Description  : Sets the GPIO type and mode
 *
 * Returns      : void
 *
 */
void GPIO_SetPinFunction( GPIO_PORT port, GPIO_PIN pin, GPIO_PUPD mode, GPIO_FUNCTION function )
{
#if DEVELOPMENT__NO_OTP
    if ( !(GPIO_status & ( ((uint64_t)1 << pin) << (port * 16) )) )
                __asm("BKPT #0\n"); // this pin has not been previously reserved!        
#endif    
    
    const int data_reg = GPIO_BASE + (port << 5);
    const int mode_reg = data_reg + 0x6 + (pin << 1);
    
    SetWord16(mode_reg, mode | function);
}

/*
 * Name         : GPIO_ConfigurePin - Combined function to set the state
 *                                     and the type and mode of the GPIO pin 
 *
 * Scope        : PUBLIC
 *
 * Arguments    : port - GPIO port
 *                pin  - pin
 *                mode - pin mode (input, output...)
 *                function - pin usage (GPIO, UART, SPI...)
 *                high - set to TRUE to set the pin into high else low
 *
 * Description  : Sets the GPIO state and then its type and mode
 *
 * Returns      : void
 *
 */
void GPIO_ConfigurePin( GPIO_PORT port, GPIO_PIN pin, GPIO_PUPD mode, GPIO_FUNCTION function,
						 const bool high )
{
#if DEVELOPMENT__NO_OTP
    if ( !(GPIO_status & ( ((uint64_t)1 << pin) << (port * 16) )) )
                __asm("BKPT #0\n"); // this pin has not been previously reserved!        
#endif    
    
    if (high)
        GPIO_SetActive( port, pin );
    else
        GPIO_SetInactive( port, pin );
    
	GPIO_SetPinFunction( port, pin, mode, function );
}

/*
 * Name         : GPIO_SetActive - Sets a pin high
 *
 * Scope        : PUBLIC
 *
 * Arguments    : port - GPIO port
 *                pin  - pin
 *
 * Description  : Sets the GPIO high. The GPIO should have been previously configured 
 *                as output!
 *
 * Returns      : void
 *
 */
void GPIO_SetActive( GPIO_PORT port, GPIO_PIN pin )
{
#if DEVELOPMENT__NO_OTP
    if ( !(GPIO_status & ( ((uint64_t)1 << pin) << (port * 16) )) )
                __asm("BKPT #0\n"); // this pin has not been previously reserved!        
#endif    
    
    const int data_reg = GPIO_BASE + (port << 5);
    const int set_data_reg = data_reg + 2;
    
	SetWord16(set_data_reg, 1 << pin);
}

/*
 * Name         : GPIO_SetInactive - Sets a pin low
 *
 * Scope        : PUBLIC
 *
 * Arguments    : port - GPIO port
 *                pin  - pin
 *
 * Description  : Sets the GPIO low. The GPIO should have been previously configured 
 *                as output!
 *
 * Returns      : void
 *
 */
void GPIO_SetInactive( GPIO_PORT port, GPIO_PIN pin )
{
#if DEVELOPMENT__NO_OTP
    if ( !(GPIO_status & ( ((uint64_t)1 << pin) << (port * 16) )) )
                __asm("BKPT #0\n"); // this pin has not been previously reserved!        
#endif    
    
    const int data_reg = GPIO_BASE + (port << 5);
    const int reset_data_reg = data_reg + 4;
    
	SetWord16(reset_data_reg, 1 << pin);
}

/*
 * Name         : GPIO_GetPinStatus - Gets the state of the pin
 *
 * Scope        : PUBLIC
 *
 * Arguments    : port - GPIO port
 *                pin  - pin
 *
 * Description  : Gets the GPIO status. The GPIO should have been previously configured 
 *                as input!
 *
 * Returns      : TRUE if the pin is high,
 *                FALSE if low.
 *
 */
bool GPIO_GetPinStatus( GPIO_PORT port, GPIO_PIN pin )
{
#if DEVELOPMENT__NO_OTP
    if ( !(GPIO_status & ( ((uint64_t)1 << pin) << (port * 16) )) )
                __asm("BKPT #0\n"); // this pin has not been previously reserved!        
#endif    
    
    const int data_reg = GPIO_BASE + (port << 5);
    
    return ( (GetWord16(data_reg) & (1 << pin)) != 0 );
}
