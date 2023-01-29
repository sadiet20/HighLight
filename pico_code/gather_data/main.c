// Reference: https://www.digikey.be/en/maker/projects/raspberry-pi-pico-rp2040-spi-example-with-micropython-and-cc/9706ea0cf3784ee98e35ff49188ee045
// Modified by: Sadie Lauser

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

/*
 * Accelerometer variables
 */

// Pins
#define CS_PIN      17
#define SCK_PIN     18
#define MOSI_PIN    19
#define MISO_PIN    16

// Ports
spi_inst_t* spi = spi0; 

// Registers
static const uint8_t REG_DEVID      = 0x00;
static const uint8_t REG_POWER_CTL  = 0x2D;
static const uint8_t REG_DATAX0     = 0x32;

// Other constants
static const uint8_t DEVID          = 0xE5;
static const float SENSITIVITY_2G   = 1.0 / 256;    // (g/LSB)
static const float SENSITIVITY_4G   = 1.0 / 128;    // (g/LSB)
static const float EARTH_GRAVITY    = 9.80665;      // Earth's gravity in [m/s^2]

/*
 * Piezo variables
 */

#define PIEZO_PIN 26


/*
 * Function Declarations
 */

void accel_init();
void reg_write(const uint cs, const uint8_t reg, const uint8_t data);
int reg_read(const uint cs, const uint8_t reg, uint8_t *buf, uint8_t nbytes);
void get_xyz_rms(float* x_ms, float* y_ms, float* z_ms, float* rms);


/*
 * Main
 */
int main() {
    float acc_x;
    float acc_y;
    float acc_z;
    float rms;
    uint16_t piezo;

    // Initialize chosen serial port
    stdio_init_all();

    // Initialize acceleromter
    accel_init();

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // initialize analog pin
    adc_init();                 // initialize ADC hardware
    adc_gpio_init(PIEZO_PIN);   // mark pin for analog instead of digital use
    adc_select_input(0);        // select which ADC pin to read from (0..3 are GPIOs 26..29)

    // Wait before taking measurements
    sleep_ms(2000);

    // Loop forever
    while (true) {
        // read analog value from piezo
        piezo = adc_read();

        //get x, y, z, and rms acceleration
        get_xyz_rms(&acc_x, &acc_y, &acc_z, &rms);
        
        if(rms >= 6){
            printf("RMS: %.2f | Piezo: %d \r\n", rms, piezo);
            gpio_put(25, 1); // Set pin 25 to high (on)
        }
        else{
            gpio_put(25, 0); // Set pin 25 to low (off)
        }
        
        // Print results
        //printf("X: %.2f m/s^2 | Y: %.2f m/s^2 | Z: %.2f m/s^2 | RMS: %.2f \r\n", acc_x, acc_y, acc_z, rms);

        sleep_ms(1);
    }
}


/*
 * Function Definitions
 */

// Initialize accelerometer (returns on success)
void accel_init(){
    // Buffer to store raw reads
    uint8_t data[6];

    // Initialize CS pin high
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    // Initialize SPI port at 1 MHz
    spi_init(spi, 1000 * 1000);

    // Set SPI format
    spi_set_format( spi0,   // SPI instance
                    8,      // Number of bits per transfer
                    1,      // Polarity (CPOL)
                    1,      // Phase (CPHA)
                    SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    // Workaround: perform throw-away read to make SCK idle high
    reg_read(CS_PIN, REG_DEVID, data, 1);

    // Read device ID to make sure that we can communicate with the ADXL343
    reg_read(CS_PIN, REG_DEVID, data, 1);
    if (data[0] != DEVID) {
        while (true){
            printf("ERROR: Could not communicate with ADXL343\r\n");
            sleep_ms(2000);
        }
    }
    
    // Read Power Control register
    reg_read(CS_PIN, REG_POWER_CTL, data, 1);
    printf("0x%X\r\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    data[0] |= (1 << 3);
    reg_write(CS_PIN, REG_POWER_CTL, data[0]);

    // Test: read Power Control register back to make sure Measure bit was set
    reg_read(CS_PIN, REG_POWER_CTL, data, 1);
    printf("0x%X\r\n", data[0]);
}


// Write 1 byte to the specified register
void reg_write(const uint cs, const uint8_t reg, const uint8_t data) {

    uint8_t msg[2];
                
    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x00 | reg;
    msg[1] = data;

    // Write to register
    gpio_put(cs, 0);
    spi_write_blocking(spi, msg, 2);
    gpio_put(cs, 1);
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(const uint cs, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Determine if multiple byte (MB) bit should be set
    if (nbytes < 1) {
        return -1;
    } else if (nbytes == 1) {
        mb = 0;
    } else {
        mb = 1;
    }

    // Construct message (set ~W bit high)
    uint8_t msg = 0x80 | (mb << 6) | reg;

    // Read from register
    gpio_put(cs, 0);
    spi_write_blocking(spi, &msg, 1);
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
    gpio_put(cs, 1);

    return num_bytes_read;
}


//get acceleration for x, y, z, and RMS 
void get_xyz_rms(float* x_ms, float* y_ms, float* z_ms, float* rms){
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    uint8_t data[6];

    // Read X, Y, and Z values from registers (16 bits each)
    reg_read(CS_PIN, REG_DATAX0, data, 6);

    // Convert 2 bytes (little-endian) into 16-bit integer (signed)
    acc_x = (int16_t)((data[1] << 8) | data[0]);
    acc_y = (int16_t)((data[3] << 8) | data[2]);
    acc_z = (int16_t)((data[5] << 8) | data[4]);

    // Convert measurements to [m/s^2]
    *x_ms = acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
    *y_ms = acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
    *z_ms = acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

    // Compute RMS acceleration
    *rms = sqrt(((*x_ms)*(*x_ms) + (*y_ms)*(*y_ms) + (*z_ms)*(*z_ms))/3);  //adjusted RMS based on m/s^2

}