/******************************************************************************
 * Author: Sadie Lauser
 * Description: This program reads from an accelerometer and piezo sensor and
 *      uses machine learning to determine if it was a rim or net shot. The
 *      program then signals the result to the user via LEDs and a speaker.
 * References: https://www.tensorflow.org/lite/microcontrollers/get_started_low_level
 *      https://www.digikey.be/en/maker/projects/raspberry-pi-pico-rp2040-spi-example-with-micropython-and-cc/9706ea0cf3784ee98e35ff49188ee045
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

//accelerometer
#include "hardware/spi.h"
#include "hardware/adc.h"

//machine learning
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

//the model
#include "hit_model.h"

//for wait until serial connection
#include <tusb.h>

//neopixels
#include "Adafruit_NeoPixel.hpp"

//speaker
#include "pico_tone.hpp"


/*
 * Variables
 */

#define SAMPLE_SIZE 350
#define NUM_SENSORS 2
#define RMS_IDX     0
#define PIEZO_IDX   1
#define NET_SHOT    0
#define RIM_SHOT    1

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
static const uint8_t REG_BW_RATE        = 0x2C;
static const uint8_t REG_DATA_FORMAT    = 0x31;

// Other constants
#define ADC_DELAY 1
static const uint8_t DEVID          = 0xE5;
static const uint8_t HZ_800         = 0x0D;         // 1101 in binary (from datasheet)
static const uint8_t G_4            = 0x01;         // +/- 4g code (from datasheet)
static const float SENSITIVITY_4G   = 1.0 / 128;    // (g/LSB)
static const float EARTH_GRAVITY    = 9.80665;      // Earth's gravity in [m/s^2]
static const float RMS_SENSITIVITY  = 7;            // sensitivity level for significant movement

/*
 * Piezo variables
 */

#define PIEZO_PIN 26

/*
 * Neopixel variables
 */

#define LED_PIN     21
#define NUM_PIXELS  16
#define BRIGHT      20

Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

static const uint32_t WHITE = pixels.Color(BRIGHT, BRIGHT, BRIGHT);
static const uint32_t RED = pixels.Color(BRIGHT, 0, 0);
static const uint32_t YELLOW = pixels.Color(BRIGHT, BRIGHT, 0);
static const uint32_t GREEN = pixels.Color(0, BRIGHT, 0);

/*
 * Speaker variables
 */

#define SPEAKER_PIN 22
Tone Speaker(SPEAKER_PIN);

/*
 * Machinie learning variables
 */

#define TENSOR_ARENA_SIZE 8 * 1024

tflite::MicroErrorReporter  micro_error_reporter;
tflite::ErrorReporter*      error_reporter  = &micro_error_reporter;
tflite::AllOpsResolver      resolver;                           //accesses all operations used by the model
const tflite::Model*        model           = nullptr;
tflite::MicroInterpreter*   interpreter     = nullptr;
TfLiteTensor*               input_tensor    = nullptr;
TfLiteTensor*               output_tensor   = nullptr;
uint8_t                     tensor_arena[TENSOR_ARENA_SIZE];    //memory for input, output, and intermediate arrays
float                       sample[350][2];

/*
 * Function Declarations
 */

void ml_init();
void check_tensor(TfLiteTensor* tensor, int exp_size, int exp_dim0, int exp_dim1, int exp_type);
int run_inference(float sample[350][2]);

void accel_init();
void reg_write(const uint cs, const uint8_t reg, const uint8_t data);
int reg_read(const uint cs, const uint8_t reg, uint8_t *buf, uint8_t nbytes);
void get_rms(float* rms);

void wait_for_movement();
void wait_for_settle();
void gather_sample(float sample[350][2]);
void rim_shot();
void net_shot();

/*
 * Main
 */
int main() {
    //initialize chosen serial port
    stdio_init_all();

    //wait for serial connection
    // while (!tud_cdc_connected()){
    //     sleep_ms(100);
    // }

    //initialize ml variables
    ml_init();

    //initialize acceleromter
    accel_init();

    //initialize analog pin
    adc_init();                 // initialize ADC hardware
    adc_gpio_init(PIEZO_PIN);   // mark pin for analog instead of digital use
    adc_select_input(0);        // select which ADC pin to read from (0..3 are GPIOs 26..29)

    //initialize speaker
    int success = Speaker.init(TONE_NON_BLOCKING, false);
	if(success < 0){
		while(1){
            printf("Failed to initialize speaker\n");
            sleep_ms(2000);
		}
	}

    //initialize NeoPixel strip
    pixels.begin();
    pixels.fill(WHITE);
    pixels.show();
    pixels.fill(WHITE);         //fill and show twice, otherwise not all pixels are white
    pixels.show();

    int hit;

    // Loop forever
    while (true) {
        pixels.fill(WHITE);
        pixels.show();

        //wait for movement to be sensed
        wait_for_movement();

        //gather and print sample
        gather_sample(sample);

        hit = run_inference(sample);
        if(hit == RIM_SHOT){
            rim_shot();
        }
        else if(hit == NET_SHOT){
            net_shot();
        }

        //wait for movement to settle down before looking for another sample
        wait_for_settle();
    }
}


/*
 * Function Definitions
 */

//initialize ml variables
void ml_init(){
    //get the TFL representation of the model byte array
    model = tflite::GetModel(hit_model);     //hit_model is the name of the char array in hit_model.h

    //make sure schema is compatible
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(error_reporter,
            "ERROR: Model provided is schema version %d not equal "
            "to supported version %d.\n",
            model->version(), TFLITE_SCHEMA_VERSION);
        while(1){ sleep_ms(2000); }
    }

    //create interpreter instance
    interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena,
                                     TENSOR_ARENA_SIZE, error_reporter);

    //alocate memory from tensor area for the model's tensors
    interpreter->AllocateTensors();

    // Obtain a pointer to the model's input tensor
    input_tensor = interpreter->input(0);
    if(input_tensor == nullptr){
        TF_LITE_REPORT_ERROR(error_reporter,
            "ERROR: Input tensor is null.\n");
        while(1){ sleep_ms(2000); }
    }

    check_tensor(input_tensor, 2, 1, 700, 1);
    
    //get the output tensor (output is single floating point number in 2D tensor)
    output_tensor = interpreter->output(0);

    check_tensor(output_tensor, 2, 1, 2, 1);
}


//checks assumptions for size, length, and type of input/output tensors
void check_tensor(TfLiteTensor* tensor, int exp_size, int exp_dim0, int exp_dim1, int exp_type){
    if(tensor->dims->size != exp_size){
        printf("ERROR: output tensor size: %d (expected %d)\n", tensor->dims->size, exp_size);
    }
    if(tensor->dims->data[0] != exp_dim0){
        printf("ERROR: output tensor length [0]: %d (expected %d)\n", tensor->dims->data[0], exp_dim0);
    }
    if(tensor->dims->data[1] != exp_dim1){
        printf("ERROR: output tensor length [1]: %d (expected %d)\n", tensor->dims->data[1], exp_dim1);
    }
    if(tensor->type != exp_type){
        printf("ERROR: output tensor type: %d (expected %d)\n", tensor->type, exp_type);     //1 == float 32
    }
}


//run inference on input rms+piezo readings
//returns NET_SHOT or RIM_SHOT depending on inference
int run_inference(float sample[350][2]){
    //provide input to model
    for(int i=0; i<SAMPLE_SIZE; i++){
        input_tensor->data.f[i*NUM_SENSORS + RMS_IDX] = sample[i][RMS_IDX];
        input_tensor->data.f[i*NUM_SENSORS + PIEZO_IDX] = sample[i][PIEZO_IDX];
    }

    //run the model
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
        while(1){ sleep_ms(2000); }
    }

    //get the values from the output tensor
    float net_percentage = output_tensor->data.f[0];
    float rim_percentage = output_tensor->data.f[1];

    //check the output value
    printf("\tCHECK: output tensor value: %.2f net, %.2f rim\n", net_percentage, rim_percentage);

    //return the detect shot type
    if(net_percentage >= rim_percentage){
        return NET_SHOT;
    }
    return RIM_SHOT;
}


// Initialize accelerometer (returns on success)
void accel_init(){
    // Buffer to store raw reads
    uint8_t data[6];
    uint baud_rate;

    // Initialize CS pin high
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    // Initialize SPI port at 1 MHz
    baud_rate = spi_init(spi, 1000 * 1000);
    printf("Baud rate: %u\n", baud_rate);

    // Set SPI format
    spi_set_format( spi0,       // SPI instance
                    8,          // Number of bits per transfer
                    SPI_CPOL_1, // Polarity (CPOL)
                    SPI_CPHA_1, // Phase (CPHA)
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

    //--- Read Power Control register ---
    reg_read(CS_PIN, REG_POWER_CTL, data, 1);
    printf("Power control register: 0x%X\r\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    data[0] |= (1 << 3);
    reg_write(CS_PIN, REG_POWER_CTL, data[0]);

    // Test: read Power Control register back to make sure Measure bit was set
    reg_read(CS_PIN, REG_POWER_CTL, data, 1);
    printf("Updated Power control register: 0x%X\r\n", data[0]);

    //--- Read Bandwidth Rate register ---
    reg_read(CS_PIN, REG_BW_RATE, data, 1);
    printf("BW Rate Register: 0x%X\r\n", data[0]);

    // Change output data rate to 800Hz
    data[0] = HZ_800;
    reg_write(CS_PIN, REG_BW_RATE, data[0]);

    // Test: read Bandwidth Rate register back to make sure data rate changed
    reg_read(CS_PIN, REG_BW_RATE, data, 1);
    printf("Updated BW Rate Register: 0x%X\r\n", data[0]);

    //--- Read Data Format register ---
    reg_read(CS_PIN, REG_DATA_FORMAT, data, 1);
    printf("Data Format Register: 0x%X\r\n", data[0]);

    // Change data format to +/- 4g
    data[0] = G_4;
    reg_write(CS_PIN, REG_DATA_FORMAT, data[0]);

    // Test: read Data Format register back to make sure data rate changed
    reg_read(CS_PIN, REG_DATA_FORMAT, data, 1);
    printf("Updated Data Format Register: 0x%X\r\n", data[0]);
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


//get RMS acceleration
void get_rms(float* rms){
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float x_ms;
    float y_ms;
    float z_ms;
    uint8_t data[6];

    // Read X, Y, and Z values from registers (16 bits each)
    reg_read(CS_PIN, REG_DATAX0, data, 6);

    // Convert 2 bytes (little-endian) into 16-bit integer (signed)
    acc_x = (int16_t)((data[1] << 8) | data[0]);
    acc_y = (int16_t)((data[3] << 8) | data[2]);
    acc_z = (int16_t)((data[5] << 8) | data[4]);

    // Convert measurements to [m/s^2]
    x_ms = acc_x * SENSITIVITY_4G * EARTH_GRAVITY;
    y_ms = acc_y * SENSITIVITY_4G * EARTH_GRAVITY;
    z_ms = acc_z * SENSITIVITY_4G * EARTH_GRAVITY;

    // Compute RMS acceleration
    *rms = sqrt(((x_ms)*(x_ms) + (y_ms)*(y_ms) + (z_ms)*(z_ms))/3);  //adjusted RMS based on m/s^2
}


//waits until movement is detected then returns
//movement = RMS_max-RMS_min of last 5 samples > 3
void wait_for_movement(){
    const int COMPARISON_SIZE = 5;
    float rms_arr[COMPARISON_SIZE];
    int idx = COMPARISON_SIZE - 1;
    float min_rms;
    float max_rms;
    const float MOVEMENT_THRESHOLD = 3;

    //fill array with data
    for(int i=0; i<COMPARISON_SIZE-1; i++){
        //get rms acceleration
        get_rms(&(rms_arr[i]));

        //delay for ADC to work properly
        sleep_ms(ADC_DELAY);
    }

    while(1){
        //get rms acceleration
        get_rms(&(rms_arr[idx]));

        //delay for ADC to work properly
        sleep_ms(ADC_DELAY);

        //find max and min of last COMPARISON_SIZE data points
        min_rms = 1000;
        max_rms = -1000;
        for(int i=0; i<COMPARISON_SIZE; i++){
            if(rms_arr[i] > max_rms){
                max_rms = rms_arr[i];
            }
            if(rms_arr[i] < min_rms){
                min_rms = rms_arr[i];
            }
        }

        //printf("\nmax: %.2f min: %.2f diff: %.2f\n", max_rms, min_rms, max_rms-min_rms);

        //check if the movement detected
        if(max_rms - min_rms > MOVEMENT_THRESHOLD){
            return;
        }

        //update index in array
        idx = (idx + 1) % COMPARISON_SIZE;
    }
}


//waits until movement has stopped (settled) and then returns
//settled = RMS_max-RMS-min of last 100 samples < 1.5
void wait_for_settle(){
    const int COMPARISON_SIZE = 100;
    float rms_arr[COMPARISON_SIZE];
    int idx = COMPARISON_SIZE - 1;
    float min_rms;
    float max_rms;
    const float MOVEMENT_THRESHOLD = 1.5;

    //fill array with data
    for(int i=0; i<COMPARISON_SIZE-1; i++){
        //get rms acceleration
        get_rms(&(rms_arr[i]));

        //delay for ADC to work properly
        sleep_ms(ADC_DELAY);
    }

    while(1){
        //get rms acceleration
        get_rms(&(rms_arr[idx]));

        //delay for ADC to work properly
        sleep_ms(ADC_DELAY);

        //find max and min of last COMPARISON_SIZE data points
        min_rms = 1000;
        max_rms = -1000;
        for(int i=0; i<COMPARISON_SIZE; i++){
            if(rms_arr[i] > max_rms){
                max_rms = rms_arr[i];
            }
            if(rms_arr[i] < min_rms){
                min_rms = rms_arr[i];
            }
        }

        //printf("\nmax: %.2f min: %.2f diff: %.2f\n", max_rms, min_rms, max_rms-min_rms);

        //check if the movement flatlined
        if(max_rms - min_rms < MOVEMENT_THRESHOLD){
            return;
        }

        //update index in array
        idx = (idx + 1) % COMPARISON_SIZE;
    }
}


//gathers RMS and piezo sample data into array then prints arrays
void gather_sample(float sample[350][2]){
    //store sample
    for(int i=0; i<SAMPLE_SIZE; i++){
        // read analog value from piezo
        sample[i][PIEZO_IDX] = adc_read();

        //get rms acceleration
        get_rms(&sample[i][RMS_IDX]);

        //delay for ADC to work properly
        sleep_ms(ADC_DELAY);
    }
}


//rim shot signal
void rim_shot(){
    pixels.fill(RED);
    pixels.show();

    //Speaker.tone(NOTE_B2, 1.2); //alternative option
    Speaker.tone(NOTE_F3, 0.3); //Note_F3 for 0.8
    sleep_ms(350);
    Speaker.tone(NOTE_F3, 0.3); //Note_F3 for 0.8
    sleep_ms(650);
}


//net shot signal
void net_shot(){
    pixels.fill(GREEN);
    pixels.show();

    sleep_ms(1000);
}