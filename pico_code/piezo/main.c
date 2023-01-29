#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define PIEZO_PIN 26

int main(){
    // initialize I/O
    stdio_init_all();

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // initialize analog pin
    adc_init();                 // initialize ADC hardware
    adc_gpio_init(PIEZO_PIN);   // mark pin for analog instead of digital use
    adc_select_input(0);        // select which ADC pin to read from (0..3 are GPIOs 26..29)

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);

    // main Loop 
    while(1){
        // 1 ms delay needed for ADC to work properly
        sleep_ms(1);

        // read analog value from piezo
        uint16_t result = adc_read();   // perform analog to digital conversion
        if(result < 40){                // found threshold via trial and error (50 for male pico, 40 for female pico)
            continue;
        }

        // print result
        printf("Raw value: %d, voltage: %f V\n", result, result * conversion_factor);
        if(result > 100){
            gpio_put(25, 1); // Set pin 25 to high (on)
        }
        else{
            gpio_put(25, 0); // Set pin 25 to low (off)
        }   
    }
}