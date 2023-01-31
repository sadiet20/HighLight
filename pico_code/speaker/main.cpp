#include <stdio.h>
#include "pico/stdlib.h"
#include "pico_tone.hpp"


#define SPEAKER_PIN 22

void play_tone(Tone);
void play_melody(Tone);

int main(){
    // initialize I/O
    stdio_init_all();

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    //initialize speaker (block while playing tone), return if 
    Tone Speaker1(SPEAKER_PIN);
    int success = Speaker1.init(TONE_BLOCKING, false);  
	if(success < 0){
		while(1){
            printf("Failed to initialize speaker\n");
            sleep_ms(2000);
		}
	}

    //create speaker with "woody tone" harmonic
    Tone Speaker2(SPEAKER_PIN,50,0,20,0,20,0,10);
    Speaker2.init(TONE_NON_BLOCKING);       //non-blocking

    // main Loop 
    while(1){
        play_tone(Speaker1);
        //play_melody(Speaker2);
    }
}

void play_tone(Tone Speaker){
    printf("tone on\n");
    gpio_put(25, 1); // Set pin 25 to high (on)
    Speaker.tone(NOTE_B2, 1.2); //Note_F3 for 0.8
    //sleep_ms(100);
    //Speaker1.tone(NOTE_F3, 0.4);
    printf("tone off\n");
    gpio_put(25, 0); // Set pin 25 to low (off)
    sleep_ms(5000);
}

void play_melody(Tone Speaker){
    int melody[] = {NOTE_C4,4,  NOTE_G3,8,  NOTE_G3,8,  NOTE_A3,4,  NOTE_G3,4, 0,4,  NOTE_B3,4,  NOTE_C4,4};
    Speaker.play_melody(T_PRESTO,8,melody);
}