// Reference: https://www.tensorflow.org/lite/microcontrollers/get_started_low_level
// Modified by: Sadie Lauser

#include <stdio.h>
#include "pico/stdlib.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

//the model
#include "hit_model.h"

//for sample data
#include "hits.h"

//for wait until serial connection
#include <tusb.h>

#define SAMPLE_SIZE 350
#define NUM_SENSORS 2
#define RMS_IDX     0
#define PIEZO_IDX   1
#define NET_SHOT    0
#define RIM_SHOT    1


int run_inference(tflite::MicroInterpreter interpreter, tflite::ErrorReporter* error_reporter, float sample[350][2]);

/*
 * Main
 */
int main() {
    // Initialize chosen serial port
    stdio_init_all();

    // wait for serial connection
    while (!tud_cdc_connected()){
        sleep_ms(100);
    }

    tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    const tflite::Model* model = ::tflite::GetModel(hit_model);     //hit_model is the name of the char array in hit_model.h
    //make sure schema is compatible
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(error_reporter,
            "ERROR: Model provided is schema version %d not equal "
            "to supported version %d.\n",
            model->version(), TFLITE_SCHEMA_VERSION);
        while(1){ sleep_ms(2000); }
    }
    printf("OK: Model schema version %d\n", model->version());

    //loads all operations (memory expensive), should only load used ones with MicroMutableOpResolver
    tflite::AllOpsResolver resolver;        //accesses operations used by the model

    //memory for input, output, and intermediate arrays
    const int tensor_arena_size = 8 * 1024;     //size depends on model, may need to experiment srt previously 2*1024
    uint8_t tensor_arena[tensor_arena_size];

    printf("Creating interpreter...\n");

    //create interpreter instance
    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena,
                                     tensor_arena_size, error_reporter);

    printf("Allocating Tensors...\n");

    //alocate memory from tensor area for the model's tensors
    interpreter.AllocateTensors();

    printf("Getting input tensor...\n");

    // Obtain a pointer to the model's input tensor
    TfLiteTensor* input = interpreter.input(0);

    if(input == nullptr){
        TF_LITE_REPORT_ERROR(error_reporter,
            "ERROR: Input tensor is null.\n");
        while(1){ sleep_ms(2000); }
    }

    //check input tensor aspects
    //the *_EXPECT_* format from the example don't work because we're not in a unit test framework
    printf("OK: input tensor is not null\n");
    printf("CHECK: input tensor size: %d (expected 2)\n", input->dims->size);
    printf("CHECK: input tensor length [0]: %d (expected 1)\n", input->dims->data[0]);
    printf("CHECK: input tensor length [1]: %d (expected 700)\n", input->dims->data[1]);
    printf("CHECK: input tensor type: %d (expected 1)\n", input->type);     //1 == float 32

    
    //provide input to model
    for(int i=0; i<SAMPLE_SIZE; i++){
        input->data.f[i*NUM_SENSORS + RMS_IDX] = net_shot[i][RMS_IDX];
        input->data.f[i*NUM_SENSORS + PIEZO_IDX] = net_shot[i][PIEZO_IDX];
    }

    //run the model
    TfLiteStatus invoke_status = interpreter.Invoke();
    if (invoke_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
        while(1){ sleep_ms(2000); }
    }
    printf("OK: invoke status is Ok\n");
 
    //make sure the invoke status is OK
    // TF_LITE_MICRO_EXPECT_EQ(kTfLiteOk, invoke_status);

    //get the output tensor (output is single floating point number in 2D tensor)
    TfLiteTensor* output = interpreter.output(0);

    printf("CHECK: output tensor size: %d (expected 2)\n", output->dims->size);
    printf("CHECK: output tensor length [0]: %d (expected 1)\n", output->dims->data[0]);
    printf("CHECK: output tensor length [1]: %d (expected 2)\n", output->dims->data[1]);
    printf("CHECK: output tensor type: %d (expected 1)\n", output->type);     //1 == float 32

    // Obtain the output value from the tensor
    float net_percentage = output->data.f[0];
    float rim_percentage = output->data.f[1];

    // Check that the output value
    printf("CHECK: output tensor value: %.2f net, %.2f rim (expect net)\n", net_percentage, rim_percentage);

    // run inference on a rim shot
    printf("EXPECT RIM: \n");
    run_inference(interpreter, error_reporter, rim_shot);
    
}


int run_inference(tflite::MicroInterpreter interpreter, tflite::ErrorReporter* error_reporter, float sample[350][2]){
    //get pointer to model's input tensor
    TfLiteTensor* input = interpreter.input(0);

    //provide input to model
    for(int i=0; i<SAMPLE_SIZE; i++){
        input->data.f[i*NUM_SENSORS + RMS_IDX] = sample[i][RMS_IDX];
        input->data.f[i*NUM_SENSORS + PIEZO_IDX] = sample[i][PIEZO_IDX];
    }

    //run the model
    TfLiteStatus invoke_status = interpreter.Invoke();
    if (invoke_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
        while(1){ sleep_ms(2000); }
    }
    printf("\tOK: invoke status is Ok\n");

    //get the output tensor (output is single floating point number in 2D tensor)
    TfLiteTensor* output = interpreter.output(0);

    //get the values from the output tensor
    float net_percentage = output->data.f[0];
    float rim_percentage = output->data.f[1];

    //check the output value
    printf("\tCHECK: output tensor value: %.2f net, %.2f rim\n", net_percentage, rim_percentage);

    //return the detect shot type
    if(net_percentage >= rim_percentage){
        return NET_SHOT;
    }
    return RIM_SHOT;
}