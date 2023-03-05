// Reference: https://www.tensorflow.org/lite/microcontrollers/get_started_low_level
// Modified by: Sadie Lauser

#include <stdio.h>
#include "pico/stdlib.h"

//machine learning includes
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

//the model
#include "hit_model.h"

//for wait until serial connection
#include <tusb.h>

//sample data of hardcoded inputs
#include "hits.h"


#define SAMPLE_SIZE 350
#define NUM_SENSORS 2
#define RMS_IDX     0
#define PIEZO_IDX   1
#define NET_SHOT    0
#define RIM_SHOT    1

#define TENSOR_ARENA_SIZE 8 * 1024

tflite::MicroErrorReporter  micro_error_reporter;
tflite::ErrorReporter*      error_reporter  = &micro_error_reporter;
tflite::AllOpsResolver      resolver;                           //accesses all operations used by the model
const tflite::Model*        model           = nullptr;
tflite::MicroInterpreter*   interpreter     = nullptr;
TfLiteTensor*               input_tensor    = nullptr;
TfLiteTensor*               output_tensor   = nullptr;
uint8_t                     tensor_arena[TENSOR_ARENA_SIZE];    //memory for input, output, and intermediate arrays

//function prototypes 
void ml_init();
void check_tensor(TfLiteTensor* tensor, int exp_size, int exp_dim0, int exp_dim1, int exp_type);
int run_inference(float sample[350][2]);


/*
 * Main
 */
int main() {
    //initialize chosen serial port
    stdio_init_all();

    //wait for serial connection
    while (!tud_cdc_connected()){
        sleep_ms(100);
    }

    //initialize ml variables
    ml_init();

    //run inference on a rim shot
    printf("EXPECT NET: \n");
    run_inference(net_shot);
    
    //run inference on a rim shot
    printf("EXPECT RIM: \n");
    run_inference(rim_shot);
}


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