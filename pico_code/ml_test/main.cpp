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
#include "model.cpp"

//for wait until serial connection
#include <tusb.h>


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

    const tflite::Model* model = ::tflite::GetModel(g_model);     //g_model is name of char array in model.cpp

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
    const int tensor_arena_size = 2 * 1024;     //size depends on model, may need to experiment
    uint8_t tensor_arena[tensor_arena_size];

    //create interpreter instance
    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena,
                                     tensor_arena_size, error_reporter);

    //alocate memory from tensor area for the model's tensors
    interpreter.AllocateTensors();

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
    printf("CHECK: input tensor length [1]: %d (expected 1)\n", input->dims->data[1]);
    printf("CHECK: input tensor type: %d (expected 1)\n", input->type);     //1 == float 32 (the model.cpp file seems to actually be type 9)

    //provide input to model
    input->data.f[0] = 0.;

    //run the model
    TfLiteStatus invoke_status = interpreter.Invoke();
    if (invoke_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
        while(1){ sleep_ms(2000); }
    }
    printf("OK: invoke status is Ok\n");

    //get the output tensor (output is single floating point number in 2D tensor)
    TfLiteTensor* output = interpreter.output(0);

    printf("CHECK: output tensor size: %d (expected 2)\n", output->dims->size);
    printf("CHECK: output tensor length [0]: %d (expected 1)\n", output->dims->data[0]);
    printf("CHECK: output tensor length [1]: %d (expected 1)\n", output->dims->data[1]);
    printf("CHECK: output tensor type: %d (expected %d)\n", output->type, kTfLiteFloat32);     //1 == float 32 (the model.cpp file seems to actually be type 9)

    // Obtain the output value from the tensor
    float value = output->data.f[0];

    //these outputs were all -0.0, probably because the model is int instead of float

    // Check that the output value is within 0.05 of the expected value
    printf("CHECK: output tensor value: %.2f (expected 0 +- 0.05)\n", value);

    input->data.f[0] = 1.;
    interpreter.Invoke();
    value = output->data.f[0];
    printf("CHECK: output tensor value: %.2f (expected 0.841 +- 0.05)\n", value);

    input->data.f[0] = 3.;
    interpreter.Invoke();
    value = output->data.f[0];
    printf("CHECK: output tensor value: %.2f (expected 0.141 +- 0.05)\n", value);

    input->data.f[0] = 5.;
    interpreter.Invoke();
    value = output->data.f[0];
    printf("CHECK: output tensor value: %.2f (expected -0.959 +- 0.05)\n", value);
}