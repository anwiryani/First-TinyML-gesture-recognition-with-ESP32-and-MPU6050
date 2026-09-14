#include <IMUgestureEXP2_inferencing.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float features[EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME];

void setup() {
  Serial.begin(115200);
  vTaskDelay(500/portTICK_PERIOD_MS);

  Serial.print("Searching mpu6050...");
  if(!mpu.begin()){
    Serial.print("MPU6050 not found");
    while(1){delay(10);}
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  for(size_t ix = 0; ix < EI_CLASSIFIER_RAW_SAMPLE_COUNT; ix++){
    sensors_event_t a, g, temp;
    mpu.getEvent (&a, &g, &temp);

    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 0] = a.acceleration.x;
    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 1] = a.acceleration.y;
    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 2] = a.acceleration.z;
    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 3] = g.gyro.x;
    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 4] = g.gyro.y;
    features[ix * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME + 5] = g.gyro.z;

    delayMicroseconds(500000/EI_CLASSIFIER_FREQUENCY);
  }

  signal_t signal;
  int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME, &signal);
  if(err != 0){
    Serial.print("Failed to generate signal, error code.: ");
    Serial.println(err);
    return;
  }

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  if(res != EI_IMPULSE_OK){
    Serial.print("Inference failed, error code: ");
    Serial.println(res);
    return;
  }

  Serial.println(" Prediction result: ");
  for(size_t ix = 0; ix <EI_CLASSIFIER_LABEL_COUNT; ix++){
    Serial.print(" ");
    Serial.print(result.classification[ix].label);
    Serial.print(" ");
    Serial.print(result.classification[ix].value, 5);
  }

  #if EI_CLASSIFIER_HAS_ANOMALY
    Serial.print(" Anomaly score: ");
    Serial.println(result.anomaly, 3);
  #endif;
}
