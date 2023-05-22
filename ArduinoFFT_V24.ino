#include <Arduino_FreeRTOS.h>
#include "arduinoFFT.h"
#include <semphr.h>
#include <SPI.h>
#include <LoRa.h>

#define Fs 26  // Sampling frequency in Hz
#define N 256  // Number of samples
#define M 10   // Monitored frequencies

TickType_t TimeStart;  // used for timetaking
TickType_t TimeEnd;    // used for timetaking
unsigned long Time_start;
unsigned long Time_end;

TickType_t xLastWakeTime;  // used for timing

const int analogPin = A1;
int LED = 5;
int t = 200;

const float g_acc = 9.82;  // Gravitational acceleration
const float AccSen = 0.5;  // Accelerometer-sensitivity of 500 mV/g

const long reference_voltage = 5000;  // 5.0V * 1000
const long offset_voltage = 2500;     // 2.5V * 1000

unsigned int T;      // Sampling period
unsigned long Time;  // Used for sampling timing
float delF;          // Frequency resolution, ∆f

double vReal[N];  // Array for sampling and real values of FFT
double vImag[N];  // Array for imaginary values of FFT
double dis[N];    // Array for displacement

arduinoFFT FFT = arduinoFFT();

SemaphoreHandle_t mutSem;  // Mutex for critical regions

SemaphoreHandle_t triggerSem;                // binary semaphore to trigger sampling
SemaphoreHandle_t samplesReadySem;           // binary semaphore to indicate samples are ready
SemaphoreHandle_t processedSamplesReadySem;  // binary semaphore to indicate processed samples are ready

void blinkk() {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
  delay(t);
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
  delay(t);
  digitalWrite(LED, HIGH);
}


/*--------------------------------------------------*/
/*---------------------- SETUP ---------------------*/
/*--------------------------------------------------*/

void setup() {
  Serial.begin(2000000);
  while (!Serial)
    ;

  T = round(1000000 * (1.0 / Fs));  // sampling period [µs]

  pinMode(LED, OUTPUT);
  pinMode(analogPin, INPUT);

  delF = (float)Fs / N;  // frequency resolution

  mutSem = xSemaphoreCreateMutex();
  triggerSem = xSemaphoreCreateBinary();
  samplesReadySem = xSemaphoreCreateBinary();
  processedSamplesReadySem = xSemaphoreCreateBinary();

  xSemaphoreGive(mutSem);  // ensures mutSem is available

  xTaskCreate(triggerTask, "Trigger", 256, NULL, 3, NULL);
  xTaskCreate(samplingTask, "Sampling", 256, NULL, 2, NULL);
  xTaskCreate(processingTask, "Processing", 256, NULL, 1, NULL);
  xTaskCreate(transmittingTask, "Transmitting", 256, NULL, 1, NULL);





  /* */
  if (!LoRa.begin(433E6)) {  // Frequency of 433 MHz
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  // LoRa parameters:
  LoRa.setSpreadingFactor(7);      // spreading factor
  LoRa.setSignalBandwidth(125E3);  // bandwidth
}



/*--------------------------------------------------*/
/*---------------------- TASKS ---------------------*/
/*--------------------------------------------------*/



void triggerTask(void *pvParameters) {

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    xSemaphoreGive(triggerSem);

    vTaskDelayUntil(&xLastWakeTime, (10500 / portTICK_PERIOD_MS));
    xLastWakeTime = xTaskGetTickCount();
  }
}



void samplingTask(void *pvParameters) {
  for (;;) {

    xSemaphoreTake(triggerSem, portMAX_DELAY);  // Acquires semaphore. Waits infinitely!

    xSemaphoreTake(mutSem, portMAX_DELAY);  // Acquires mutex. Waits infinitely!

    Serial.println("SAMPLING");

    for (int i = 0; i < N; i++) {  // sampling runs from 0 to N - 1 Time = micros();
      Time = micros();

      //float value = analogRead(analogPin);

      ADMUX |= B01000000;   // REFS1=0 and REFS0=1 -> Reference voltage of Vcc.
      ADMUX |= B00000001;   // Read A1
      ADCSRA |= B11000000;  // Start AD conversion - ADEN & ADSC equal to 1

      while (bit_is_set(ADCSRA, ADSC));       // Detect end-of-conversion
      int value = ADCL | (ADCH << 8);  // Shifting 8 bits to the left.

      long mapped_voltage = map(value, 0, 1023, 0, 5000);  // Map ADC value to voltage range
      mapped_voltage = mapped_voltage - offset_voltage;    // Remove the offset

      // Convert the mapped voltage back to float and scale down by 1000
      float voltage = (float)mapped_voltage / 1000.0;

      vReal[i] = voltage;
      vImag[i] = 0;
      Serial.println(vReal[i]);

      while (micros() < (Time + T)) {  // checks if current time is smaller than "Time+T"; we do nothing. If else, a new sample is taken
      }
    }
    xSemaphoreGive(mutSem);  // release mutex
    Serial.println("SAMPLING DONE");


    xSemaphoreGive(samplesReadySem);  // Semaphore for processingTask to begin
  }
}



void processingTask(void *pvParameters) {
  for (;;) {

    xSemaphoreTake(samplesReadySem, portMAX_DELAY);  // Acquires semaphore. Waits infinitely!

    xSemaphoreTake(mutSem, portMAX_DELAY);  // Acquires mutex. Waits infinitely!

    Serial.println("PROCESSING");

    /*  DC removal  */
    double mean = 0;
    for (int i = 0; i < N; i++) {
      mean += vReal[i];  // Sum up all elements from vReal
    }
    mean /= N;  // Calculate mean value
    for (int i = 0; i < N; i++) {
      vReal[i] -= mean;  // Subtract mean value from all elements
    }

    /* Hamming window */
    for (int i = 0; i < N; i++) {
      vReal[i] *= 0.54 - 0.46 * cos(2 * PI * i / (N - 1));
    }

    /* FFT */
    FFT.Compute(vReal, vImag, N, FFT_FORWARD);  // FFT is calculated and stored in vReal and vImag
    for (int i = 0; i < N; i++) {
      vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));  // Absolute value of all FFT points are saved in vReal
    }

    /* NORMALIZE FREQUENCY DATA.  */
    for (int i = 0; i < N; i++) {
      vReal[i] /= 34.73;
    }

    /* Accelertion -> Displacement conversion  */
    for (int i = 0; i < (N / 2); i++) {  // Conversion from 0 to N/2 - 1
      double omega_squared = pow(2 * PI * i * delF, 2);
      if (i == 0) {  // ignoring DC (∆f = 0)
        dis[i] = 0;
      } else {
        dis[i] = round(((vReal[i] / AccSen) * g_acc / omega_squared) * 100);  // Measured voltage divided by accelerometer sensitivity and multiplied by gravitational acceleration.
      }                                                                       // Then divided by omega_squared. Lastly multiplied by 100 to get it in cm.
    }



    /* DISPLAYING DATA */
    for (int k = 1; k < (M + 1); k++) {  // Displaying from 1 ≤ k ≤ M
      Serial.print(k * delF);
      Serial.print(" Hz");
      Serial.print("   ");
      Serial.print(vReal[k], 3);
      Serial.print(" V");
      Serial.print("   ");
      Serial.print(dis[k]);
      Serial.println(" cm");
    }



    xSemaphoreGive(mutSem);  // release mutex
    Serial.println("PROCESSING DONE");

    vTaskDelay(pdMS_TO_TICKS(1));
    xSemaphoreGive(processedSamplesReadySem);  // Semaphore for transmittingTask to begin
  }
}



void transmittingTask(void *pvParameters) {
  for (;;) {
    xSemaphoreTake(processedSamplesReadySem, portMAX_DELAY);  // Acquires sempahore. Waits infinitely!

    xSemaphoreTake(mutSem, portMAX_DELAY);  // Acquires mutex. Waits infinitely!

    Serial.println("TRANSMITTING");


    /* TRANSMITTING */
    for (int k = 1; k < (M + 1); k++) {  // Transmitting from 1 ≤ k ≤ M

      LoRa.beginPacket(false);  // Begin data package. "False" for explicit header mode.

      // Data package
      LoRa.print(27);  // ID
      LoRa.print(",");
      LoRa.print(k);  // Frequency bin
      LoRa.print(",");
      LoRa.print(round(dis[k]));  // Displacement [cm]

      Serial.print("Package ");
      Serial.println(k);
      LoRa.endPacket();                     // End data package.
      vTaskDelay(50 / portTICK_PERIOD_MS);  // non-blocking delay of 100 ms.
    }

    Serial.println("TRANSMITTING DONE");

    xSemaphoreGive(mutSem);  // release mutex
  }
}




void loop() {}  // not used