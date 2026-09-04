#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DS18B20_PIN      4
#define HCSR04_TRIG      5
#define HCSR04_ECHO      18
#define ENC_CLK          25
#define ENC_DT           26
#define ENC_SW           27
#define LIMIT_SW_PIN     14
#define PROXIMITY_PIN    13
#define IR_SENSOR_PIN    23
#define LOADCELL_ADC     34   
#define CURRENT_ADC      35   
#define VOLTAGE_ADC      32   

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
Adafruit_MPU6050 mpu;

volatile long encoderCount = 0;
volatile int lastCLKState;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 1000;

const float ENCODER_PPR = 20.0;

void IRAM_ATTR handleEncoder() {
  int clkState = digitalRead(ENC_CLK);
  if (clkState != lastCLKState) {
    if (digitalRead(ENC_DT) != clkState) encoderCount++;
    else encoderCount--;
  }
  lastCLKState = clkState;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Conveyor Belt Health Monitoring - Boot");

  tempSensor.begin();

  if (!mpu.begin()) {
    Serial.println("WARNING: MPU6050 not detected");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastCLKState = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, CHANGE);

  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  pinMode(PROXIMITY_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);

  analogReadResolution(12); // ESP32 ADC: 0-4095
}

float readUltrasonicCM() {
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG, LOW);
  long duration = pulseIn(HCSR04_ECHO, HIGH, 30000UL);
  if (duration == 0) return -1.0; // out of range / timeout
  return duration * 0.0343 / 2.0;
}

void loop() {
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float vibMagnitude = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));

  float distanceCM = readUltrasonicCM();

  noInterrupts();
  long pulses = encoderCount;
  encoderCount = 0;
  interrupts();
  float rpm = (pulses / ENCODER_PPR) * 60.0; 

  bool limitTriggered    = (digitalRead(LIMIT_SW_PIN)  == LOW);
  bool proximityDetected = (digitalRead(PROXIMITY_PIN) == LOW);
  bool materialDetected  = (digitalRead(IR_SENSOR_PIN) == LOW);


  int loadRaw = analogRead(LOADCELL_ADC);
  float tensionKg = map(loadRaw, 0, 4095, 0, 500); 


  int currentRaw = analogRead(CURRENT_ADC);
  float currentVoltage = currentRaw * (3.3 / 4095.0);
  float motorCurrentA = (currentVoltage - 1.65) / 0.100; 
  
  int voltRaw = analogRead(VOLTAGE_ADC);
  float sensedVoltage = voltRaw * (3.3 / 4095.0);
  float motorVoltage = sensedVoltage * 11.0; 

 
  Serial.print("{");
  Serial.print("\"temp_C\":");         Serial.print(tempC, 1);          Serial.print(",");
  Serial.print("\"vibration\":");      Serial.print(vibMagnitude, 2);   Serial.print(",");
  Serial.print("\"distance_cm\":");    Serial.print(distanceCM, 1);     Serial.print(",");
  Serial.print("\"rpm\":");            Serial.print(rpm, 1);            Serial.print(",");
  Serial.print("\"limit_sw\":");       Serial.print(limitTriggered);    Serial.print(",");
  Serial.print("\"proximity\":");      Serial.print(proximityDetected); Serial.print(",");
  Serial.print("\"material\":");       Serial.print(materialDetected);  Serial.print(",");
  Serial.print("\"tension_kg\":");     Serial.print(tensionKg, 1);      Serial.print(",");
  Serial.print("\"motor_A\":");        Serial.print(motorCurrentA, 2);  Serial.print(",");
  Serial.print("\"motor_V\":");        Serial.print(motorVoltage, 1);
  Serial.println("}");

  
  if (vibMagnitude > 20.0)  Serial.println("ALERT: Abnormal vibration - possible joint damage");
  if (tempC > 60.0)         Serial.println("ALERT: Overheating detected");
  if (limitTriggered)       Serial.println("ALERT: Belt misalignment / emergency triggered");
  if (motorCurrentA > 15.0) Serial.println("ALERT: Motor overload");
  if (distanceCM > 0 && distanceCM < 5) Serial.println("ALERT: Material blockage / chute overflow risk");
}