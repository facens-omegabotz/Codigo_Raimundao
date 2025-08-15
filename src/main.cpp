#include <Arduino.h>
#include <IRremote.hpp>
#include <Itamotorino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ipc.h>

#define DECODE_SONY

constexpr uint_least8_t IR_SIGNAL = 4;
constexpr uint_least8_t ONBOARD_LED = 2;
constexpr bool DEBUG = true; // set to true for serial messaging

uint_fast32_t startTime = millis();
uint_fast32_t nowTime;
bool flashing = true;

enum State {
  READY = 0x0, // same as controller command for number 1.
  RUNNING,
  STOP
};

enum Strategy {
  RADAR = 0x3,
  CURVA_ABERTA
};

State state = READY;
Strategy strategy = RADAR;

void receiveSignal(void *arg);
void blinkLed(void *arg);

void setup() {
  IrReceiver.begin(IR_SIGNAL, false);
  pinMode(ONBOARD_LED, OUTPUT);
  if (DEBUG){
    Serial.begin(115200);
    while (!Serial){;}
  }
  digitalWrite(ONBOARD_LED, HIGH);
}

void loop() {
  esp_ipc_call(PRO_CPU_NUM, receiveSignal, NULL);
  if (state == RUNNING){
    esp_ipc_call(APP_CPU_NUM, blinkLed, NULL);
  }
  if (state == STOP){
    if (DEBUG)
      Serial.println("Stopped reception and every other task. Locked functionality.");
    while (true){
      digitalWrite(ONBOARD_LED, LOW);
      delay(500);
      digitalWrite(ONBOARD_LED, HIGH);
      delay(500);
    }
  }
}

void receiveSignal(void *arg){
  (void)arg;
  if (IrReceiver.decode()){
    IrReceiver.resume();
    if (state != STOP && state != RUNNING && IrReceiver.decodedIRData.command == READY)
      state = READY;

    if (state == READY){
      switch(IrReceiver.decodedIRData.command){
        case RUNNING:
          state = RUNNING;
          break;
        case RADAR:
          strategy = RADAR;
          break;
        case CURVA_ABERTA:
          strategy = CURVA_ABERTA;
          break;
        default:
          break;
      }
    }
    if (IrReceiver.decodedIRData.command == STOP)
      state = STOP;

    if (DEBUG){
      Serial.print("Command from Sony SIRC protocol: 0x");
      Serial.println(IrReceiver.decodedIRData.command);
      Serial.print("State based on command: 0x");
      Serial.println(state);
      Serial.print("Current strategy: 0x");
      Serial.println(strategy);
      Serial.print("Core ID for this function: ");
      Serial.println(xPortGetCoreID());
    }
  }
  IrReceiver.resume(); // might be unnecessary
}

void blinkLed(void *arg){
  (void)arg;
  nowTime = millis();
  if (nowTime - startTime >= 2000L){
    startTime = millis();
    if (flashing)
      digitalWrite(ONBOARD_LED, LOW);
    else
      digitalWrite(ONBOARD_LED, HIGH);
    flashing = !flashing;
    if (DEBUG){
      Serial.print("Core ID for blinking function: ");
      Serial.println(xPortGetCoreID());
    }
  }
}