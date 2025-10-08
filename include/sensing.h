#ifndef INCLUDE_SENSING_H_
#define INCLUDE_SENSING_H_
#endif

#include <IRremote.hpp>
#include <QTRSensors.h>
#include "enumerators.h"

#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.
#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4

constexpr uint8_t kSensor1 = 35;
constexpr uint8_t kSensor2 = 33;
constexpr uint8_t kSensor3 = 25;
constexpr uint8_t kSensor4 = 27;
constexpr uint8_t kIrPin = 4;
constexpr uint8_t kQtr1 = 36;
constexpr uint8_t kQtr2 = 39;

class SensingHandler {
    private:
        RobotState state = kReady;
        Strategy strat = kRadarDir;
        bool useNVSCalibration;
        QTRSensorsAnalog qtra;
    public:
        SensingHandler(bool useNVSCalibration, bool aEnableLEDFeedback){
            this->useNVSCalibration = useNVSCalibration;
            qtra.init((uint8_t[]){kQtr1, kQtr2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
            IrReceiver.begin(kIrPin, false);
        }

        void receiveIrSignal(){
            if (IrReceiver.decode()){
                IrReceiver.resume();
                if (state != kStop && state != kRunning && IrReceiver.decodedIRData.command == kReady)
                    state = kReady;

                if (state == kReady){
                    switch(IrReceiver.decodedIRData.command){
                        case kRunning:
                        state = kRunning;
                        break;
                        case kRadarEsq:
                        strat = kRadarEsq;
                        break;
                        case kRadarDir:
                        strat = kRadarDir;
                        break;
                        case kCurvaAberta:
                        strat = kCurvaAberta;
                        break;
                        case kFollowOnly:
                        strat = kFollowOnly;
                        break;
                        case  kWoodPecker:
                        strat = kWoodPecker;
                        default:
                        break;
                    }
                }
                if (IrReceiver.decodedIRData.command == kStop)
                    state = kStop;
            }
            IrReceiver.resume(); // might be unnecessary
        }
};