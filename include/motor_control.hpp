#ifndef INCLUDE_MOTOR_CONTROL_H_
#define INCLUDE_MOTOR_CONTROL_H_
#endif

#include <Itamotorino.h>

// Pinos da ponte H (TB6612FNG)

constexpr uint8_t kPwmA = 4;
constexpr uint8_t kPwmB = 21; 
constexpr uint8_t kAIn1 = 16;
constexpr uint8_t kAIn2 = 22;
constexpr uint8_t kBIn1 = 23;
constexpr uint8_t kBIn2 = 5;

// Valores para configuração de PWM

constexpr int kPwmChannelM1 = 1;
constexpr int kPwmFreqM1 = 1000;
constexpr int kPwmResolutionM1 = 8;

constexpr int kPwmChannelM2 = 2;
constexpr int kPwmFreqM2 = 1000;
constexpr int kPwmResolutionM2 = 8;

class MotorControl {
    private:
        Itamotorino itamotorino = Itamotorino(kAIn1, kAIn2, kBIn1, kBIn2, kPwmA, kPwmB);
    
    public:
        void SetSpeeds(int32_t motor_a, int32_t motor_b);
        void StopMotors();
};

MotorControl::MotorControl(){
    this->itamotorino.setupADC(kPwmChannelM1, kPwmFreqM1, kPwmResolutionM1, 
                               kPwmChannelM2, kPwmFreqM2, kPwmResolutionM2);
}

void MotorControl::SetSpeeds(int32_t motor_a, int32_t motor_b){
    this->itamotorino.setSpeeds(motor_a, motor_b);
}

void MotorControl::StopMotors(){
    this->SetSpeeds(0, 0);
}
