/**
 * Este headerfile define a classe MotorControl, a fim de facilitar
 * a definição dos comportamentos do robô no código principal.
 */

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
        MotorControl();
        void SetSpeeds(int32_t m1, int32_t m2);
        void StopMotors();
        void Turn(Direction d);
        void Accelerate(Direction d);
};

MotorControl::MotorControl(){
    this->itamotorino.setupADC(kPwmChannelM1, kPwmFreqM1, kPwmResolutionM1, 
                               kPwmChannelM2, kPwmFreqM2, kPwmResolutionM2);
}

// Talvez seja um método desnecessário.
void MotorControl::SetSpeeds(int32_t m1, int32_t m2){
    this->itamotorino.setSpeeds(m1, m2);
}

void MotorControl::StopMotors(){
    this->SetSpeeds(0, 0);
}

void MotorControl::Turn(Direction d){
    switch (d){
        case Direction::kLeft:
            this->SetSpeeds(191, -191);
            break;

        case Direction::kRight:
            this->SetSpeeds(-191, 191);
            break;

        default:
            this->Accelerate(d);
            break;
    }
}

void MotorControl::Accelerate(Direction d){
    switch (d){
        case Direction::kForward:
            this->SetSpeeds(-255, -255);
            break;
        
        case Direction::kBackward:
            this->SetSpeeds(255, 255);
            break;
        
        default:
            this->Turn(d);
            break;
    }
}
