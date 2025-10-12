#ifndef INCLUDE_IR_CONTROL_H_
#define INCLUDE_IR_CONTROL_H_
#endif

#define DECODE_SONY
#include <IRremote.hpp>

class IRControl{
  private:
    RobotState *state;
    Strategy *strat;
  public:
    IRControl(const uint8_t ir_pin, RobotState *p_state, Strategy *p_strat);
    void ExpectIRSignal();
    void DecodeIRSignal();
};

IRControl::IRControl(const uint8_t ir_pin, RobotState *p_state, Strategy *p_strat){
  state = p_state;
  strat = p_strat;
  pinMode(LED_BUILTIN, OUTPUT);
  IrReceiver.begin(ir_pin, true, LED_BUILTIN); // LED_BUILTIN como feedback de resposta.
  IrReceiver.enableIRIn(); // Linha possivelmente desnecessária.
}

/// @brief Função para alterar os valores importantes ao robô de acordo com o sinal recebido via infravermelho.
void IRControl::DecodeIRSignal(){
  IrReceiver.resume();
  
  if (*state != RobotState::kStop && *state != RobotState::kRunning 
      && IrReceiver.decodedIRData.command == static_cast<uint16_t>(RobotState::kReady))
    *state = RobotState::kReady;
  
  if (*state == RobotState::kReady){
    switch (IrReceiver.decodedIRData.command){
      case static_cast<uint16_t>(RobotState::kRunning):
        *state = RobotState::kRunning;
        break;

      case static_cast<uint16_t>(Strategy::kRadarEsq):
        *strat = Strategy::kRadarEsq;
        break;

      case static_cast<uint16_t>(Strategy::kRadarDir):
        *strat = Strategy::kRadarDir;
        break;

      case static_cast<uint16_t>(Strategy::kCurvaAberta):
        *strat = Strategy::kCurvaAberta;
        break;

      case static_cast<uint16_t>(Strategy::kFollowOnly):
        *strat = Strategy::kFollowOnly;
        break;

      case static_cast<uint16_t>(Strategy::kWoodPecker):
        *strat = Strategy::kWoodPecker;
      
      default:
        break;
    }
  }

  if (IrReceiver.decodedIRData.command == static_cast<uint16_t>(RobotState::kStop))
    *state = RobotState::kStop;
}

void IRControl::ExpectIRSignal(){
  if (IrReceiver.decode())
    DecodeIRSignal();
}