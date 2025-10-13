/**
 * Este headerfile define as classes de enumeração a serem usadas no código.
 */

#ifndef INCLUDE_ENUMERATORS_H_
#define INCLUDE_ENUMERATORS_H_
#endif

enum class RobotState {
  kReady = 0x0, // Comando HEX recebido pelo número 1.
  kRunning,
  kStop
};

enum class Strategy {
  kRadarEsq = 0x3, // Comando HEX recebido pelo número 4.
  kRadarDir,
  kCurvaAbertaEsq,
  kCurvaAbertaDir,
  kFollowOnly,
  kWoodPecker
};

enum class Direction {
  kLeft,
  kRight,
  kForward,
  kBackward
};