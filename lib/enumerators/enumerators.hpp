#ifndef HEADERS_ENUMERATORS_H_
#define HEADERS_ENUMERATORS_H_

enum class FightState {
  kReady = 0x0, // Comando HEX recebido pelo número 1.
  kRunning,
  kStop
};

enum class Strategy {
  kRadarEsq = 0x3, // Comando HEX recebido pelo número 4.
  kRadarDir,
  kFollowOnly,
  kWoodPecker
};

enum class Direction {
  kLeft,
  kRight,
  kForward,
  kBackward
};

enum class QTRCalibration : unsigned char {
  kUseNVS,
  kUseCalibration
};

enum class IrDetectionState {
  kFront,
  kLeft,
  kRight,
  kNone
};

#endif