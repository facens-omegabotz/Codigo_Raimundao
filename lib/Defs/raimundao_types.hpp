#ifndef INCLUDE_RAIMUNDAO_TYPES_HPP_
#define INCLUDE_RAIMUNDAO_TYPES_HPP_

enum class FightState : unsigned char {
  kReady = 0x0,
  kFighting,
  kStop
};

enum class Strategy : unsigned char {
  kSearchLeft = 0x3,
  kSearchRight,
  kFollowEnemy,
};

enum class SensorState : unsigned char {
  kLeft,
  kFrontLeft,
  kFront,
  kFrontRight,
  kRight,
  kNone
};

enum class QTRState : unsigned char {
  kLeft,
  kRight,
  kBoth,
  kNone
};

struct States {
  FightState fight_state {FightState::kReady};
  Strategy strategy {Strategy::kSearchLeft};
  SensorState sensor_state {SensorState::kNone};
  SensorState last_state {SensorState::kNone};
  QTRState qtr_state {QTRState::kNone};
};

#endif