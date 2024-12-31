enum events {
    TIME = 0,
    ANGLE,
    DISTANCE,
    CRASH,
    ROLLOVER = 3,
    IGN_ON = 33,
    IGN_OFF = 34

};

enum cmdType {
  TEST = 0,
  READ = 1,
  WRITE = 2,
  EXECUTE = 3,
  SEND = 4,
  UNKNOWN = -1
};