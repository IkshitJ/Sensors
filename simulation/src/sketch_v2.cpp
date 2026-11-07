// Compiles the counter sketch UNMODIFIED, twice: once as the ENTRY gate board
// and once as the EXIT gate board (the two Arduinos of the real installation,
// which differ only in BOARD_ROLE). Each copy lives in its own namespace so
// that its globals (plates, engines, counts) are its own, exactly as on two
// separate boards. The shared headers are included once, globally.
#include "../arduino/Arduino.h"
#include "../arduino/HX711.h"
#include "sketch_boards.h"
#define PC_MAX_PADS 7
#define DT_MAX_PADS 7
#include "../../firmware/ZebraCrossingPedestrianCounter/PedestrianCounter.h"
#include "../../firmware/ZebraCrossingPedestrianCounter/DirectionTracker.h"

#define BOARD_ROLE 1  // ROLE_ENTRY
namespace board_entry {
#include "../../firmware/ZebraCrossingPedestrianCounter/ZebraCrossingPedestrianCounter.ino"
}
#undef BOARD_ROLE
#undef ROLE_TWO_WAY
#undef ROLE_ENTRY
#undef ROLE_EXIT
#undef ONE_WAY
#undef LINK
#undef LINK_MS
#undef NUM_PLATES
#undef HALF
#undef CROWD_LIMIT_DEFAULT
#undef ALERT_PIN
#undef ALERT_REPEAT_MS
#undef FRAME_US
#undef TELEMETRY_MS
#undef DEBUG_MS
#undef HX711_MAX
#undef HX711_MIN
#define BOARD_ROLE 2  // ROLE_EXIT
namespace board_exit {
#include "../../firmware/ZebraCrossingPedestrianCounter/ZebraCrossingPedestrianCounter.ino"
}

const char* SIM_FIRMWARE_ID = "v2";
const SimBoard SIM_BOARDS[] = {
    {"ENTRY", board_entry::setup, board_entry::loop},
    {"EXIT", board_exit::setup, board_exit::loop},
};
const int SIM_NUM_BOARDS = 2;
