# Builds two native simulator binaries, each running one REAL Arduino sketch.
CXX      ?= clang++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wno-unused-function -Iarduino
LDFLAGS  ?= -pthread

COMMON_SRC = src/harness.cpp src/world.cpp arduino/core.cpp
COMMON_HDR = src/world.h src/sketch_boards.h arduino/Arduino.h arduino/HX711.h arduino/sim_hooks.h
FW_V2  = ../firmware/ZebraCrossingPedestrianCounter/ZebraCrossingPedestrianCounter.ino \
         ../firmware/ZebraCrossingPedestrianCounter/PedestrianCounter.h \
         ../firmware/ZebraCrossingPedestrianCounter/DirectionTracker.h
FW_CAL = ../firmware/HX711_Calibration/HX711_Calibration.ino

all: bin/zebra_v2 bin/zebra_cal

bin/zebra_v2: $(COMMON_SRC) $(COMMON_HDR) src/sketch_v2.cpp $(FW_V2)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) -o $@ $(COMMON_SRC) src/sketch_v2.cpp $(LDFLAGS)

bin/zebra_cal: $(COMMON_SRC) $(COMMON_HDR) src/sketch_cal.cpp $(FW_CAL)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) -o $@ $(COMMON_SRC) src/sketch_cal.cpp $(LDFLAGS)

clean:
	rm -rf bin

.PHONY: all clean
