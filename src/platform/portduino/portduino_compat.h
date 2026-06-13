#pragma once
// Force-included into every compilation unit for the rpi3bplus_rfm95_oled Portduino build.
// Provides Arduino interrupt wrappers that third-party libs (e.g. OneWire) expect
// but that Portduino/Linux does not supply.

#ifndef noInterrupts
static inline void noInterrupts() {}
#endif

#ifndef interrupts
static inline void interrupts() {}
#endif
