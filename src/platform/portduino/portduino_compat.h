#pragma once
// Force-included into every compilation unit for the rpi3bplus_rfm95_oled Portduino build.
// Provides Arduino interrupt wrappers that third-party libs (e.g. OneWire) expect
// but that Portduino/Linux does not supply.
#if defined(ARCH_PORTDUINO)
static inline void noInterrupts() {}
static inline void interrupts() {}
#endif
