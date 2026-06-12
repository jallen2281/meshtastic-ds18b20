#include "configuration.h"

#if defined(ARCH_PORTDUINO)

// OneWire relies on Arduino interrupt wrappers that are not provided by Portduino.
// These no-op shims keep the API contract while running in Linux user space.
void portduino_noInterrupts() {}
void portduino_interrupts() {}

#endif
