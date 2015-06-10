#include "hardware.h"

configuration PlatformC {
  provides {
    interface Init as PlatformInit;
    interface Platform;
    interface BootParams;
  }
  uses interface Init as PeripheralInit;
}

implementation {
  components PlatformP, StackC;
  Platform = PlatformP;
  BootParams = PlatformP;
  PlatformInit = PlatformP;
  PeripheralInit = PlatformP.PeripheralInit;

  PlatformP.Stack -> StackC;

  components PlatformPinsC;
  PlatformP.PlatformPins -> PlatformPinsC;

  components PlatformLedsC;
  PlatformP.PlatformLeds -> PlatformLedsC;

  components PlatformUsciMapC;
  // No code initialization required; just connect the pins

  components PlatformClockC;
  PlatformP.PlatformClock -> PlatformClockC;
}
