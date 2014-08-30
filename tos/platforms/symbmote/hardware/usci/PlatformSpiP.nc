
#warning symbmote/hardware/usci/PlatformSpi being used.

module PlatformSpiP {
  provides interface Init;
  uses {
    interface Resource;
    interface Leds;
  }
}

implementation {

  command error_t Init.init() {
    return call Resource.immediateRequest();
  }

  event void Resource.granted() { }
}
