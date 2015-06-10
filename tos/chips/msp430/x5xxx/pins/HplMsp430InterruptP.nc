/*
 * Copyright (c) 2011,2013 Eric B. Decker
 * Copyright (c) 2000-2005 The Regents of the University of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Joe Polastre
 * @author Eric B. Decker <cire831@gmail.com>
 *
 * x5 version.  The x1 and x2 processors do not have the P1IV or P2IV
 * interrupt vector registers.  x1x2 mcus need to manuall clear the
 * corresponding interrupt flag.  While the x5 mcus automatically clear
 * the highest priority pending interrupt flag when P1IV or P2IV are
 * acessed.
 */

module HplMsp430InterruptP @safe() {

#if defined(__msp430_have_port1) || defined(__MSP430_HAS_PORT1__) || defined(__MSP430_HAS_PORT1_R__)
  provides interface HplMsp430Interrupt as Port10;
  provides interface HplMsp430Interrupt as Port11;
  provides interface HplMsp430Interrupt as Port12;
  provides interface HplMsp430Interrupt as Port13;
  provides interface HplMsp430Interrupt as Port14;
  provides interface HplMsp430Interrupt as Port15;
  provides interface HplMsp430Interrupt as Port16;
  provides interface HplMsp430Interrupt as Port17;
#endif
#if defined(__msp430_have_port2) || defined(__MSP430_HAS_PORT2__) || defined(__MSP430_HAS_PORT2_R__)
  provides interface HplMsp430Interrupt as Port20;
  provides interface HplMsp430Interrupt as Port21;
  provides interface HplMsp430Interrupt as Port22;
  provides interface HplMsp430Interrupt as Port23;
  provides interface HplMsp430Interrupt as Port24;
  provides interface HplMsp430Interrupt as Port25;
  provides interface HplMsp430Interrupt as Port26;
  provides interface HplMsp430Interrupt as Port27;
#endif
}

implementation {

#if defined(__msp430_have_port1) || defined(__MSP430_HAS_PORT1__) || defined(__MSP430_HAS_PORT1_R__)
  TOSH_SIGNAL(PORT1_VECTOR) {
    uint8_t n = P1IV;

    switch(n) {
      case 0:   { return; }
      case 2:   { signal Port10.fired(); return; }
      case 4:   { signal Port11.fired(); return; }
      case 6:   { signal Port12.fired(); return; }
      case 8:   { signal Port13.fired(); return; }
      case 10:  { signal Port14.fired(); return; }
      case 12:  { signal Port15.fired(); return; }
      case 14:  { signal Port16.fired(); return; }
      case 16:  { signal Port17.fired(); return; }

      default:
                return;
    }
  }

  default async event void Port10.fired() { }
  default async event void Port11.fired() { }
  default async event void Port12.fired() { }
  default async event void Port13.fired() { }
  default async event void Port14.fired() { }
  default async event void Port15.fired() { }
  default async event void Port16.fired() { }
  default async event void Port17.fired() { }

  async command void Port10.enable() { P1IE |= (1 << 0); }
  async command void Port11.enable() { P1IE |= (1 << 1); }
  async command void Port12.enable() { P1IE |= (1 << 2); }
  async command void Port13.enable() { P1IE |= (1 << 3); }
  async command void Port14.enable() { P1IE |= (1 << 4); }
  async command void Port15.enable() { P1IE |= (1 << 5); }
  async command void Port16.enable() { P1IE |= (1 << 6); }
  async command void Port17.enable() { P1IE |= (1 << 7); }

  async command void Port10.disable() { P1IE &= ~(1 << 0); }
  async command void Port11.disable() { P1IE &= ~(1 << 1); }
  async command void Port12.disable() { P1IE &= ~(1 << 2); }
  async command void Port13.disable() { P1IE &= ~(1 << 3); }
  async command void Port14.disable() { P1IE &= ~(1 << 4); }
  async command void Port15.disable() { P1IE &= ~(1 << 5); }
  async command void Port16.disable() { P1IE &= ~(1 << 6); }
  async command void Port17.disable() { P1IE &= ~(1 << 7); }

  async command void Port10.clear() { P1IFG &= ~(1 << 0); }
  async command void Port11.clear() { P1IFG &= ~(1 << 1); }
  async command void Port12.clear() { P1IFG &= ~(1 << 2); }
  async command void Port13.clear() { P1IFG &= ~(1 << 3); }
  async command void Port14.clear() { P1IFG &= ~(1 << 4); }
  async command void Port15.clear() { P1IFG &= ~(1 << 5); }
  async command void Port16.clear() { P1IFG &= ~(1 << 6); }
  async command void Port17.clear() { P1IFG &= ~(1 << 7); }

  async command bool Port10.getValue() { bool b; atomic b=(P1IN >> 0) & 1; return b; }
  async command bool Port11.getValue() { bool b; atomic b=(P1IN >> 1) & 1; return b; }
  async command bool Port12.getValue() { bool b; atomic b=(P1IN >> 2) & 1; return b; }
  async command bool Port13.getValue() { bool b; atomic b=(P1IN >> 3) & 1; return b; }
  async command bool Port14.getValue() { bool b; atomic b=(P1IN >> 4) & 1; return b; }
  async command bool Port15.getValue() { bool b; atomic b=(P1IN >> 5) & 1; return b; }
  async command bool Port16.getValue() { bool b; atomic b=(P1IN >> 6) & 1; return b; }
  async command bool Port17.getValue() { bool b; atomic b=(P1IN >> 7) & 1; return b; }

/*  async command void Port10.edgeRising() { P1IES &= 0xfe; }
  async command void Port11.edgeRising() { P1IES &= 0xfd; }
  async command void Port12.edgeRising() { P1IES &= 0xfb; }
  async command void Port13.edgeRising() { P1IES &= 0xf7; }
  async command void Port14.edgeRising() { P1IES &= 0xef; }
  async command void Port15.edgeRising() { P1IES &= 0xdf; }
  async command void Port16.edgeRising() { P1IES &= 0xbf; }
  async command void Port17.edgeRising() { P1IES &= 0x7f; }

  async command void Port10.edgeFalling() { P1IES |= 0x01; }
  async command void Port11.edgeFalling() { P1IES |= 0x02; }
  async command void Port12.edgeFalling() { P1IES |= 0x04; }
  async command void Port13.edgeFalling() { P1IES |= 0x08; }
  async command void Port14.edgeFalling() { P1IES |= 0x10; }
  async command void Port15.edgeFalling() { P1IES |= 0x20; }
  async command void Port16.edgeFalling() { P1IES |= 0x40; }
  async command void Port17.edgeFalling() { P1IES |= 0x80; }*/

  async command void Port10.edge(bool l2h) {
    atomic {
      if (l2h)  P1IES &= ~(1 << 0);
      else      P1IES |=  (1 << 0);
    }
  }

  async command void Port11.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 1); 
      else      P1IES |=  (1 << 1);
    }
  }

  async command void Port12.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 2); 
      else      P1IES |=  (1 << 2);
    }
  }

  async command void Port13.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 3); 
      else      P1IES |=  (1 << 3);
    }
  }

  async command void Port14.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 4); 
      else      P1IES |=  (1 << 4);
    }
  }

  async command void Port15.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 5); 
      else      P1IES |=  (1 << 5);
    }
  }

  async command void Port16.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 6); 
      else      P1IES |=  (1 << 6);
    }
  }

  async command void Port17.edge(bool l2h) { 
    atomic {
      if (l2h)  P1IES &= ~(1 << 7); 
      else      P1IES |=  (1 << 7);
    }
  }
#endif

#if defined(__msp430_have_port2) || defined(__MSP430_HAS_PORT2__) || defined(__MSP430_HAS_PORT2_R__)
  TOSH_SIGNAL(PORT2_VECTOR) {
    uint8_t n = P2IV;

    switch(n) {
      case 0:   { return; }
      case 2:   { signal Port20.fired(); return; }
      case 4:   { signal Port21.fired(); return; }
      case 6:   { signal Port22.fired(); return; }
      case 8:   { signal Port23.fired(); return; }
      case 10:  { signal Port24.fired(); return; }
      case 12:  { signal Port25.fired(); return; }
      case 14:  { signal Port26.fired(); return; }
      case 16:  { signal Port27.fired(); return; }

      default:
                return;
    }
  }

  default async event void Port20.fired() { }
  default async event void Port21.fired() { }
  default async event void Port22.fired() { }
  default async event void Port23.fired() { }
  default async event void Port24.fired() { }
  default async event void Port25.fired() { }
  default async event void Port26.fired() { }
  default async event void Port27.fired() { }

  async command void Port20.enable() { P2IE |= (1 << 0); }
  async command void Port21.enable() { P2IE |= (1 << 1); }
  async command void Port22.enable() { P2IE |= (1 << 2); }
  async command void Port23.enable() { P2IE |= (1 << 3); }
  async command void Port24.enable() { P2IE |= (1 << 4); }
  async command void Port25.enable() { P2IE |= (1 << 5); }
  async command void Port26.enable() { P2IE |= (1 << 6); }
  async command void Port27.enable() { P2IE |= (1 << 7); }

  async command void Port20.disable() { P2IE &= ~(1 << 0); }
  async command void Port21.disable() { P2IE &= ~(1 << 1); }
  async command void Port22.disable() { P2IE &= ~(1 << 2); }
  async command void Port23.disable() { P2IE &= ~(1 << 3); }
  async command void Port24.disable() { P2IE &= ~(1 << 4); }
  async command void Port25.disable() { P2IE &= ~(1 << 5); }
  async command void Port26.disable() { P2IE &= ~(1 << 6); }
  async command void Port27.disable() { P2IE &= ~(1 << 7); }

  async command void Port20.clear() { P2IFG &= ~(1 << 0); }
  async command void Port21.clear() { P2IFG &= ~(1 << 1); }
  async command void Port22.clear() { P2IFG &= ~(1 << 2); }
  async command void Port23.clear() { P2IFG &= ~(1 << 3); }
  async command void Port24.clear() { P2IFG &= ~(1 << 4); }
  async command void Port25.clear() { P2IFG &= ~(1 << 5); }
  async command void Port26.clear() { P2IFG &= ~(1 << 6); }
  async command void Port27.clear() { P2IFG &= ~(1 << 7); }

  async command bool Port20.getValue() { bool b; atomic b=(P2IN >> 0) & 1; return b; }
  async command bool Port21.getValue() { bool b; atomic b=(P2IN >> 1) & 1; return b; }
  async command bool Port22.getValue() { bool b; atomic b=(P2IN >> 2) & 1; return b; }
  async command bool Port23.getValue() { bool b; atomic b=(P2IN >> 3) & 1; return b; }
  async command bool Port24.getValue() { bool b; atomic b=(P2IN >> 4) & 1; return b; }
  async command bool Port25.getValue() { bool b; atomic b=(P2IN >> 5) & 1; return b; }
  async command bool Port26.getValue() { bool b; atomic b=(P2IN >> 6) & 1; return b; }
  async command bool Port27.getValue() { bool b; atomic b=(P2IN >> 7) & 1; return b; }

/*  async command void Port20.edgeRising() { P2IES &= 0xfe; }
  async command void Port21.edgeRising() { P2IES &= 0xfd; }
  async command void Port22.edgeRising() { P2IES &= 0xfb; }
  async command void Port23.edgeRising() { P2IES &= 0xf7; }
  async command void Port24.edgeRising() { P2IES &= 0xef; }
  async command void Port25.edgeRising() { P2IES &= 0xdf; }
  async command void Port26.edgeRising() { P2IES &= 0xbf; }
  async command void Port27.edgeRising() { P2IES &= 0x7f; }

  async command void Port20.edgeFalling() { P2IES |= 0x01; }
  async command void Port21.edgeFalling() { P2IES |= 0x02; }
  async command void Port22.edgeFalling() { P2IES |= 0x04; }
  async command void Port23.edgeFalling() { P2IES |= 0x08; }
  async command void Port24.edgeFalling() { P2IES |= 0x10; }
  async command void Port25.edgeFalling() { P2IES |= 0x20; }
  async command void Port26.edgeFalling() { P2IES |= 0x40; }
  async command void Port27.edgeFalling() { P2IES |= 0x80; }*/

  async command void Port20.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 0);
      else      P2IES |=  (1 << 0);
    }
  }

  async command void Port21.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 1);
      else      P2IES |=  (1 << 1);
    }
  }  

  async command void Port22.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 2);
      else      P2IES |=  (1 << 2);
    }
  }  

  async command void Port23.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 3);
      else      P2IES |=  (1 << 3);
    }
  }  

  async command void Port24.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 4);
      else      P2IES |=  (1 << 4);
    }
  }

  async command void Port25.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 5);
      else      P2IES |=  (1 << 5);
    }
  }

  async command void Port26.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 6);
      else      P2IES |=  (1 << 6);
    }
  }

  async command void Port27.edge(bool l2h) {
    atomic {
      if (l2h)  P2IES &= ~(1 << 7);
      else      P2IES |=  (1 << 7);
    }
  }
#endif
}
