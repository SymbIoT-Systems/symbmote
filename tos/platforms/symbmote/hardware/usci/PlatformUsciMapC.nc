/*
 * Copyright (c) 2014 Laksh Bhatia
 * Copyright (c) 2013 Eric B. Decker
 * Copyright (c) 2009-2010 People Power Co.
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
 * - Neither the name of the copyright holders nor the names of
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
 */

#include "msp430usci.h"

/*
 * 
 * Connect the appropriate pins for USCI support on a 5438a
 *
 * @author Peter A. Bigot <pab@peoplepowerco.com>
 * @author Eric B. Decker <cire831@gmail.com>
 * @author Laksh Bhatia
 */

configuration PlatformUsciMapC {
} implementation {
  components HplMsp430GeneralIOC as GIO;
  components PanicC, PlatformC;

  components Msp430UsciUartA0P as UartA0C;
  UartA0C.URXD    -> GIO.UCA0RXD;
  UartA0C.UTXD    -> GIO.UCA0TXD;

  components Msp430UsciUartA1P as UartA1C;
  UartA1C.URXD    -> GIO.UCA1RXD;
  UartA1C.UTXD    -> GIO.UCA1TXD;

  components Msp430UsciSpiB0P as SpiB0C;
  SpiB0C.SIMO     -> GIO.UCB0SIMO;
  SpiB0C.SOMI     -> GIO.UCB0SOMI;
  SpiB0C.CLK      -> GIO.UCB0CLK;
  SpiB0C.Platform -> PlatformC;
  SpiB0C.Panic    -> PanicC;

  components Msp430UsciSpiA1P as SpiA1C;
  SpiA1C.SIMO     -> GIO.UCA1SIMO;
  SpiA1C.SOMI     -> GIO.UCA1SOMI;
  SpiA1C.CLK      -> GIO.UCA1CLK;
  SpiA1C.Platform -> PlatformC;
  SpiA1C.Panic    -> PanicC;
}
