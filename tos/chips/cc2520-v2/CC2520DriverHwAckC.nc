/*
 * Copyright (c) 2014, Laksh Bhatia
 * Copyright (c) 2013, Eric B. Decker
 * Copyright (c) 2010, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:  
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Author: Janos Sallai, Miklos Maroti
 * Author: Thomas Schmid (adapted to CC2520)
 * Author: Eric B. Decker (rewritten for cc2520/cc2591)
 *
 * Requires Panic and Platform.
 */

#include <RadioConfig.h>
#include <CC2520DriverLayer.h>

configuration CC2520DriverHwAckC {
  provides {
    interface RadioState;
    interface RadioSend;
    interface RadioReceive;
    interface RadioCCA;
    interface RadioPacket;

    interface PacketField<uint8_t> as PacketTransmitPower;
    interface PacketField<uint8_t> as PacketRSSI;
    interface PacketField<uint8_t> as PacketTimeSyncOffset;
    interface PacketField<uint8_t> as PacketLinkQuality;
    //interface PacketField<uint8_t> as AckReceived;

    interface LocalTime<TRadio> as LocalTimeRadio;
    interface Alarm<TRadio, tradio_size>;

    interface PacketAcknowledgements;
  }
  uses {
    interface CC2520DriverConfig as Config;
    interface PacketTimeStamp<TRadio, uint32_t>;
    interface Ieee154PacketLayer;

    interface PacketFlag as TransmitPowerFlag;
    interface PacketFlag as RSSIFlag;    
    interface PacketFlag as TimeSyncFlag; 
    interface PacketFlag as AckReceivedFlag;
    interface RadioAlarm;     
  }
}

implementation {
  components CC2520DriverHwAckP as DriverLayerP,
             new TaskletC(),
	     MainC,
             HplCC2520C as HplChip,ActiveMessageAddressC;

  MainC.SoftwareInit -> DriverLayerP.SoftwareInit;

  RadioState = DriverLayerP;
  RadioSend = DriverLayerP;
  RadioReceive = DriverLayerP;
  RadioCCA = DriverLayerP;
  RadioPacket = DriverLayerP;
  PacketAcknowledgements = DriverLayerP;

  LocalTimeRadio = HplChip;
  Config = DriverLayerP;

  DriverLayerP.RSTN  -> HplChip.RSTN;
  DriverLayerP.VREN  -> HplChip.VREN;
  DriverLayerP.CSN   -> HplChip.CSN;

  DriverLayerP.SFD   -> HplChip.SFD;
  DriverLayerP.TXA   -> HplChip.TXA;
  DriverLayerP.EXCA  -> HplChip.EXCA;

  PacketTransmitPower = DriverLayerP.PacketTransmitPower;
  DriverLayerP.TransmitPowerFlag = TransmitPowerFlag;

  PacketRSSI = DriverLayerP.PacketRSSI;
  DriverLayerP.RSSIFlag = RSSIFlag;

  PacketTimeSyncOffset = DriverLayerP.PacketTimeSyncOffset;
  DriverLayerP.TimeSyncFlag = TimeSyncFlag;

/*
  AckReceived = DriverLayerP.AckReceived;
  components new CC2520MetadataFlagC() as AckFlagC;
  DriverLayerP.AckFlag -> AckFlagC;
*/

  AckReceivedFlag = DriverLayerP.AckReceivedFlag;

  PacketLinkQuality = DriverLayerP.PacketLinkQuality;
  PacketTimeStamp = DriverLayerP.PacketTimeStamp;

  RadioAlarm = DriverLayerP.RadioAlarm;
  Alarm = HplChip.Alarm;

  DriverLayerP.SpiResource -> HplChip.SpiResource;
  DriverLayerP.FastSpiByte -> HplChip;
  DriverLayerP.SpiByte     -> HplChip;

  DriverLayerP.ExcAInterrupt -> HplChip.ExcAInterrupt;
  DriverLayerP.SfdCapture    -> HplChip.SfdCapture;

  DriverLayerP.Tasklet -> TaskletC;

  DriverLayerP.LocalTime-> HplChip.LocalTimeRadio;
  Ieee154PacketLayer = DriverLayerP;
  DriverLayerP.ActiveMessageAddress -> ActiveMessageAddressC;

#ifdef RADIO_DEBUG_MESSAGES
  components DiagMsgC;
  DriverLayerP.DiagMsg -> DiagMsgC;
#endif

  components CC2520SecurityP;
  DriverLayerP.CC2520Security -> CC2520SecurityP;

  components PlatformCC2520P;
  DriverLayerP.PlatformCC2520       -> PlatformCC2520P;
  PlatformCC2520P.CC2520BasicAccess -> DriverLayerP;

  components TraceC;
  DriverLayerP.Trace       -> TraceC;

#ifdef REQUIRE_PLATFORM
  components PlatformC;
  DriverLayerP.Platform    -> PlatformC;
#endif

#ifdef REQUIRE_PANIC
  components PanicC;
  DriverLayerP.Panic       -> PanicC;
#endif

#ifdef CC2520_HWACK_64BIT
  components LocalIeeeEui64C;
  DriverLayerP.LocalIeeeEui64 -> LocalIeeeEui64C;
  #endif

}
