/*
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

/**
 * Implement the UART-related interfaces for a MSP430 USCI module
 * instance.
 *
 * Interrupt Management
 * --------------------
 *
 * Upon grant of the USCI in UART mode to a client, interrupts are
 * turned off.
 *
 * On the MSP430, when the TX interrupt is raised the MCU
 * automatically clears the UCTXIFG bit that indicates that the TXBUF
 * is available for writing characters.  Rather than maintain local
 * state managed by cooperation between the TX interrupt handler and
 * the send code, we leave the TX interrupt disabled and rely on the
 * UCTXIFG flag to indicate that single-byte transmission is
 * permitted.  It is better and simpler to let the h/w maintain the
 * state.
 *
 * An exception to this is in support of the UartSerial.send()
 * function.  The transmit interrupt is enabled when the outgoing
 * message is provided; subsequent sends are interrupt-driven, and the
 * interrupt is disabled just prior to transmitting the last character
 * of the packet.  This leaves the UCTXIFG flag set upon completion of
 * the transfer.
 *
 * The receive interrupt is enabled upon configuration.  It is
 * controlled using the UartStream functions.  While a buffered
 * receive operation is active, received characters will be stored and
 * no notification provided until the full packet has been received.
 * If no buffered receive operation is active, the receivedByte()
 * event will be signaled for each received character.  Per byte
 * signal per each byte interrupt.
 *
 * As with the transmit interrupt, MCU execution of the receive
 * interrupt clears the UCRXIFG flag, making interrupt-driven
 * reception fundamentally incompatible with the busy-waiting
 * UartByte.receive() method.
 *
 * @author Peter A. Bigot <pab@peoplepowerco.com>
 * @author Eric B. Decker <cire831@gmail.com>
 */

generic module Msp430UsciUartP () @safe() {
  provides {
    interface UartStream[ uint8_t client ];
    interface UartByte[ uint8_t client ];
    interface ResourceConfigure[ uint8_t client ];
    interface Msp430UsciError[ uint8_t client ];
  }
  uses {
    interface HplMsp430Usci as Usci;
    interface HplMsp430UsciInterrupts as Interrupts;
    interface HplMsp430GeneralIO as URXD;
    interface HplMsp430GeneralIO as UTXD;

    interface Msp430UsciConfigure[ uint8_t client ];
    interface ArbiterInfo;
    interface LocalTime<TMilli> as LocalTime_bms;
  }
}
implementation {

  norace uint16_t m_tx_len, m_rx_len;
  norace uint8_t * COUNT_NOK(m_tx_len) m_tx_buf, * COUNT_NOK(m_rx_len) m_rx_buf;
  norace uint16_t m_tx_pos, m_rx_pos;

  /**
   * The UART is busy if it's actively transmitting/receiving, or if
   * there is an active buffered I/O operation.
   */
  bool isBusy () {
    while (call Usci.isBusy()) {
      ;/* busy-wait */
    }
    return (m_tx_buf || m_rx_buf);
  }

  /**
   * The given client is the owner if the USCI is in UART mode and
   * the client is the user stored in the UART arbiter.
   */
  error_t checkIsOwner (uint8_t client) {
    /* Ensure the USCI is in UART mode and we're the owning client */
    const uint8_t current_client = call ArbiterInfo.userId();

    if (0xFF == current_client)
      return EOFF;

    if (current_client != client)
      return EBUSY;

    return SUCCESS;
  }

  /**
   * Take the USCI out of UART mode.
   *
   * Assumes the USCI is currently in UART mode.  This will busy-wait
   * until any characters being actively transmitted or received are
   * out of their shift register.  It disables the interrupts, puts
   * the USCI into reset, and returns the UART-related pins back to
   * IO mode.
   *
   * The USCI is left in reset mode to avoid power drain per UCS6.
   */
  void unconfigure_ () {
    while (UCBUSY & (call Usci.getStat())) {
      ;/* busy-wait, FIX-ME */
    }
    /*
     * formerly, first thing we turned off the interrupt enables
     * but kicking reset turns off the interrupt enables.
     */
    call Usci.enterResetMode_();
    call URXD.makeOutput();
    call URXD.selectIOFunc();
    call UTXD.makeOutput();
    call UTXD.selectIOFunc();
  }

  /**
   * Configure the USCI for UART mode.
   *
   * Invoke the USCI configuration to set up the serial speed, but
   * leaves USCI in reset mode on completion.  This function then
   * follows up by setting the UART-related pins to their module role
   * prior to taking the USCI out of reset mode.  The RX interrupt is
   * enabled, and TX is disabled..
   */
  error_t configure_ (const msp430_usci_config_t* config) {
    if (! config)
      return FAIL;

    /*
     * Do basic configuration, leaving USCI in reset mode.  Configure
     * the UART pins, enable the USCI, and turn on the rx interrupt.
     */
    atomic {
      call Usci.configure(config, TRUE);
      call URXD.makeInput();
      call URXD.selectModuleFunc();
      call UTXD.makeOutput();
      call UTXD.selectModuleFunc();

      /*
       * all configured.  before leaving reset and turning on interrupts
       * reset the state variables about where we are in the buffer.
       */
      m_tx_buf = m_rx_buf = 0;		/* really?  do we want to do this? */
      call Usci.leaveResetMode_();

      /* any IE bits are cleared on reset, turn on RX interrupt */
      call Usci.enableRxIntr();
    }
    return SUCCESS;
  }

  /**
   * Transmit the next character in the outgoing message.
   *
   * Assumes the USCI is in UART mode and the owning client has
   * supplied a transmission buffer using UartStream.  This method is
   * only invoked by the transmit interrupt handler when TXBUF is
   * ready to receive a new character.
   */
  void nextStreamTransmit (uint8_t client) {
    uint8_t ch;
    bool    last_char;

    atomic {
      ch = m_tx_buf[m_tx_pos++];
      last_char = (m_tx_pos >= m_tx_len);

      if (last_char) {
        /*
	 * Disable TX interrupt, UCTXIFG will still be asserted
	 * once the next char going out finishes.
	 */
	call Usci.disableTxIntr();
      }
      call Usci.setTxbuf(ch);

      /*
       * On completion, disable the transmit infrastructure prior to
       * signaling completion.
       */
      if (last_char) {
        uint8_t* tx_buf;
        uint16_t tx_len;

	tx_buf = m_tx_buf;
	tx_len = m_tx_len;
        m_tx_buf = 0;
        signal UartStream.sendDone[client](tx_buf, tx_len, SUCCESS);
      }
    }
  }

  async command error_t UartStream.send[uint8_t client]( uint8_t* buf, uint16_t len ) {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero -> error */
      return rv;

    if (isBusy())
      return EBUSY;

    if (!len || !buf)
      return FAIL;

    m_tx_buf = buf;
    m_tx_len = len;
    m_tx_pos = 0;
    /*
     * On start up UCTXIFG should be asserted, so enabling the TX interrupt
     * should cause the ISR to get invoked.
     */
    call Usci.enableTxIntr();
    return SUCCESS;
  }


  default async event void UartStream.sendDone[uint8_t client]
    (uint8_t* buf, uint16_t len, error_t error ) { }


  /*
   * The behavior of UartStream during reception is not well defined.
   * In the original Msp430UartP implementation, both transmit and
   * receive interrupts were enabled upon UART configuration.  As
   * noted earlier, we keep the transmit interrupt disabled to
   * simplify control flow, but we do enable the receive interrupt for
   * backwards compatibility.
   * 
   * If Usci.receive(uint8_t*,uint16_t) is called, then subsequent received
   * characters will be stored into the buffer until completion, and
   * the receivedByte(uint8_t) event will not be signaled.  If no
   * buffered receive is active, then receivedByte(uint8_t) will be
   * signaled.
   *
   * There is no coordination with UartByte, for which the receive
   * operation simply busy-waits until the interrupt register
   * indicates data is available.  If UartStream's
   * enableReceiveInterrupt() is in force, it is probable that the
   * loop will timeout as the interrupt will clear the flag
   * register.
   *
   * When the UART client releases control (unconfigures the UART),
   * the UART is left in reset which also disables all interrupt enables.
   */

  async command error_t UartStream.enableReceiveInterrupt[uint8_t client]() {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return rv;
    
    call Usci.enableRxIntr();
    return rv;				/* SUCCESS */
  }

  async command error_t UartStream.disableReceiveInterrupt[uint8_t client]() {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return rv;
    
    call Usci.disableRxIntr();
    return rv;				/* SUCCESS */
  }

  default async event void UartStream.receivedByte[uint8_t client]( uint8_t byte ) { }

  async command error_t UartStream.receive[uint8_t client]( uint8_t* buf, uint16_t len ) {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return rv;
    
    if (!len || !buf)
      return FAIL;

    atomic {
      if (m_rx_buf)
        return EBUSY;

      m_rx_buf = buf;
      m_rx_len = len;
      m_rx_pos = 0;
    }
    return SUCCESS;
  }


  default async event void UartStream.receiveDone[uint8_t client]
    (uint8_t* buf, uint16_t len, error_t error) { }


  async command error_t UartByte.send[uint8_t client]( uint8_t byte ) {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return rv;
    
    if (m_tx_buf)
      return EBUSY;

    /* Wait for TXBUF to become available */
    while (!(call Usci.isTxIntrPending())) {
    }

    /*
     * Transmit the character.  Note that it hasn't actually gone out
     * over the wire until UCBUSY (see UCmxSTAT) is cleared.
     */
    call Usci.setTxbuf(byte);

    /*
     * wait until it's actually sent.   This kills the pipeline and sucks
     * performancewise.
     */
    while(call Usci.isBusy()) {
    }
    return SUCCESS;
  }


  /*
   * Check to see if space is available for another transmit byte to go out.
   *
   * If something goes wrong, just return FALSE (no space is available).
   */
  async command bool UartByte.sendAvail[uint8_t client]() {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return FALSE;

    /* isTxIntrPending returns TRUE if space is available */
    return (call Usci.isTxIntrPending());
  }


  enum {
    /**
     * The timeout for UartByte.receive is specified in "byte times",
     * which we can't know without reverse engineering the clock
     * subsystem.  Assuming a 57600 baud system, one byte takes
     * roughly 170usec to transmit (ten bits per byte), or about five
     * byte times per (binary) millisecond.
     *
     * This is seriously wedged because it is platform dependent.
     * The platform interface should specify a byte time in number
     * of microseconds (of whatever flavor the platform is providing).
     */
    ByteTimesPerMillisecond = 5,

    /**
     * Using an 8-bit value to represent a count of events with
     * sub-millisecond duration is a horrible interface for humans:
     * gives us at most 52msec to react.  For testing purposes, scale
     * that by some value (e.g., 100 will increase the maximum delay
     * to 5 seconds).
     */
    ByteTimeScaleFactor = 1,
  };

  async command error_t UartByte.receive[uint8_t client]( uint8_t* byte, uint8_t timeout_bt ) {
    error_t rv;
    uint32_t startTime_bms, timeout_bms;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return FALSE;

    if (! byte)
      return FAIL;

    if (m_rx_buf)
      return EBUSY;

    startTime_bms = call LocalTime_bms.get();
    timeout_bms = ByteTimeScaleFactor * ((ByteTimesPerMillisecond + timeout_bt - 1) / ByteTimesPerMillisecond);

    while (! call Usci.isRxIntrPending()) {
      if((call LocalTime_bms.get() - startTime_bms) > timeout_bms)
        return FAIL;
    }

    *byte = call Usci.getRxbuf();
    return SUCCESS;
  }

  /*
   * Check to see if another Rx byte is available.
   *
   * If something goes wrong, just return FALSE (no byte is available).
   */
  async command bool UartByte.receiveAvail[uint8_t client]() {
    error_t rv;

    if ((rv = checkIsOwner(client)))	/* non-zero, error bail out */
      return FALSE;

    /* isRxIntrPending returns TRUE if another rx byte is available */
    return (call Usci.isRxIntrPending());
  }


  async event void Interrupts.interrupted (uint8_t iv) {
    uint8_t current_client;

    current_client = call ArbiterInfo.userId();

    if (0xFF == current_client)
      return;

    if (USCI_UCRXIFG == iv) {
      uint8_t stat = call Usci.getStat();
      uint8_t data = call Usci.getRxbuf();

      /*
       * SLAU259 16.3.6: Errors are cleared by reading UCAxRXD.  Grab
       * the old errors, read the incoming data, then read the errors
       * again in case an overrun occurred between reading STATx and
       * RXD.  Mask off the bits we don't care about, and if there are
       * any left on notify somebody.
       */
      stat = MSP430_USCI_ERR_UCxySTAT & (stat | (call Usci.getStat()));
      if (stat)
        signal Msp430UsciError.condition[current_client](stat);

      if (m_rx_buf) {
        m_rx_buf[m_rx_pos++] = data;
        if (m_rx_len == m_rx_pos) {
          uint8_t* rx_buf = m_rx_buf;
          uint16_t rx_len = m_rx_len;
          m_rx_buf = 0;
          signal UartStream.receiveDone[current_client](rx_buf, rx_len, SUCCESS);
        }
      } else
        signal UartStream.receivedByte[current_client](data);
      return;
    }

    if (USCI_UCTXIFG == iv)
      nextStreamTransmit(current_client);
  }

  default async command const msp430_usci_config_t*
    Msp430UsciConfigure.getConfiguration[uint8_t client] () {
      return &msp430_usci_uart_default_config;
  }

  async command void ResourceConfigure.configure[uint8_t client] () {
    configure_(call Msp430UsciConfigure.getConfiguration[client]());
  }

  async command void ResourceConfigure.unconfigure[uint8_t client] () {
    unconfigure_();
  }

  default async event void Msp430UsciError.condition[uint8_t client] (unsigned int errors) { }
}
