/**
 * Defining the platform-independently named packet structures to be the
 * chip-specific CC2520 packet structures.
 */

#ifndef PLATFORM_MESSAGE_H
#define PLATFORM_MESSAGE_H

#include <Serial.h>
#include "CC2520Radio.h"

typedef union message_header {
  cc2520packet_header_t cc2520;
  serial_header_t serial;
} message_header_t;

typedef union message_footer {
  cc2520packet_footer_t cc2520;
} message_footer_t;

typedef union message_metadata {
  cc2520packet_metadata_t cc2520;
	serial_metadata_t serial;
} message_metadata_t;

#endif
