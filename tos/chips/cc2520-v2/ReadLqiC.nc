
/*
 * Different platforms have different ways of getting in touch with
 * the LQI reading the radio provides.  This module wraps the
 * different ways in platform-independent logic.
 *
 *
 * @author Stephen Dawson-Haggerty <stevedh@eecs.berkeley.edu>
 */

configuration ReadLqiC {
  provides interface ReadLqi;
} implementation {

  // cc2520 platforms
  components CC2520ReadLqiC, CC2520RadioC;
  ReadLqi = CC2520ReadLqiC;
  CC2520ReadLqiC.PacketLinkQuality -> CC2520RadioC.PacketLinkQuality;
  CC2520ReadLqiC.PacketRSSI -> CC2520RadioC.PacketRSSI;
}
/*configuration ReadLqiC {
	provides interface ReadLqi;
} implementation {
	components CC2520RadioC;
	ReadLqi = CC2520RadioC;
}*/