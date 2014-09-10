'''
Script to offer simpler user functionality for overall testbed usage by calling tos-deluge functions in the background

Functions to be provided:
1.Code inject and flash i.e. call -ls and -s followed by -i and -dr [slot number]
2.Code switching on all devices using -dr 
3.Pinging to see if a remote node is alive using -pr
4.Switch the edgerouter to sniffer mode

'''


from tosdeluge import *
import sys

print "Usage: python deluge1.py <command> <parameters>\n"
#eg. python deluge1.py flash serial@/dev/ttyUSB0 -ls -s -i 1 build/telosb/tinyos_main.xml -dr 1
#eg. python deluge1.py flash 1 build/telosb/tos_image.xml 

motepath = "serial@/dev/ttyUSB0"


# print "<command> can be :"
if (sys.argv[1] == "flash"):
	slot = int(sys.arv[2])
	filename = sys.argv[3]

	if localstop():
		if stop ():
			if inject (slot,filename):
				print "Command sent"
	else:
		print "Error"

# if __name__ == "__main__":
#     print "Using as a file"


