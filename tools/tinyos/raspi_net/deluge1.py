'''
Script to offer simpler user functionality for overall testbed usage by calling tos-deluge functions in the background

Functions to be provided:
1.Code inject and flash i.e. call -ls and -s followed by -i and -dr [slot number]
2.Code switching on all devices using -dr 
3.Pinging to see if a remote node is alive using -pr
4.Switch the edgerouter to sniffer mode

'''

import tosdeluge

print "Usage: python deluge1.py <command> <parameters>\n"
# print "<command> can be :"
if (sys.argv[1] == "flash"):
	if (localstop):
		if(stop):
			print "Command sent"
	else:
		print "Error"

if __name__ == "__main__":
    print "Using as a file"


