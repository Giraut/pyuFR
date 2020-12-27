#!/usr/bin/python3
"""Scan a local network for Nano Online readers and get their serial numbers
using the HTTP protocol. Transparent mode must be enabled in the readers.
"""

### Modules
import re
import sys
import ndef
import argparse
from datetime import datetime

sys.path.append("..")
from pyufr import uFR, uFRemuMode



### Main program
if __name__ == "__main__":

  # Parse the command line arguments
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
	  "-n", "--network",
	  help = "Network to scan (default 192.168.1.0/24)",
	  type = str,
	  default = "192.168.1.0/24",
	)
  args = argparser.parse_args()

  ufr = uFR()

  # Probe the network for Nano Online devices
  for ip in ufr.probe_subnet_nano_onlines(args.network):

    sys.stdout.write("IP={}, serial=".format(ip))

    # Open the reader, try the uart1 reader first
    with ufr.open("http://" + ip + "/uart1") as ufrcomm:
      try:
        sys.stdout.write(ufrcomm.get_serial_number() + "\n")
        continue
      except:
        pass

    # Open the reader again, try the uart2 reader next
    with ufr.open("http://" + ip + "/uart2") as ufrcomm:
      try:
        sys.stdout.write(ufrcomm.get_serial_number() + "\n")
      except:
        sys.stdout.write("?\n")
