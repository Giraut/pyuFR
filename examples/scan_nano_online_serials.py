#!/usr/bin/python3
"""Scan a local network for uFR Nano Online readers, get their serial numbers
and make then beep and flash the LEDs. The readers must be in slave mode.
"""

### Modules
import sys
import argparse

sys.path.append("..")
from pyufr import uFR, uFRlightSignal, uFRbeepSignal



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
  for response in ufr.nano_online_subnet_discovery(args.network):

    sys.stdout.write("IP={}, serial=".format(response.ip))

    # Open the reader, try the uart1 reader first
    with ufr.open("{}://{}:{}".format("udp" if response.uart1.is_udp else "tcp",
				response.ip, response.uart1.port)) as ufrcomm:
      try:
        sys.stdout.write(ufrcomm.get_serial_number() + "\n")
        ufrcomm.user_interface_signal(uFRlightSignal.ALTERNATION,
					uFRbeepSignal.SHORT)
        continue
      except:
        pass

    # Open the reader again, try the uart2 reader next
    with ufr.open("{}://{}:{}".format("udp" if response.uart2.is_udp else "tcp",
				response.ip, response.uart2.port)) as ufrcomm:
      try:
        sys.stdout.write(ufrcomm.get_serial_number() + "\n")
        ufrcomm.user_interface_signal(uFRlightSignal.ALTERNATION,
					uFRbeepSignal.SHORT)
      except:
        sys.stdout.write("?\n")
