#!/usr/bin/python3

### Modules
import sys
import argparse

sys.path.append("..")
from pyufr import uFR, uFRtagCommType



### Parameters
tag_type = uFRtagCommType.ISO14443_TYPE_A
set_factory_setting = False



### Main program
if __name__ == "__main__":

  # Parse the command line arguments
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
	  "-d", "--device",
	  help = "uFR device (e.g. serial:///dev/ttyUSB0:1000000)",
	  type = str,
	  required = True
	)
  args = argparser.parse_args()

  # Open the reader
  with uFR.open(args.device, restore_on_close = True) as ufr:

    # Get current values
    rfas = ufr.get_rf_analog_settings(tag_type)

    # Decode current RFCfgReg
    rfcfgreg = rfas[0]
    rflevelamp = (rfcfgreg & 0x80) != 0
    rxgain = (rfcfgreg & 0x70) >> 4
    rflevel = rfcfgreg & 0x0f

    # Print current values
    print("Current RF configuration:")
    print("  RFCfgRed = x{v:02x} (b{v:08b})".format(v = rfcfgreg))
    print("    RFLevelAmp = {}".format(rflevelamp))
    print("    RxGain = x{v:01x} (b{v:03b})".format(v = rxgain))
    print("    RFLevel = x{v:01x} (b{v:04b})".format(v = rflevel))
    print()

    # Modify values
    rflevelamp = False
    rxgain = 0b111	# 48 dB - Factory value is 33 dB (0b100)

    # Encode new RFCfgReg
    rfcfgreg = rflevel + (rxgain << 4) + (0x80 if rflevelamp else 0)
    rfas[0] = rfcfgreg

    # Print new value
    print("New RF configuration:")
    print("  RFCfgRed = x{v:02x} (b{v:08b})".format(v = rfcfgreg))
    print()

    # Ask if the user wants to write the new value
    do_write = input("Write [Y/N]? ") in "yY"

    # Write the value if asked
    if do_write:
      ufr.set_rf_analog_settings(tag_type, set_factory_setting, rfas)

  print("Done")
