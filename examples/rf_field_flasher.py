#!/usr/bin/python3
"""Turn the RF field on and off repeatedly at the specified frequency
Requires uFR firmware v5.0.51 or greater
"""

### Modules
import sys
import argparse
from time import sleep
from datetime import datetime

sys.path.append("..")
from pyufr import uFR, uFRRFfieldCtl



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
  argparser.add_argument(
	  "-f", "--frequency",
	  help = "Frequency to flash the RF field at in Hz",
	  type = float,
	  required = True
	)
  args = argparser.parse_args()

  # Open the reader (assert restore_on_close so the reader is automatically
  # restored to the state we found it in upon closing)
  with uFR.open(args.device, restore_on_close = True) as ufr:

    # Enable anti-collision to disable polling
    ufr.enable_anti_collision()

    rf_field_on = True
    warn_if_too_fast = True

    half_period = .5 / args.frequency

    while True:

      # Invert the field, measure how long it takes to send the command and
      # calculate what additional delay to wait
      cmd_start_tstamp = datetime.now().timestamp()

      rf_field_on = not rf_field_on
      ufr.rf_reset(uFRRFfieldCtl.ON if rf_field_on else uFRRFfieldCtl.OFF)

      cmd_duration = datetime.now().timestamp() - cmd_start_tstamp
      remaininng_wait = half_period - cmd_duration

      # If the command took longer than one half period, warn the user
      if remaininng_wait <= 0:
        if warn_if_too_fast:
          print("Warning: {} too fast for your uFR reader and/or your " \
		"computer.".format(args.frequency))
          warn_if_too_fast = False
        continue

      # Sleep long enough to wait for the end of the half period
      sleep(remaininng_wait)
