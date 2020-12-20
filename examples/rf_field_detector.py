#!/usr/bin/python3
"""Continuously monitor external RF field changes and print the pulses' timings
Useful to determine if a cellphone's NFC chip is working properly for example
"""

### Modules
import re
import sys
import six
import ndef
import argparse
from datetime import datetime

sys.path.append("..")
from pyufr import uFR, uFRemumode



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

  # Initialize the reader
  ufr = uFR()
  ufr.open(args.device)

  # If we find the reader already in ad-hoc mode, disable it, otherwise it
  # won't accept the reset and anti-collision commands
  if ufr.get_reader_status()[1] != uFRemumode.TAG_EMU_DISABLED:
    ufr.ad_hoc_emulation_stop()

  ufr.self_reset()

  try:

    ufr.enable_anti_collision()	# Disable polling in ad-hoc emulation mode
    ufr.ad_hoc_emulation_start()

    # Measure RF field on / off timings until the user hits CTRL-C
    rf_state = None
    last_state_change_tstamp = None

    print("Bring NFC device within range of the reader...")

    while True:

      # Read the state of the external field
      rf_state_prev = rf_state
      rf_state = ufr.get_external_field_state()
      if rf_state_prev is None:
        rf_state_prev = rf_state

      # Output state changes and timings
      if rf_state != rf_state_prev:

        s = "RF field {}".format("ON " if rf_state else "OFF")

        now = datetime.now().timestamp()
        if last_state_change_tstamp is not None:
          duration = now - last_state_change_tstamp
          if duration > 0.1:
            s += " after {:0.1f} s".format(duration)
          else:
            s += " after {:0.0f} ms".format(duration * 1000)
        last_state_change_tstamp = now

        print(s)
        sys.stdout.flush()

  except KeyboardInterrupt:

    t, v, tb = sys.exc_info()

    # Return the reader to the default configuration
    print("Restoring reader...")

    ufr.flush()

    if ufr.get_reader_status()[1] != uFRemumode.TAG_EMU_DISABLED:
      ufr.ad_hoc_emulation_stop()

    if ufr.get_anti_collision_status():
      ufr.disable_anti_collision()	# re-enable polling

    # Close the reader
    ufr.close()

    # Re-raise KeyboardInterrupt
    six.reraise(t, v, tb)

  except:
    raise
