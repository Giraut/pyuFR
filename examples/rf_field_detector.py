#!/usr/bin/python3
"""Continuously monitor external RF field changes and print the pulses' timings
Useful to determine if a cellphone's NFC chip is working properly for example
"""

### Modules
import re
import sys
import ndef
import argparse
from datetime import datetime

sys.path.append("..")
from pyufr import uFRcomm, uFRemuMode



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

  # Open the reader (assert restore_on_close so the reader is automatically
  # restored to the state we found it in upon closing)
  with uFRcomm(args.device, restore_on_close = True) as ufr:

    # Unconditionally exit any emulation mode the reader might be in, otherwise
    # it won't accept the reset and anti-collision commands
    ufr.tag_emulation_stop()
    ufr.ad_hoc_emulation_stop()

    # Stop polling and disable the field
    ufr.self_reset()			# Do this immediately before enabling
					# anti-collision to force the reader to
					# "let go" of any card in the field
					# long enough to prevent polling from
					# restarting before anti-collision is on
    ufr.enable_anti_collision()		# This disables polling in ad-hoc
					# emulation mode
    ufr.ad_hoc_emulation_start()	# Without polling, this turns off the
					# RF field

    # Measure RF field on / off timings until the user hits CTRL-C
    rf_state = None
    last_state_change_tstamp = None

    print("Bring NFC device within range of the reader...")

    while True:

      rf_state_prev = rf_state

      # Read the state of the external field
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
