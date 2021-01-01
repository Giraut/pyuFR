#!/usr/bin/python3
"""Enable asynchronous ID, wait for the reader to report cards coming in the
field, and make the reader beep with a different frequency for each card
"""

### Modules
import sys
import argparse

sys.path.append("..")
from pyufr import uFR



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
  with uFR.open(args.device, restore_on_close = True) as ufr:

    # Enable asynchronous ID sending
    ufr.set_card_id_send_conf(True)

    print("Bring NFC card close to the reader...")

    # Wait for cards to come in or go out of the field. Quit after 30 seconds of
    # inactivity
    while True:

      # Get an ID if a card comes in the field, or None if the card goes off
      # the field
      try:
        uid = ufr.get_async_id(30)
      except TimeoutError:
        break

      # Determine a beep frequency based on the card's UId - or 0 to disable
      # beeping - and set the reader's buzzer
      if uid:
        print("Card scanned: UID = {}".format(uid))
        beep_frequency = 200 + int(uid[-8:], 16) / 2000000
      else:
        print("Card removed")
        beep_frequency = 0

      ufr.set_speaker_frequency(beep_frequency)

  print("Done")
