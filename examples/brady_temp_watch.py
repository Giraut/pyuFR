#!/usr/bin/python3
"""Continuously read a Brady temperature sensing NFC tag
https://www.brady.co.uk/labels/rfid-temperature-labels
"""

### Modules
import re
import sys
import six
import ndef
import argparse

sys.path.append("..")
import pyufr



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
  ufr = pyufr.ufr()
  ufr.open(args.device)

  try:

    ufr.self_reset()
    ufr.enable_anti_collision()

    # Continuously read the tag
    while True:

      uid = None
      uri = None

      # Get the Brady URI
      try:
        ufr.enum_cards()
        uid = ufr.list_cards()[0]
        ufr.select_card(uid)
        n = ufr.linear_read(pyufr.ufrauthmode.T2T_NO_PWD_AUTH, 10, 150)
        uri = list(ndef.message_decoder(n))[0].iri
        ufr.deselect_card()
      except KeyboardInterrupt:
        raise
      except:
        continue

      # Process the URI
      m = re.findall("rfidtemperaturemonitoring\?n=([0-9]+)&d=([0-9]+)C&"
			"o=([0-9a-f]+)&i=([0-9a-f]+)&h=([0-9a-f]+)", uri, re.I)
      if not m:
        continue

      n, d, o, i, h = m[0]

      counter = int(n)
      temp_c = -40 + int(d) / 10
      temp_f = temp_c * 1.8 + 32
      serial = i

      # Display the tag's data
      print("Brady temperature sensing label:")
      print("  UID          = {}".format(uid))
      print("  S/N          = {}".format(serial))
      print("  Read counter = {}".format(counter))
      print("  Temp         = {:.1f}C / {:.1f}F".format(temp_c, temp_f))

      sys.stdout.flush()

  except KeyboardInterrupt:

    t, v, tb = sys.exc_info()

    # Return the reader to the default configuration
    ufr.flush()

    if ufr.get_anti_collision_status():
      ufr.disable_anti_collision()	# re-enable polling

    # Close the reader
    ufr.close()

    # Re-raise KeyboardInterrupt
    six.reraise(t, v, tb)

  except:
    raise
