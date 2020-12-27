#!/usr/bin/python3
"""Continuously read a Brady temperature sensing NFC tag
https://www.brady.co.uk/labels/rfid-temperature-labels
"""

### Modules
import re
import sys
import ndef
import argparse

sys.path.append("..")
from pyufr import uFR, uFRauthMode, uFRresponseError



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

    # Unconditionally exit any emulation mode the reader might be in, otherwise
    # it won't accept the reset and anti-collision commands
    ufr.tag_emulation_stop()
    ufr.ad_hoc_emulation_stop()

    # Stop polling and enable anti-collision
    ufr.self_reset()			# Do this immediately before enabling
					# anti-collision to force the reader to
					# "let go" of any card in the field
					# long enough to prevent polling from
					# restarting before anti-collision is on
    ufr.enable_anti_collision()

    # Continuously read the tag
    while True:

      uid = None
      uri = None

      # Get the Brady URI
      try:

        # Enumerate the cards in the field
        if not ufr.enum_cards():
          continue

        # Retrieve the list of UIDs enumerated
        uids = ufr.list_cards()
        if not uids:
          continue

        # Select the tag, read the memory zone where the NDEF should be,
        # then deselect the tag (the tag's counter increments upon selecting)
        ufr.select_card(uids[0])
        ndef_mem_zone = ufr.linear_read(uFRauthMode.T2T_NO_PWD_AUTH, 10, 150)
        ufr.deselect_card()

        # Try to decode the NDEF
        ndef_records = list(ndef.message_decoder(ndef_mem_zone))

        # Do we have at least one NDEF record?
        if not ndef_records:
          continue

        # Is the NDEF record a URI?
        if not isinstance(ndef_records[0],ndef.uri.UriRecord):
          continue

        # Get the URI as a unicode string
        uri = list(ndef.message_decoder(ndef_mem_zone))[0].iri

      except (TimeoutError, uFRresponseError, ndef.record.DecodeError):
        continue

      except KeyboardInterrupt:

        # Do our own cleanup here instead of letting the class do it
        # automatically, for the sake of example

        print()
        print("Interrupted: restoring and closing the reader")

        ufr.flush()
        ufr.close()

        break

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

    print("Done")
