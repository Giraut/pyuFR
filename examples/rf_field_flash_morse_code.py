#!/usr/bin/python3
"""Modulate a uFR reader's RF field with a Morse code message. The message may
be made visible using a LED connected to tank tuned to 13.56 MHz, such as a
Dangerous Things X Field Detector (https://dangerousthings.com/product/xfd/) or
RFID Diagnostic Card (https://dangerousthings.com/product/rdc/).
Requires uFR firmware v5.0.51 or greater
"""

### Parameters
default_wpm         = 10 #words per minute
message_start_pause = "   "
message_end_pause   = "   "



### Modules
import sys
import argparse
from time import sleep
from datetime import datetime

sys.path.append("..")
from pyufr import uFR, uFRRFfieldCtl



### Defines
# Morse code dictionary
morsecode = {
	"A":     ".-",
	"B":     "-...",
	"C":     "-.-.",
	"D":     "-..",
	"E":     ".",
	"F":     "..-.",
	"G":     "--.",
	"H":     "....",
	"I":     "..",
	"J":     ".---",
	"K":     "-.-",
	"L":     ".-..",
	"M":     "--",
	"N":     "-.",
	"O":     "---",
	"P":     ".--.",
	"Q":     "--.-",
	"R":     ".-.",
	"S":     "...",
	"T":     "-",
	"U":     "..-",
	"V":     "...-",
	"W":     ".--",
	"X":     "-..-",
	"Y":     "-.--",
	"Z":     "--..",
	"1":     ".----",
	"2":     "..---",
	"3":     "...--",
	"4":     "....-",
	"5":     ".....",
	"6":     "-....",
	"7":     "--...",
	"8":     "---..",
	"9":     "----.",
	"0":     "-----",
	"=":     "-...-",
	"/":     "-..-.",
	"?":     "..--..",
	",":     "--..--",
	".":     ".-.-.-",
	":":     "---...",
	"'":     ".----.",
	'"':     ".-..-.",
	"_":     "..--.-",
	"(":     "-.--.",
	")":     "-.--.-",
	"#":     "-.---",
	"-":     "-....-",
	"|":     "...-..",
	"\\":    "-.....",
	"*":     "-----.",
	";":     "-.-.-.",
	"@":     ".--.-.",
	"^":     "....--.-.",
	"$":     "...-..-",
	"!":     "....-.",
	">":     "....---.",
	"]":     "....-....",
	"[":     "....-..",
	"<":     "....-.-..",
	"&":     "....--.",
	"%":     "....-.--.",
	"~":     "....--",
	"+":     ".-.-.",
	"{":     "....-.--",
	"}":     "....--..-",
	"[AR]":  ".-.-.",
	"[AS]":  ".-...",
	"[BK]":  "-...-.-",
	"[BT]":  "-...-",
	"[KA]":  "-.-.-",
	"[CL]":  "-.-..-..",
	"[KN]":  "-.--.",
	"[VA]":  "...-.-",
	"[VE]":  "...-.",
	"[GR]":  "--..-.",
	"[HM]":  "....--",
	"[IX]":  "..-..-",
	"[IMI]": "..--..",
	"[INT]": "..-.-",
	"[SOS]": "...---..."}



### Main routine
if __name__ == "__main__":

  # Read the command line arguments
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
	  "-d", "--device",
	  help = "uFR device (e.g. serial:///dev/ttyUSB0:1000000)",
	  type = str,
	  required = True
	)
  argparser.add_argument(
	  "-w", "--words-per-minute",
	  type = int,
	  help = "Rate of the Morse code (default: {} WPM)".format(default_wpm),
	  default = default_wpm,
          required = False
	)
  argparser.add_argument(
	  "message",
	  type = str,
	  help = "Text to translate into Morse code and modulate " \
			"the RF field with",
	)
  args = argparser.parse_args()

  # Add a pause at the beginning and at the end of the message
  msg = message_start_pause + args.message + message_end_pause

  # Turn the message into a sequence of field-on durations (positive) and
  # field-off durations (negative)
  msg = msg.upper()
  morsechars = []
  i = 0
  while i < len(msg):

    if msg[i] == "[":
      j = msg.find("]", i + 1)
      if j > 0 and msg[i : j + 1] in morsecode:
        morsechars.append(morsecode[msg[i : j + 1]])
        i = j + 1
        continue

    if msg[i] == " " or msg[i] == "\t":
      morsechars.append(" ")

    elif msg[i] in morsecode:
      morsechars.append(morsecode[msg[i]])

    else:
      print("Untranslatable in Morse code: {} - dropped".format(msg[i]))

    i += 1

  ditlen = 1.2 / args.words_per_minute 
      
  morseseq = []
  for mc in morsechars:

    if mc == " ":
      if morseseq:
        morseseq[-1] = -ditlen * 7
      else:
        morseseq.append(-ditlen * 7)
      continue

    for c in mc:
      if c == ".":
        morseseq.append(ditlen)
      else:
        morseseq.append(ditlen * 3)
      morseseq.append(-ditlen)

    morseseq[-1] = -ditlen * 3

  # Open the reader (assert restore_on_close so the reader is automatically
  # restored to the state we found it in upon closing)
  with uFR.open(args.device, restore_on_close = True) as ufr:

    # Enable anti-collision to disable polling
    ufr.enable_anti_collision()

    # Turn on the red LED, so the green LED stops blinking
    ufr.red_light_control(True)

    # Turn off ESP lights to see the implant's LED better. Fail silently if the
    # command isn't supported (i.e. not a uFR Nano Online)
    try:
      ufr.esp_set_display_data((0, 0, 0), (0, 0, 0), 0)
    except:
      pass

    # Play the morse code sequence
    warn_if_too_fast = True

    for d in morseseq:

      # Flip the field on and off, measure how long it takes to send the command
      # and calculate what additional delay to wait
      cmd_start_tstamp = datetime.now().timestamp()

      ufr.rf_reset(uFRRFfieldCtl.OFF if d < 0 else uFRRFfieldCtl.ON)

      cmd_duration = datetime.now().timestamp() - cmd_start_tstamp
      remaining_wait = abs(d) - cmd_duration

      # If the command took longer than this Morse code element, warn the user
      if remaining_wait <= 0:
        if warn_if_too_fast:
          print("Warning: {} WPM is too fast for your computer / " \
			"device connection".format(args.words_per_minute))
          print("Morse code timing may appear incorrect")
          warn_if_too_fast = False        
        continue

      # Sleep long enough to wait for the end of this Morse code element
      sleep(remaining_wait)
