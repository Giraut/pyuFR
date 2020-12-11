#!/usr/bin/python3
"""Pure Python class to communicate with uFR series readers
"""

### Parameters
default_ufr_device = "serial:///dev/ttyUSB0:1000000"	# Nano USB serial
#default_ufr_device = "udp://ufr:8881"			# Nano Online slave UDP
#default_ufr_device = "tcp://ufr:8881"			# Nano Online slave TCP
#default_ufr_device = "http://ufr/uart1"		# Nano Online REST UART1
default_ufr_timeout = 1 #s

# API tests
test_network_probe_functions  = False
test_eeprom_writing_functions = False
test_reset_functions          = True
test_sleep_functions          = True
test_led_sound_functions      = True
test_esp_io                   = True
test_uid_functions            = True
test_tag_emulation            = True



### Modules
import re
import requests
from time import sleep
from enum import IntEnum
from datetime import datetime
from multiprocessing.pool import ThreadPool
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM, SOCK_STREAM

# Try to import optional modules but fail silently if they're not needed later
try:
  from serial import Serial
except:
  pass
try:
  from ipaddress import ip_network
except:
  pass



### Enums
class ufrhead(IntEnum):
  CMD_HEADER                              = 0x55
  ACK_HEADER                              = 0xac
  RESPONSE_HEADER                         = 0xde
  ERR_HEADER                              = 0xec

class ufrcmdextpartack(IntEnum):
  ACK_PART                                = 0xad
  ACK_LAST_PART                           = 0xdd

class ufrtrail(IntEnum):
  CMD_TRAILER                             = 0xaa
  ACK_TRAILER                             = 0xca
  RESPONSE_TRAILER                        = 0xed
  ERR_TRAILER                             = 0xce

class ufrcmd(IntEnum):
  GET_READER_TYPE                         = 0x10
  GET_READER_SERIAL                       = 0x11
  GET_SERIAL_NUMBER                       = 0x40
  GET_HARDWARE_VERSION                    = 0x2a
  GET_FIRMWARE_VERSION                    = 0x29
  GET_BUILD_NUMBER                        = 0x2b
  READER_KEY_WRITE                        = 0x12
  USER_DATA_READ                          = 0x1b
  USER_DATA_WRITE                         = 0x1c
  READER_KEYS_LOCK                        = 0x27
  READER_KEYS_UNLOCK                      = 0x28
  READER_PASSWORD_WRITE                   = 0x33
  SELF_RESET                              = 0x30
  SET_SPEED_PERMANENTLY                   = 0x4b
  GET_SPEED_PARAMETERS                    = 0x4c
  SET_UART_SPEED                          = 0x70
  RED_LIGHT_CONTROL                       = 0x71
  USER_INTERFACE_SIGNAL                   = 0x26
  SET_RF_ANALOG_SETTINGS                  = 0x7d
  GET_RF_ANALOG_SETTINGS                  = 0x7e
  SET_LED_CONFIG                          = 0x6e
  DEFAULT_UART_SPEED_SESSION              = 0xf1
  GET_CARD_ID                             = 0x13
  GET_CARD_ID_EX                          = 0x2c
  GET_DLOGIC_CARD_TYPE                    = 0x3c
  GET_LAST_CARD_ID_EX                     = 0x7c
  SECTOR_TRAILER_WRITE                    = 0x1a
  SECTOR_TRAILER_WRITE_UNSAFE             = 0x2f
  BLOCK_READ                              = 0x16
  BLOCK_WRITE                             = 0x17
  BLOCK_IN_SECTOR_READ                    = 0x18
  BLOCK_IN_SECTOR_WRITE                   = 0x19
  LINEAR_READ                             = 0x14
  LINEAR_WRITE                            = 0x15
  LINEAR_FORMAT_CARD                      = 0x25
  LIN_ROW_READ                            = 0x45
  VALUE_BLOCK_READ                        = 0x1d
  VALUE_BLOCK_WRITE                       = 0x1e
  VALUE_BLOCK_INC                         = 0x21
  VALUE_BLOCK_DEC                         = 0x22
  VALUE_BLOCK_IN_SECTOR_READ              = 0x1f
  VALUE_BLOCK_IN_SECTOR_WRITE             = 0x20
  VALUE_BLOCK_IN_SECTOR_INC               = 0x23
  VALUE_BLOCK_IN_SECTOR_DEC               = 0x24
  GET_DESFIRE_UID                         = 0x80
  SET_DESFIRE_KEY                         = 0x81
  DESFIRE_WRITE_TO_FILE                   = 0x82
  DESFIRE_READ_FROM_FILE                  = 0x83
  DESFIRE_CREATE_APPLICATION              = 0x84
  DESFIRE_CREATE_FILE                     = 0x85
  DESFIRE_CREATE_AES_KEY                  = 0x86
  DESFIRE_GET_KEY_CONFIG                  = 0x87
  DESFIRE_CHANGE_KEY_CONFIG               = 0x88
  DESFIRE_DELETE_APPLICATION              = 0x89
  DESFIRE_DELETE_FILE                     = 0x8a
  DESFIRE_SET_CONFIGURATION               = 0x8b
  DESFIRE_FORMAT_CARD                     = 0x8c
  DESFIRE_FREE_MEM                        = 0x8d
  DESFIRE_WRITE_AES_KEY                   = 0x8e
  DESFIRE_CREATE_VALUE_FILE               = 0x8f
  DESFIRE_READ_VALUE_FILE                 = 0x9a
  DESFIRE_INCREASE_VALUE_FILE             = 0x9b
  DESFIRE_DECREASE_VALUE_FILE             = 0x9c
  DESFIRE_CREATE_RECORD_FILE              = 0x97
  DESFIRE_WRITE_RECORD                    = 0x98
  DESFIRE_READ_RECORDS                    = 0x99
  DESFIRE_CLEAR_RECORD                    = 0x6d
  DESFIRE_GET_APPLICATION_IDS             = 0xc0
  MFP_FIRST_AUTHENTICATE                  = 0x6a
  MFP_CHANGE_REG_KEY                      = 0x6b
  MFP_GET_UID                             = 0x6c
  GET_NFC_T2T_VERSION                     = 0xb0
  READ_COUNTER                            = 0xb1
  INCREMENT_COUNTER                       = 0xb2
  NT4H_COMMON_CMD                         = 0xb3
  READ_ECC_SIGNATURE                      = 0xbf
  SET_CARD_ID_SEND_CONF                   = 0x3d
  GET_CARD_ID_SEND_CONF                   = 0x3e
  SET_BAD_SELECT_NR_MAX                   = 0x3f
  GET_BAD_SELECT_NR_MAX                   = 0x44
  ENTER_SLEEP_MODE                        = 0x46
  LEAVE_SLEEP_MODE                        = 0x47
  AUTO_SLEEP_SET                          = 0x4d
  AUTO_SLEEP_GET                          = 0x4e
  WRITE_EMULATION_NDEF                    = 0x4a
  TAG_EMULATION_START                     = 0x48
  TAG_EMULATION_STOP                      = 0x49
  AD_HOC_EMULATION_START                  = 0x76
  AD_HOC_EMULATION_STOP                   = 0x77
  GET_EXTERNAL_FIELD                      = 0x9f
  GET_AD_HOC_EMULATION_PARAMS             = 0x9d
  SET_AD_HOC_EMULATION_PARAMS             = 0x9e
  SET_DISPLAY_DATA                        = 0x72
  SET_SPEAKER_FREQUENCY                   = 0x73
  SET_DISPLAY_INTENSITY                   = 0x74
  GET_DISPLAY_INTENSITY                   = 0x75
  UFR_XRC_LOCK_OPEN                       = 0x60
  UFR_XRC_SET_RELAY_STATE                 = 0x61
  UFR_XRC_GET_IO_STATE                    = 0x62
  ENTER_SHARE_RAM_COMM_MODE               = 0x78
  EXIT_SHARE_RAM_COMM_MODE                = 0x79
  READ_SHARE_RAM                          = 0x7a
  WRITE_SHARE_RAM                         = 0x7b
  I_BLOCK_TRANSCEIVE                      = 0x90
  R_BLOCK_TRANSCEIVE                      = 0x91
  S_BLOCK_DESELECT                        = 0x92
  SET_ISO14443_4_MODE                     = 0x93
  APDU_TRANSCEIVE                         = 0x94
  ESP_SET_IO_STATE                        = 0xf3
  ESP_GET_IO_STATE                        = 0xf4
  ESP_READER_TIME_WRITE                   = 0xf5
  ESP_READER_TIME_READ                    = 0xf6
  ESP_READER_EEPROM_READ                  = 0xf7
  ESP_SET_DISPLAY_DATA                    = 0xf8
  ESP_READER_RESET                        = 0xf9
  ESP_READER_PASSWORD_WRITE               = 0xfa
  ESP_READER_EEPROM_WRITE                 = 0xfb
  CHECK_UID_CHANGE                        = 0xe4
  RF_RESET                                = 0xe5
  GET_READER_STATUS                       = 0xe6

class ufrerr(IntEnum):
  OK                                      = 0x00
  COMMUNICATION_ERROR                     = 0x01
  CHKSUM_ERROR                            = 0x02
  READING_ERROR                           = 0x03
  WRITING_ERROR                           = 0x04
  BUFFER_OVERFLOW                         = 0x05
  MAX_ADDRESS_EXCEEDED                    = 0x06
  MAX_KEY_INDEX_EXCEEDED                  = 0x07
  NO_CARD                                 = 0x08
  COMMAND_NOT_SUPPORTED                   = 0x09
  FORBIDEN_DIRECT_WRITE_IN_SECTOR_TRAILER = 0x0a
  ADDRESSED_BLOCK_IS_NOT_SECTOR_TRAILER   = 0x0b
  WRONG_ADDRESS_MODE                      = 0x0c
  WRONG_ACCESS_BITS_VALUES                = 0x0d
  AUTH_ERROR                              = 0x0e
  PARAMETERS_ERROR                        = 0x0f
  MAX_SIZE_EXCEEDED                       = 0x10
  UNSUPPORTED_CARD_TYPE                   = 0x11
  COUNTER_ERROR                           = 0x12
  WRITE_VERIFICATION_ERROR                = 0x70
  BUFFER_SIZE_EXCEEDED                    = 0x71
  VALUE_BLOCK_INVALID                     = 0x72
  VALUE_BLOCK_ADDR_INVALID                = 0x73
  VALUE_BLOCK_MANIPULATION_ERROR          = 0x74
  WRONG_UI_MODE                           = 0x75
  KEYS_LOCKED                             = 0x76
  KEYS_UNLOCKED                           = 0x77
  WRONG_PASSWORD                          = 0x78
  CAN_NOT_LOCK_DEVICE                     = 0x79
  CAN_NOT_UNLOCK_DEVICE                   = 0x7a
  DEVICE_EEPROM_BUSY                      = 0x7b
  RTC_SET_ERROR                           = 0x7c
  EEPROM_ERROR                            = 0x7d
  NO_CARDS_ENUMERRATED                    = 0x7e
  CARD_ALREADY_SELECTED                   = 0x7f
  WRONG_CARD_TYPE                         = 0x80
  FORBIDDEN_IN_TAG_EMULATION_MODE         = 0x90
  MFP_COMMAND_OVERFLOW                    = 0xb0
  MFP_INVALID_MAC                         = 0xb1
  MFP_INVALID_BLOCK_NR                    = 0xb2
  MFP_NOT_EXIST_BLOCK_NR                  = 0xb3
  MFP_COND_OF_USE_ERROR                   = 0xb4
  MFP_LENGTH_ERROR                        = 0xb5
  MFP_GENERAL_MANIP_ERROR                 = 0xb6
  MFP_SWITCH_TO_ISO14443_4_ERROR          = 0xb7
  MFP_ILLEGAL_STATUS_CODE                 = 0xb8
  MFP_MULTI_BLOCKS_READ                   = 0xb9
  NT4H_COMMAND_ABORTED                    = 0xc0
  NT4H_LENGTH_ERROR                       = 0xc1
  NT4H_PARAMETER_ERROR                    = 0xc2
  NT4H_NO_SUCH_KEY                        = 0xc3
  NT4H_PERMISSION_DENIED                  = 0xc4
  NT4H_AUTHENTICATION_DELAY               = 0xc5
  NT4H_MEMORY_ERROR                       = 0xc6
  NT4H_INTEGRITY_ERROR                    = 0xc7
  NT4H_FILE_NOT_FOUND                     = 0xc8
  NT4H_BOUNDARY_ERROR                     = 0xc9
  NT4H_INVALID_MAC                        = 0xca
  NT4H_NO_CHANGES                         = 0xcb

class ufrauthmode(IntEnum):
  RKA_AUTH1A                              = 0x00
  RKA_AUTH1B                              = 0x01
  AKM1_AUTH1A                             = 0x20
  AKM1_AUTH1B                             = 0x21
  AKM2_AUTH1A                             = 0x40
  AKM2_AUTH1B                             = 0x41
  PK_AUTH1A                               = 0x60
  PK_AUTH1B                               = 0x61
  PK_AUTH1A_AES                           = 0x80
  PK_AUTH1B_AES                           = 0x81
  SAM_KEY_AUTH1A                          = 0x10
  SAM_KEY_AUTH1B                          = 0x11
  MFP_RKA_AUTH1A                          = 0x02
  MFP_RKA_AUTH1B                          = 0x03
  MFP_AKM1_AUTH1A                         = 0x22
  MFP_AKM1_AUTH1B                         = 0x23
  MFP_AKM2_AUTH1A                         = 0x42
  MFP_AKM2_AUTH1B                         = 0x43

class ufrtagcommtype(IntEnum):
  ISO14443_TYPE_A                         = 0X01
  ISO14443_TYPE_B                         = 0X02
  ISO14443_4_212_KBPS                     = 0X03
  ISO14443_4_424_KBPS                     = 0X04

class pn53xanalogsettingsreg(IntEnum):
  RFCFG                                   = 0
  RXTHRESHOLD                             = 1
  GSNON                                   = 2
  CWGSP                                   = 3
  GSNOFF                                  = 4
  MODGSP                                  = 4

class ufrlightsignal(IntEnum):
  NONE                                    = 0
  LONG_GREEN                              = 1
  LONG_RED                                = 2
  ALTERNATION                             = 3
  FLASH                                   = 4

class ufrbeepsignal(IntEnum):
  NONE                                    = 0
  SHORT                                   = 1
  LONG                                    = 2
  DOUBLE_SHORT                            = 3
  TRIPLE_SHORT                            = 4
  TRIPLET_MELODY                          = 5

class ufriostate(IntEnum):
  LOW                                     = 0
  HIGH                                    = 1
  INPUT                                   = 2



### Defines
ufr_header_vals = tuple(map(int, ufrhead)) 
ufr_trailer_vals = tuple(map(int, ufrtrail)) 
ufr_cmd_vals = tuple(map(int, ufrcmd))
ufr_err_vals = tuple(map(int, ufrerr))
ufr_val_to_cmd = {cmd.value: cmd for cmd in ufrcmd}
ufr_val_to_err = {err.value: err for err in ufrerr}
ufr_val_to_iostate = {iostate.value: iostate for iostate in ufriostate}

# Number of concurrent connection when scanning a subnet for Nano Onlines
subnet_probe_concurrent_connections = 100



# Leave sleep mode parameters
WAKE_UP_BYTE                   = 0x00
wake_up_wait                   = .01 #s
post_wake_up_wait              = .1 #s	Apparently needed by the reader

# Soft reader restart and hard reader reset parameters
post_reset_wait                = .1 #s	Apparently needed by the reader

# Write emulation NDEF parameters
post_write_emulation_ndef_wait = .1 #s	Apparently needed after writing EEPROM



### Classes
class ufr_answer:

  def __init__(self):

    self.is_ack = False
    self.is_err = False
    self.is_rsp = False
    self.has_ext = False

    self.header = None
    self.code = None
    self.trailer = None

    self.ext_len = None

    self.val0 = None
    self.val1 = None

    self.checksum = None

    self.ext = None
    self.ext_checksum = None



  def printable(self):
    """Return a human-readable description of the answer
    """

    desc = "ACK" if self.is_ack else "ERR" if self.is_err else "RSP"
    desc += "_EXT" if self.has_ext and not self.is_ack else ""
    desc += ", cmd=" if self.is_ack or self.is_rsp else ", err="
    desc += self.code.name
    desc += ", ext_len={}".format(self.ext_len)
    desc += ", val0={:02x}h, val1={:02x}h".format(self.val0, self.val1)
    if self.has_ext and not self.is_ack:
      desc += ", ext=("
      desc += ", ".join(["{:02x}h".format(v) for v in self.ext])
      desc += ")"
    return(desc)



class ufr:

  def __init__(self):

    self.serdev = None

    self.udpsock = None
    self.udphost = None
    self.udpport = None

    self.tcpsock = None

    self.resturl = None
    self.postdata = None

    self.default_timeout = None
    self.current_timeout = None

    self.recbuf = []

    self.last_cmd = None

    self.answer = ufr_answer()



  def open(self, dev = default_ufr_device, timeout = default_ufr_timeout):
    """Open a connection. The device format is one of:

    serial://<device file>:<baudrate>
    udp://<host>:<port>
    tcp://<host>:<port>
    http://<host>/uartX
    """

    # Find out the protocol and associated parameters
    m = re.findall("^(serial|udp|tcp)://(.+):([0-9]+)/*$", dev)
    if m:
      proto, devhost, baudport = m[0]
    else:
      m = re.findall("^(http://.+/uart[12])/*$", dev)
      if m:
        proto = "http"
        url = m[0]
      else:
        proto = None

    # Open the device
    if proto == "serial":
      self.serdev = Serial(devhost, baudport, timeout = timeout)

    elif proto == "udp":
      self.udpsock = socket(AF_INET, SOCK_DGRAM)
      self.udpsock.settimeout(timeout)
      self.udphost = gethostbyname(devhost)
      self.udpport = int(baudport)

    elif proto == "tcp":
      self.tcpsock = socket(AF_INET, SOCK_STREAM)
      self.tcpsock.settimeout(timeout)
      self.tcpsock.connect((gethostbyname(devhost), int(baudport)))

    elif proto == "http":
      self.resturl = url

    else:
      raise ValueError("unknown uFR device {}".format(dev))
      return

    self.default_timeout = timeout
    self.current_timeout = timeout
    return



  def _checksum(self, data):
    """Calculate the checksum of a row of bytes
    """

    csum = 0
    for b in data:
      csum ^= b

    return((csum + 0x07) % 256)
    


  def _send_data(self, data):
    """Send a data packet
    """

    # Send to a serial device
    if self.serdev is not None:
      self.serdev.write(data)
      self.serdev.flush()

    # Send to a UDP host
    elif self.udpsock is not None:
      self.udpsock.sendto(bytes(data), (self.udphost, self.udpport))

    # Send to a TCP host
    elif self.tcpsock is not None:
      self.tcpsock.sendall(bytes(data))

    # "Send" to a HTTP server
    elif self.resturl is not None:
      self.postdata = "".join(["{:02X}".format(b) for b in data])



  def _get_data(self, timeout = None):
    """Receive data
    """

    # Change the timeout as needed
    if timeout is None:
      reset_timeout = (self.current_timeout != self.default_timeout)
      self.current_timeout = self.default_timeout
    else:
      reset_timeout = (self.current_timeout != timeout)
      self.current_timeout = timeout

    # Receive from a serial device
    if self.serdev is not None:
      if reset_timeout:
        self.serdev.timeout = self.current_timeout
      data = self.serdev.read(1)
      if not data:
        raise TimeoutError

    # Receive from a UDP host
    elif self.udpsock is not None:
      if reset_timeout:
        self.udpsock.settimeout(self.current_timeout)
      timeout_tstamp = datetime.now().timestamp() + self.current_timeout
      data = None
      while not data:
        data, fromhostport = self.udpsock.recvfrom(1024)
        if fromhostport[0] != self.udphost:
          data = None
        if not data and datetime.now().timestamp() >= timeout_tstamp:
          raise TimeoutError

    # Receive from a TCP host
    elif self.tcpsock is not None:
      if reset_timeout:
        self.tcpsock.settimeout(self.current_timeout)
      data = self.tcpsock.recv(1024)

    # Receive a POST reply from a HTTP server
    elif self.resturl is not None:
      data = requests.post(self.resturl, data = self.postdata,
			timeout = self.current_timeout).text.rstrip("\r\n\0 ")
      if not re.match("^([0-9a-zA-Z][0-9a-zA-Z])+$", data):
        if not data:
          raise ValueError("empty HTTP POST response")
        else:
          raise ValueError("invalid HTTP POST response: {}".format(data))
      data = [int(data[i:i+2], 16) for i in range(0, len(data), 2)]
      self.postdata = None

    return(data)



  def send_cmd(self, cmd, par0 = 0, par1 = 0, ext_len = 0):
    """Send a short command
    """

    packet = [ufrhead.CMD_HEADER.value, cmd.value, ufrtrail.CMD_TRAILER.value,
		ext_len, par0, par1]
    packet.append(self._checksum(packet))
    self._send_data(packet)

    self.last_cmd = cmd



  def send_ext(self, ext_parms):
    """Sent extended command parameters
    """

    packet = list(ext_parms)
    packet.append(self._checksum(packet))
    self._send_data(packet)



  def send_cmd_ext(self, cmd, par0, par1, ext_parms, timeout = None):
    """Send an extended command in two steps: first the short command, wait for
    an ACK, then send the extended command parameters
    """

    ext_len = len(ext_parms) + 1

    if not ext_len:
      return(self.send_cmd(cmd, par0, par1, 0))

    self.send_cmd(cmd, par0, par1, ext_len)

    answer = self.get_answer(timeout)

    if not answer.is_ack or answer.code != cmd.value:
      raise ValueError("expected ACK to {}, ext_len={}, "
			"par0={:02x}h, par1={:02x}h - got {}".format(
			cmd.name, ext_len, par0, par1, answer.printable()))

    self.send_ext(ext_parms)



  def get_answer(self, timeout = None):
    """Get an answer packet
    """

    self.answer.__init__()

    while True:

      # Read data if the receive buffer is empty
      if not self.recbuf:
        data = self._get_data(timeout)
        self.recbuf.extend(data)

      # Parse the receive buffer
      b = self.recbuf.pop(0)

      # Get header
      if self.answer.header is None:
        if b in ufr_header_vals:
          self.answer.header = b
          self.answer.is_ack = (b == ufrhead.ACK_HEADER)
          self.answer.is_err = (b == ufrhead.ERR_HEADER)
          self.answer.is_rsp = (b == ufrhead.RESPONSE_HEADER)
        continue

      # Get the code (either command or error)
      if self.answer.code is None:
        if (self.answer.is_ack or self.answer.is_rsp) and b in ufr_cmd_vals:
          self.answer.code = ufr_val_to_cmd[b]
        elif self.answer.is_err and b in ufr_err_vals:
          self.answer.code = ufr_val_to_err[b]
        else:
          self.answer.__init__()
        continue

      # Get the trailer
      if self.answer.trailer is None:
        if (self.answer.header == ufrhead.ACK_HEADER and \
			b == ufrtrail.ACK_TRAILER) or \
		(self.answer.header == ufrhead.ERR_HEADER and \
			b == ufrtrail.ERR_TRAILER) or \
		(self.answer.header == ufrhead.RESPONSE_HEADER and \
			b == ufrtrail.RESPONSE_TRAILER):
          self.answer.trailer = b
        else:
          self.answer.__init__()
        continue

      # Get the length of the returned parameters
      if self.answer.ext_len is None:
        if b == 0 or b >= 2:
          self.answer.ext_len = b
          self.answer.has_ext = (b != 0)
        else:
          self.answer.__init__()
        continue

      # Get val0
      if self.answer.val0 is None:
        self.answer.val0 = b
        continue

      # Get val1
      if self.answer.val1 is None:
        self.answer.val1 = b
        continue

      # Get the checksum
      if self.answer.checksum is None:
        if self._checksum((self.answer.header, self.answer.code,
		self.answer.trailer, self.answer.ext_len,
		self.answer.val0, self.answer.val1)) == b:
          self.answer.checksum = b

          # If the response is short, return it immediately
          if not self.answer.has_ext or self.answer.is_ack:
            return(self.answer)

        else:
          self.answer.__init__()
        continue

      # Get the first byte of the extended packet
      if self.answer.ext is None:
        self.answer.ext = [b]
        nb_ext_bytes_remaining = self.answer.ext_len - 2
        continue

      # Get the rest of the extended packet
      if nb_ext_bytes_remaining:
        self.answer.ext.append(b)
        nb_ext_bytes_remaining -= 1
        continue

      # Get the extended packet's checksum
      if self._checksum(self.answer.ext) == b:
        self.answer.ext = tuple(self.answer.ext)
        self.answer.ext_checksum = b

        # Return the long answer
        return(self.answer)

      else:
        self.answer.__init__()



  def get_cmd_ext_part_ack(self, timeout = None):
    """Get a multipart CMD_EXT acknowledgment.
    Return True if it's a part acknowledgment, False if it's the last part.
    If we get anything else, raise an exception.
    """

    # Read data if the receive buffer is empty
    if not self.recbuf:
      data = self._get_data(timeout)
      self.recbuf.extend(data)

    # Parse one byte
    b = self.recbuf.pop(0)

    # Did we get an ACK?
    if b == ufrcmdextpartack.ACK_PART:
      return(True)
    if b == ufrcmdextpartack.ACK_LAST_PART:
      return(False)

    # We got an expected byte
    raise ValueError("expected {} ({:02x}h) or {} ({:02x}h) - got {:02x}h".
			format(ufrcmdextpartack.ACK_PART.name,
			ufrcmdextpartack.ACK_PART.value,
			ufrcmdextpartack.ACK_LAST_PART.name,
			ufrcmdextpartack.ACK_LAST_PART.value,
			b))



  def get_last_command_response(self, timeout = None):
    """Get a responde to the last command sent. Throw an exception if the
    answer is unexpected
    """

    answer = self.get_answer(timeout)
    if not answer.is_rsp or answer.code != self.last_cmd:
      raise ValueError("expected response to {} - got {}".format(
			self.last_cmd.name, answer.printable()))
    return(answer)



  def close(self):
    """Close any open connection
    """

    if self.serdev is not None:
      self.serdev.close()
      self.serdev = None

    if self.udpsock is not None:
      self.udpsock.close()
      self.udpsock = None
      self.udphost = None
      self.udpport = None

    if self.tcpsock is not None:
      self.tcpsock.close()
      self.tcpsock = None

    self.resturl = None
    self.postdata = None

    self.default_timeout = None
    self.current_timeout = None


  def __del__(self):

    self.close()



  def is_host_nano_online(self, host, timeout = None):
    """Try to contact a host to see if it's running a HTTP server serving a
    Nano online page
    """

    # Are we being called as a thread from probe_subnet_nano_onlines()?
    if type(host) == tuple:
      is_thread = True
      host, timeout = host
    else:
      is_thread = False

    # Try to get the Nano Online's login page
    try:
      response = requests.get("http://" + host, timeout = \
			self.default_timeout if timeout is None else timeout)
      is_nano_online = (response.status_code == 200 and \
		re.search("uFR login", response.text) is not None)
    except:
      is_nano_online = False

    # If we're called as a thread, return a tuple containing the host and the
    # probe result. Otherwise return just the probe result
    return((host, is_nano_online) if is_thread else is_nano_online)



  def probe_subnet_nano_onlines(self, netaddr, timeout = None):
    """Probe an entire subnet for Nano Onlines. Uses threads
    """

    ip_net = ip_network(netaddr)

    threadpool = ThreadPool(processes = subnet_probe_concurrent_connections)

    nano_online_ips = []
    for host, is_nano_online in threadpool.map(self.is_host_nano_online,
			[(str(host), timeout) for host in ip_net.hosts()]):
      if is_nano_online:
        nano_online_ips.append(host)

    threadpool.close()

    return(nano_online_ips)



  # Front-end API functions - added as needed
  def get_reader_type(self, timeout = None):

    self.send_cmd(ufrcmd.GET_READER_TYPE)
    rsp = self.get_last_command_response(timeout)
    return(rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24))

  def get_serial_number(self, timeout = None):

    self.send_cmd(ufrcmd.GET_SERIAL_NUMBER)
    rsp = self.get_last_command_response(timeout)
    return(bytes(rsp.ext).decode("ascii"))

  def get_hardware_version(self, timeout = None):

    self.send_cmd(ufrcmd.GET_HARDWARE_VERSION)
    rsp = self.get_last_command_response(timeout)
    return((rsp.val0 << 8) + rsp.val1)

  def get_firmware_version(self, timeout = None):

    self.send_cmd(ufrcmd.GET_FIRMWARE_VERSION)
    rsp = self.get_last_command_response(timeout)
    return((rsp.val0 << 8) + rsp.val1)

  def get_build_number(self, timeout = None):

    self.send_cmd(ufrcmd.GET_BUILD_NUMBER)
    rsp = self.get_last_command_response(timeout)
    return(rsp.val0)

  def get_card_id_ex(self, timeout = None):

    self.send_cmd(ufrcmd.GET_CARD_ID_EX)
    try:
      rsp = self.get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise
    return(":".join(["{:02X}".format(v) for v in rsp.ext[:rsp.val1]]))

  def get_analog_settings(self, tag_comm_type, timeout = None):

    self.send_cmd(ufrcmd.GET_RF_ANALOG_SETTINGS, tag_comm_type)
    rsp = self.get_last_command_response(timeout)
    return(rsp.ext)

  def set_analog_settings(self, tag_comm_type, factory_settings, settings,
				timeout = None):
    self.send_cmd_ext(ufrcmd.SET_RF_ANALOG_SETTINGS, tag_comm_type,
			1 if factory_settings else 0, settings, timeout)
    rsp = self.get_last_command_response(timeout)

  def set_led_config(self, blink, timeout = None):
    self.send_cmd(ufrcmd.SET_LED_CONFIG, 1 if blink else 0)
    rsp = self.get_last_command_response(timeout)

  def enter_sleep_mode(self, timeout = None):

    self.send_cmd(ufrcmd.ENTER_SLEEP_MODE)
    rsp = self.get_last_command_response(timeout)

  def leave_sleep_mode(self, timeout = None):

    self._send_data((WAKE_UP_BYTE,))
    sleep(wake_up_wait)
    self.send_cmd(ufrcmd.LEAVE_SLEEP_MODE)
    rsp = self.get_last_command_response(timeout)
    sleep(post_wake_up_wait)

  def rf_reset(self, timeout = None):

    self.send_cmd(ufrcmd.RF_RESET)
    rsp = self.get_last_command_response(timeout)

  def self_reset(self, timeout = None):

    self.send_cmd(ufrcmd.SELF_RESET)
    rsp = self.get_last_command_response(timeout)
    sleep(post_reset_wait)

  def write_emulation_ndef(self, ndef, in_ram = False, timeout = None):

    ndeflen = len(ndef)

    if (not in_ram and ndeflen > 144) or (in_ram and ndeflen > 1008):
      raise ValueError("NDEF too long")

    # Split the ndef into 240-byte-long parts
    ext_parts = [ndef[i:i + 240] for i in range(0, len(ndef), 240)]
    if not ext_parts:
      ext_parts = [b""]

    # First CMD_EXT part is prefixed with the length of the NDEF, and suffixed
    # with the checksum of that part (but send_cmd_ext() will take care of
    # appending the checksum)
    ext_parts[0] = bytes([ndeflen & 0xff, ndeflen >> 8]) + ext_parts[0]

    # Subsequent CMD_EXT parts are only prefixed with the length of that part
    for i in range(1, len(ext_parts)):
      ext_parts[i] = bytes([len(ext_parts[i])]) + ext_parts[i]

    # Send the command and first CMD_EXT part
    self.send_cmd_ext(ufrcmd.WRITE_EMULATION_NDEF, 1 if in_ram else 0, 0,
			ext_parts.pop(0), timeout)

    # Wait for ACKs and send subsequent parts if we have more than one part
    if ext_parts:
      while self.get_cmd_ext_part_ack(timeout):
        if not ext_parts:
          raise ValueError("expected {} ({:02x}h) - got {} ({:02x}h) "
			"with no more CMD_EXT parts to send".format(
			ufrcmdextpartack.ACK_LAST_PART.name,
			ufrcmdextpartack.ACK_LAST_PART.value,
			ufrcmdextpartack.ACK_PART.name,
			ufrcmdextpartack.ACK_PART.value))
        self._send_data(ext_parts.pop(0))

    if ext_parts:
      raise ValueError("expected {} ({:02x}h) - got {} ({:02x}h) "
			"before sending the last CMD_EXT part".format(
			ufrcmdextpartack.ACK_PART.name,
			ufrcmdextpartack.ACK_PART.value,
			ufrcmdextpartack.ACK_LAST_PART.name,
			ufrcmdextpartack.ACK_LAST_PART.value))

    rsp = self.get_last_command_response(timeout)
    sleep(post_write_emulation_ndef_wait)


  def tag_emulation_start(self, ram_ndef = False, timeout = None):

    self.send_cmd(ufrcmd.TAG_EMULATION_START, 1 if ram_ndef else 0)
    rsp = self.get_last_command_response(timeout)

  def tag_emulation_stop(self, timeout = None):

    self.send_cmd(ufrcmd.TAG_EMULATION_STOP)
    rsp = self.get_last_command_response(timeout)

  def red_light_control(self, state, timeout = None):

    self.send_cmd(ufrcmd.RED_LIGHT_CONTROL, 1 if state else 0)
    rsp = self.get_last_command_response(timeout)

  def user_interface_signal(self, light_signal_mode, beep_signal_mode,
				timeout = None):
    self.send_cmd(ufrcmd.USER_INTERFACE_SIGNAL, light_signal_mode.value,
			beep_signal_mode.value)
    rsp = self.get_last_command_response(timeout)

  def set_speaker_frequency(self, frequency, timeout = None):

    period = ((round(65535 - 1500000 / (2 * frequency))) & 0xffff) \
		if frequency else 0xffff
    self.send_cmd(ufrcmd.SET_SPEAKER_FREQUENCY, period & 0xff, period >> 8)
    rsp = self.get_last_command_response(timeout)

  def set_iso14443_4_mode(self, timeout = None):

    self.send_cmd(ufrcmd.SET_ISO14443_4_MODE)
    rsp = self.get_last_command_response(timeout)

  def esp_set_io_state(self, pin, state, timeout = None):

    self.send_cmd(ufrcmd.ESP_SET_IO_STATE, pin, state.value)
    rsp = self.get_last_command_response(timeout)

  def esp_get_io_state(self, timeout = None):

    self.send_cmd(ufrcmd.ESP_GET_IO_STATE)
    rsp = self.get_last_command_response(timeout)
    return([ufr_val_to_iostate[st] for st in rsp.ext])

  def esp_set_display_data(self, rgb1, rgb2, duration, timeout = None):

    self.send_cmd_ext(ufrcmd.ESP_SET_DISPLAY_DATA,
			duration & 0xff, duration >> 8, rgb1 + rgb2, timeout)
    rsp = self.get_last_command_response(timeout)

  def esp_reader_reset(self, timeout = None):

    self.send_cmd(ufrcmd.ESP_READER_RESET, 0)
    rsp = self.get_last_command_response(timeout)
    sleep(post_reset_wait)



### Test routine
if __name__ == "__main__":
  """Test routine
  """

  ufr = ufr()

  if test_network_probe_functions:
    print("IS_HOST_NANO_ONLINE:       ", ufr.is_host_nano_online("ufr"))
    print("PROBE_SUBNET_NANO_ONLINES: ",
			ufr.probe_subnet_nano_onlines("192.168.1.0/24"))

  ufr.open()

  print("GET_READER_TYPE:           ", hex(ufr.get_reader_type()))
  print("GET_SERIAL_NUMBER:         ", ufr.get_serial_number())
  print("GET_HARDWARE_VERSION:      ", hex(ufr.get_hardware_version()))
  print("GET_FIRMWARE_VERSION:      ", hex(ufr.get_firmware_version()))
  print("GET_BUILD_NUMBER:          ", hex(ufr.get_build_number()))

  for tct in map(int, ufrtagcommtype):
    print("GET_RF_ANALOG_SETTINGS: ", ufr.get_analog_settings(tct))

    if test_eeprom_writing_functions:
      new_settings = list(ufr.answer.ext)
      new_settings[pn53xanalogsettingsreg.RXTHRESHOLD] = 255
      print("SET_RF_ANALOG_SETTINGS")
      ufr.set_analog_settings(tct, False, new_settings)
      print("SET_LED_CONFIG")
      ufr.set_led_config(True)

  if test_reset_functions:
    print("SELF_RESET")
    ufr.self_reset()
    if ufr.udpsock is not None or ufr.tcpsock:
      print("ESP_READER_RESET")
      ufr.esp_reader_reset()

  if test_sleep_functions:
    if ufr.tcpsock is None and ufr.resturl is None:
      print("ENTER_SLEEP_MODE")
      ufr.enter_sleep_mode()
      print("LEAVE_SLEEP_MODE")
      ufr.leave_sleep_mode()

  if test_led_sound_functions:
    print("RED_LIGHT_CONTROL")
    ufr.red_light_control(True)
    sleep(2)
    ufr.red_light_control(False)
    print("SET_SPEAKER_FREQUENCY")
    freq = 1480 / 4
    for i in range(3):
      ufr.set_speaker_frequency(freq)
      freq *= 2
      sleep(.1)
    ufr.set_speaker_frequency(0)
    print("USER_INTERFACE_SIGNAL")
    ufr.user_interface_signal(ufrlightsignal.ALTERNATION, ufrbeepsignal.SHORT)
    if ufr.udpsock is not None or ufr.tcpsock is not None:
      print("ESP_SET_DISPLAY_DATA")
      for i in range(3):
        ufr.esp_set_display_data((0xff, 0, 0), (0, 0xff, 0), 0)
        sleep(0.1)
        ufr.esp_set_display_data((0, 0xff, 0), (0, 0, 0xff), 0)
        sleep(0.1)
        ufr.esp_set_display_data((0, 0, 0xff), (0xff, 0, 0), 0)
        sleep(0.1)
      ufr.esp_set_display_data((0, 0, 0), (0, 0, 0), 1000)

  if test_esp_io and (ufr.udpsock is not None or ufr.tcpsock):
    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, ufriostate.HIGH)
    print("ESP_GET_IO_STATE", ufr.esp_get_io_state())
    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, ufriostate.LOW)
    print("ESP_GET_IO_STATE", ufr.esp_get_io_state())

  if test_uid_functions:
    for i in range(10):
      print("GET_CARD_ID_EX:         ", ufr.get_card_id_ex())
      sleep(.5)

  if test_tag_emulation:
    eeprom_ndef = b"\x03\x10\xd1\x01\x0cU\x01d-logic.net\xfe"
    ram_ndef = b"\x03\xff\x03\xeb\xc1\x01\x00\x00\x03\xe4T\x02en3456" + \
		b"7890123456" * 99
    if test_eeprom_writing_functions:
      print("WRITE_EMULATION_NDEF (EEPROM)")
      ufr.write_emulation_ndef(eeprom_ndef, False)
    print("WRITE_EMULATION_NDEF (RAM)")
    ufr.write_emulation_ndef(ram_ndef, True)
  print("TAG_EMULATION_START (RAM NDEF)")
  ufr.tag_emulation_start(True)
  sleep(10)
  print("TAG_EMULATION_STOP")
  ufr.tag_emulation_stop()

  ufr.close()
  del(ufr)
