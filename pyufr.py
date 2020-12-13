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
test_network_probe_functions      = False
test_eeprom_writing_functions     = False
test_reader_info_functions        = False
test_ad_hoc_functions             = False
test_rf_analog_settings_functions = False
test_reset_functions              = False
test_sleep_functions              = False
test_led_sound_functions          = False
test_esp_io                       = False
test_uid_functions                = True
test_read_functions               = False
test_iso14443_4_functions         = False
test_anti_collision_functions     = False
test_tag_emulation                = False



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
  GET_EXTERNAL_FIELD_STATE                = 0x9f
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
  ENABLE_ANTI_COLLISION                   = 0x2d
  DISABLE_ANTI_COLLISION                  = 0x2e
  ENUM_CARDS                              = 0x37
  LIST_CARDS                              = 0x38
  SELECT_CARD                             = 0x39
  DESELECT_CARD                           = 0x3a
  GET_ANTI_COLLISION_STATUS               = 0x3b
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
  NO_CARDS_ENUMERATED                     = 0x7e
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

class ufrcardtype(IntEnum):	# Partially documented - may be wrong/incomplete

  GENERIC                                 = 0x00	# Undocumented
  MIFARE_CLASSIC_1K                       = 0x08
  MIFARE_CLASSIC_4K                       = 0x18
  MIFARE_MINI                             = 0x09
  CONTACTLESS_EMV                         = 0x0b	# Undocumented
  MIFARE_DESFIRE                          = 0x20	# Undocumented

class ufrdlcardtype(IntEnum):
  DL_MIFARE_ULTRALIGHT                    = 0x01
  DL_MIFARE_ULTRALIGHT_EV1_11             = 0x02
  DL_MIFARE_ULTRALIGHT_EV1_21             = 0x03
  DL_MIFARE_ULTRALIGHT_C                  = 0x04
  DL_NTAG_203                             = 0x05
  DL_NTAG_210                             = 0x06
  DL_NTAG_212                             = 0x07
  DL_NTAG_213                             = 0x08
  DL_NTAG_215                             = 0x09
  DL_NTAG_216                             = 0x0a
  MIKRON_MIK640D                          = 0x0b
  NFC_T2T_GENERIC                         = 0x0c
  DL_MIFARE_MINI                          = 0x20
  DL_MIFARE_CLASSIC_1K                    = 0x21
  DL_MIFARE_CLASSIC_4K                    = 0x22
  DL_MIFARE_PLUS_S_2K                     = 0x23
  DL_MIFARE_PLUS_S_4K                     = 0x24
  DL_MIFARE_PLUS_X_2K                     = 0x25
  DL_MIFARE_PLUS_X_4K                     = 0x26
  DL_MIFARE_DESFIRE                       = 0x27
  DL_MIFARE_DESFIRE_EV1_2K                = 0x28
  DL_MIFARE_DESFIRE_EV1_4K                = 0x29
  DL_MIFARE_DESFIRE_EV1_8K                = 0x2a
  DL_MIFARE_DESFIRE_EV2_2K                = 0x2b
  DL_MIFARE_DESFIRE_EV2_4K                = 0x2c
  DL_MIFARE_DESFIRE_EV2_8K                = 0x2d
  DL_GENERIC_ISO14443_4                   = 0x40
  DL_GENERIC_ISO14443_TYPE_B              = 0x41
  DL_IMEI_UID                             = 0x80

class ufrauthmode(IntEnum):
  T2T_NO_PWD_AUTH                         = 0x00
  T2T_RKA_PWD_AUTH                        = 0x01
  T2T_PK_PWD_AUTH                         = 0x61
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

class ufremumode(IntEnum):
  TAG_EMU_DISABLED                        = 0,
  TAG_EMU_DEDICATED                       = 1,
  TAG_EMU_COMBINED                        = 2,
  TAG_EMU_AUTO_AD_HOC                     = 3

class ufremustate(IntEnum):
  EMULATION_NONE                          = 0,
  EMULATION_IDLE                          = 1,
  EMULATION_AUTO_COLL                     = 2,
  EMULATION_ACTIVE                        = 3,
  EMULATION_HALT                          = 4,
  EMULATION_POWER_OFF                     = 5

class ufrpcdmgrstate(IntEnum):
  PCD_MGR_NO_RF_GENERATED                 = 0,
  PCD_MGR_14443A_POLLING                  = 1,
  PCD_MGR_14443A_SELECTED                 = 2,
  PCD_MGR_CE_DEDICATED                    = 3,
  PCD_MGR_CE_COMBO_START                  = 4,
  PCD_MGR_CE_COMBO                        = 5,
  PCD_MGR_CE_COMBO_IN_FIELD               = 6



### Defines
ufr_header_vals = tuple(map(int, ufrhead)) 
ufr_trailer_vals = tuple(map(int, ufrtrail)) 
ufr_cmd_vals = tuple(map(int, ufrcmd))
ufr_err_vals = tuple(map(int, ufrerr))
ufr_val_to_card_type = {ct.value: ct for ct in ufrcardtype}
ufr_val_to_dl_card_type = {dlct.value: dlct for dlct in ufrdlcardtype}
ufr_val_to_emu_mode = {em.value: em for em in ufremumode}
ufr_val_to_emu_state = {st.value: st for st in ufremustate}
ufr_val_to_pcd_mgr_state = {pmst.value: pmst for pmst in ufrpcdmgrstate}
ufr_val_to_cmd = {cmd.value: cmd for cmd in ufrcmd}
ufr_val_to_err = {err.value: err for err in ufrerr}
ufr_val_to_iostate = {iostate.value: iostate for iostate in ufriostate}

# Number of concurrent connection when scanning a subnet for Nano Onlines
subnet_probe_concurrent_connections = 100

# Leave sleep mode parameters
wake_up_byte                   = 0x00
wake_up_wait                   = .01 #s

# Extra delays following certain commands that aren't prescribed in the COM
# protocol, but that are apparently needed to prevent the reader from going
# unresponsive after the command
post_wake_up_wait              = .1 #s
post_reset_wait                = .1 #s
post_write_emulation_ndef_wait = .1 #s
post_emulation_start_stop_wait = .01 #s



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
    


  def _uid_bytes2str(self, bytesuid):
    """Convert bytes or a list of integers into a human-readable UID
    """
    return(":".join(["{:02X}".format(b) for b in bytesuid]))



  def _uid_str2bytes(self, struid):
    """Convert a human-readable UID into bytes
    """
    return(bytes([int(v, 16) for v in struid.split(":")]))



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



  def _send_cmd(self, cmd, par0 = 0, par1 = 0, ext_len = 0):
    """Send a short command
    """

    packet = [ufrhead.CMD_HEADER.value, cmd.value, ufrtrail.CMD_TRAILER.value,
		ext_len, par0, par1]
    packet.append(self._checksum(packet))
    self._send_data(packet)

    self.last_cmd = cmd



  def _send_ext(self, ext_parms):
    """Sent extended command parameters
    """

    packet = list(ext_parms)
    packet.append(self._checksum(packet))
    self._send_data(packet)



  def _send_cmd_ext(self, cmd, par0, par1, ext_parms, timeout = None):
    """Send an extended command in two steps: first the short command, wait for
    an ACK, then send the extended command parameters
    """

    ext_len = len(ext_parms) + 1

    if not ext_len:
      return(self._send_cmd(cmd, par0, par1, 0))

    self._send_cmd(cmd, par0, par1, ext_len)

    answer = self._get_answer(timeout)

    if not answer.is_ack or answer.code != cmd.value:
      raise ValueError("expected ACK to {}, ext_len={}, "
			"par0={:02x}h, par1={:02x}h - got {}".format(
			cmd.name, ext_len, par0, par1, answer.printable()))

    self._send_ext(ext_parms)



  def _get_answer(self, timeout = None):
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



  def _get_cmd_ext_part_ack(self, timeout = None):
    """Get a multipart CMD_EXT acknowledgment
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



  def _get_last_command_response(self, timeout = None):
    """Get a responde to the last command sent. Throw an exception if the
    answer is unexpected
    """

    answer = self._get_answer(timeout)
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



  # Front-end API functions - roughly 1:1 with the Digital Logic COM protocol
  # with a few additional convenience functions
  #
  # Incomplete - functions will be added as needed

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



  def get_reader_type(self, timeout = None):
    """Get the reader's type
    """

    self._send_cmd(ufrcmd.GET_READER_TYPE)
    rsp = self._get_last_command_response(timeout)
    return(rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24))



  def get_reader_serial(self, timeout = None):
    """Get the reader's serial number as an integer
    """
    self._send_cmd(ufrcmd.GET_READER_SERIAL)
    rsp = self._get_last_command_response(timeout)
    return(rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24))



  def get_serial_number(self, timeout = None):
    """Get the reader's serial number as a string
    """

    self._send_cmd(ufrcmd.GET_SERIAL_NUMBER)
    rsp = self._get_last_command_response(timeout)
    return(bytes(rsp.ext).decode("ascii"))



  def get_hardware_version(self, timeout = None):
    """Get the reader's hardware version
    """

    self._send_cmd(ufrcmd.GET_HARDWARE_VERSION)
    rsp = self._get_last_command_response(timeout)
    return((rsp.val0 << 8) + rsp.val1)



  def get_firmware_version(self, timeout = None):
    """Get the reader's firmware version
    """

    self._send_cmd(ufrcmd.GET_FIRMWARE_VERSION)
    rsp = self._get_last_command_response(timeout)
    return((rsp.val0 << 8) + rsp.val1)



  def get_build_number(self, timeout = None):
    """Get the reader's firmware's build number
    """

    self._send_cmd(ufrcmd.GET_BUILD_NUMBER)
    rsp = self._get_last_command_response(timeout)
    return(rsp.val0)



  def get_card_id(self, timeout = None):
    """Get the card type and UID (4 bytes only)
    Return the UID or None if no card is in the field
    If the card type is unknown, return the integer value
    """

    self._send_cmd(ufrcmd.GET_CARD_ID)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise
    return(ufr_val_to_card_type.get(rsp.val0, rsp.val0),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_card_id_ex(self, timeout = None):
    """Get the card type and UID (4, 7 or 10 bytes)
    Return the UID or None if no card is in the field
    If the card type is unknown, return the integer value
    """

    self._send_cmd(ufrcmd.GET_CARD_ID_EX)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise
    return(ufr_val_to_card_type.get(rsp.val0, rsp.val0),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_last_card_id_ex(self, timeout = None):
    """Get the last read card type and UID (4, 7 or 10 bytes)
    Return the UID or None if no card was last read
    If the card type is unknown, return the integer value
    """

    self._send_cmd(ufrcmd.GET_LAST_CARD_ID_EX)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise
    return(ufr_val_to_card_type.get(rsp.val0, rsp.val0),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_dlogic_card_type(self, timeout = None):
    """Get the Digital Logic card type or None if no card is in the field
    """

    self._send_cmd(ufrcmd.GET_DLOGIC_CARD_TYPE)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise
    return(ufr_val_to_dl_card_type[rsp.val0])



  def linear_read(self, authmode, addr, length, key = 0, multiblock = False,
			timeout = None):
    """Linear read from a card. Return None if no card was present in the field
    STATUS: partially tested
    """

    # Define the command parameters
    cmdext = [addr & 0xff, addr >> 8]

    if authmode in (ufrauthmode.RKA_AUTH1A, ufrauthmode.RKA_AUTH1B):
      par1 = key
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (ufrauthmode.AKM1_AUTH1A, ufrauthmode.AKM1_AUTH1B,
		ufrauthmode.AKM2_AUTH1A, ufrauthmode.AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    elif authmode in (ufrauthmode.PK_AUTH1A, ufrauthmode.PK_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      cmdext.extend(list(key))

    elif authmode in (ufrauthmode.SAM_KEY_AUTH1A, ufrauthmode.SAM_KEY_AUTH1B):
      par1 = key
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (ufrauthmode.PK_AUTH1A_AES, ufrauthmode.PK_AUTH1B_AES):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      cmdext.extend(list(key))

    elif authmode in (ufrauthmode.MFP_RKA_AUTH1A, ufrauthmode.MFP_RKA_AUTH1B):
      par1 = key
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (ufrauthmode.MFP_AKM1_AUTH1A, ufrauthmode.MFP_AKM1_AUTH1B,
		ufrauthmode.MFP_AKM2_AUTH1A, ufrauthmode.MFP_AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    # Send the command and read back the data
    self._send_cmd_ext(ufrcmd.LINEAR_READ, par1, 0, cmdext, timeout)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARD:
        return(None)
      else:
        raise

    return(bytes(rsp.ext))



  def get_rf_analog_settings(self, tag_comm_type, timeout = None):
    """Get the RF frontend's analog settings
    """

    self._send_cmd(ufrcmd.GET_RF_ANALOG_SETTINGS, tag_comm_type)
    rsp = self._get_last_command_response(timeout)
    return(rsp.ext)



  def set_rf_analog_settings(self, tag_comm_type, factory_settings, settings,
				timeout = None):
    """Set the RF frontend's analog settings
    """

    self._send_cmd_ext(ufrcmd.SET_RF_ANALOG_SETTINGS, tag_comm_type,
			1 if factory_settings else 0, settings, timeout)
    rsp = self._get_last_command_response(timeout)



  def set_led_config(self, blink, timeout = None):
    """Set the green LED's configuration
    """

    self._send_cmd(ufrcmd.SET_LED_CONFIG, 1 if blink else 0)
    rsp = self._get_last_command_response(timeout)



  def enter_sleep_mode(self, timeout = None):
    """Send the reader to sleep
    """

    self._send_cmd(ufrcmd.ENTER_SLEEP_MODE)
    rsp = self._get_last_command_response(timeout)



  def leave_sleep_mode(self, timeout = None):
    """Wake up the reader
    """

    self._send_data((wake_up_byte,))
    sleep(wake_up_wait)
    self._send_cmd(ufrcmd.LEAVE_SLEEP_MODE)
    rsp = self._get_last_command_response(timeout)
    sleep(post_wake_up_wait)



  def rf_reset(self, timeout = None):
    """Reset the RF field
    """

    self._send_cmd(ufrcmd.RF_RESET)
    rsp = self._get_last_command_response(timeout)



  def check_uid_change(self, timeout = None):
    """Return True if the card's UID is changeable (magic Mifare Classic gen2),
    False if it isn't of if no card is in the field
    NOTE: Does NOT test if the card responds to the magic Mifare command
          (magic Mifare Classic gen 1a), only if the UID is directly writeable
    """

    self._send_cmd(ufrcmd.CHECK_UID_CHANGE)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.READING_ERROR:
        return(False)
      else:
        raise
    return(True)



  def get_reader_status(self, timeout = None):
    """Get the states of the reader
    """

    self._send_cmd(ufrcmd.GET_READER_STATUS)
    rsp = self._get_last_command_response(timeout)
    return(ufr_val_to_pcd_mgr_state[rsp.ext[0]],
		ufr_val_to_emu_mode[rsp.ext[1]],
		ufr_val_to_emu_state[rsp.ext[2]])



  def self_reset(self, timeout = None):
    """Soft-restart the reader
    """

    self._send_cmd(ufrcmd.SELF_RESET)
    rsp = self._get_last_command_response(timeout)
    sleep(post_reset_wait)



  def write_emulation_ndef(self, ndef, in_ram = False, timeout = None):
    """Write the emulation NDEF in EEPROM or in RAM. The NDEF in EEPROM survives
    resets but is 144 bytes long at the most. The NDEF in RAM can be 1008 bytes
    long but is wiped after a reset
    """

    ndeflen = len(ndef)

    if (not in_ram and ndeflen > 144) or (in_ram and ndeflen > 1008):
      raise ValueError("NDEF too long")

    # Split the NDEF into 240-byte-long parts
    ext_parts = [ndef[i:i + 240] for i in range(0, ndeflen, 240)]
    if not ext_parts:
      ext_parts = [b""]

    # First CMD_EXT part is prefixed with the length of the NDEF, and suffixed
    # with the checksum of that part (but _send_cmd_ext() will take care of
    # appending the checksum)
    ext_parts[0] = bytes([ndeflen & 0xff, ndeflen >> 8]) + ext_parts[0]

    # Subsequent CMD_EXT parts are only prefixed with the length of that part
    for i in range(1, len(ext_parts)):
      ext_parts[i] = bytes([len(ext_parts[i])]) + ext_parts[i]

    # Send the command and first CMD_EXT part
    self._send_cmd_ext(ufrcmd.WRITE_EMULATION_NDEF, 1 if in_ram else 0, 0,
			ext_parts.pop(0), timeout)

    # Wait for ACKs and send subsequent parts if we have more than one part
    if ext_parts:
      while self._get_cmd_ext_part_ack(timeout):
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

    rsp = self._get_last_command_response(timeout)
    sleep(post_write_emulation_ndef_wait)



  def tag_emulation_start(self, ram_ndef = False, timeout = None):
    """Start tag emulation. Use either the NDEF in EEPROM or in RAM
    """

    self._send_cmd(ufrcmd.TAG_EMULATION_START, 1 if ram_ndef else 0)
    rsp = self._get_last_command_response(timeout)
    sleep(post_emulation_start_stop_wait)



  def tag_emulation_stop(self, timeout = None):
    """Stop tag emulation
    """

    self._send_cmd(ufrcmd.TAG_EMULATION_STOP)
    rsp = self._get_last_command_response(timeout)
    sleep(post_emulation_start_stop_wait)



  def ad_hoc_emulation_start(self, timeout = None):
    """Start ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(ufrcmd.AD_HOC_EMULATION_START)
    rsp = self._get_last_command_response(timeout)
    sleep(post_emulation_start_stop_wait)



  def ad_hoc_emulation_stop(self, timeout = None):
    """Stop ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(ufrcmd.AD_HOC_EMULATION_STOP)
    rsp = self._get_last_command_response(timeout)
    sleep(post_emulation_start_stop_wait)



  def get_external_field_state(self, timeout = None):
    """Test the presence of an external RF field in ad-hoc (peer-to-peer) mode
    """

    self._send_cmd(ufrcmd.GET_EXTERNAL_FIELD_STATE)
    rsp = self._get_last_command_response(timeout)
    return(rsp.val0 == 1)



  def get_ad_hoc_emulation_params(self, timeout = None):
    """Get current ad-hoc (peer-to-peer) emulation parameters
    Return RxThreshold and RFCfg
    """

    self._send_cmd(ufrcmd.GET_AD_HOC_EMULATION_PARAMS)
    rsp = self._get_last_command_response(timeout)
    return(rsp.val0, rsp.val1)



  def set_ad_hoc_emulation_params(self, rxthreshold, rfcfg, timeout = None):
    """Set current ad-hoc (peer-to-peer) emulation parameters
    """

    self._send_cmd(ufrcmd.SET_AD_HOC_EMULATION_PARAMS, rxthreshold,
							rfcfg & 0x7f)
    rsp = self._get_last_command_response(timeout)



  def red_light_control(self, state, timeout = None):
    """Turn the red LED on or off
    """

    self._send_cmd(ufrcmd.RED_LIGHT_CONTROL, 1 if state else 0)
    rsp = self._get_last_command_response(timeout)



  def user_interface_signal(self, light_signal_mode, beep_signal_mode,
				timeout = None):
    """Trigger a LED sequence or beep sequence
    """
    self._send_cmd(ufrcmd.USER_INTERFACE_SIGNAL, light_signal_mode.value,
			beep_signal_mode.value)
    rsp = self._get_last_command_response(timeout)



  def set_speaker_frequency(self, frequency, timeout = None):
    """Make the buzzer emit a continuous sound. Set frequency to 0 or None to
    stop the sound
    """

    period = ((round(65535 - 1500000 / (2 * frequency))) & 0xffff) \
		if frequency else 0xffff
    self._send_cmd(ufrcmd.SET_SPEAKER_FREQUENCY, period & 0xff, period >> 8)
    rsp = self._get_last_command_response(timeout)



  def set_iso14443_4_mode(self, timeout = None):
    """Set ISO14443-4 mode
    """

    self._send_cmd(ufrcmd.SET_ISO14443_4_MODE)
    rsp = self._get_last_command_response(timeout)



  def s_block_deselect(self, timeout = None):
    """Deselect tag and restore polling
    STATUS: untested
    """

    self._send_cmd(ufrcmd.S_BLOCK_DESELECT)
    rsp = self._get_last_command_response(timeout)



  def apdu_transceive(self, c_apdu, apdu_timeout = None, timeout = None):
    """Send a command APDU to the ISO14443-4 transponder and get the response
    APDU
    STATUS: untested
    """

    self._send_cmd_ext(ufrcmd.APDU_TRANSCEIVE, 0, self.default_timeout \
			if apdu_timeout is None else apdu_timeout,
			c_apdu, timeout)
    rsp = self._get_last_command_response(timeout)
    return(rsp.ext)



  def enable_anti_collision(self, timeout = None):
    """Enable anti-collision mode: leave single-card mode and stop polling
    """

    self._send_cmd(ufrcmd.ENABLE_ANTI_COLLISION)
    rsp = self._get_last_command_response(timeout)

  def disable_anti_collision(self, timeout = None):
    """Disable anti-collision mode; return to single-card mode and start polling
    """

    self._send_cmd(ufrcmd.DISABLE_ANTI_COLLISION)
    rsp = self._get_last_command_response(timeout)



  def enum_cards(self, timeout = None):
    """Enumerate cards in the field in anti-collision mode
    Return True if at least one card was enumerated, False if no card was found
    """

    self._send_cmd(ufrcmd.ENUM_CARDS)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARDS_ENUMERATED:
        return(False)
      else:
        raise
    return(True)



  def list_cards(self, timeout = None):
    """List cards previously enumerated by enum_cards()
    Return a list of UIDs or an empty list if no card was enumerated
    """

    self._send_cmd(ufrcmd.LIST_CARDS)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == ufrerr.NO_CARDS_ENUMERATED:
        return([])
      else:
        raise
    return([self._uid_bytes2str(rsp.ext[i + 1 : i + rsp.ext[i] + 1]) \
		for i in range(0, len(rsp.ext), 11)])



  def select_card(self, uid, timeout = None):
    """Select a card in the field in anti-collision mode
    """

    bytesuid = self._uid_str2bytes(uid)
    self._send_cmd_ext(ufrcmd.SELECT_CARD, len(bytesuid), 0, bytesuid, timeout)
    rsp = self._get_last_command_response(timeout)
    return(ufr_val_to_dl_card_type[rsp.val0])



  def deselect_card(self, timeout = None):
    """Deselect the currently selected card in anti-collision mode
    """

    self._send_cmd(ufrcmd.DESELECT_CARD)
    rsp = self._get_last_command_response(timeout)



  def get_anti_collision_status(self, timeout = None):
    """Return the status of the anti-collision mode, and whether a card is
    currently selected
    """

    self._send_cmd(ufrcmd.GET_ANTI_COLLISION_STATUS)
    rsp = self._get_last_command_response(timeout)
    return(rsp.val0 != 0, rsp.val1 != 0)



  def esp_set_io_state(self, pin, state, timeout = None):
    """Set the state of one of the 6 ESP I/O pins
    """

    self._send_cmd(ufrcmd.ESP_SET_IO_STATE, pin, state.value)
    rsp = self._get_last_command_response(timeout)



  def esp_get_io_state(self, timeout = None):
    """Get the states of the 6 ESP I/O pins
    return the states as a list
    """

    self._send_cmd(ufrcmd.ESP_GET_IO_STATE)
    rsp = self._get_last_command_response(timeout)
    return([ufr_val_to_iostate[st] for st in rsp.ext])



  def esp_set_display_data(self, rgb1, rgb2, duration, timeout = None):
    """Set the color of the two ESP LEDs for a certain duration in ms. Set the
    duration to 0 to keep those colors permanently. Set a short duration to
    return to reader-managed colors
    WARNING: don't set two non-zero timeouts in a row, or the reader will go
             unresponsive when the first delay elapses. If you want to
             play a color sequence, only the last command should have a
             non-zero timeout, and you should manage the sequence's delays
             yourself
    """

    self._send_cmd_ext(ufrcmd.ESP_SET_DISPLAY_DATA,
			duration & 0xff, duration >> 8, rgb1 + rgb2, timeout)
    rsp = self._get_last_command_response(timeout)



  def esp_reader_reset(self, timeout = None):
    """Ask the ESP to reset the reader
    """

    self._send_cmd(ufrcmd.ESP_READER_RESET, 0)
    rsp = self._get_last_command_response(timeout)
    sleep(post_reset_wait)



### Routines
def print_api_soc(ufr):
  """Print the API's state of completion - i.e. which Digital Logic COM
  protocol functions are implemented in this class, which aren't, which are
  untested or partially tested, and what percentage of the COM protocol is
  implemented
  """

  comcmds = [cmd.name for cmd in ufrcmd]
  pubclassfcts = [attribute for attribute in dir(ufr) \
			if callable(getattr(ufr, attribute)) and \
			not attribute.startswith('_')]

  # Get the status of all the commands in the COM protocol from whether the
  # corresponding functions exists in lowercase in the class and from status
  # markers in their docstrings
  cmd_impl_status = {}
  nb_impl = 0
  max_cmd_name_len = 0
  max_status_len = 0
  for cmd in comcmds:
    if cmd.lower() in pubclassfcts:
      localparms = {'ufr': ufr}
      exec("doc = ufr.{}.__doc__".format(cmd.lower()), {}, localparms)
      m = re.findall("(STATUS|COMPLETION)\s*:\s*(.*)\n", localparms["doc"])
      cmd_impl_status[cmd] = "Implemented" + (" - {}".format(m[0][1]) \
					if m else "")
      nb_impl += 1
    else:
      cmd_impl_status[cmd] = "Not implemented"
    max_cmd_name_len = max(max_cmd_name_len, len(cmd))
    max_status_len = max(max_status_len, len(cmd_impl_status[cmd]))

  # Dump the implementation status of all the commands and the percentage of
  # implemented commands
  max_cmd_name_len += 2
  padded = lambda s: ("{" + ":<{}".format(max_cmd_name_len) + "}").format(s)
  separator = "-" * (max_cmd_name_len + max_status_len)

  print(padded("UFR COM protocol command"), "Status")
  print(separator)
  for cmd in comcmds:
    print(padded(cmd), cmd_impl_status[cmd])

  print(separator)
  print(padded("State of completion:"),
		"{}%".format(round(100 * nb_impl / len(ufrcmd))))

  # Dump the list of API functions that are specific to this class
  print()
  print("Class-specific API functions")
  print(separator)
  for fct in sorted(pubclassfcts, key = lambda fct: len(fct)):
    if fct.upper() not in comcmds:
      print("{}()".format(fct))



def test_api(ufr):
  """Test the API
  """

  # Pretty line formatting
  padded = lambda s: "{:<30}".format(s)

  # Network probing functions - the device doesn't need to be open for this
  if test_network_probe_functions:

    sys.stdout.write(padded("PROBE_SUBNET_NANO_ONLINES: "))
    sys.stdout.flush()
    nos = ufr.probe_subnet_nano_onlines("192.168.1.0/24")
    print(nos)
    for no in nos:
      print(padded("IS_HOST_NANO_ONLINE:"), ufr.is_host_nano_online(no))

  # Open the device
  ufr.open(args.device)

  # Reader information functions
  if test_reader_info_functions:

    print(padded("GET_READER_TYPE:"), hex(ufr.get_reader_type()))
    print(padded("GET_READER_SERIAL:"), ufr.get_reader_serial())
    print(padded("GET_SERIAL_NUMBER:"), ufr.get_serial_number())
    print(padded("GET_HARDWARE_VERSION:"), hex(ufr.get_hardware_version()))
    print(padded("GET_FIRMWARE_VERSION:"), hex(ufr.get_firmware_version()))
    print(padded("GET_BUILD_NUMBER:"), hex(ufr.get_build_number()))
    print(padded("GET_READER_STATUS:"), ufr.get_reader_status())

  # Ad-hoc (peer-to-peer) functions
  if test_ad_hoc_functions:

    print("AD_HOC_EMULATION_START")
    ufr.ad_hoc_emulation_start()
    rxthreshold, rfcfg = ufr.get_ad_hoc_emulation_params()
    print(padded("GET_AD_HOC_EMULATION_PARAMS:"), hex(rxthreshold), hex(rfcfg))
    print("SET_AD_HOC_EMULATION_PARAMS:")
    ufr.set_ad_hoc_emulation_params(rxthreshold, rfcfg)
    print(padded("GET_READER_STATUS:"), ufr.get_reader_status())
    print(padded("GET_EXTERNAL_FIELD_STATE"), ufr.get_external_field_state())
    print("AD_HOC_EMULATION_STOP")
    ufr.ad_hoc_emulation_stop()

  # RF analog settings functions
  if test_rf_analog_settings_functions:

    for tct in map(int, ufrtagcommtype):
      print(padded("GET_RF_ANALOG_SETTINGS:"), ufr.get_rf_analog_settings(tct))

      if test_eeprom_writing_functions:

        new_settings = list(ufr.answer.ext)
        new_settings[pn53xanalogsettingsreg.RXTHRESHOLD] = 255
        print("SET_RF_ANALOG_SETTINGS")
        ufr.set_rf_analog_settings(tct, False, new_settings)
        print("SET_LED_CONFIG")
        ufr.set_led_config(True)

  # Reset functions
  if test_reset_functions:

    print("RF_RESET")
    ufr.rf_reset()
    print("SELF_RESET")
    ufr.self_reset()

    # Only test the ESP reset function if the device is a Nano Online connected
    # through the network, but not in HTTP transparent mode, as transparent
    # mode bypasses the the ESP and sends the commands directly to the UART
    if ufr.udpsock is not None or ufr.tcpsock:

      print("ESP_READER_RESET")
      ufr.esp_reader_reset()

  # Sleep functions - only works if the device is connected directly to a
  # a serial port or through the network in HTTP transparent mode, as we'll
  # lose communication with it and won't be able to wake it back up in TCP
  # or UDP mode
  if test_sleep_functions and ufr.tcpsock is None and ufr.resturl is None:

    print("ENTER_SLEEP_MODE")
    ufr.enter_sleep_mode()
    print("LEAVE_SLEEP_MODE")
    ufr.leave_sleep_mode()

  # LED and buzzer functions
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

    # Only test the ESP LED function if the device is a Nano Online connected
    # through the network, but not in HTTP transparent mode, as transparent
    # mode bypasses the the ESP and sends the commands directly to the UART
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

  # ESP I/O functions - only works if the device is a Nano Online connected
  # through the network, but not in HTTP transparent mode, as transparent
  # mode bypasses the the ESP and sends the commands directly to the UART
  if test_esp_io and (ufr.udpsock is not None or ufr.tcpsock):

    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, ufriostate.HIGH)
    print(padded("ESP_GET_IO_STATE"), ufr.esp_get_io_state())
    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, ufriostate.LOW)
    print(padded("ESP_GET_IO_STATE"), ufr.esp_get_io_state())

  # UID functions
  if test_uid_functions:

      print(padded("GET_CARD_ID:"), ufr.get_card_id_ex())
      print(padded("GET_CARD_ID_EX:"), ufr.get_card_id_ex())
      print(padded("GET_LAST_CARD_ID_EX:"), ufr.get_last_card_id_ex())
      print(padded("GET_DLOGIC_CARD_TYPE:"), ufr.get_dlogic_card_type())
      print(padded("CHECK_UID_CHANGE:"), ufr.check_uid_change())

  # Test read functions
  if test_read_functions:
      print(padded("LINEAR_READ:"), ufr.linear_read(ufrauthmode.T2T_NO_PWD_AUTH,
							0, 10))

  # Get ISO14443-4 functions
  if test_iso14443_4_functions:

    print("SET_ISO_14443_4_MODE")
    try:
      ufr.set_iso14443_4_mode()
    except:
      if ufr.answer.code != ufrerr.NO_CARD:
        raise

  # Anti-collision functions
  if test_anti_collision_functions:

    print("ENABLE_ANTI_COLLISION")
    ufr.enable_anti_collision()
    print(padded("GET_ANTI_COLLISION_STATUS:"), ufr.get_anti_collision_status())
    print(padded("ENUM_CARDS:"), ufr.enum_cards())
    uids = ufr.list_cards()
    print(padded("LIST_CARDS:"), uids)
    if uids:
      print(padded("SELECT CARD:"), ufr.select_card(uids[0]))
      print(padded("GET_ANTI_COLLISION_STATUS:"),
		ufr.get_anti_collision_status())
      print("DESELECT CARD")
      ufr.deselect_card()
      print(padded("GET_ANTI_COLLISION_STATUS:"),
		ufr.get_anti_collision_status())
    print("DISABLE_ANTI_COLLISION")
    ufr.disable_anti_collision()
    print(padded("GET_ANTI_COLLISION_STATUS:"), ufr.get_anti_collision_status())

  # Tag emulation functions
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
    print(padded("GET_READER_STATUS:"), ufr.get_reader_status())
    print("TAG_EMULATION_STOP")
    ufr.tag_emulation_stop()



### Main routine - run if the class is called as a standalone program
if __name__ == "__main__":

  ### Modules
  import sys
  import argparse

  # Parse the command line arguments
  argparser=argparse.ArgumentParser()
  argparser.add_argument(
	  "-d", "--device",
	  help = "uFR device to test (default {})".format(default_ufr_device),
	  type = str,
	  default = default_ufr_device
	)
  argparser.add_argument(
	  "-soc", "--state-of-completion",
	  help = "Print the API's state of completion",
	  action = "store_true"
	)
  args=argparser.parse_args()

  # Create the ufr object
  ufr = ufr()

  # Dump the API's state of completion
  if args.state_of_completion:
    print_api_soc(ufr)

  # Test the API
  else:
    test_api(ufr)

  # Close the device
  ufr.close()

  # Delete the ufr object
  del(ufr)
