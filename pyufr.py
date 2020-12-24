#!/usr/bin/python3
"""Pure Python class to communicate with uFR series readers
"""

from __future__ import annotations

### Parameters
_default_ufr_device = "serial:///dev/ttyUSB0:1000000"	# Nano USB serial
#_default_ufr_device = "udp://ufr:8881"			# Nano Online slave UDP
#_default_ufr_device = "tcp://ufr:8881"			# Nano Online slave TCP
#_default_ufr_device = "http://ufr/uart1"		# Nano Online REST UART1
_default_ufr_timeout = 1 #s

# Extra delays following certain commands, that aren't prescribed in the COM
# protocol, but that are apparently needed to prevent the reader from going
# unresponsive after the command
_post_wake_up_wait: float                 = .1 #s
_post_reset_wait: float                   = .1 #s
_post_write_emulation_ndef_wait: float    = .1 #s
_post_emulation_start_stop_wait: float    = .01 #s

# Number of concurrent connection when scanning a subnet for Nano Onlines
_subnet_probe_concurrent_connections: int = 100

# API tests
__test_network_probe_functions      = False
__test_eeprom_writing_functions     = False
__test_reader_info_functions        = True
__test_ad_hoc_functions             = True
__test_rf_analog_settings_functions = True
__test_reset_functions              = True
__test_sleep_functions              = True
__test_led_sound_functions          = True
__test_esp_io                       = True
__test_uid_functions                = True
__test_read_functions               = True
__test_iso14443_4_functions         = True
__test_anti_collision_functions     = True
__test_tag_emulation                = True



### Modules
from typing import Any, List, Tuple, Dict, Callable, Union, Optional
import re
import socket
import requests
from time import sleep
from enum import IntEnum
from datetime import datetime
from multiprocessing.pool import ThreadPool

# Try to import optional modules but fail silently if they're not needed later
try:
  from serial import Serial					# type: ignore
except:
  pass
try:
  from ipaddress import ip_network, IPv4Network
except:
  pass



### Enums
class uFRhead(IntEnum):
  CMD_HEADER: int                              = 0x55
  ACK_HEADER: int                              = 0xac
  RESPONSE_HEADER: int                         = 0xde
  ERR_HEADER: int                              = 0xec

class uFRcmdextpartack(IntEnum):
  ACK_PART: int                                = 0xad
  ACK_LAST_PART: int                           = 0xdd

class uFRtrail(IntEnum):
  CMD_TRAILER: int                             = 0xaa
  ACK_TRAILER: int                             = 0xca
  RESPONSE_TRAILER: int                        = 0xed
  ERR_TRAILER: int                             = 0xce

class uFRcmd(IntEnum):
  _UNDEFINED: int                              = -1
  GET_READER_TYPE: int                         = 0x10
  GET_READER_SERIAL: int                       = 0x11
  GET_SERIAL_NUMBER: int                       = 0x40
  GET_HARDWARE_VERSION: int                    = 0x2a
  GET_FIRMWARE_VERSION: int                    = 0x29
  GET_BUILD_NUMBER: int                        = 0x2b
  READER_KEY_WRITE: int                        = 0x12
  USER_DATA_READ: int                          = 0x1b
  USER_DATA_WRITE: int                         = 0x1c
  READER_KEYS_LOCK: int                        = 0x27
  READER_KEYS_UNLOCK: int                      = 0x28
  READER_PASSWORD_WRITE: int                   = 0x33
  SELF_RESET: int                              = 0x30
  SET_SPEED_PERMANENTLY: int                   = 0x4b
  GET_SPEED_PARAMETERS: int                    = 0x4c
  SET_UART_SPEED: int                          = 0x70
  RED_LIGHT_CONTROL: int                       = 0x71
  USER_INTERFACE_SIGNAL: int                   = 0x26
  SET_RF_ANALOG_SETTINGS: int                  = 0x7d
  GET_RF_ANALOG_SETTINGS: int                  = 0x7e
  SET_LED_CONFIG: int                          = 0x6e
  DEFAULT_UART_SPEED_SESSION: int              = 0xf1
  GET_CARD_ID: int                             = 0x13
  GET_CARD_ID_EX: int                          = 0x2c
  GET_DLOGIC_CARD_TYPE: int                    = 0x3c
  GET_LAST_CARD_ID_EX: int                     = 0x7c
  SECTOR_TRAILER_WRITE: int                    = 0x1a
  SECTOR_TRAILER_WRITE_UNSAFE: int             = 0x2f
  BLOCK_READ: int                              = 0x16
  BLOCK_WRITE: int                             = 0x17
  BLOCK_IN_SECTOR_READ: int                    = 0x18
  BLOCK_IN_SECTOR_WRITE: int                   = 0x19
  LINEAR_READ: int                             = 0x14
  LINEAR_WRITE: int                            = 0x15
  LINEAR_FORMAT_CARD: int                      = 0x25
  LIN_ROW_READ: int                            = 0x45
  VALUE_BLOCK_READ: int                        = 0x1d
  VALUE_BLOCK_WRITE: int                       = 0x1e
  VALUE_BLOCK_INC: int                         = 0x21
  VALUE_BLOCK_DEC: int                         = 0x22
  VALUE_BLOCK_IN_SECTOR_READ: int              = 0x1f
  VALUE_BLOCK_IN_SECTOR_WRITE: int             = 0x20
  VALUE_BLOCK_IN_SECTOR_INC: int               = 0x23
  VALUE_BLOCK_IN_SECTOR_DEC: int               = 0x24
  GET_DESFIRE_UID: int                         = 0x80
  SET_DESFIRE_KEY: int                         = 0x81
  DESFIRE_WRITE_TO_FILE: int                   = 0x82
  DESFIRE_READ_FROM_FILE: int                  = 0x83
  DESFIRE_CREATE_APPLICATION: int              = 0x84
  DESFIRE_CREATE_FILE: int                     = 0x85
  DESFIRE_CREATE_AES_KEY: int                  = 0x86
  DESFIRE_GET_KEY_CONFIG: int                  = 0x87
  DESFIRE_CHANGE_KEY_CONFIG: int               = 0x88
  DESFIRE_DELETE_APPLICATION: int              = 0x89
  DESFIRE_DELETE_FILE: int                     = 0x8a
  DESFIRE_SET_CONFIGURATION: int               = 0x8b
  DESFIRE_FORMAT_CARD: int                     = 0x8c
  DESFIRE_FREE_MEM: int                        = 0x8d
  DESFIRE_WRITE_AES_KEY: int                   = 0x8e
  DESFIRE_CREATE_VALUE_FILE: int               = 0x8f
  DESFIRE_READ_VALUE_FILE: int                 = 0x9a
  DESFIRE_INCREASE_VALUE_FILE: int             = 0x9b
  DESFIRE_DECREASE_VALUE_FILE: int             = 0x9c
  DESFIRE_CREATE_RECORD_FILE: int              = 0x97
  DESFIRE_WRITE_RECORD: int                    = 0x98
  DESFIRE_READ_RECORDS: int                    = 0x99
  DESFIRE_CLEAR_RECORD: int                    = 0x6d
  DESFIRE_GET_APPLICATION_IDS: int             = 0xc0
  MFP_FIRST_AUTHENTICATE: int                  = 0x6a
  MFP_CHANGE_REG_KEY: int                      = 0x6b
  MFP_GET_UID: int                             = 0x6c
  GET_NFC_T2T_VERSION: int                     = 0xb0
  READ_COUNTER: int                            = 0xb1
  INCREMENT_COUNTER: int                       = 0xb2
  NT4H_COMMON_CMD: int                         = 0xb3
  READ_ECC_SIGNATURE: int                      = 0xbf
  SET_CARD_ID_SEND_CONF: int                   = 0x3d
  GET_CARD_ID_SEND_CONF: int                   = 0x3e
  SET_BAD_SELECT_NR_MAX: int                   = 0x3f
  GET_BAD_SELECT_NR_MAX: int                   = 0x44
  ENTER_SLEEP_MODE: int                        = 0x46
  LEAVE_SLEEP_MODE: int                        = 0x47
  AUTO_SLEEP_SET: int                          = 0x4d
  AUTO_SLEEP_GET: int                          = 0x4e
  WRITE_EMULATION_NDEF: int                    = 0x4a
  TAG_EMULATION_START: int                     = 0x48
  TAG_EMULATION_STOP: int                      = 0x49
  AD_HOC_EMULATION_START: int                  = 0x76
  AD_HOC_EMULATION_STOP: int                   = 0x77
  GET_EXTERNAL_FIELD_STATE: int                = 0x9f
  GET_AD_HOC_EMULATION_PARAMS: int             = 0x9d
  SET_AD_HOC_EMULATION_PARAMS: int             = 0x9e
  SET_DISPLAY_DATA: int                        = 0x72
  SET_SPEAKER_FREQUENCY: int                   = 0x73
  SET_DISPLAY_INTENSITY: int                   = 0x74
  GET_DISPLAY_INTENSITY: int                   = 0x75
  UFR_XRC_LOCK_OPEN: int                       = 0x60
  UFR_XRC_SET_RELAY_STATE: int                 = 0x61
  UFR_XRC_GET_IO_STATE: int                    = 0x62
  ENTER_SHARE_RAM_COMM_MODE: int               = 0x78
  EXIT_SHARE_RAM_COMM_MODE: int                = 0x79
  READ_SHARE_RAM: int                          = 0x7a
  WRITE_SHARE_RAM: int                         = 0x7b
  I_BLOCK_TRANSCEIVE: int                      = 0x90
  R_BLOCK_TRANSCEIVE: int                      = 0x91
  S_BLOCK_DESELECT: int                        = 0x92
  SET_ISO14443_4_MODE: int                     = 0x93
  APDU_TRANSCEIVE: int                         = 0x94
  ENABLE_ANTI_COLLISION: int                   = 0x2d
  DISABLE_ANTI_COLLISION: int                  = 0x2e
  ENUM_CARDS: int                              = 0x37
  LIST_CARDS: int                              = 0x38
  SELECT_CARD: int                             = 0x39
  DESELECT_CARD: int                           = 0x3a
  GET_ANTI_COLLISION_STATUS: int               = 0x3b
  ESP_SET_IO_STATE: int                        = 0xf3
  ESP_GET_IO_STATE: int                        = 0xf4
  ESP_READER_TIME_WRITE: int                   = 0xf5
  ESP_READER_TIME_READ: int                    = 0xf6
  ESP_READER_EEPROM_READ: int                  = 0xf7
  ESP_SET_DISPLAY_DATA: int                    = 0xf8
  ESP_READER_RESET: int                        = 0xf9
  ESP_READER_PASSWORD_WRITE: int               = 0xfa
  ESP_READER_EEPROM_WRITE: int                 = 0xfb
  CHECK_UID_CHANGE: int                        = 0xe4
  RF_RESET: int                                = 0xe5
  GET_READER_STATUS: int                       = 0xe6

class uFRerr(IntEnum):
  OK: int                                      = 0x00
  COMMUNICATION_ERROR: int                     = 0x01
  CHKSUM_ERROR: int                            = 0x02
  READING_ERROR: int                           = 0x03
  WRITING_ERROR: int                           = 0x04
  BUFFER_OVERFLOW: int                         = 0x05
  MAX_ADDRESS_EXCEEDED: int                    = 0x06
  MAX_KEY_INDEX_EXCEEDED: int                  = 0x07
  NO_CARD: int                                 = 0x08
  COMMAND_NOT_SUPPORTED: int                   = 0x09
  FORBIDEN_DIRECT_WRITE_IN_SECTOR_TRAILER: int = 0x0a
  ADDRESSED_BLOCK_IS_NOT_SECTOR_TRAILER: int   = 0x0b
  WRONG_ADDRESS_MODE: int                      = 0x0c
  WRONG_ACCESS_BITS_VALUES: int                = 0x0d
  AUTH_ERROR: int                              = 0x0e
  PARAMETERS_ERROR: int                        = 0x0f
  MAX_SIZE_EXCEEDED: int                       = 0x10
  UNSUPPORTED_CARD_TYPE: int                   = 0x11
  COUNTER_ERROR: int                           = 0x12
  WRITE_VERIFICATION_ERROR: int                = 0x70
  BUFFER_SIZE_EXCEEDED: int                    = 0x71
  VALUE_BLOCK_INVALID: int                     = 0x72
  VALUE_BLOCK_ADDR_INVALID: int                = 0x73
  VALUE_BLOCK_MANIPULATION_ERROR: int          = 0x74
  WRONG_UI_MODE: int                           = 0x75
  KEYS_LOCKED: int                             = 0x76
  KEYS_UNLOCKED: int                           = 0x77
  WRONG_PASSWORD: int                          = 0x78
  CAN_NOT_LOCK_DEVICE: int                     = 0x79
  CAN_NOT_UNLOCK_DEVICE: int                   = 0x7a
  DEVICE_EEPROM_BUSY: int                      = 0x7b
  RTC_SET_ERROR: int                           = 0x7c
  EEPROM_ERROR: int                            = 0x7d
  NO_CARDS_ENUMERATED: int                     = 0x7e
  CARD_ALREADY_SELECTED: int                   = 0x7f
  WRONG_CARD_TYPE: int                         = 0x80
  FORBIDDEN_IN_TAG_EMULATION_MODE: int         = 0x90
  MFP_COMMAND_OVERFLOW: int                    = 0xb0
  MFP_INVALID_MAC: int                         = 0xb1
  MFP_INVALID_BLOCK_NR: int                    = 0xb2
  MFP_NOT_EXIST_BLOCK_NR: int                  = 0xb3
  MFP_COND_OF_USE_ERROR: int                   = 0xb4
  MFP_LENGTH_ERROR: int                        = 0xb5
  MFP_GENERAL_MANIP_ERROR: int                 = 0xb6
  MFP_SWITCH_TO_ISO14443_4_ERROR: int          = 0xb7
  MFP_ILLEGAL_STATUS_CODE: int                 = 0xb8
  MFP_MULTI_BLOCKS_READ: int                   = 0xb9
  NT4H_COMMAND_ABORTED: int                    = 0xc0
  NT4H_LENGTH_ERROR: int                       = 0xc1
  NT4H_PARAMETER_ERROR: int                    = 0xc2
  NT4H_NO_SUCH_KEY: int                        = 0xc3
  NT4H_PERMISSION_DENIED: int                  = 0xc4
  NT4H_AUTHENTICATION_DELAY: int               = 0xc5
  NT4H_MEMORY_ERROR: int                       = 0xc6
  NT4H_INTEGRITY_ERROR: int                    = 0xc7
  NT4H_FILE_NOT_FOUND: int                     = 0xc8
  NT4H_BOUNDARY_ERROR: int                     = 0xc9
  NT4H_INVALID_MAC: int                        = 0xca
  NT4H_NO_CHANGES: int                         = 0xcb

class uFRcardtype(IntEnum):	# Partially documented - may be wrong/incomplete

  _NO_CARD: int                                = -2
  _UNDEFINED: int                              = -1
  GENERIC: int                                 = 0x00	# Undocumented
  MIFARE_CLASSIC_1K: int                       = 0x08
  MIFARE_CLASSIC_4K: int                       = 0x18
  MIFARE_MINI: int                             = 0x09
  CONTACTLESS_EMV: int                         = 0x0b	# Undocumented
  MIFARE_DESFIRE: int                          = 0x20	# Undocumented

class uFRdlcardtype(IntEnum):
  _NO_CARD: int                                = -2
  _UNDEFINED: int                              = -1
  DL_MIFARE_ULTRALIGHT: int                    = 0x01
  DL_MIFARE_ULTRALIGHT_EV1_11: int             = 0x02
  DL_MIFARE_ULTRALIGHT_EV1_21: int             = 0x03
  DL_MIFARE_ULTRALIGHT_C: int                  = 0x04
  DL_NTAG_203: int                             = 0x05
  DL_NTAG_210: int                             = 0x06
  DL_NTAG_212: int                             = 0x07
  DL_NTAG_213: int                             = 0x08
  DL_NTAG_215: int                             = 0x09
  DL_NTAG_216: int                             = 0x0a
  MIKRON_MIK640D: int                          = 0x0b
  NFC_T2T_GENERIC: int                         = 0x0c
  DL_MIFARE_MINI: int                          = 0x20
  DL_MIFARE_CLASSIC_1K: int                    = 0x21
  DL_MIFARE_CLASSIC_4K: int                    = 0x22
  DL_MIFARE_PLUS_S_2K: int                     = 0x23
  DL_MIFARE_PLUS_S_4K: int                     = 0x24
  DL_MIFARE_PLUS_X_2K: int                     = 0x25
  DL_MIFARE_PLUS_X_4K: int                     = 0x26
  DL_MIFARE_DESFIRE: int                       = 0x27
  DL_MIFARE_DESFIRE_EV1_2K: int                = 0x28
  DL_MIFARE_DESFIRE_EV1_4K: int                = 0x29
  DL_MIFARE_DESFIRE_EV1_8K: int                = 0x2a
  DL_MIFARE_DESFIRE_EV2_2K: int                = 0x2b
  DL_MIFARE_DESFIRE_EV2_4K: int                = 0x2c
  DL_MIFARE_DESFIRE_EV2_8K: int                = 0x2d
  DL_GENERIC_ISO14443_4: int                   = 0x40
  DL_GENERIC_ISO14443_TYPE_B: int              = 0x41
  DL_IMEI_UID: int                             = 0x80

class uFRauthmode(IntEnum):
  T2T_NO_PWD_AUTH: int                         = 0x00
  T2T_RKA_PWD_AUTH: int                        = 0x01
  T2T_PK_PWD_AUTH: int                         = 0x61
  RKA_AUTH1A: int                              = 0x00
  RKA_AUTH1B: int                              = 0x01
  AKM1_AUTH1A: int                             = 0x20
  AKM1_AUTH1B: int                             = 0x21
  AKM2_AUTH1A: int                             = 0x40
  AKM2_AUTH1B: int                             = 0x41
  PK_AUTH1A: int                               = 0x60
  PK_AUTH1B: int                               = 0x61
  PK_AUTH1A_AES: int                           = 0x80
  PK_AUTH1B_AES: int                           = 0x81
  SAM_KEY_AUTH1A: int                          = 0x10
  SAM_KEY_AUTH1B: int                          = 0x11
  MFP_RKA_AUTH1A: int                          = 0x02
  MFP_RKA_AUTH1B: int                          = 0x03
  MFP_AKM1_AUTH1A: int                         = 0x22
  MFP_AKM1_AUTH1B: int                         = 0x23
  MFP_AKM2_AUTH1A: int                         = 0x42
  MFP_AKM2_AUTH1B: int                         = 0x43

class uFRtagcommtype(IntEnum):
  ISO14443_TYPE_A: int                         = 0X01
  ISO14443_TYPE_B: int                         = 0X02
  ISO14443_4_212_KBPS: int                     = 0X03
  ISO14443_4_424_KBPS: int                     = 0X04

class PN53xanalogsettingsreg(IntEnum):
  RFCFG: int                                   = 0
  RXTHRESHOLD: int                             = 1
  GSNON: int                                   = 2
  CWGSP: int                                   = 3
  GSNOFF: int                                  = 4
  MODGSP: int                                  = 4

class uFRlightsignal(IntEnum):
  NONE: int                                    = 0
  LONG_GREEN: int                              = 1
  LONG_RED: int                                = 2
  ALTERNATION: int                             = 3
  FLASH: int                                   = 4

class uFRbeepsignal(IntEnum):
  NONE: int                                    = 0
  SHORT: int                                   = 1
  LONG: int                                    = 2
  DOUBLE_SHORT: int                            = 3
  TRIPLE_SHORT: int                            = 4
  TRIPLET_MELODY: int                          = 5

class uFRiostate(IntEnum):
  LOW: int                                     = 0
  HIGH: int                                    = 1
  INPUT: int                                   = 2

class uFRemumode(IntEnum):
  TAG_EMU_DISABLED: int                        = 0
  TAG_EMU_DEDICATED: int                       = 1
  TAG_EMU_COMBINED: int                        = 2
  TAG_EMU_AUTO_AD_HOC: int                     = 3

class uFRemustate(IntEnum):
  EMULATION_NONE: int                          = 0
  EMULATION_IDLE: int                          = 1
  EMULATION_AUTO_COLL: int                     = 2
  EMULATION_ACTIVE: int                        = 3
  EMULATION_HALT: int                          = 4
  EMULATION_POWER_OFF: int                     = 5

class uFRpcdmgrstate(IntEnum):
  PCD_MGR_NO_RF_GENERATED: int                 = 0
  PCD_MGR_14443A_POLLING: int                  = 1
  PCD_MGR_14443A_SELECTED: int                 = 2
  PCD_MGR_CE_DEDICATED: int                    = 3
  PCD_MGR_CE_COMBO_START: int                  = 4
  PCD_MGR_CE_COMBO: int                        = 5
  PCD_MGR_CE_COMBO_IN_FIELD: int               = 6



### Classes
class uFRanswer:

  def __init__(self: uFRanswer) \
		-> None:
    """__init__ method
    """
    self.wipe()



  def wipe(self: uFRanswer) \
		-> None:
    """Reinitialize answer fields
    """

    self.is_ack: bool = False
    self.is_err: bool = False
    self.is_rsp: bool = False
    self.has_ext: bool = False

    self._got_header: bool = False
    self.header: int = 0

    self._got_code: bool = False
    self.code: Union[uFRcmd, uFRerr] = uFRcmd._UNDEFINED

    self._got_trailer: bool = False
    self.trailer: int = 0

    self._got_ext_len: bool = False
    self.ext_len: int = 0

    self._got_val0: bool = False
    self.val0: int = 0

    self._got_val1: bool = False
    self.val1: int = 0

    self._got_checksum: bool = False
    self.checksum: int = 0

    self._got_ext: bool = False
    self.ext: List[int] = []

    self._got_ext_checksum: bool = False
    self.ext_checksum: int = 0



  def __repr__(self: uFRanswer) \
		-> str:
    """Return a one-line human-readable description of the answer
    """

    desc: str = "ACK" if self.is_ack else "ERR" if self.is_err else "RSP"
    desc += "_EXT" if self.has_ext and not self.is_ack else ""
    desc += ", cmd=" if self.is_ack or self.is_rsp else ", err="
    desc += self.code.name if self.code is not None else "None"
    desc += ", ext_len={}".format(self.ext_len)
    if self.val0 is not None:
      desc += ", val0={:02x}h".format(self.val0)
    if self.val1 is not None:
      desc += ", val1={:02x}h".format(self.val1)
    if self.has_ext and self.ext is not None and not self.is_ack:
      desc += ", ext=("
      desc += ", ".join(["{:02x}h".format(v) for v in self.ext])
      desc += ")"
    return desc



class uFR:

  def __init__(self: uFR) \
		-> None:
    """__init__ method
    """

    ### Constants
    # Reverse lookup tables
    self.__UFR_HEADER_VALS: Tuple[int, ...]  = tuple(map(int, uFRhead))
    self.__UFR_CMD_VALS: Tuple[int, ...]     = tuple(map(int, uFRcmd))
    self.__UFR_ERR_VALS: Tuple[int, ...]     = tuple(map(int, uFRerr))
    self.__UFR_VAL_TO_CARD_TYPE: Dict[int, uFRcardtype] = \
			{ct.value: ct for ct in uFRcardtype}
    self.__UFR_VAL_TO_DL_CARD_TYPE: Dict[int, uFRdlcardtype] = \
			{dlct.value: dlct for dlct in uFRdlcardtype}
    self.__UFR_VAL_TO_EMU_MODE: Dict[int, uFRemumode] = \
			{em.value: em for em in uFRemumode}
    self.__UFR_VAL_TO_EMU_STATE: Dict[int, uFRemustate] = \
			{st.value: st for st in uFRemustate}
    self.__UFR_VAL_TO_PCD_MGR_STATE: Dict[int, uFRpcdmgrstate] = \
			{pmst.value: pmst for pmst in uFRpcdmgrstate}
    self.__UFR_VAL_TO_CMD: Dict[int, uFRcmd] = \
			{cmd.value: cmd for cmd in uFRcmd}
    self.__UFR_VAL_TO_ERR: Dict[int, uFRerr] = \
			{err.value: err for err in uFRerr}
    self.__UFR_VAL_TO_IOSTATE: Dict[int, uFRiostate] = \
			{iostate.value: iostate for iostate in uFRiostate}

    # Leave sleep mode parameters
    self.__WAKE_UP_BYTE: int                        = 0x00
    self.__WAKE_UP_WAIT: float                      = .01 #s




    self.serdev: Optional[Serial] = None

    self.udpsock: Optional[socket.socket] = None
    self.udphost: Optional[str] = None
    self.udpport: Optional[int] = None

    self.tcpsock: Optional[socket.socket] = None

    self.resturl: Optional[str] = None
    self.postdata: str = ""

    self.default_timeout: float = _default_ufr_timeout
    self.current_timeout: float = _default_ufr_timeout

    self.recbuf: list = []

    self.last_cmd: uFRcmd = uFRcmd._UNDEFINED

    self.answer = uFRanswer()



  def open(self: uFR,
		dev: str = _default_ufr_device,
		timeout: float = _default_ufr_timeout) \
		-> None:
    """Open a connection. The device format is one of:
    serial://<device file>:<baudrate>
    udp://<host>:<port>
    tcp://<host>:<port>
    http://<host>/uartX
    """

    # Find out the protocol and associated parameters
    proto: str
    p1: str
    p2: str
    m: Optional[List]

    m = re.findall("^(serial|udp|tcp)://(.+):([0-9]+)/*$", dev)
    if m:
      proto, p1, p2 = m[0]
    else:
      m = re.findall("^(http://.+/uart[12])/*$", dev)
      if m:
        proto = "http"
        p1 = m[0]
      else:
        proto = ""

    # Open the device
    if proto == "serial":
      self.serdev = Serial(p1, int(p2), timeout = timeout)

    elif proto == "udp":
      self.udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.udpsock.settimeout(timeout)
      self.udphost = socket.gethostbyname(p1)
      self.udpport = int(p2)

    elif proto == "tcp":
      self.tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.tcpsock.settimeout(timeout)
      self.tcpsock.connect((socket.gethostbyname(p1), int(p2)))

    elif proto == "http":
      self.resturl = p1

    else:
      raise ValueError("unknown uFR device {}".format(dev))
      return

    self.default_timeout = timeout
    self.current_timeout = timeout
    return



  def flush(self: uFR,
		timeout: Optional[float] = None) \
		-> None:
    """ Wait until the reception times out, to clear any buffered data
    """

    while self.serdev is not None or self.udpsock is not None or \
		self.tcpsock is not None:
      try:
        self._get_data(timeout)
      except TimeoutError:
        return
      except socket.timeout:
        return



  def _checksum(self: uFR,
		data: Union[List[int], Tuple[int, ...], bytes]) \
		-> int:
    """Calculate the checksum of a row of bytes
    """

    csum: int = 0
    b: int
    for b in data:
      csum ^= b
    return (csum + 0x07) % 256



  def _uid_bytes2str(self: uFR,
			bytesuid: Union[List[int], bytes]) \
			-> str:
    """Convert bytes or a list of integers into a human-readable UID
    """
    return ":".join(["{:02X}".format(b) for b in bytesuid])



  def _uid_str2bytes(self: uFR,
			struid: str) \
			-> bytes:
    """Convert a human-readable UID into bytes
    """
    return bytes([int(v, 16) for v in struid.split(":")])



  def _send_data(self: uFR,
			data: Union[List[int], bytes]) \
			-> None:
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



  def _get_data(self: uFR,
		timeout: Optional[float] = None) \
		-> bytes:
    """Receive data
    """

    data: bytes

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
      data = b""
      while not data:
        data, fromhostport = self.udpsock.recvfrom(1024)
        if fromhostport[0] != self.udphost:
          data = b""
        if not data and datetime.now().timestamp() >= timeout_tstamp:
          raise TimeoutError

    # Receive from a TCP host
    elif self.tcpsock is not None:
      if reset_timeout:
        self.tcpsock.settimeout(self.current_timeout)
      data = self.tcpsock.recv(1024)

    # Receive a POST reply from a HTTP server
    elif self.resturl is not None:
      resp: str = requests.post(self.resturl, data = self.postdata,
			timeout = self.current_timeout).text.rstrip("\r\n\0 ")
      if not re.match("^([0-9a-zA-Z][0-9a-zA-Z])+$", resp):
        if not resp:
          raise ValueError("empty HTTP POST response")
        else:
          raise ValueError("invalid HTTP POST response: {}".format(resp))
      data = bytes([int(resp[i:i+2], 16) for i in range(0, len(resp), 2)])
      self.postdata = ""

    return data



  def _send_cmd(self: uFR,
		cmd: uFRcmd,
		par0: int = 0,
		par1: int = 0,
		ext_len: int  = 0) \
		-> None:
    """Send a short command
    """

    packet: List[int] = [uFRhead.CMD_HEADER.value, cmd.value,
			uFRtrail.CMD_TRAILER.value, ext_len, par0, par1]
    packet.append(self._checksum(packet))
    self._send_data(packet)

    self.last_cmd = cmd



  def _send_ext(self: uFR,
		ext_parms: Union[List[int], bytes]) \
		-> None:
    """Sent extended command parameters
    """

    packet = list(ext_parms)
    packet.append(self._checksum(packet))
    self._send_data(packet)



  def _send_cmd_ext(self: uFR,
			cmd: uFRcmd,
			par0: int,
			par1: int,
			ext_parms: Union[List[int], bytes],
			timeout: Optional[float] = None) \
			-> None:
    """Send an extended command in two steps: first the short command, wait for
    an ACK, then send the extended command parameters
    """

    ext_len: int = len(ext_parms) + 1

    if not ext_len:
      self._send_cmd(cmd, par0, par1, 0)
      return

    self._send_cmd(cmd, par0, par1, ext_len)

    answer: uFRanswer = self._get_answer(timeout)

    if not answer.is_ack or answer.code != cmd.value:
      raise ValueError("expected ACK to {}, ext_len={}, "
			"par0={:02x}h, par1={:02x}h - got {}".format(
			cmd.name, ext_len, par0, par1, answer))

    self._send_ext(ext_parms)



  def _get_answer(self: uFR,
			timeout: Optional[float] = None) \
			-> uFRanswer:
    """Get an answer packet
    """

    self.answer.wipe()

    nb_ext_bytes_remaining: int = -1

    while True:

      # Read data if the receive buffer is empty
      if not self.recbuf:
        self.recbuf.extend(self._get_data(timeout))

      # Parse the receive buffer
      b: int = self.recbuf.pop(0)

      # Get header
      if not self.answer._got_header:
        if b in self.__UFR_HEADER_VALS:
          self.answer.header = b
          self.answer.is_ack = (b == uFRhead.ACK_HEADER)
          self.answer.is_err = (b == uFRhead.ERR_HEADER)
          self.answer.is_rsp = (b == uFRhead.RESPONSE_HEADER)
          self.answer._got_header = True
        continue

      # Get the code (either command or error)
      if not self.answer._got_code:
        if (self.answer.is_ack or self.answer.is_rsp) and \
		b in self.__UFR_CMD_VALS:
          self.answer.code = self.__UFR_VAL_TO_CMD[b]
          self.answer._got_code = True
        elif self.answer.is_err and b in self.__UFR_ERR_VALS:
          self.answer.code = self.__UFR_VAL_TO_ERR[b]
          self.answer._got_code = True
        else:
          self.answer.wipe()
        continue

      # Get the trailer
      if not self.answer._got_trailer:
        if (self.answer.header == uFRhead.ACK_HEADER and \
			b == uFRtrail.ACK_TRAILER) or \
		(self.answer.header == uFRhead.ERR_HEADER and \
			b == uFRtrail.ERR_TRAILER) or \
		(self.answer.header == uFRhead.RESPONSE_HEADER and \
			b == uFRtrail.RESPONSE_TRAILER):
          self.answer.trailer = b
          self.answer._got_trailer = True
        else:
          self.answer.wipe()
        continue

      # Get the length of the returned parameters
      if not self.answer._got_ext_len:
        if b == 0 or b >= 2:
          self.answer.ext_len = b
          self.answer.has_ext = (b != 0)
          self.answer._got_ext_len = True
        else:
          self.answer.wipe()
        continue

      # Get val0
      if not self.answer._got_val0:
        self.answer.val0 = b
        self.answer._got_val0 = True
        continue

      # Get val1
      if not self.answer._got_val1:
        self.answer.val1 = b
        self.answer._got_val1 = True
        continue

      # Get the checksum
      if not self.answer._got_checksum:
        if self._checksum((self.answer.header, self.answer.code.value,
		self.answer.trailer, self.answer.ext_len,
		self.answer.val0, self.answer.val1)) == b:
          self.answer.checksum = b
          self.answer._got_checksum = True

          # If the response is short, return it immediately
          if not self.answer.has_ext or self.answer.is_ack:
            return self.answer

        else:
          self.answer.wipe()
        continue

      # Get the extended packet
      if not self.answer._got_ext:

        # Get the first byte of the extended packet
        if nb_ext_bytes_remaining < 0:
          self.answer.ext = [b]
          nb_ext_bytes_remaining = self.answer.ext_len - 2
          continue

        # Get the rest of the extended packet
        elif nb_ext_bytes_remaining > 0:
          self.answer.ext.append(b)
          nb_ext_bytes_remaining -= 1
          continue

        else:
          self.answer._got_ext = True

      # Get the extended packet's checksum
      if self._checksum(self.answer.ext) == b:
        self.answer.ext_checksum = b
        self.answer._got_ext_checksum = True

        # Return the long answer
        return self.answer

      else:
        self.answer.wipe()



  def _get_cmd_ext_part_ack(self: uFR,
				timeout: Optional[float] = None) \
				-> bool:
    """Get a multipart CMD_EXT acknowledgment
    Return True if it's a part acknowledgment, False if it's the last part.
    If we get anything else, raise an exception.
    """

    # Read data if the receive buffer is empty
    if not self.recbuf:
      data = self._get_data(timeout)
      self.recbuf.extend(data)

    # Parse one byte
    b: int = self.recbuf.pop(0)

    # Did we get an ACK?
    if b == uFRcmdextpartack.ACK_PART:
      return True
    if b == uFRcmdextpartack.ACK_LAST_PART:
      return False

    # We got an expected byte
    raise ValueError("expected {} ({:02x}h) or {} ({:02x}h) - got {:02x}h".
			format(uFRcmdextpartack.ACK_PART.name,
			uFRcmdextpartack.ACK_PART.value,
			uFRcmdextpartack.ACK_LAST_PART.name,
			uFRcmdextpartack.ACK_LAST_PART.value,
			b))



  def _get_last_command_response(self: uFR,
				timeout: Optional[float] = None) \
				-> uFRanswer:
    """Get a responde to the last command sent. Throw an exception if the
    answer is unexpected
    """

    answer: uFRanswer = self._get_answer(timeout)
    if not answer.is_rsp or answer.code != self.last_cmd:
      raise ValueError("expected response to {} - got {}".format(
			self.last_cmd.name, answer))
    return answer



  def close(self: uFR) \
		-> None:
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
    self.postdata = ""

    self.default_timeout = _default_ufr_timeout
    self.current_timeout = _default_ufr_timeout

    self.recbuf = []

    self.last_cmd = uFRcmd._UNDEFINED

    self.answer.wipe()



  def __del__(self: uFR) \
		-> None:
    """__del__ method
    """

    self.close()



  # Front-end API functions - roughly 1:1 with the Digital Logic COM protocol
  # with a few additional convenience functions
  #
  # Incomplete - functions will be added as needed

  def is_host_nano_online(self: uFR,
				host: str,
				timeout: Optional[float] = None) \
				-> bool:
    """Try to contact a host to see if it's running a HTTP server serving a
    Nano online page
    """

    # Try to get the Nano Online's login page
    try:
      response = requests.get("http://" + host, timeout = \
			self.default_timeout if timeout is None else timeout)
      return response.status_code == 200 and \
		re.search("uFR login", response.text) is not None
    except:
      return False



  def _is_host_nano_online_threadpool_wrapper(self: uFR,
					ht: Tuple[str, Optional[float]]) \
					-> Tuple[str, bool]:
    """Wrapper to call is_host_nano_online() from a thread pool
    """
    return (ht[0], self.is_host_nano_online(ht[0], timeout = ht[1]))



  def probe_subnet_nano_onlines(self: uFR,
				netaddr: str,
				timeout: Optional[float] = None) \
				-> list:
    """Probe an entire subnet for Nano Onlines. Uses threads
    """

    ip_net: IPv4Network = ip_network(netaddr)

    t: ThreadPool = ThreadPool(processes = _subnet_probe_concurrent_connections)

    nano_online_ips = []
    for host, is_nano_online in t.map(
			self._is_host_nano_online_threadpool_wrapper,
			[(str(host), timeout) for host in ip_net.hosts()]):
      if is_nano_online:
        nano_online_ips.append(host)

    t.close()

    return nano_online_ips



  def get_reader_type(self: uFR,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's type
    """

    self._send_cmd(uFRcmd.GET_READER_TYPE)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24) \
		if isinstance(rsp.ext, list) else -1



  def get_reader_serial(self: uFR,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's serial number as an integer
    """

    self._send_cmd(uFRcmd.GET_READER_SERIAL)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24)



  def get_serial_number(self: uFR,
			timeout: Optional[float] = None) \
			-> str:
    """Get the reader's serial number as a string
    """

    self._send_cmd(uFRcmd.GET_SERIAL_NUMBER)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return bytes(rsp.ext).decode("ascii")



  def get_hardware_version(self: uFR,
				timeout: Optional[float] = None) \
				-> int:
    """Get the reader's hardware version
    """

    self._send_cmd(uFRcmd.GET_HARDWARE_VERSION)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return (rsp.val0 << 8) + rsp.val1



  def get_firmware_version(self: uFR,
				timeout: Optional[float] = None) \
				-> int:
    """Get the reader's firmware version
    """

    self._send_cmd(uFRcmd.GET_FIRMWARE_VERSION)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return (rsp.val0 << 8) + rsp.val1



  def get_build_number(self: uFR,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's firmware's build number
    """

    self._send_cmd(uFRcmd.GET_BUILD_NUMBER)
    return self._get_last_command_response(timeout).val0



  def get_card_id(self: uFR,
			timeout: Optional[float] = None) \
			-> Tuple[uFRcardtype, int]:
    """Get the card type and UID (4 bytes only)
    """

    self._send_cmd(uFRcmd.GET_CARD_ID)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardtype._NO_CARD, 0)
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardtype._UNDEFINED),
		(rsp.ext[0] << 24) + (rsp.ext[1] << 16) + \
		(rsp.ext[2] << 8) + rsp.ext[3])



  def get_card_id_ex(self: uFR,
			timeout: Optional[float] = None) \
			-> Tuple[uFRcardtype, str]:
    """Get the card type and UID (4, 7 or 10 bytes)
    """

    self._send_cmd(uFRcmd.GET_CARD_ID_EX)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardtype._NO_CARD, "")
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardtype._UNDEFINED),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_last_card_id_ex(self: uFR,
				timeout: Optional[float] = None) \
				-> Tuple[uFRcardtype, str]:
    """Get the last read card type and UID (4, 7 or 10 bytes)
    Return () if no card was last read
    """

    self._send_cmd(uFRcmd.GET_LAST_CARD_ID_EX)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardtype._NO_CARD, "")
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardtype._UNDEFINED),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_dlogic_card_type(self: uFR,
				timeout: Optional[float] = None) \
				-> uFRdlcardtype:
    """Get the Digital Logic card type
    """

    self._send_cmd(uFRcmd.GET_DLOGIC_CARD_TYPE)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return uFRdlcardtype._NO_CARD
      else:
        raise
    return self.__UFR_VAL_TO_DL_CARD_TYPE.get(rsp.val0,
						uFRdlcardtype._UNDEFINED)



  def linear_read(self: uFR,
			authmode: uFRauthmode,
			addr: int,
			length: int,
			key: Union[List[int], Tuple[int, ...], bytes, int] = 0,
			multiblock: bool = False,
			timeout: Optional[float] = None) \
			-> bytes:
    """Linear read from a card. Return b"" if no card was present in the field
    STATUS: partially tested
    """

    # Define the command parameters
    part1: int
    cmdext: List[int] = [addr & 0xff, addr >> 8]

    if authmode in (uFRauthmode.RKA_AUTH1A, uFRauthmode.RKA_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise ValueError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthmode.AKM1_AUTH1A, uFRauthmode.AKM1_AUTH1B,
		uFRauthmode.AKM2_AUTH1A, uFRauthmode.AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    elif authmode in (uFRauthmode.PK_AUTH1A, uFRauthmode.PK_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      if isinstance(key, list) or isinstance(key, tuple) or \
		isinstance(key, bytes):
        cmdext.extend(list(key))
      else:
        raise ValueError("key be a tuple, list or bytes")

    elif authmode in (uFRauthmode.SAM_KEY_AUTH1A, uFRauthmode.SAM_KEY_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise ValueError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthmode.PK_AUTH1A_AES, uFRauthmode.PK_AUTH1B_AES):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      if isinstance(key, list) or isinstance(key, tuple) or \
		isinstance(key, bytes):
        cmdext.extend(list(key))
      else:
        raise ValueError("key be a tuple, list or bytes")

    elif authmode in (uFRauthmode.MFP_RKA_AUTH1A, uFRauthmode.MFP_RKA_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise ValueError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthmode.MFP_AKM1_AUTH1A, uFRauthmode.MFP_AKM1_AUTH1B,
		uFRauthmode.MFP_AKM2_AUTH1A, uFRauthmode.MFP_AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    # Send the command and read back the data
    self._send_cmd_ext(uFRcmd.LINEAR_READ, par1, 0, cmdext, timeout)
    try:
      rsp = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return b""
      else:
        raise

    return bytes(rsp.ext)



  def get_rf_analog_settings(self: uFR,
				tag_comm_type: uFRtagcommtype,
				timeout: Optional[float] = None) \
				-> List[int]:
    """Get the RF frontend's analog settings
    """

    self._send_cmd(uFRcmd.GET_RF_ANALOG_SETTINGS, tag_comm_type.value)
    rsp = self._get_last_command_response(timeout)
    return rsp.ext



  def set_rf_analog_settings(self: uFR,
				tag_comm_type: uFRtagcommtype,
				factory_settings: bool,
				settings: List[int],
				timeout: Optional[float] = None) \
				-> None:
    """Set the RF frontend's analog settings
    """

    self._send_cmd_ext(uFRcmd.SET_RF_ANALOG_SETTINGS, tag_comm_type,
			1 if factory_settings else 0, settings, timeout)
    self._get_last_command_response(timeout)



  def set_led_config(self: uFR,
			blink: bool,
			timeout: Optional[float] = None) \
			-> None:
    """Set the green LED's configuration
    """

    self._send_cmd(uFRcmd.SET_LED_CONFIG, 1 if blink else 0)
    self._get_last_command_response(timeout)



  def enter_sleep_mode(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Send the reader to sleep
    """

    self._send_cmd(uFRcmd.ENTER_SLEEP_MODE)
    self._get_last_command_response(timeout)



  def leave_sleep_mode(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Wake up the reader
    """

    self._send_data([self.__WAKE_UP_BYTE])
    sleep(self.__WAKE_UP_WAIT)
    self._send_cmd(uFRcmd.LEAVE_SLEEP_MODE)
    self._get_last_command_response(timeout)
    sleep(_post_wake_up_wait)



  def rf_reset(self: uFR,
		timeout: Optional[float] = None) \
		-> None:
    """Reset the RF field
    """

    self._send_cmd(uFRcmd.RF_RESET)
    self._get_last_command_response(timeout)



  def check_uid_change(self: uFR,
			timeout: Optional[float] = None) \
			-> bool:
    """Return True if the card's UID is changeable (magic Mifare Classic gen2),
    False if it isn't of if no card is in the field
    NOTE: Does NOT test if the card responds to the magic Mifare command
          (magic Mifare Classic gen 1a), only if the UID is directly writeable
    """

    self._send_cmd(uFRcmd.CHECK_UID_CHANGE)
    try:
      self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.READING_ERROR:
        return False
      else:
        raise
    return True



  def get_reader_status(self: uFR,
			timeout: Optional[float] = None) \
			-> Tuple[uFRpcdmgrstate, uFRemumode, uFRemustate]:
    """Get the states of the reader
    """

    self._send_cmd(uFRcmd.GET_READER_STATUS)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return (self.__UFR_VAL_TO_PCD_MGR_STATE[rsp.ext[0]],
		self.__UFR_VAL_TO_EMU_MODE[rsp.ext[1]],
		self.__UFR_VAL_TO_EMU_STATE[rsp.ext[2]])



  def self_reset(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Soft-restart the reader
    """

    self._send_cmd(uFRcmd.SELF_RESET)
    self._get_last_command_response(timeout)
    sleep(_post_reset_wait)



  def write_emulation_ndef(self: uFR,
				ndef: bytes,
				in_ram: bool = False,
				timeout: Optional[float] = None) \
				-> None:
    """Write the emulation NDEF in EEPROM or in RAM. The NDEF in EEPROM survives
    resets but is 144 bytes long at the most. The NDEF in RAM can be 1008 bytes
    long but is wiped after a reset
    """

    i: int
    ndeflen: int = len(ndef)

    if timeout is None:
      timeout = _default_ufr_timeout

    if (not in_ram and ndeflen > 144) or (in_ram and ndeflen > 1008):
      raise ValueError("NDEF too long")

    # Split the NDEF into 240-byte-long parts
    ext_parts: List[bytes] = [ndef[i:i + 240] for i in range(0, ndeflen, 240)]
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
    self._send_cmd_ext(uFRcmd.WRITE_EMULATION_NDEF, 1 if in_ram else 0, 0,
			ext_parts.pop(0), timeout)

    # Wait for ACKs and send subsequent parts if we have more than one part
    if ext_parts:
      while self._get_cmd_ext_part_ack(timeout):
        if not ext_parts:
          raise ValueError("expected {} ({:02x}h) - got {} ({:02x}h) "
			"with no more CMD_EXT parts to send".format(
			uFRcmdextpartack.ACK_LAST_PART.name,
			uFRcmdextpartack.ACK_LAST_PART.value,
			uFRcmdextpartack.ACK_PART.name,
			uFRcmdextpartack.ACK_PART.value))
        self._send_data(ext_parts.pop(0))

    if ext_parts:
      raise ValueError("expected {} ({:02x}h) - got {} ({:02x}h) "
			"before sending the last CMD_EXT part".format(
			uFRcmdextpartack.ACK_PART.name,
			uFRcmdextpartack.ACK_PART.value,
			uFRcmdextpartack.ACK_LAST_PART.name,
			uFRcmdextpartack.ACK_LAST_PART.value))

    self._get_last_command_response(timeout)
    sleep(_post_write_emulation_ndef_wait)



  def tag_emulation_start(self: uFR,
				ram_ndef: bool = False,
				timeout: Optional[float] = None) \
				-> None:
    """Start tag emulation. Use either the NDEF in EEPROM or in RAM
    """

    self._send_cmd(uFRcmd.TAG_EMULATION_START, 1 if ram_ndef else 0)
    self._get_last_command_response(timeout)
    sleep(_post_emulation_start_stop_wait)



  def tag_emulation_stop(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Stop tag emulation
    """

    self._send_cmd(uFRcmd.TAG_EMULATION_STOP)
    self._get_last_command_response(timeout)
    sleep(_post_emulation_start_stop_wait)



  def ad_hoc_emulation_start(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Start ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(uFRcmd.AD_HOC_EMULATION_START)
    self._get_last_command_response(timeout)
    sleep(_post_emulation_start_stop_wait)



  def ad_hoc_emulation_stop(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Stop ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(uFRcmd.AD_HOC_EMULATION_STOP)
    self._get_last_command_response(timeout)
    sleep(_post_emulation_start_stop_wait)



  def get_external_field_state(self: uFR,
				timeout: Optional[float] = None) \
				-> bool:
    """Test the presence of an external RF field in ad-hoc (peer-to-peer) mode
    """

    self._send_cmd(uFRcmd.GET_EXTERNAL_FIELD_STATE)
    return self._get_last_command_response(timeout).val0 == 1



  def get_ad_hoc_emulation_params(self: uFR,
					timeout: Optional[float] = None) \
					-> Tuple[int, int]:
    """Get current ad-hoc (peer-to-peer) emulation parameters
    Return RxThreshold and RFCfg
    """

    self._send_cmd(uFRcmd.GET_AD_HOC_EMULATION_PARAMS)
    rsp: uFRanswer = self._get_last_command_response(timeout)
    return (rsp.val0, rsp.val1)



  def set_ad_hoc_emulation_params(self: uFR,
					rxthreshold: int,
					rfcfg: int,
					timeout: Optional[float] = None) \
					-> None:
    """Set current ad-hoc (peer-to-peer) emulation parameters
    """

    self._send_cmd(uFRcmd.SET_AD_HOC_EMULATION_PARAMS, rxthreshold,
							rfcfg & 0x7f)
    self._get_last_command_response(timeout)



  def red_light_control(self: uFR,
			state: bool,
			timeout: Optional[float] = None) \
			-> None:
    """Turn the red LED on or off
    """

    self._send_cmd(uFRcmd.RED_LIGHT_CONTROL, 1 if state else 0)
    self._get_last_command_response(timeout)



  def user_interface_signal(self: uFR,
				light_signal_mode: uFRlightsignal,
				beep_signal_mode: uFRbeepsignal,
				timeout: Optional[float] = None) \
				-> None:
    """Trigger a LED sequence or beep sequence
    """

    self._send_cmd(uFRcmd.USER_INTERFACE_SIGNAL, light_signal_mode.value,
			beep_signal_mode.value)
    self._get_last_command_response(timeout)



  def set_speaker_frequency(self: uFR,
				frequency: float,
				timeout: Optional[float] = None) \
				-> None:
    """Make the buzzer emit a continuous sound. Set frequency to 0 or None to
    stop the sound
    """

    period: int = ((round(65535 - 1500000 / (2 * frequency))) & 0xffff) \
		if frequency else 0xffff
    self._send_cmd(uFRcmd.SET_SPEAKER_FREQUENCY, period & 0xff, period >> 8)
    self._get_last_command_response(timeout)



  def set_iso14443_4_mode(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Set ISO14443-4 mode
    """

    self._send_cmd(uFRcmd.SET_ISO14443_4_MODE)
    self._get_last_command_response(timeout)



  def s_block_deselect(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Deselect tag and restore polling
    STATUS: untested
    """

    self._send_cmd(uFRcmd.S_BLOCK_DESELECT)
    self._get_last_command_response(timeout)



  def apdu_transceive(self: uFR,
			c_apdu: bytes,
			apdu_timeout_ms: Optional[int] = None,
			timeout: Optional[float] = None) \
			-> bytes:
    """Send a command APDU to the ISO14443-4 transponder and get the response
    APDU
    STATUS: untested
    """

    self._send_cmd_ext(uFRcmd.APDU_TRANSCEIVE, 0,
			round(self.default_timeout * 1000) \
			if apdu_timeout_ms is None else apdu_timeout_ms,
			c_apdu, timeout)
    return bytes(self._get_last_command_response(timeout).ext)



  def enable_anti_collision(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Enable anti-collision mode: leave single-card mode and stop polling
    """

    self._send_cmd(uFRcmd.ENABLE_ANTI_COLLISION)
    self._get_last_command_response(timeout)



  def disable_anti_collision(self: uFR,
				timeout: Optional[float] = None) \
				-> None:
    """Disable anti-collision mode; return to single-card mode and start polling
    """

    self._send_cmd(uFRcmd.DISABLE_ANTI_COLLISION)
    self._get_last_command_response(timeout)



  def enum_cards(self: uFR,
			timeout: Optional[float] = None) \
			-> bool:
    """Enumerate cards in the field in anti-collision mode
    Return True if at least one card was enumerated, False if no card was found
    """

    self._send_cmd(uFRcmd.ENUM_CARDS)
    try:
      self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARDS_ENUMERATED:
        return False
      else:
        raise
    return True



  def list_cards(self: uFR,
			timeout: Optional[float] = None) \
			-> List[str]:
    """List cards previously enumerated by enum_cards()
    Return a list of UIDs or an empty list if no card was enumerated
    """

    self._send_cmd(uFRcmd.LIST_CARDS)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout)
    except:
      if self.answer.code == uFRerr.NO_CARDS_ENUMERATED:
        return []
      else:
        raise
    return [self._uid_bytes2str(rsp.ext[i + 1 : i + rsp.ext[i] + 1]) \
		for i in range(0, len(rsp.ext), 11)]



  def select_card(self: uFR,
			uid: str,
			timeout: Optional[float] = None) \
			-> uFRdlcardtype:
    """Select a card in the field in anti-collision mode
    """

    bytesuid: bytes = self._uid_str2bytes(uid)
    self._send_cmd_ext(uFRcmd.SELECT_CARD, len(bytesuid), 0, bytesuid, timeout)
    return self.__UFR_VAL_TO_DL_CARD_TYPE[
		self._get_last_command_response(timeout).val0]



  def deselect_card(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Deselect the currently selected card in anti-collision mode
    """

    self._send_cmd(uFRcmd.DESELECT_CARD)
    self._get_last_command_response(timeout)



  def get_anti_collision_status(self: uFR,
				timeout: Optional[float] = None) \
				-> Tuple[bool, bool]:
    """Return the status of the anti-collision mode, and whether a card is
    currently selected
    """

    self._send_cmd(uFRcmd.GET_ANTI_COLLISION_STATUS)
    rsp = self._get_last_command_response(timeout)
    return (rsp.val0 != 0, rsp.val1 != 0)



  def esp_set_io_state(self: uFR,
			pin: int,
			state: uFRiostate,
			timeout: Optional[float] = None) \
			-> None:
    """Set the state of one of the 6 ESP I/O pins
    """

    self._send_cmd(uFRcmd.ESP_SET_IO_STATE, pin, state.value)
    self._get_last_command_response(timeout)



  def esp_get_io_state(self: uFR,
			timeout: Optional[float] = None) \
			-> List[uFRiostate]:
    """Get the states of the 6 ESP I/O pins
    return the states as a list
    """

    self._send_cmd(uFRcmd.ESP_GET_IO_STATE)
    return [self.__UFR_VAL_TO_IOSTATE[st]
		for st in self._get_last_command_response(timeout).ext]



  def esp_set_display_data(self: uFR,
				rgb1: Union[List[int], Tuple[int, int, int]],
				rgb2: Union[List[int], Tuple[int, int, int]],
				duration_ms: int,
				timeout: Optional[float] = None) \
				-> None:
    """Set the color of the two ESP LEDs for a certain duration in ms. Set the
    duration to 0 to keep those colors permanently. Set a short duration to
    return to reader-managed colors
    WARNING: don't set two non-zero timeouts in a row, or the reader will go
             unresponsive when the first delay elapses. If you want to
             play a color sequence, only the last command should have a
             non-zero timeout, and you should manage the sequence's delays
             yourself
    """

    self._send_cmd_ext(uFRcmd.ESP_SET_DISPLAY_DATA, duration_ms & 0xff,
			duration_ms >> 8,list(rgb1) + list(rgb2), timeout)
    self._get_last_command_response(timeout)



  def esp_reader_reset(self: uFR,
			timeout: Optional[float] = None) \
			-> None:
    """Ask the ESP to reset the reader
    """

    self._send_cmd(uFRcmd.ESP_READER_RESET, 0)
    self._get_last_command_response(timeout)
    sleep(_post_reset_wait)



### Routines
def __print_api_state_of_completion(ufr: uFR) \
			-> None:
  """Print the API's state of completion - i.e. which Digital Logic COM
  protocol functions are implemented in this class, which aren't, which are
  untested or partially tested, and what percentage of the COM protocol is
  implemented
  """

  comcmds: List[str] = [cmd.name for cmd in uFRcmd]
  pubclassfcts: List[str] = [attribute for attribute in dir(ufr) \
			if callable(getattr(ufr, attribute)) and \
			not attribute.startswith('_')]

  cmd_impl_status: Dict[str, str] = {}
  nb_impl: int = 0
  max_cmd_name_len: int = 0
  max_status_len: int = 0
  cmd: str
  localparms: Dict[str, Any]
  m: Optional[List]

  # Get the status of all the commands in the COM protocol from whether the
  # corresponding functions exists in lowercase in the class and from status
  # markers in their docstrings
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
  padded: Callable = lambda s: \
		("{" + ":<{}".format(max_cmd_name_len) + "}").format(s)
  separator: str = "-" * (max_cmd_name_len + max_status_len)

  print(padded("UFR COM protocol command"), "Status")
  print(separator)
  for cmd in comcmds:
    print(padded(cmd), cmd_impl_status[cmd])

  print(separator)
  print(padded("State of completion:"),
		"{}%".format(round(100 * nb_impl / len(uFRcmd))))

  # Dump the list of API functions that are specific to this class
  print()
  print("Class-specific API functions")
  print(separator)
  for cmd in sorted(pubclassfcts, key = lambda s: len(s)):
    if cmd.upper() not in comcmds:
      print("{}()".format(cmd))



def __test_api(ufr: uFR) \
		-> None:
  """Test the API
  """

  i: int

  # Pretty line formatting
  padded: Callable = lambda s: "{:<30}".format(s)

  # Network probing functions - the device doesn't need to be open for this
  if __test_network_probe_functions:

    sys.stdout.write(padded("PROBE_SUBNET_NANO_ONLINES..."))
    sys.stdout.flush()
    nos: List[str] = ufr.probe_subnet_nano_onlines("192.168.1.0/24")
    print("\r" + padded("PROBE_SUBNET_NANO_ONLINES:"), nos)
    no: str
    for no in nos:
      print(padded("IS_HOST_NANO_ONLINE:"), ufr.is_host_nano_online(no))

  # Open the device
  ufr.open(args.device)

  # Reader information functions
  if __test_reader_info_functions:

    print(padded("GET_READER_TYPE:"), hex(ufr.get_reader_type()))
    print(padded("GET_READER_SERIAL:"), ufr.get_reader_serial())
    print(padded("GET_SERIAL_NUMBER:"), ufr.get_serial_number())
    print(padded("GET_HARDWARE_VERSION:"), hex(ufr.get_hardware_version()))
    print(padded("GET_FIRMWARE_VERSION:"), hex(ufr.get_firmware_version()))
    print(padded("GET_BUILD_NUMBER:"), hex(ufr.get_build_number()))
    print(padded("GET_READER_STATUS:"), ufr.get_reader_status())

  # Ad-hoc (peer-to-peer) functions
  if __test_ad_hoc_functions:

    print("AD_HOC_EMULATION_START")
    ufr.ad_hoc_emulation_start()
    rxthreshold: int
    rfcfg: int
    rxthreshold, rfcfg = ufr.get_ad_hoc_emulation_params()
    print(padded("GET_AD_HOC_EMULATION_PARAMS:"),
		(hex(rxthreshold), hex(rfcfg)))
    print("SET_AD_HOC_EMULATION_PARAMS:")
    ufr.set_ad_hoc_emulation_params(rxthreshold, rfcfg)
    print(padded("GET_READER_STATUS:"), ufr.get_reader_status())
    print(padded("GET_EXTERNAL_FIELD_STATE"), ufr.get_external_field_state())
    print("AD_HOC_EMULATION_STOP")
    ufr.ad_hoc_emulation_stop()

  # RF analog settings functions
  if __test_rf_analog_settings_functions:

    tct: uFRtagcommtype
    for tct in uFRtagcommtype:
      print(padded("GET_RF_ANALOG_SETTINGS:"), ufr.get_rf_analog_settings(tct))

      if __test_eeprom_writing_functions:

        new_settings: List[int] = list(ufr.answer.ext)
        new_settings[PN53xanalogsettingsreg.RXTHRESHOLD] = 255
        print("SET_RF_ANALOG_SETTINGS")
        ufr.set_rf_analog_settings(tct, False, new_settings)

  # Reset functions
  if __test_reset_functions:

    print("RF_RESET")
    ufr.rf_reset()
    print("SELF_RESET")
    ufr.self_reset()

    # Only test the ESP reset function if the device is a Nano Online connected
    # through the network, but not in HTTP transparent mode, as transparent
    # mode bypasses the the ESP and sends the commands directly to the UART
    if ufr.udpsock is not None or ufr.tcpsock is not None:

      print("ESP_READER_RESET")
      ufr.esp_reader_reset()

  # Sleep functions - only works if the device is connected directly to a
  # a serial port
  if __test_sleep_functions and ufr.serdev is not None:

    print("ENTER_SLEEP_MODE")
    ufr.enter_sleep_mode()
    print("LEAVE_SLEEP_MODE")
    ufr.leave_sleep_mode()

  # LED and buzzer functions
  if __test_led_sound_functions:

    if __test_eeprom_writing_functions:

      print("SET_LED_CONFIG")
      ufr.set_led_config(True)

    print("RED_LIGHT_CONTROL")
    ufr.red_light_control(True)
    sleep(2)
    ufr.red_light_control(False)

    print("SET_SPEAKER_FREQUENCY")
    freq: float = 1480 / 4
    for i in range(3):
      ufr.set_speaker_frequency(freq)
      freq *= 2
      sleep(.1)
    ufr.set_speaker_frequency(0)

    print("USER_INTERFACE_SIGNAL")
    ufr.user_interface_signal(uFRlightsignal.ALTERNATION, uFRbeepsignal.SHORT)

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
  if __test_esp_io and (ufr.udpsock is not None or ufr.tcpsock is not None):

    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, uFRiostate.HIGH)
    print(padded("ESP_GET_IO_STATE"), ufr.esp_get_io_state())
    print("ESP_SET_IO_STATE")
    ufr.esp_set_io_state(6, uFRiostate.LOW)
    print(padded("ESP_GET_IO_STATE"), ufr.esp_get_io_state())

  # UID functions
  if __test_uid_functions:

      cid: Tuple[uFRcardtype, int] = ufr.get_card_id()
      print(padded("GET_CARD_ID:"),
		(cid[0], "0x{:08X}".format(cid[1])) if cid else ())
      print(padded("GET_CARD_ID_EX:"), ufr.get_card_id_ex())
      print(padded("GET_LAST_CARD_ID_EX:"), ufr.get_last_card_id_ex())
      print(padded("GET_DLOGIC_CARD_TYPE:"), ufr.get_dlogic_card_type())
      print(padded("CHECK_UID_CHANGE:"), ufr.check_uid_change())

  # Test read functions
  if __test_read_functions:
      print(padded("LINEAR_READ:"), ufr.linear_read(uFRauthmode.T2T_NO_PWD_AUTH,
							0, 10))

  # Get ISO14443-4 functions
  if __test_iso14443_4_functions:

    print("SET_ISO_14443_4_MODE")
    try:
      ufr.set_iso14443_4_mode()
    except:
      if ufr.answer.code != uFRerr.NO_CARD:
        raise

  # Anti-collision functions
  if __test_anti_collision_functions:

    print("SELF_RESET")
    ufr.self_reset()
    print("ENABLE_ANTI_COLLISION")
    ufr.enable_anti_collision()
    print(padded("GET_ANTI_COLLISION_STATUS:"), ufr.get_anti_collision_status())
    print(padded("ENUM_CARDS:"), ufr.enum_cards())
    uids: List[str] = ufr.list_cards()
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
  if __test_tag_emulation:

    eeprom_ndef: bytes = b"\x03\x10\xd1\x01\x0cU\x01d-logic.net\xfe"
    ram_ndef: bytes = b"\x03\xff\x03\xeb\xc1\x01\x00\x00\x03\xe4T\x02en3456" + \
			b"7890123456" * 99

    if __test_eeprom_writing_functions:
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
  argparser: argparse.ArgumentParser = argparse.ArgumentParser()
  argparser.add_argument(
	  "-d", "--device",
	  help = "uFR device to test (default {})".format(_default_ufr_device),
	  type = str,
	  default = _default_ufr_device
	)
  argparser.add_argument(
	  "-soc", "--state-of-completion",
	  help = "Print the API's state of completion",
	  action = "store_true"
	)
  args: argparse.Namespace = argparser.parse_args()

  # Create the ufr object
  ufr: uFR = uFR()

  # Dump the API's state of completion
  if args.state_of_completion:
    __print_api_state_of_completion(ufr)

  # Test the API
  else:
    __test_api(ufr)

  # Close the device
  ufr.close()

  # Delete the ufr object
  del(ufr)
