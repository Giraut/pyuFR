#!/usr/bin/python3
"""Pure Python class to communicate with uFR series readers
"""

from __future__ import annotations

### Parameters
_default_ufr_timeout = 1 #s

# Extra delays following certain commands, that aren't prescribed in the COM
# protocol, but that are apparently needed to prevent the reader from going
# unresponsive after the command
_post_wake_up_wait: float              = .1 #s
_post_reset_wait: float                = .1 #s
_post_write_emulation_ndef_wait: float = .1 #s
_post_emulation_start_stop_wait: float = .1 #s

# Number of concurrent connection when scanning a subnet for Nano Onlines
_subnet_probe_concurrent_connections: int = 100

# API tests
__test_network_probe_functions      = True
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
__test_asynchronous_card_id_sending = True



### Modules
from typing import Any, Type, List, Tuple, Dict, Callable, Generator, \
			Union, Optional
from types import TracebackType
import re
import socket
import requests
from time import sleep
from enum import IntEnum
from datetime import datetime

# Try to import optional modules but fail silently if they're not needed later
try:
  import serial							# type: ignore
except:
  pass
try:
  import websocket						# type: ignore
except:
  pass
try:
  import ipaddress
except:
  pass



### Enums
class uFRhead(IntEnum):
  CMD_HEADER: int                              = 0x55
  ACK_HEADER: int                              = 0xac
  RESPONSE_HEADER: int                         = 0xde
  ERR_HEADER: int                              = 0xec

class uFRcmdExtPartAck(IntEnum):
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

class uFRcardType(IntEnum):	# Partially documented - may be wrong/incomplete
  _NO_CARD: int                                = -2
  _UNDEFINED: int                              = -1
  GENERIC: int                                 = 0x00	# Undocumented
  MIFARE_CLASSIC_1K: int                       = 0x08
  MIFARE_CLASSIC_4K: int                       = 0x18
  MIFARE_MINI: int                             = 0x09
  CONTACTLESS_EMV: int                         = 0x0b	# Undocumented
  MIFARE_DESFIRE: int                          = 0x20	# Undocumented

class uFRDLCardType(IntEnum):
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

class uFRauthMode(IntEnum):
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

class uFRtagCommType(IntEnum):
  ISO14443_TYPE_A: int                         = 0X01
  ISO14443_TYPE_B: int                         = 0X02
  ISO14443_4_212_KBPS: int                     = 0X03
  ISO14443_4_424_KBPS: int                     = 0X04

class PN53xAnalogSettingsReg(IntEnum):
  RFCFG: int                                   = 0
  RXTHRESHOLD: int                             = 1
  GSNON: int                                   = 2
  CWGSP: int                                   = 3
  GSNOFF: int                                  = 4
  MODGSP: int                                  = 4

class uFRlightSignal(IntEnum):
  NONE: int                                    = 0
  LONG_GREEN: int                              = 1
  LONG_RED: int                                = 2
  ALTERNATION: int                             = 3
  FLASH: int                                   = 4

class uFRbeepSignal(IntEnum):
  NONE: int                                    = 0
  SHORT: int                                   = 1
  LONG: int                                    = 2
  DOUBLE_SHORT: int                            = 3
  TRIPLE_SHORT: int                            = 4
  TRIPLET_MELODY: int                          = 5

class uFRIOState(IntEnum):
  LOW: int                                     = 0
  HIGH: int                                    = 1
  INPUT: int                                   = 2

class uFRemuMode(IntEnum):
  TAG_EMU_DISABLED: int                        = 0
  TAG_EMU_DEDICATED: int                       = 1
  TAG_EMU_COMBINED: int                        = 2
  TAG_EMU_AUTO_AD_HOC: int                     = 3

class uFRemuState(IntEnum):
  EMULATION_NONE: int                          = 0
  EMULATION_IDLE: int                          = 1
  EMULATION_AUTO_COLL: int                     = 2
  EMULATION_ACTIVE: int                        = 3
  EMULATION_HALT: int                          = 4
  EMULATION_POWER_OFF: int                     = 5

class uFRPCDMgrState(IntEnum):
  PCD_MGR_NO_RF_GENERATED: int                 = 0
  PCD_MGR_14443A_POLLING: int                  = 1
  PCD_MGR_14443A_SELECTED: int                 = 2
  PCD_MGR_CE_DEDICATED: int                    = 3
  PCD_MGR_CE_COMBO_START: int                  = 4
  PCD_MGR_CE_COMBO: int                        = 5
  PCD_MGR_CE_COMBO_IN_FIELD: int               = 6



### Classes
class uFRanswer:
  """uFR command answer
  """

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
    self.is_async_id: bool = False

    self._got_async_id_prefix = False
    self.async_id: str = ""

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

    self.ext_checksum: int = 0



  def __repr__(self: uFRanswer) \
		-> str:
    """Return a one-line human-readable description of the answer
    """

    desc: str = "ACK" if self.is_ack else "ERR" if self.is_err else \
		"RSP" if self.is_rsp else "ASYNC_ID" if self.is_async_id else \
		"INVALID"

    if self.is_ack or self.is_err or self.is_rsp:

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

    elif self.is_async_id:

      desc += ", async_id=" + self.async_id

    return desc



class uFRresponseError(Exception):
  """Exception raised when receiving a malformed or unexpected response from
     a uFR device
  """

  def __init__(self: uFRresponseError,
		message: str = "") \
		-> None:
    """__init__ method
    """
    self.message = message
    super().__init__(self.message)



class uFRopenError(Exception):
  """Exception raised when an error occurs when opening a uFR device
  """

  def __init__(self: uFRopenError,
		message: str = "") \
		-> None:
    """__init__ method
    """
    self.message = message
    super().__init__(self.message)



class uFRIOError(Exception):
  """Exception raised when an error occurs doing an invalid I/O operation
  """

  def __init__(self: uFRIOError,
		message: str = "") \
		-> None:
    """__init__ method
    """
    self.message = message
    super().__init__(self.message)



class uFRdiscoveryResponse:
  """uFR Nano Online UDP discovery server response
  """

  class uart:
    port: int = 0
    is_udp: bool = False
    baudrate: int = 0

  # Variables
  ip: str = ""
  uart1: uart = uart()
  uart2: uart = uart()
  serial: Optional[str] = None



  def __init__(self: uFRdiscoveryResponse,
		dgram: bytes) \
		-> None:
    """__init__ method
    Extract values from a UDP discovery response datagram
    """

    if len(dgram) < 18 or dgram[6] not in (b"TU") or dgram[13] not in (b"TU"):
      raise ValueError("invalid UDP discovery datagram")

    self.ip = ".".join([str(b) for b in dgram[:4]])
    self.uart1.port = dgram[4] + (dgram[5] << 8)
    self.uart1.is_udp = (dgram[6] == ord("U"))
    self.uart1.baudrate = dgram[7] + (dgram[8] << 8) + \
				(dgram[9] << 16) + (dgram[10] << 24)
    self.uart2.port = dgram[11] + (dgram[12] << 8)
    self.uart2.is_udp = (dgram[13] == ord("U"))
    self.uart2.baudrate = dgram[14] + (dgram[15] << 8) + \
				(dgram[16] << 16) + (dgram[17] << 24)
    if len(dgram) > 20 and dgram[18] == 0x0b and dgram[-1] == 0:
      self.serial = dgram[19:-1].decode("ascii")



  def __repr__(self: uFRdiscoveryResponse) \
		-> str:
    """Return a one-line human-readable description of the discovery response
    """

    return "ip={}, uart1.port={}, uart1.is_udp={}, uart1.baudrate={}, " \
		"uart2.port={}, uart2.is_udp={}, uart2.baudrate={}, " \
		"serial={}".format(self.ip,
		self.uart1.port, self.uart1.is_udp, self.uart1.baudrate,
		self.uart2.port, self.uart2.is_udp, self.uart2.baudrate,
		self.serial)



class uFRcomm:

  def __init__(self: uFRcomm,
		dev: str,
		restore_on_close: bool = False,
		timeout: float = _default_ufr_timeout) \
		-> None:
    """__init__
    Open a connection. The device format is one of:

    serial://<device file>:<baudrate>
    udp://<host>:<port>
    tcp://<host>:<port>
    ws://<host>:<port>
    http://<host>/uartX

    examples:

    serial:///dev/ttyUSB0:1000000		# Nano USB serial
    udp://192.168.1.123:8881			# Nano Online slave UDP
    tcp://192.168.1.123:8881			# Nano Online slave TCP
    ws://192.168.1.123:8881			# Nano Online slave websocket
    http://ufr.localnet.org/uart1"		# Nano Online REST UART1
    """

    ### Constants
    # Reverse lookup tables
    self.__UFR_HEADER_VALS: Tuple[int, ...] = tuple(map(int, uFRhead))
    self.__UFR_CMD_VALS: Tuple[int, ...] = tuple(map(int, uFRcmd))
    self.__UFR_ERR_VALS: Tuple[int, ...] = tuple(map(int, uFRerr))
    self.__UFR_VAL_TO_CARD_TYPE: Dict[int, uFRcardType] = \
			{ct.value: ct for ct in uFRcardType}
    self.__UFR_VAL_TO_DL_CARD_TYPE: Dict[int, uFRDLCardType] = \
			{dlct.value: dlct for dlct in uFRDLCardType}
    self.__UFR_VAL_TO_EMU_MODE: Dict[int, uFRemuMode] = \
			{em.value: em for em in uFRemuMode}
    self.__UFR_VAL_TO_EMU_STATE: Dict[int, uFRemuState] = \
			{st.value: st for st in uFRemuState}
    self.__UFR_VAL_TO_PCD_MGR_STATE: Dict[int, uFRPCDMgrState] = \
			{pmst.value: pmst for pmst in uFRPCDMgrState}
    self.__UFR_VAL_TO_CMD: Dict[int, uFRcmd] = \
			{cmd.value: cmd for cmd in uFRcmd}
    self.__UFR_VAL_TO_ERR: Dict[int, uFRerr] = \
			{err.value: err for err in uFRerr}
    self.__UFR_VAL_TO_IOSTATE: Dict[int, uFRIOState] = \
			{iostate.value: iostate for iostate in uFRIOState}

    # Leave sleep mode parameters
    self.__WAKE_UP_BYTE: int = 0x00
    self.__WAKE_UP_WAIT: float = .01 #s

    # Default asynchronous ID sending prefix and suffix
    self.__DEFAULT_ASYNC_ID_PREFIX: int = 0xcc
    self.__DEFAULT_ASYNC_ID_SUFFIX: int = 0xee

    ### Variables
    self.serdev: Optional[serial.Serial] = None

    self.udpsock: Optional[socket.socket] = None
    self._udphost: Optional[str] = None
    self._udpport: Optional[int] = None

    self.tcpsock: Optional[socket.socket] = None

    self.websock: Optional[websocket] = None

    self.resturl: Optional[str] = None
    self.__postdata: str = ""

    self._default_timeout: float = timeout
    self._current_timeout: float = timeout

    self.__recbuf: list = []

    self._last_cmd: uFRcmd = uFRcmd._UNDEFINED

    self.__async_id_enabled: bool = False
    self.__async_id_prefix: int = 0x00
    self.__async_id_suffix: int = 0x00

    self.answer = uFRanswer()

    self.__saved_pcd_mgr_state: Optional[uFRPCDMgrState] = None
    self.__saved_emu_mode: Optional[uFRemuMode] = None
    self.__saved_anti_collision_enabled: Optional[bool] = None
    self.__saved_async_flags: Optional[int] = None
    self.__saved_async_prefix: Optional[int] = None
    self.__saved_async_suffix: Optional[int] = None
    self.__saved_async_baudrate: Optional[int] = None

    # Find out the protocol and associated parameters
    proto: str
    p1: str
    p2: str
    m: Optional[List]

    m = re.findall("^(serial|udp|tcp|ws)://(.+):([0-9]+)/*$", dev)
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
      self.serdev = serial.Serial(p1, int(p2), timeout = timeout)

    elif proto == "udp":
      self.udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.udpsock.settimeout(timeout)
      self._udphost = socket.gethostbyname(p1)
      self._udpport = int(p2)

    elif proto == "tcp":
      self.tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.tcpsock.settimeout(timeout)
      self.tcpsock.connect((socket.gethostbyname(p1), int(p2)))

    elif proto == "ws":
      self.websock = websocket.create_connection("ws://{}:{}".format(p1, p2))
      self.websock.settimeout(timeout)

    elif proto == "http":
      self.resturl = p1

    else:
      raise uFRopenError("unknown uFR device {}".format(dev))

    self._default_timeout = timeout
    self._current_timeout = timeout

    # Get the current state of the reader if needed
    if restore_on_close:
      if self.get_reader_status(save_status = True, timeout = timeout)[1] == \
		uFRemuMode.TAG_EMU_DISABLED:
        self.get_anti_collision_status(save_status = True, timeout = timeout)
        self.get_card_id_send_conf(save_status = True, timeout = timeout)



  def flush(self: uFRcomm,
		timeout: Optional[float] = None) \
		-> None:
    """ Wait until the reception times out, to clear any buffered data
    """

    while self.serdev is not None or self.udpsock is not None or \
		self.tcpsock is not None or self.websock is not None:
      try:
        self._get_data(timeout = timeout)
      except TimeoutError:
        return
      except socket.timeout:
        return



  def _checksum(self: uFRcomm,
		data: Union[List[int], Tuple[int, ...], bytes]) \
		-> int:
    """Calculate the checksum of a row of bytes
    """

    csum: int = 0
    b: int
    for b in data:
      csum ^= b
    return (csum + 0x07) % 256



  def _uid_bytes2str(self: uFRcomm,
			bytesuid: Union[List[int], bytes]) \
			-> str:
    """Convert bytes or a list of integers into a human-readable UID
    """

    return ":".join(["{:02X}".format(b) for b in bytesuid])



  def _uid_str2bytes(self: uFRcomm,
			struid: str) \
			-> bytes:
    """Convert a human-readable UID into bytes
    """

    return bytes([int(v, 16) for v in struid.split(":")])



  def _send_data(self: uFRcomm,
			data: Union[List[int], bytes]) \
			-> None:
    """Send a data packet
    """

    # Throw an exception if a device is not open
    if self.serdev is None and self.udpsock is None and self.tcpsock is None \
		and self.websock is None and self.resturl is None:
      raise uFRIOError("device not open")

    # Send to a serial device
    if self.serdev is not None:
      self.serdev.write(data)
      self.serdev.flush()

    # Send to a UDP host
    elif self.udpsock is not None:
      self.udpsock.sendto(bytes(data), (self._udphost, self._udpport))

    # Send to a TCP host
    elif self.tcpsock is not None:
      self.tcpsock.sendall(bytes(data))

    # Send to a websocket host
    elif self.websock is not None:
      self.websock.send_binary(bytes(data))

    # "Send" to a HTTP server
    elif self.resturl is not None:
      self.__postdata = "".join(["{:02X}".format(b) for b in data])



  def _get_data(self: uFRcomm,
		timeout: Optional[float] = None) \
		-> bytes:
    """Receive data
    """

    # Throw an exception if a device is not open
    if self.serdev is None and self.udpsock is None and self.tcpsock is None \
		and self.websock is None and self.resturl is None:
      raise uFRIOError("device not open")

    data: bytes
    ip: str
    reset_timeout: bool
    timeout_tstamp: float

    # Change the timeout as needed
    if timeout is None:
      reset_timeout = (self._current_timeout != self._default_timeout)
      self._current_timeout = self._default_timeout
    else:
      reset_timeout = (self._current_timeout != timeout)
      self._current_timeout = timeout

    # Receive from a serial device
    if self.serdev is not None:
      if reset_timeout:
        self.serdev.timeout = self._current_timeout
      data = self.serdev.read(1)
      if not data:
        raise TimeoutError

    # Receive from a UDP host
    elif self.udpsock is not None:
      if reset_timeout:
        self.udpsock.settimeout(self._current_timeout)
      timeout_tstamp = datetime.now().timestamp() + self._current_timeout
      data = b""
      while not data:
        try:
          data, (ip, _) = self.udpsock.recvfrom(1024)
        except socket.timeout:
          raise TimeoutError
        if ip != self._udphost:
          data = b""
        if not data and datetime.now().timestamp() >= timeout_tstamp:
          raise TimeoutError

    # Receive from a TCP host
    elif self.tcpsock is not None:
      if reset_timeout:
        self.tcpsock.settimeout(self._current_timeout)
      try:
        data = self.tcpsock.recv(1024)
      except socket.timeout:
        raise TimeoutError

    # Receive from a websocket host
    elif self.websock is not None:
      if reset_timeout:
        self.websock.settimeout(self._current_timeout)
      try:
        wsframe: websocket.ABNF = self.websock.recv_frame()
      except websocket._exceptions.WebSocketTimeoutException as e:
        if str(e)== "timed out":
          raise TimeoutError
        raise
      if wsframe.opcode != websocket.ABNF.OPCODE_BINARY:
        raise uFRresponseError("websocket response is not binary")
      data = wsframe.data

    # Receive a POST reply from a HTTP server
    elif self.resturl is not None:
      try:
        resp: str = requests.post(self.resturl, data = self.__postdata,
			timeout = self._current_timeout).text.rstrip("\r\n\0 ")
      except requests.exceptions.ConnectTimeout:
        raise TimeoutError
      if not re.match("^([0-9a-zA-Z][0-9a-zA-Z])+$", resp):
        if not resp:
          raise uFRresponseError("empty HTTP POST response")
        else:
          raise uFRresponseError("invalid HTTP POST response: {}".format(resp))
      data = bytes([int(resp[i:i+2], 16) for i in range(0, len(resp), 2)])
      self.__postdata = ""

    return data



  def _send_cmd(self: uFRcomm,
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

    self._last_cmd = cmd



  def _send_ext(self: uFRcomm,
		ext_parms: Union[List[int], bytes]) \
		-> None:
    """Sent extended command parameters
    """

    packet = list(ext_parms)
    packet.append(self._checksum(packet))
    self._send_data(packet)



  def _send_cmd_ext(self: uFRcomm,
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

    answer: uFRanswer = self._get_answer(timeout = timeout)

    if not answer.is_ack or answer.code != cmd.value:
      raise uFRresponseError("expected ACK to {}, ext_len={}, "
			"par0={:02x}h, par1={:02x}h - got {}".format(
			cmd.name, ext_len, par0, par1, answer))

    self._send_ext(ext_parms)



  def _get_answer(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> uFRanswer:
    """Get an answer packet
    """

    self.answer.wipe()

    nb_ext_bytes_remaining: int = -1

    while True:

      # Read data if the receive buffer is empty
      if not self.__recbuf:
        self.__recbuf.extend(self._get_data(timeout = timeout))

      # Parse the receive buffer
      b: int = self.__recbuf.pop(0)

      # Get header, or the asynchronous ID sending prefix if it's enabled
      if not self.answer._got_header and not self.answer._got_async_id_prefix:
        if b in self.__UFR_HEADER_VALS:
          self.answer.header = b
          self.answer.is_ack = (b == uFRhead.ACK_HEADER)
          self.answer.is_err = (b == uFRhead.ERR_HEADER)
          self.answer.is_rsp = (b == uFRhead.RESPONSE_HEADER)
          self.answer._got_header = True
        elif self.__async_id_enabled and b == self.__async_id_prefix:
          self.answer._got_async_id_prefix = True
          self.answer.async_id = ""
        continue

      # If asynchronous ID sending is enabled and we got the prefix, get the
      # ID until we hit the suffix
      if self.__async_id_enabled and self.answer._got_async_id_prefix:

        # If we got a hex digit, add it to the ID
        if b in b"0123456789ABCDEF":
          self.answer.async_id += chr(b)

        # If we hit the suffix and the ID we got is an even number of digits,
        # normalize it and return the answer
        elif b == self.__async_id_suffix and not len(self.answer.async_id) & 1:
            self.answer.async_id = ":".join([self.answer.async_id[i:i + 2] \
					for i in range(0,
					len(self.answer.async_id), 2)])
            self.answer.is_async_id = True
            return self.answer

        else:
          self.answer.wipe()

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

        # Return the long answer
        return self.answer

      else:
        self.answer.wipe()



  def _get_cmd_ext_part_ack(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> bool:
    """Get a multipart CMD_EXT acknowledgment
    Return True if it's a part acknowledgment, False if it's the last part.
    If we get anything else, raise an exception.
    """

    # Read data if the receive buffer is empty
    if not self.__recbuf:
      data = self._get_data(timeout = timeout)
      self.__recbuf.extend(data)

    # Parse one byte
    b: int = self.__recbuf.pop(0)

    # Did we get an ACK?
    if b == uFRcmdExtPartAck.ACK_PART:
      return True
    if b == uFRcmdExtPartAck.ACK_LAST_PART:
      return False

    # We got an expected byte
    raise uFRresponseError("expected {} ({:02x}h) or {} ({:02x}h) - "
				"got {:02x}h".format(
				uFRcmdExtPartAck.ACK_PART.name,
				uFRcmdExtPartAck.ACK_PART.value,
				uFRcmdExtPartAck.ACK_LAST_PART.name,
				uFRcmdExtPartAck.ACK_LAST_PART.value, b))



  def _get_last_command_response(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> uFRanswer:
    """Get a responde to the last command sent. Throw an exception if the
    answer is unexpected
    """

    answer: uFRanswer = self._get_answer(timeout = timeout)
    if not answer.is_rsp or answer.code != self._last_cmd:
      raise uFRresponseError("expected response to {} - got {}".format(
				self._last_cmd.name, answer))
    return answer



  def _restore_reader(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Restore the saved state of the reader, if it has been saved
    """

    pcd_mgr_state: uFRPCDMgrState
    emu_mode: uFRemuMode

    # Should we restore the emulation / ad-hoc (peer-to-peer) modes?
    if self.__saved_pcd_mgr_state is not None and \
		self.__saved_emu_mode is not None:

      pcd_mgr_state, emu_mode, _ = self.get_reader_status(timeout = timeout)

      # Should we disable emulation - tag emulation or ad-hoc (peer-to-peer)?
      if self.__saved_emu_mode == uFRemuMode.TAG_EMU_DISABLED:
        if emu_mode != uFRemuMode.TAG_EMU_DISABLED:
          if pcd_mgr_state == uFRPCDMgrState.PCD_MGR_CE_DEDICATED:
            self.tag_emulation_stop(timeout = timeout)
          else:
            self.ad_hoc_emulation_stop(timeout = timeout)

      # Should we enable tag emulation?
      elif self.__saved_pcd_mgr_state == uFRPCDMgrState.PCD_MGR_CE_DEDICATED:
        if emu_mode == uFRemuMode.TAG_EMU_DISABLED or \
		pcd_mgr_state != uFRPCDMgrState.PCD_MGR_CE_DEDICATED:
            self.tag_emulation_start(timeout = timeout)

      # Should we enable ad-hoc (peer-to-peer)?
      else:
        if pcd_mgr_state == uFRPCDMgrState.PCD_MGR_CE_DEDICATED:
          self.tag_emulation_stop(timeout = timeout)
        self.ad_hoc_emulation_start(timeout = timeout)

    # Is emulation disabled?
    if self.get_reader_status(timeout = timeout)[1] == \
		uFRemuMode.TAG_EMU_DISABLED:

      # Should we restore the anti-collision mode?
      if self.__saved_anti_collision_enabled is not None:

        ac_enabled: bool
        ac_tag_sel: bool
        ac_enabled, ac_tag_sel = self.get_anti_collision_status(
							timeout = timeout)

        if self.__saved_anti_collision_enabled and not ac_enabled:
          self.enable_anti_collision(timeout = timeout)

        elif not self.__saved_anti_collision_enabled:
          if ac_tag_sel:
            self.deselect_card(timeout = timeout)
          if ac_enabled:
            self.disable_anti_collision(timeout = timeout)

      # Should we restore the asynchronous ID sending parameters?
      if self.__saved_async_flags is not None and \
		self.__saved_async_prefix is not None and \
		self.__saved_async_suffix is not None and \
		self.__saved_async_baudrate is not None:
        self.set_card_id_send_conf(baudrate = self.__saved_async_baudrate,
					prefix = self.__saved_async_prefix,
					suffix = self.__saved_async_suffix,
					flags = self.__saved_async_flags,
					timeout = timeout)



  def get_async_id(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> Optional[str]:
    """Get asynchronous IDs
    """

    if not self.__async_id_enabled:
      raise uFRIOError("asynchronous ID sending not enabled")

    answer: uFRanswer = self._get_answer(timeout = timeout)

    if not answer.is_async_id:
      raise uFRresponseError("expected asynchronous ID answer "
				"- got {}".format(answer))

    return answer.async_id if answer.async_id else None



  def close(self: uFRcomm,
		restore = True,
		timeout: Optional[float] = None) \
		-> None:
    """Close any open connections
    By default, restore the reader if restore_on_reader was asserted on opening
    """

    if restore and self.serdev is not None or self.udpsock is not None or \
		self.tcpsock is not None or self.websock is not None or \
		self.resturl is not None:
      self._restore_reader(timeout = timeout)

    if self.serdev is not None:
      self.serdev.close()
      self.serdev = None

    if self.udpsock is not None:
      self.udpsock.close()
      self.udpsock = None
      self._udphost = None
      self._udpport = None

    if self.tcpsock is not None:
      self.tcpsock.close()
      self.tcpsock = None

    if self.websock is not None:
      self.websock.close()
      self.websock = None

    self.resturl = None
    self.__postdata = ""

    self._default_timeout = _default_ufr_timeout
    self._current_timeout = _default_ufr_timeout

    self.__recbuf = []

    self._last_cmd = uFRcmd._UNDEFINED

    self.answer.wipe()



  def __enter__(self: uFRcomm) \
		-> uFRcomm:
    """__enter__ method
    """

    return self



  def __exit__(self: uFRcomm,
		exc_type: Optional[Type[BaseException]],
		exc_value: Optional[BaseException],
		exc_traceback: Optional[TracebackType]) \
		-> None:
    """__exit__ method
    """

    # If we got here from an exception and we have saved reader states to
    # restore, make sure the read buffer is empty before issuing further
    # commands to the device
    if exc_type is not None and (
		self.__saved_pcd_mgr_state is not None or \
		self.__saved_emu_mode is not None or \
		self.__saved_anti_collision_enabled is not None):
      self.flush()

    self.close()



  # Front-end API functions - roughly 1:1 with the Digital Logic COM protocol
  #
  # Incomplete - functions will be added as needed

  def get_reader_type(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's type
    """

    self._send_cmd(uFRcmd.GET_READER_TYPE)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24) \
		if isinstance(rsp.ext, list) else -1



  def get_reader_serial(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's serial number as an integer
    """

    self._send_cmd(uFRcmd.GET_READER_SERIAL)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return rsp.ext[0] + (rsp.ext[1] << 8) + \
		(rsp.ext[2] << 16) + (rsp.ext[3] << 24)



  def get_serial_number(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> str:
    """Get the reader's serial number as a string
    """

    self._send_cmd(uFRcmd.GET_SERIAL_NUMBER)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return bytes(rsp.ext).decode("ascii")



  def get_hardware_version(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> int:
    """Get the reader's hardware version
    """

    self._send_cmd(uFRcmd.GET_HARDWARE_VERSION)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return (rsp.val0 << 8) + rsp.val1



  def get_firmware_version(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> int:
    """Get the reader's firmware version
    """

    self._send_cmd(uFRcmd.GET_FIRMWARE_VERSION)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return (rsp.val0 << 8) + rsp.val1



  def get_build_number(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> int:
    """Get the reader's firmware's build number
    """

    self._send_cmd(uFRcmd.GET_BUILD_NUMBER)
    return self._get_last_command_response(timeout = timeout).val0



  def get_card_id(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> Tuple[uFRcardType, int]:
    """Get the card type and UID (4 bytes only)
    """

    self._send_cmd(uFRcmd.GET_CARD_ID)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardType._NO_CARD, 0)
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardType._UNDEFINED),
		(rsp.ext[0] << 24) + (rsp.ext[1] << 16) + \
		(rsp.ext[2] << 8) + rsp.ext[3])



  def get_card_id_ex(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> Tuple[uFRcardType, str]:
    """Get the card type and UID (4, 7 or 10 bytes)
    """

    self._send_cmd(uFRcmd.GET_CARD_ID_EX)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardType._NO_CARD, "")
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardType._UNDEFINED),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_last_card_id_ex(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> Tuple[uFRcardType, str]:
    """Get the last read card type and UID (4, 7 or 10 bytes)
    Return () if no card was last read
    """

    self._send_cmd(uFRcmd.GET_LAST_CARD_ID_EX)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return (uFRcardType._NO_CARD, "")
      else:
        raise
    return (self.__UFR_VAL_TO_CARD_TYPE.get(rsp.val0, uFRcardType._UNDEFINED),
		self._uid_bytes2str(rsp.ext[:rsp.val1]))



  def get_dlogic_card_type(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> uFRDLCardType:
    """Get the Digital Logic card type
    """

    self._send_cmd(uFRcmd.GET_DLOGIC_CARD_TYPE)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return uFRDLCardType._NO_CARD
      else:
        raise
    return self.__UFR_VAL_TO_DL_CARD_TYPE.get(rsp.val0,
						uFRDLCardType._UNDEFINED)



  def linear_read(self: uFRcomm,
			authmode: uFRauthMode,
			addr: int,
			length: int,
			key: Union[List[int], Tuple[int, ...], bytes, int] = 0,
			multiblock: bool = False,
			timeout: Optional[float] = None) \
			-> bytes:
    """Linear read from a card. Return b"" if no card was present in the field
    STATUS: Partially tested
    """

    # Define the command parameters
    part1: int
    cmdext: List[int] = [addr & 0xff, addr >> 8]

    if authmode in (uFRauthMode.RKA_AUTH1A, uFRauthMode.RKA_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise TypeError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthMode.AKM1_AUTH1A, uFRauthMode.AKM1_AUTH1B,
		uFRauthMode.AKM2_AUTH1A, uFRauthMode.AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    elif authmode in (uFRauthMode.PK_AUTH1A, uFRauthMode.PK_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      if isinstance(key, list) or isinstance(key, tuple) or \
		isinstance(key, bytes):
        cmdext.extend(list(key))
      else:
        raise TypeError("key should be a tuple, list or bytes")

    elif authmode in (uFRauthMode.SAM_KEY_AUTH1A, uFRauthMode.SAM_KEY_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise TypeError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthMode.PK_AUTH1A_AES, uFRauthMode.PK_AUTH1B_AES):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])
      if isinstance(key, list) or isinstance(key, tuple) or \
		isinstance(key, bytes):
        cmdext.extend(list(key))
      else:
        raise TypeError("key should be a tuple, list or bytes")

    elif authmode in (uFRauthMode.MFP_RKA_AUTH1A, uFRauthMode.MFP_RKA_AUTH1B):
      if isinstance(key, int):
        par1 = key
      else:
        raise TypeError("key should be an int")
      if length < 192 or not multiblock:
        cmdext.extend([length & 0xff, length >> 8])
      else:
        cmdext.extend([0, 192, length & 0xff, length >> 8])

    elif authmode in (uFRauthMode.MFP_AKM1_AUTH1A, uFRauthMode.MFP_AKM1_AUTH1B,
		uFRauthMode.MFP_AKM2_AUTH1A, uFRauthMode.MFP_AKM2_AUTH1B):
      par1 = 0
      cmdext.extend([length & 0xff, length >> 8])

    # Send the command and read back the data
    self._send_cmd_ext(uFRcmd.LINEAR_READ, par1, 0, cmdext, timeout)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARD:
        return b""
      else:
        raise

    return bytes(rsp.ext)



  def set_card_id_send_conf(self: uFRcomm,
				enable: bool = True,
				baudrate: int = -1,
				prefix: int = -1,
				suffix: int = -1,
                                flags: int = -1,
				timeout: Optional[float] = None) \
				-> None:
    """Set the asynchronous card ID sending parameters
    """

    # Contrary to Digital Logic's recommendation, this library doesn't set a
    # different baudrate for asynchronous card ID sending. By default, if the
    # method was called without specifying a baudrate and the connection is
    # serial, reuse the serial connection's baudrate. If the connection is over
    # the network, the device is a uFR Nano Online, therefore set 115200 bps
    # as this is the baudrate used by the reader internally.
    #
    # If flags and baudrate are valid and were specified explicitely, use the
    # baudrate as passed
    if baudrate < 0 or flags < 0 or flags > 0xff:
      if self.serdev is not None:
        baudrate = baudrate if baudrate >=0 else self.serdev.baudrate
      else:
        baudrate = 115200

    # If the prefix or suffix weren't specified or are invalid, use the
    # default values
    if prefix < 0 or prefix > 0xff:
      prefix = self.__DEFAULT_ASYNC_ID_PREFIX
    if suffix < 0 or suffix > 0xff:
      suffix = self.__DEFAULT_ASYNC_ID_SUFFIX

    # If the flags weren't specified explicitely or are invalid, use the enable
    # parameter to determine them and the final baudrate
    par0: int
    if flags < 0 or flags > 0xff:
      par0 = 0x07 if enable else 0x00
      baudrate = baudrate if enable else 0
    else:
      par0 = flags

    par1: int = 0x00

    params: List[int] = [par0, par1, prefix, suffix,
			baudrate & 0xff, (baudrate >> 8) & 0xff,
			(baudrate >> 16) & 0xff, (baudrate >> 24) & 0xff]
    params.append(self._checksum(params))

    self._send_cmd_ext(uFRcmd.SET_CARD_ID_SEND_CONF, par0, par1, params[2:],
			timeout = timeout)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)

    # Save the state of the asynchronous ID sending feature and the
    # prefix / suffix for _get_answer() to catch start IDs
    self.__async_id_enabled = enable
    self.__async_id_prefix = prefix
    self.__async_id_suffix = suffix



  def get_card_id_send_conf(self: uFRcomm,
				save_status: bool = False,
				timeout: Optional[float] = None) \
				-> Tuple[int, int, int, int]:
    """Get the asynchronous card ID sending parameters
    """

    self._send_cmd(uFRcmd.GET_CARD_ID_SEND_CONF)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)

    if rsp.ext[7] != self._checksum(rsp.ext[:7]):
      raise uFRresponseError("asynchronous parameters checksum error")

    baudrate: int = rsp.ext[3] + (rsp.ext[4] << 8) + \
			(rsp.ext[5] << 16) + (rsp.ext[6] << 24)
    if save_status:
      self.__saved_async_flags = rsp.ext[0]
      self.__saved_async_prefix = rsp.ext[1]
      self.__saved_async_suffix = rsp.ext[2]
      self.__saved_async_baudrate = baudrate

    return (rsp.ext[0], rsp.ext[1], rsp.ext[2], baudrate)



  def get_rf_analog_settings(self: uFRcomm,
				tag_comm_type: uFRtagCommType,
				timeout: Optional[float] = None) \
				-> List[int]:
    """Get the RF frontend's analog settings
    """

    self._send_cmd(uFRcmd.GET_RF_ANALOG_SETTINGS, tag_comm_type.value)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return rsp.ext



  def set_rf_analog_settings(self: uFRcomm,
				tag_comm_type: uFRtagCommType,
				factory_settings: bool,
				settings: List[int],
				timeout: Optional[float] = None) \
				-> None:
    """Set the RF frontend's analog settings
    """

    self._send_cmd_ext(uFRcmd.SET_RF_ANALOG_SETTINGS, tag_comm_type,
			1 if factory_settings else 0, settings, timeout)
    self._get_last_command_response(timeout = timeout)



  def set_led_config(self: uFRcomm,
			blink: bool,
			timeout: Optional[float] = None) \
			-> None:
    """Set the green LED's configuration
    """

    self._send_cmd(uFRcmd.SET_LED_CONFIG, 1 if blink else 0)
    self._get_last_command_response(timeout = timeout)



  def enter_sleep_mode(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Send the reader to sleep
    """

    self._send_cmd(uFRcmd.ENTER_SLEEP_MODE)
    self._get_last_command_response(timeout = timeout)



  def leave_sleep_mode(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Wake up the reader
    """

    self._send_data([self.__WAKE_UP_BYTE])
    sleep(self.__WAKE_UP_WAIT)
    self._send_cmd(uFRcmd.LEAVE_SLEEP_MODE)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_wake_up_wait)



  def rf_reset(self: uFRcomm,
		timeout: Optional[float] = None) \
		-> None:
    """Reset the RF field
    """

    self._send_cmd(uFRcmd.RF_RESET)
    self._get_last_command_response(timeout = timeout)



  def check_uid_change(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> bool:
    """Return True if the card's UID is changeable (magic Mifare Classic gen2),
    False if it isn't of if no card is in the field
    NOTE: Does NOT test if the card responds to the magic Mifare command
          (magic Mifare Classic gen 1a), only if the UID is directly writeable
    """

    self._send_cmd(uFRcmd.CHECK_UID_CHANGE)
    try:
      self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.READING_ERROR:
        return False
      else:
        raise
    return True



  def get_reader_status(self: uFRcomm,
			save_status: bool = False,
			timeout: Optional[float] = None) \
			-> Tuple[uFRPCDMgrState, uFRemuMode, uFRemuState]:
    """Get the status of the reader
    If save_status is asserted, save the status to restore it on closing
    """

    self._send_cmd(uFRcmd.GET_READER_STATUS)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)

    pcd_mgr_state: uFRPCDMgrState = self.__UFR_VAL_TO_PCD_MGR_STATE[rsp.ext[0]]
    emu_mode: uFRemuMode = self.__UFR_VAL_TO_EMU_MODE[rsp.ext[1]]
    emu_state: uFRemuState = self.__UFR_VAL_TO_EMU_STATE[rsp.ext[2]]

    if save_status:
      self.__saved_pcd_mgr_state = pcd_mgr_state
      self.__saved_emu_mode = emu_mode

    return (pcd_mgr_state, emu_mode, emu_state)



  def self_reset(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Soft-restart the reader
    """

    self._send_cmd(uFRcmd.SELF_RESET)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_reset_wait)



  def write_emulation_ndef(self: uFRcomm,
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
      while self._get_cmd_ext_part_ack(timeout = timeout):
        if not ext_parts:
          raise uFRresponseError("expected {} ({:02x}h) - got {} ({:02x}h) with"
					"no more CMD_EXT parts to send".format(
					uFRcmdExtPartAck.ACK_LAST_PART.name,
					uFRcmdExtPartAck.ACK_LAST_PART.value,
					uFRcmdExtPartAck.ACK_PART.name,
					uFRcmdExtPartAck.ACK_PART.value))
        self._send_data(ext_parts.pop(0))

    if ext_parts:
      raise uFRresponseError("expected {} ({:02x}h) - got {} ({:02x}h) "
				"before sending the last CMD_EXT part".format(
				uFRcmdExtPartAck.ACK_PART.name,
				uFRcmdExtPartAck.ACK_PART.value,
				uFRcmdExtPartAck.ACK_LAST_PART.name,
				uFRcmdExtPartAck.ACK_LAST_PART.value))

    self._get_last_command_response(timeout = timeout)
    sleep(_post_write_emulation_ndef_wait)



  def tag_emulation_start(self: uFRcomm,
				ram_ndef: bool = False,
				timeout: Optional[float] = None) \
				-> None:
    """Start tag emulation. Use either the NDEF in EEPROM or in RAM
    """

    self._send_cmd(uFRcmd.TAG_EMULATION_START, 1 if ram_ndef else 0)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_emulation_start_stop_wait)



  def tag_emulation_stop(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Stop tag emulation
    """

    self._send_cmd(uFRcmd.TAG_EMULATION_STOP)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_emulation_start_stop_wait)



  def ad_hoc_emulation_start(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Start ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(uFRcmd.AD_HOC_EMULATION_START)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_emulation_start_stop_wait)



  def ad_hoc_emulation_stop(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Stop ad-hoc (peer-to-peer) emulation
    """

    self._send_cmd(uFRcmd.AD_HOC_EMULATION_STOP)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_emulation_start_stop_wait)



  def get_external_field_state(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> bool:
    """Test the presence of an external RF field in ad-hoc (peer-to-peer) mode
    """

    self._send_cmd(uFRcmd.GET_EXTERNAL_FIELD_STATE)
    return self._get_last_command_response(timeout = timeout).val0 == 1



  def get_ad_hoc_emulation_params(self: uFRcomm,
					timeout: Optional[float] = None) \
					-> Tuple[int, int]:
    """Get current ad-hoc (peer-to-peer) emulation parameters
    Return RxThreshold and RFCfg
    """

    self._send_cmd(uFRcmd.GET_AD_HOC_EMULATION_PARAMS)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    return (rsp.val0, rsp.val1)



  def set_ad_hoc_emulation_params(self: uFRcomm,
					rxthreshold: int,
					rfcfg: int,
					timeout: Optional[float] = None) \
					-> None:
    """Set current ad-hoc (peer-to-peer) emulation parameters
    """

    self._send_cmd(uFRcmd.SET_AD_HOC_EMULATION_PARAMS, rxthreshold,
							rfcfg & 0x7f)
    self._get_last_command_response(timeout = timeout)



  def red_light_control(self: uFRcomm,
			state: bool,
			timeout: Optional[float] = None) \
			-> None:
    """Turn the red LED on or off
    """

    self._send_cmd(uFRcmd.RED_LIGHT_CONTROL, 1 if state else 0)
    self._get_last_command_response(timeout = timeout)



  def user_interface_signal(self: uFRcomm,
				light_signal_mode: uFRlightSignal,
				beep_signal_mode: uFRbeepSignal,
				timeout: Optional[float] = None) \
				-> None:
    """Trigger a LED sequence or beep sequence
    """

    self._send_cmd(uFRcmd.USER_INTERFACE_SIGNAL, light_signal_mode.value,
			beep_signal_mode.value)
    self._get_last_command_response(timeout = timeout)



  def set_speaker_frequency(self: uFRcomm,
				frequency: float,
				timeout: Optional[float] = None) \
				-> None:
    """Make the buzzer emit a continuous sound. Set frequency to 0 to stop
    the sound
    """

    period: int = ((round(65535 - 1500000 / (2 * frequency))) & 0xffff) \
		if frequency > 0 else 0xffff
    self._send_cmd(uFRcmd.SET_SPEAKER_FREQUENCY, period & 0xff, period >> 8)
    self._get_last_command_response(timeout = timeout)



  def set_iso14443_4_mode(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Set ISO14443-4 mode
    """

    self._send_cmd(uFRcmd.SET_ISO14443_4_MODE)
    self._get_last_command_response(timeout = timeout)



  def s_block_deselect(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Deselect tag and restore polling
    STATUS: Untested
    """

    self._send_cmd(uFRcmd.S_BLOCK_DESELECT)
    self._get_last_command_response(timeout = timeout)



  def apdu_transceive(self: uFRcomm,
			c_apdu: bytes,
			apdu_timeout_ms: Optional[int] = None,
			timeout: Optional[float] = None) \
			-> bytes:
    """Send a command APDU to the ISO14443-4 transponder and get the response
    APDU
    STATUS: Untested
    """

    self._send_cmd_ext(uFRcmd.APDU_TRANSCEIVE, 0,
			round(self._default_timeout * 1000) \
			if apdu_timeout_ms is None else apdu_timeout_ms,
			c_apdu, timeout)
    return bytes(self._get_last_command_response(timeout = timeout).ext)



  def enable_anti_collision(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Enable anti-collision mode: leave single-card mode and stop polling
    """

    self._send_cmd(uFRcmd.ENABLE_ANTI_COLLISION)
    self._get_last_command_response(timeout = timeout)



  def disable_anti_collision(self: uFRcomm,
				timeout: Optional[float] = None) \
				-> None:
    """Disable anti-collision mode; return to single-card mode and start polling
    """

    self._send_cmd(uFRcmd.DISABLE_ANTI_COLLISION)
    self._get_last_command_response(timeout = timeout)



  def enum_cards(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> bool:
    """Enumerate cards in the field in anti-collision mode
    Return True if at least one card was enumerated, False if no card was found
    """

    self._send_cmd(uFRcmd.ENUM_CARDS)
    try:
      self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARDS_ENUMERATED:
        return False
      else:
        raise
    return True



  def list_cards(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> List[str]:
    """List cards previously enumerated by enum_cards()
    Return a list of UIDs or an empty list if no card was enumerated
    """

    self._send_cmd(uFRcmd.LIST_CARDS)
    try:
      rsp: uFRanswer = self._get_last_command_response(timeout = timeout)
    except:
      if self.answer.code == uFRerr.NO_CARDS_ENUMERATED:
        return []
      else:
        raise
    return [self._uid_bytes2str(rsp.ext[i + 1 : i + rsp.ext[i] + 1]) \
		for i in range(0, len(rsp.ext), 11)]



  def select_card(self: uFRcomm,
			uid: str,
			timeout: Optional[float] = None) \
			-> uFRDLCardType:
    """Select a card in the field in anti-collision mode
    """

    bytesuid: bytes = self._uid_str2bytes(uid)
    self._send_cmd_ext(uFRcmd.SELECT_CARD, len(bytesuid), 0, bytesuid, timeout)
    return self.__UFR_VAL_TO_DL_CARD_TYPE[
		self._get_last_command_response(timeout = timeout).val0]



  def deselect_card(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Deselect the currently selected card in anti-collision mode
    """

    self._send_cmd(uFRcmd.DESELECT_CARD)
    self._get_last_command_response(timeout = timeout)



  def get_anti_collision_status(self: uFRcomm,
				save_status: bool = False,
				timeout: Optional[float] = None) \
				-> Tuple[bool, bool]:
    """Return the status of the anti-collision mode, and whether a card is
    currently selected
    If save_status is asserted, save the status to restore it on closing
    """

    self._send_cmd(uFRcmd.GET_ANTI_COLLISION_STATUS)
    rsp: uFRanswer = self._get_last_command_response(timeout = timeout)

    anti_collision_enabled: bool = rsp.val0 != 0
    anti_collision_card_selected: bool = rsp.val1 != 0

    if save_status:
      self.__saved_anti_collision_enabled = anti_collision_enabled

    return (anti_collision_enabled, anti_collision_card_selected)



  def esp_set_io_state(self: uFRcomm,
			pin: int,
			state: uFRIOState,
			timeout: Optional[float] = None) \
			-> None:
    """Set the state of one of the 6 ESP I/O pins
    """

    self._send_cmd(uFRcmd.ESP_SET_IO_STATE, pin, state.value)
    self._get_last_command_response(timeout = timeout)



  def esp_get_io_state(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> List[uFRIOState]:
    """Get the states of the 6 ESP I/O pins
    return the states as a list
    """

    self._send_cmd(uFRcmd.ESP_GET_IO_STATE)
    return [self.__UFR_VAL_TO_IOSTATE[st]
		for st in self._get_last_command_response(
		timeout = timeout).ext]



  def esp_set_display_data(self: uFRcomm,
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
    self._get_last_command_response(timeout = timeout)



  def esp_reader_reset(self: uFRcomm,
			timeout: Optional[float] = None) \
			-> None:
    """Ask the ESP to reset the reader
    """

    self._send_cmd(uFRcmd.ESP_READER_RESET, 0)
    self._get_last_command_response(timeout = timeout)
    sleep(_post_reset_wait)



class uFR:

  open = uFRcomm



  def nano_online_host_discovery(self: uFR,
					hostaddr: str,
					timeout: float = _default_ufr_timeout) \
					-> Optional[uFRdiscoveryResponse]:
    """Try to send a UDP packet to a single host on port 8880 / UDP and return
    the discovery response or None
    """

    hostaddr = socket.gethostbyname(hostaddr)

    # Set up the UDP socket
    udpsock: socket.socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udpsock:

      udpsock.settimeout(timeout)

      # Send the host a UDP datagram on port 8880
      udpsock.sendto(b"", (hostaddr, 8880))

      # Wait for a UDP datagram back and only keep it if it's a valid response
      # from that host
      data: bytes
      ip: str
      try:
        data, (ip, _) = udpsock.recvfrom(1024)
      except socket.timeout:
        return None

      if ip != hostaddr:
        return None

      try:
        response: uFRdiscoveryResponse = uFRdiscoveryResponse(data)
      except:
        return None

      if response.ip != ip:
        del(response)
        return None

      return uFRdiscoveryResponse(data)



  def nano_online_subnet_discovery(self: uFR,
					netaddr: str,
					timeout: float = _default_ufr_timeout) \
					-> Generator:
    """Try to send a UDP packet to all the hosts on a subnet on port 8880 / UDP
    and return which ones respond with a UDP discovery response datagram
    """

    ip_net: ipaddress.IPv4Network = ipaddress.ip_network(netaddr)

    # Set up the UDP socket
    udpsock: socket.socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udpsock:

      udpsock.settimeout(.1)	# Very short timeout for sending to begin with

      # Increase the socket send buffer to ping an entire /24 subnet in one go
      udpsock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2000000)

      # Get the first host in the subnet
      addr_generator: Generator = ipaddress.ip_network(netaddr).hosts()
      next_addr: str = str(next(addr_generator, ""))
      if not next_addr:
        return

      # Send probe databrams, get UDP datagrams back and only keep valid
      # responses from hosts we've interested in
      data: bytes
      ip: str

      while True:

        # Send as many UDP packets as we can then switch to listening to
        # responses back if we fill up the send buffer before reaching the end
        # of the list of hosts to ping. If we've sent all the UDP packets we
        # have to send, switch to the normal timeout for the final round of
        # listening
        while next_addr:

          try:
            udpsock.sendto(b"", (next_addr, 8880))
            next_addr = str(next(addr_generator, ""))
          except socket.timeout:
            break

          if not next_addr:
            udpsock.settimeout(timeout)

        # Wait for a UDP datagram back and only yield it if it's a valid
        # response from a host in the subnet
        try:
          data, (ip, _) = udpsock.recvfrom(1024)
        except socket.timeout:
          if next_addr:
            continue
          else:
            break

        if ipaddress.IPv4Address(ip) not in ip_net:
          continue

        try:
          response: uFRdiscoveryResponse = uFRdiscoveryResponse(data)
        except:
          continue

        if response.ip != ip:
          del(response)
          continue

        yield response



### Routines
def __print_api_state_of_completion() \
			-> None:
  """Print the API's state of completion - i.e. which Digital Logic COM
  protocol functions are implemented in this class, which aren't, which are
  untested or partially tested, and what percentage of the COM protocol is
  implemented
  """

  comcmds: List[str] = [cmd.name for cmd in uFRcmd]
  ufrcomm_pubclassmets: List[str] = [attribute for attribute in dir(uFRcomm) \
			if callable(getattr(uFRcomm, attribute)) and \
			not attribute.startswith('_')]
  ufr_pubclassmets: List[str] = [attribute for attribute in dir(uFR) \
			if callable(getattr(uFR, attribute)) and \
			not attribute.startswith('_')]

  cmd_method: Dict[str, str] = {}
  cmd_impl_status: Dict[str, str] = {}
  nb_impl: int = 0
  max_cmd_name_len: int = 0
  max_method_name_len: int = 0
  max_status_len: int = 0
  cmd: str
  localparms: Dict[str, Any]
  m: Optional[List]

  # Get the status of all the commands in the COM protocol from whether the
  # corresponding methods exists in lowercase in the uFRcomm class and from
  # status markers in their docstrings
  for cmd in comcmds:
    if cmd.lower() in ufrcomm_pubclassmets:
      localparms = {'uFRcomm': uFRcomm}
      exec("doc = uFRcomm.{}.__doc__".format(cmd.lower()), {}, localparms)
      m = re.findall("(STATUS|COMPLETION)\s*:\s*(.*)\n", localparms["doc"])
      cmd_method[cmd] = cmd.lower() + "()"
      cmd_impl_status[cmd] = m[0][1] if m else "Implemented"
      nb_impl += 1
    else:
      cmd_method[cmd] = "N/A"
      cmd_impl_status[cmd] = "Not implemented"
    max_cmd_name_len = max(max_cmd_name_len, len(cmd))
    max_method_name_len = max(max_method_name_len, len(cmd_method[cmd]))
    max_status_len = max(max_status_len, len(cmd_impl_status[cmd]))

  # Dump the implementation status of all the commands and the percentage of
  # implemented commands
  max_cmd_name_len += 2
  pad_cmd: Callable = lambda s: \
		("{" + ":<{}".format(max_cmd_name_len) + "}").format(s)
  max_method_name_len += 2
  pad_method: Callable = lambda s: \
		("{" + ":<{}".format(max_method_name_len) + "}").format(s)
  separator: str
  separator = "-" * (max_cmd_name_len + max_method_name_len + max_status_len)

  print(pad_cmd("uFR COM protocol command"), pad_method("uFRcomm class method"),
	"Status")
  print(separator)
  for cmd in comcmds:
    print(pad_cmd(cmd), pad_method(cmd_method[cmd]), cmd_impl_status[cmd])

  print(separator)
  print(pad_cmd("State of completion:"),
		"{}%".format(round(100 * nb_impl / len(uFRcmd))))

  # Dump the list of methods that are specific to the uFRcomm class
  print()
  print("uFRcomm class-specific methods")
  print(separator)
  for met in sorted(ufrcomm_pubclassmets, key = lambda s: len(s)):
    if met.upper() not in comcmds:
      print("{}()".format(met))

  # Dump the list of methods in the uFR class
  print()
  print("uFR class methods")
  print(separator)
  for met in sorted(ufr_pubclassmets, key = lambda s: len(s)):
    print("{}()".format(met))



def __test_api(device: Optional[str],
		network: Optional[str]) \
		-> None:
  """Test the API
  """

  i: int

  # Pretty line formatting
  pad: Callable = lambda s: "{:<30}".format(s)

  ufr: uFR = uFR()

  # Test network probing functions - the device doesn't need to be open for this
  if __test_network_probe_functions:

    if not network:
      print("No network specified to test network probing")

    else:
      print("NANO_ONLINE_SUBNET_DISCOVERY:")
      discovery_response: Optional[uFRdiscoveryResponse] = None
      for discovery_response in ufr.nano_online_subnet_discovery(network):
        print(discovery_response)
      if discovery_response is not None:
        print(pad("NANO_ONLINE_HOST_DISCOVERY:"),
			ufr.nano_online_host_discovery(discovery_response.ip))
      else:
        print("No uFR Nano Online reader discovered")

  # Open the device
  if device:
    ufrcomm = ufr.open(device)
  else:
    print("No uFR device specified to test communication")
    return

  # Test reader information functions
  if __test_reader_info_functions:

    print(pad("GET_READER_TYPE:"), hex(ufrcomm.get_reader_type()))
    print(pad("GET_READER_SERIAL:"), ufrcomm.get_reader_serial())
    print(pad("GET_SERIAL_NUMBER:"), ufrcomm.get_serial_number())
    print(pad("GET_HARDWARE_VERSION:"), hex(ufrcomm.get_hardware_version()))
    print(pad("GET_FIRMWARE_VERSION:"), hex(ufrcomm.get_firmware_version()))
    print(pad("GET_BUILD_NUMBER:"), hex(ufrcomm.get_build_number()))
    print(pad("GET_READER_STATUS:"), ufrcomm.get_reader_status())

  # Test ad-hoc (peer-to-peer) functions
  if __test_ad_hoc_functions:

    print("AD_HOC_EMULATION_START")
    ufrcomm.ad_hoc_emulation_start()
    rxthreshold: int
    rfcfg: int
    rxthreshold, rfcfg = ufrcomm.get_ad_hoc_emulation_params()
    print(pad("GET_AD_HOC_EMULATION_PARAMS:"),
		(hex(rxthreshold), hex(rfcfg)))
    print("SET_AD_HOC_EMULATION_PARAMS:")
    ufrcomm.set_ad_hoc_emulation_params(rxthreshold, rfcfg)
    print(pad("GET_READER_STATUS:"), ufrcomm.get_reader_status())
    print(pad("GET_EXTERNAL_FIELD_STATE"), ufrcomm.get_external_field_state())
    print("AD_HOC_EMULATION_STOP")
    ufrcomm.ad_hoc_emulation_stop()

  # Test RF analog settings functions
  if __test_rf_analog_settings_functions:

    tct: uFRtagCommType
    for tct in uFRtagCommType:
      print(pad("GET_RF_ANALOG_SETTINGS:"), ufrcomm.get_rf_analog_settings(tct))

      if __test_eeprom_writing_functions:

        new_settings: List[int] = list(ufrcomm.answer.ext)
        new_settings[PN53xAnalogSettingsReg.RXTHRESHOLD] = 255
        print("SET_RF_ANALOG_SETTINGS")
        ufrcomm.set_rf_analog_settings(tct, False, new_settings)

  # Test reset functions
  if __test_reset_functions:

    print("RF_RESET")
    ufrcomm.rf_reset()
    print("SELF_RESET")
    ufrcomm.self_reset()

    # Only test the ESP reset function if the device is a Nano Online connected
    # through the network, but not in HTTP mode, as it bypasses the ESP and
    # sends the commands directly to the UART
    if ufrcomm.udpsock is not None or ufrcomm.tcpsock is not None or \
		ufrcomm.websock is not None:

      print("ESP_READER_RESET")
      ufrcomm.esp_reader_reset()

  # Test sleep functions - only works if the device is connected directly to a
  # a serial port
  if __test_sleep_functions and ufrcomm.serdev is not None:

    print("ENTER_SLEEP_MODE")
    ufrcomm.enter_sleep_mode()
    print("LEAVE_SLEEP_MODE")
    ufrcomm.leave_sleep_mode()

  # Test LED and buzzer functions
  if __test_led_sound_functions:

    if __test_eeprom_writing_functions:

      print("SET_LED_CONFIG")
      ufrcomm.set_led_config(True)

    print("RED_LIGHT_CONTROL")
    ufrcomm.red_light_control(True)
    sleep(2)
    ufrcomm.red_light_control(False)

    print("SET_SPEAKER_FREQUENCY")
    freq: float = 1480 / 4
    for i in range(3):
      ufrcomm.set_speaker_frequency(freq)
      freq *= 2
      sleep(.1)
    ufrcomm.set_speaker_frequency(0)

    print("USER_INTERFACE_SIGNAL")
    ufrcomm.user_interface_signal(uFRlightSignal.ALTERNATION,
					uFRbeepSignal.SHORT)

    # Only test the ESP LED function if the device is a Nano Online connected
    # through the network, but not in HTTP mode, as it bypasses the ESP and
    # sends the commands directly to the UART
    if ufrcomm.udpsock is not None or ufrcomm.tcpsock is not None or \
		ufrcomm.websock is not None:

      print("ESP_SET_DISPLAY_DATA")
      for i in range(3):
        ufrcomm.esp_set_display_data((0xff, 0, 0), (0, 0xff, 0), 0)
        sleep(0.1)
        ufrcomm.esp_set_display_data((0, 0xff, 0), (0, 0, 0xff), 0)
        sleep(0.1)
        ufrcomm.esp_set_display_data((0, 0, 0xff), (0xff, 0, 0), 0)
        sleep(0.1)
      ufrcomm.esp_set_display_data((0, 0, 0), (0, 0, 0), 1000)

  # Test ESP I/O functions - only works if the device is a Nano Online connected
  # through the network, but not in HTTP mode, as it mode bypasses the ESP and
  # sends the commands directly to the UART
  if __test_esp_io and (ufrcomm.udpsock is not None or \
		ufrcomm.tcpsock is not None or ufrcomm.websock is not None):

    print("ESP_SET_IO_STATE")
    ufrcomm.esp_set_io_state(6, uFRIOState.HIGH)
    print(pad("ESP_GET_IO_STATE"), ufrcomm.esp_get_io_state())
    print("ESP_SET_IO_STATE")
    ufrcomm.esp_set_io_state(6, uFRIOState.LOW)
    print(pad("ESP_GET_IO_STATE"), ufrcomm.esp_get_io_state())

  # Test UID functions
  if __test_uid_functions:

      cid: Tuple[uFRcardType, int] = ufrcomm.get_card_id()
      print(pad("GET_CARD_ID:"),
		(cid[0], "0x{:08X}".format(cid[1])) if cid else ())
      print(pad("GET_CARD_ID_EX:"), ufrcomm.get_card_id_ex())
      print(pad("GET_LAST_CARD_ID_EX:"), ufrcomm.get_last_card_id_ex())
      print(pad("GET_DLOGIC_CARD_TYPE:"), ufrcomm.get_dlogic_card_type())
      print(pad("CHECK_UID_CHANGE:"), ufrcomm.check_uid_change())

  # Test read functions
  if __test_read_functions:
      print(pad("LINEAR_READ:"),
			ufrcomm.linear_read(uFRauthMode.T2T_NO_PWD_AUTH, 0, 10))

  # Test ISO14443-4 functions
  if __test_iso14443_4_functions:

    print("SET_ISO_14443_4_MODE")
    try:
      ufrcomm.set_iso14443_4_mode()
    except:
      if ufrcomm.answer.code != uFRerr.NO_CARD:
        raise

  # Test anti-collision functions
  if __test_anti_collision_functions:

    print("SELF_RESET")
    ufrcomm.self_reset()
    print("ENABLE_ANTI_COLLISION")
    ufrcomm.enable_anti_collision()
    print(pad("GET_ANTI_COLLISION_STATUS:"),
					ufrcomm.get_anti_collision_status())
    print(pad("ENUM_CARDS:"), ufrcomm.enum_cards())
    uids: List[str] = ufrcomm.list_cards()
    print(pad("LIST_CARDS:"), uids)
    if uids:
      print(pad("SELECT CARD:"), ufrcomm.select_card(uids[0]))
      print(pad("GET_ANTI_COLLISION_STATUS:"),
		ufrcomm.get_anti_collision_status())
      print("DESELECT CARD")
      ufrcomm.deselect_card()
      print(pad("GET_ANTI_COLLISION_STATUS:"),
		ufrcomm.get_anti_collision_status())
    print("DISABLE_ANTI_COLLISION")
    ufrcomm.disable_anti_collision()
    print(pad("GET_ANTI_COLLISION_STATUS:"),
					ufrcomm.get_anti_collision_status())

  # Test tag emulation functions
  if __test_tag_emulation:

    eeprom_ndef: bytes = b"\x03\x10\xd1\x01\x0cU\x01d-logic.net\xfe"
    ram_ndef: bytes = b"\x03\xff\x03\xeb\xc1\x01\x00\x00\x03\xe4T\x02en3456" + \
			b"7890123456" * 99

    if __test_eeprom_writing_functions:
      print("WRITE_EMULATION_NDEF (EEPROM)")
      ufrcomm.write_emulation_ndef(eeprom_ndef, False)

    print("WRITE_EMULATION_NDEF (RAM)")
    ufrcomm.write_emulation_ndef(ram_ndef, True)
    print("TAG_EMULATION_START (RAM NDEF)")
    ufrcomm.tag_emulation_start(True)
    print(pad("GET_READER_STATUS:"), ufrcomm.get_reader_status())
    print("TAG_EMULATION_STOP")
    ufrcomm.tag_emulation_stop()

  # Test asynchronous card ID sending functions, but not in HTTP mode, as it
  # is synchronous by nature
  if __test_asynchronous_card_id_sending and ufrcomm.resturl is None:
    print("SET_CARD_ID_SEND_CONF")
    ufrcomm.set_card_id_send_conf(True)
    print(pad("GET_CARD_ID_SEND_CONF"), ufrcomm.get_card_id_send_conf())
    print("Waiting for asynchronous IDs...")
    while True:
      try:
        print(ufrcomm.get_async_id(3))
      except:
        break
    print("SET_CARD_ID_SEND_CONF")
    ufrcomm.set_card_id_send_conf(False)
    print(pad("GET_CARD_ID_SEND_CONF"), ufrcomm.get_card_id_send_conf())

  # Close the device
  ufrcomm.close()



### Main routine - run if the class is called as a standalone program
if __name__ == "__main__":

  ### Modules
  import sys
  import argparse

  # Parse the command line arguments
  argparser: argparse.ArgumentParser = argparse.ArgumentParser()
  argparser.add_argument(
	  "-d", "--device",
	  help = "uFR device to test communication (e.g. serial:///dev/ttyUSB0"
			":1000000)",
	  type = str,
	)
  argparser.add_argument(
	  "-n", "--network",
	  help = "Network to test network probing (e.g. 192.168.1.0/24)",
	  type = str,
	)
  argparser.add_argument(
	  "-soc", "--state-of-completion",
	  help = "Print the API's state of completion",
	  action = "store_true"
	)
  args: argparse.Namespace = argparser.parse_args()

  # Dump the API's state of completion
  if args.state_of_completion:
    __print_api_state_of_completion()

  # Test the API
  else:
    __test_api(args.device, args.network)
