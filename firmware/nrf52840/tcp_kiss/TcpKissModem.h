#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <RAK13800_W5100S.h>

// Standard KISS framing bytes (KA9Q / MeshCore KISS spec)
#define KISS_FEND  0xC0
#define KISS_FESC  0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

#define KISS_MAX_FRAME_SIZE          512
#define KISS_MAX_PACKET_SIZE         255
#define KISS_FRAME_BOUNDARY_BYTES    2
#define KISS_TYPE_BYTES              1
#define KISS_HW_SUBCMD_BYTES         1
// Worst-case escaped 255-byte MeshCore packet: 2 + 2 * (1 + 255) = 514 bytes.
// Keep contiguous RAM frame buffer >= 520 bytes to avoid worst-case overflow.
#define KISS_MAX_ENCODED_FRAME_SIZE  520
#define KISS_TX_FRAME_QUEUE_DEPTH    4
#define KISS_RX_CHUNK_SIZE           128

// Standard KISS commands (port 0)
#define KISS_CMD_DATA        0x00
#define KISS_CMD_TXDELAY     0x01
#define KISS_CMD_PERSISTENCE 0x02
#define KISS_CMD_SLOTTIME    0x03
#define KISS_CMD_TXTAIL      0x04
#define KISS_CMD_FULLDUPLEX  0x05
#define KISS_CMD_SETHARDWARE 0x06
#define KISS_CMD_RETURN      0xFF

#define KISS_DEFAULT_TXDELAY     50
#define KISS_DEFAULT_PERSISTENCE 63
#define KISS_DEFAULT_SLOTTIME    10
#define KISS_TX_TIMEOUT_NUM      3
#define KISS_TX_TIMEOUT_DEN      2   // 1.5x estimated airtime

// MeshCore SetHardware (0x06) sub-commands
#define HW_CMD_GET_IDENTITY      0x01
#define HW_CMD_GET_RANDOM        0x02
#define HW_CMD_VERIFY_SIGNATURE  0x03
#define HW_CMD_SIGN_DATA         0x04
#define HW_CMD_ENCRYPT_DATA      0x05
#define HW_CMD_DECRYPT_DATA      0x06
#define HW_CMD_KEY_EXCHANGE      0x07
#define HW_CMD_HASH              0x08
#define HW_CMD_SET_RADIO         0x09
#define HW_CMD_SET_TX_POWER      0x0A
#define HW_CMD_GET_RADIO         0x0B
#define HW_CMD_GET_TX_POWER      0x0C
#define HW_CMD_GET_CURRENT_RSSI  0x0D
#define HW_CMD_IS_CHANNEL_BUSY   0x0E
#define HW_CMD_GET_AIRTIME       0x0F
#define HW_CMD_GET_NOISE_FLOOR   0x10
#define HW_CMD_GET_VERSION       0x11
#define HW_CMD_GET_STATS         0x12
#define HW_CMD_GET_BATTERY       0x13
#define HW_CMD_GET_MCU_TEMP      0x14
#define HW_CMD_GET_SENSORS       0x15
#define HW_CMD_GET_DEVICE_NAME   0x16
#define HW_CMD_PING              0x17
#define HW_CMD_REBOOT            0x18
#define HW_CMD_SET_SIGNAL_REPORT 0x19
#define HW_CMD_GET_SIGNAL_REPORT 0x1A
#define HW_CMD_ENTER_OTA_DFU     0xFE

#define HW_RESP(cmd)             ((cmd) | 0x80)
#define HW_RESP_OK               0xF0
#define HW_RESP_ERROR            0xF1
#define HW_RESP_TX_DONE          0xF8
#define HW_RESP_RX_META          0xF9

#define HW_ERR_INVALID_LENGTH    0x01
#define HW_ERR_INVALID_PARAM     0x02
#define HW_ERR_NO_CALLBACK       0x03
#define HW_ERR_MAC_FAILED        0x04
#define HW_ERR_UNKNOWN_CMD       0x05
#define HW_ERR_ENCRYPT_FAILED    0x06
#define HW_ERR_TX_BUSY           0x07

#define KISS_FIRMWARE_VERSION    1

struct KissRadioConfig {
  uint32_t freq_hz;
  uint32_t bw_hz;
  uint8_t sf;
  uint8_t cr;
  uint8_t tx_power;
};

typedef void (*SetRadioCallback)(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr);
typedef void (*SetTxPowerCallback)(uint8_t power_dbm);
typedef float (*GetCurrentRssiCallback)();
typedef void (*GetStatsCallback)(uint32_t* rx, uint32_t* tx, uint32_t* errors);
typedef void (*ControlActionCallback)();

class TcpKissModem {
  enum TxState : uint8_t {
    TX_IDLE,
    TX_WAIT_CLEAR,
    TX_SLOT_WAIT,
    TX_DELAY,
    TX_SENDING,
    TX_DONE_PENDING
  };

  struct OutboundFrame {
    uint16_t len;
    uint8_t data[KISS_MAX_ENCODED_FRAME_SIZE];
  };

  mesh::Radio& _radio;
  mesh::MainBoard& _board;
  uint32_t _prng_state;

  // Non-blocking KISS RX frame parser
  uint8_t _rx_buf[KISS_MAX_FRAME_SIZE];
  uint16_t _rx_len;
  bool _rx_escaped;
  bool _rx_active;

  // Pending radio TX state machine
  uint8_t _pending_tx[KISS_MAX_PACKET_SIZE];
  uint16_t _pending_tx_len;
  bool _has_pending_tx;

  uint8_t _txdelay;
  uint8_t _persistence;
  uint8_t _slottime;
  uint8_t _txtail;
  uint8_t _fullduplex;

  TxState _tx_state;
  uint32_t _tx_timer;

  // Outbound contiguous KISS frame ring buffer
  OutboundFrame _tx_queue[KISS_TX_FRAME_QUEUE_DEPTH];
  uint8_t _tx_head;
  uint8_t _tx_tail;
  uint8_t _tx_count;
  bool _tx_busy_error_pending;
  bool _tx_done_pending;
  uint8_t _tx_done_result;
  uint8_t _hw_payload[KISS_MAX_PACKET_SIZE + KISS_HW_SUBCMD_BYTES];

  KissRadioConfig _config;
  bool _signal_report_enabled;

  SetRadioCallback _setRadioCallback;
  SetTxPowerCallback _setTxPowerCallback;
  GetCurrentRssiCallback _getCurrentRssiCallback;
  GetStatsCallback _getStatsCallback;
  ControlActionCallback _rebootCallback;
  ControlActionCallback _enterOtaCallback;

  EthernetClient* _active_client;
  bool _pending_reboot;
  bool _pending_ota_dfu;

  uint8_t nextRandomByte();
  static uint16_t appendEscapedByte(uint8_t* dest, uint16_t idx, uint16_t max_len, uint8_t b);
  static uint16_t encodeFrame(uint8_t type, const uint8_t* data, uint16_t len, uint8_t* dest, uint16_t max_len);
  void resetOutputQueue();
  bool tryFlushFrames(EthernetClient* client);
  bool queueFrame(uint8_t type, const uint8_t* data, uint16_t len, bool mark_busy_error = true);
  bool queueHardwareFrame(uint8_t sub_cmd, const uint8_t* data, uint16_t len, bool mark_busy_error = true);
  bool queuePendingBusyError();
  bool queuePendingTxDone();
  void setTxDonePending(uint8_t result);
  void writeHardwareError(uint8_t error_code);

  void feedRxByte(uint8_t b);
  void processFrame();
  void handleHardwareCommand(uint8_t sub_cmd, const uint8_t* data, uint16_t len);
  void processTx();

  void handleGetRandom(const uint8_t* data, uint16_t len);
  void handleSetRadio(const uint8_t* data, uint16_t len);
  void handleSetTxPower(const uint8_t* data, uint16_t len);
  void handleGetRadio();
  void handleGetTxPower();
  void handleGetCurrentRssi();
  void handleIsChannelBusy();
  void handleGetAirtime(const uint8_t* data, uint16_t len);
  void handleGetNoiseFloor();
  void handleGetVersion();
  void handleGetStats();
  void handleGetBattery();
  void handleGetMCUTemp();
  void handleGetDeviceName();
  void handlePing();
  void handleReboot();
  void handleSetSignalReport(const uint8_t* data, uint16_t len);
  void handleGetSignalReport();
  void handleEnterOtaDfu();

public:
  TcpKissModem(mesh::Radio& radio, mesh::MainBoard& board);

  void begin(const KissRadioConfig& initial_config, uint32_t rng_seed);
  void resetSession();

  // Returns true if any bytes were read from the TCP client during this tick.
  bool loop(EthernetClient* client);

  void onPacketReceived(int8_t snr_x4, int8_t rssi_dbm, const uint8_t* packet, uint16_t len);

  void setRadioCallback(SetRadioCallback cb) { _setRadioCallback = cb; }
  void setTxPowerCallback(SetTxPowerCallback cb) { _setTxPowerCallback = cb; }
  void setGetCurrentRssiCallback(GetCurrentRssiCallback cb) { _getCurrentRssiCallback = cb; }
  void setGetStatsCallback(GetStatsCallback cb) { _getStatsCallback = cb; }
  void setRebootCallback(ControlActionCallback cb) { _rebootCallback = cb; }
  void setEnterOtaCallback(ControlActionCallback cb) { _enterOtaCallback = cb; }

  const KissRadioConfig& radioConfig() const { return _config; }
  bool isTxBusy() const { return _tx_state != TX_IDLE; }
  bool isActuallyTransmitting() const { return _tx_state == TX_SENDING; }
  bool isHostOutputBackedUp() const {
    return _tx_count >= (KISS_TX_FRAME_QUEUE_DEPTH - 1) || _tx_busy_error_pending || _tx_done_pending;
  }
};
