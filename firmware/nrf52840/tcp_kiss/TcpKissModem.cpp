#include "TcpKissModem.h"
#include <math.h>
#include <string.h>

TcpKissModem::TcpKissModem(mesh::Radio& radio, mesh::MainBoard& board)
    : _radio(radio),
      _board(board),
      _prng_state(0xA5A55A5Au),
      _rx_len(0),
      _rx_escaped(false),
      _rx_active(false),
      _pending_tx_len(0),
      _has_pending_tx(false),
      _txdelay(KISS_DEFAULT_TXDELAY),
      _persistence(KISS_DEFAULT_PERSISTENCE),
      _slottime(KISS_DEFAULT_SLOTTIME),
      _txtail(0),
      _fullduplex(0),
      _tx_state(TX_IDLE),
      _tx_timer(0),
      _tx_head(0),
      _tx_tail(0),
      _tx_count(0),
      _tx_busy_error_pending(false),
      _tx_done_pending(false),
      _tx_done_result(0),
      _config{0, 0, 0, 0, 0},
      _signal_report_enabled(true),
      _setRadioCallback(nullptr),
      _setTxPowerCallback(nullptr),
      _getCurrentRssiCallback(nullptr),
      _getStatsCallback(nullptr),
      _rebootCallback(nullptr),
      _enterOtaCallback(nullptr),
      _active_client(nullptr),
      _pending_reboot(false),
      _pending_ota_dfu(false) {}

void TcpKissModem::begin(const KissRadioConfig& initial_config, uint32_t rng_seed) {
  _config = initial_config;
  _prng_state = rng_seed ? rng_seed : (micros() ^ 0x9E3779B9u);
  _has_pending_tx = false;
  _pending_tx_len = 0;
  _tx_state = TX_IDLE;
  _pending_reboot = false;
  _pending_ota_dfu = false;
  resetSession();
}

void TcpKissModem::resetSession() {
  _rx_len = 0;
  _rx_escaped = false;
  _rx_active = false;
  _signal_report_enabled = true;
  resetOutputQueue();
}

void TcpKissModem::resetOutputQueue() {
  _tx_head = 0;
  _tx_tail = 0;
  _tx_count = 0;
  _tx_busy_error_pending = false;
  if (_tx_state == TX_DONE_PENDING) {
    _tx_done_pending = false;
    _has_pending_tx = false;
    _tx_state = TX_IDLE;
  }
}

uint8_t TcpKissModem::nextRandomByte() {
  // xorshift32 seeded from SX1262 hardware noise
  uint32_t x = _prng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  _prng_state = x;
  return (uint8_t)(x & 0xFFu);
}

uint16_t TcpKissModem::appendEscapedByte(uint8_t* dest, uint16_t idx, uint16_t max_len, uint8_t b) {
  if (b == KISS_FEND || b == KISS_FESC) {
    if (idx + 2 > max_len) {
      return 0;
    }
    dest[idx++] = KISS_FESC;
    dest[idx++] = (b == KISS_FEND) ? KISS_TFEND : KISS_TFESC;
    return idx;
  }
  if (idx + 1 > max_len) {
    return 0;
  }
  dest[idx++] = b;
  return idx;
}

uint16_t TcpKissModem::encodeFrame(uint8_t type, const uint8_t* data, uint16_t len,
                                   uint8_t* dest, uint16_t max_len) {
  if (max_len < KISS_FRAME_BOUNDARY_BYTES) {
    return 0;
  }

  uint16_t idx = 0;
  dest[idx++] = KISS_FEND;

  idx = appendEscapedByte(dest, idx, max_len, type);
  if (idx == 0) {
    return 0;
  }

  for (uint16_t i = 0; i < len; i++) {
    idx = appendEscapedByte(dest, idx, max_len, data[i]);
    if (idx == 0) {
      return 0;
    }
  }

  if (idx + 1 > max_len) {
    return 0;
  }
  dest[idx++] = KISS_FEND;
  return idx;
}

bool TcpKissModem::tryFlushFrames(EthernetClient* client) {
  if (!client || !client->connected()) {
    return false;
  }

  while (_tx_count > 0) {
    const OutboundFrame& frame = _tx_queue[_tx_head];
    const int avail = client->availableForWrite();
    if (avail < (int)frame.len) {
      return false;
    }

    // Transmit entire assembled KISS frame in one contiguous SPI write, then flush.
    const size_t written = client->write(frame.data, frame.len);
    client->flush();
    if (written != frame.len) {
      return false;
    }

    _tx_head = (uint8_t)((_tx_head + 1) % KISS_TX_FRAME_QUEUE_DEPTH);
    _tx_count--;
  }
  return true;
}

bool TcpKissModem::queueFrame(uint8_t type, const uint8_t* data, uint16_t len, bool mark_busy_error) {
  if (_tx_count >= KISS_TX_FRAME_QUEUE_DEPTH && !tryFlushFrames(_active_client)) {
    if (mark_busy_error) {
      _tx_busy_error_pending = true;
    }
    return false;
  }

  OutboundFrame& slot = _tx_queue[_tx_tail];
  const uint16_t frame_len = encodeFrame(type, data, len, slot.data, sizeof(slot.data));
  if (frame_len == 0) {
    return false;
  }

  slot.len = frame_len;
  _tx_tail = (uint8_t)((_tx_tail + 1) % KISS_TX_FRAME_QUEUE_DEPTH);
  _tx_count++;
  tryFlushFrames(_active_client);
  return true;
}

bool TcpKissModem::queueHardwareFrame(uint8_t sub_cmd, const uint8_t* data, uint16_t len, bool mark_busy_error) {
  if (len > KISS_MAX_PACKET_SIZE) {
    return false;
  }
  _hw_payload[0] = sub_cmd;
  if (len > 0 && data != nullptr) {
    memcpy(_hw_payload + 1, data, len);
  }
  return queueFrame(KISS_CMD_SETHARDWARE, _hw_payload, (uint16_t)(len + 1), mark_busy_error);
}

bool TcpKissModem::queuePendingBusyError() {
  if (!_tx_busy_error_pending) {
    return true;
  }
  const uint8_t err = HW_ERR_TX_BUSY;
  if (!queueHardwareFrame(HW_RESP_ERROR, &err, 1, false)) {
    return false;
  }
  _tx_busy_error_pending = false;
  return true;
}

bool TcpKissModem::queuePendingTxDone() {
  if (!_tx_done_pending) {
    return true;
  }
  if (!queueHardwareFrame(HW_RESP_TX_DONE, &_tx_done_result, 1, false)) {
    return false;
  }
  _tx_done_pending = false;
  return true;
}

void TcpKissModem::setTxDonePending(uint8_t result) {
  _tx_done_result = result;
  _tx_done_pending = true;
  _tx_state = TX_DONE_PENDING;
}

void TcpKissModem::writeHardwareError(uint8_t error_code) {
  queueHardwareFrame(HW_RESP_ERROR, &error_code, 1, true);
}

void TcpKissModem::feedRxByte(uint8_t b) {
  if (b == KISS_FEND) {
    if (_rx_active && _rx_len > 0) {
      processFrame();
    }
    _rx_len = 0;
    _rx_escaped = false;
    _rx_active = true;
    return;
  }

  if (!_rx_active) {
    return;
  }

  if (b == KISS_FESC) {
    _rx_escaped = true;
    return;
  }

  if (_rx_escaped) {
    _rx_escaped = false;
    if (b == KISS_TFEND) {
      b = KISS_FEND;
    } else if (b == KISS_TFESC) {
      b = KISS_FESC;
    } else {
      return;
    }
  }

  if (_rx_len < KISS_MAX_FRAME_SIZE) {
    _rx_buf[_rx_len++] = b;
  } else {
    // Frame overflow without FEND; drop frame cleanly.
    _rx_len = 0;
    _rx_escaped = false;
    _rx_active = false;
  }
}

bool TcpKissModem::loop(EthernetClient* client) {
  _active_client = (client && client->connected()) ? client : nullptr;
  bool received_bytes = false;

  tryFlushFrames(_active_client);

  if (_active_client) {
    const int avail = _active_client->available();
    if (avail > 0) {
      uint8_t chunk[KISS_RX_CHUNK_SIZE];
      const size_t to_read = (avail < (int)sizeof(chunk)) ? (size_t)avail : sizeof(chunk);
      const int n = _active_client->read(chunk, to_read);
      if (n > 0) {
        received_bytes = true;
        for (int i = 0; i < n; i++) {
          feedRxByte(chunk[i]);
          if (_pending_ota_dfu || _pending_reboot) {
            break;
          }
        }
      }
    }
  }

  processTx();
  tryFlushFrames(_active_client);
  queuePendingBusyError();

  if (_pending_ota_dfu) {
    _pending_ota_dfu = false;
    tryFlushFrames(_active_client);
    if (_enterOtaCallback) {
      _enterOtaCallback();
    }
  } else if (_pending_reboot) {
    _pending_reboot = false;
    tryFlushFrames(_active_client);
    if (_rebootCallback) {
      _rebootCallback();
    } else {
      _board.reboot();
    }
  }

  return received_bytes;
}

void TcpKissModem::processFrame() {
  if (_rx_len < 1) {
    return;
  }

  const uint8_t type_byte = _rx_buf[0];
  if (type_byte == KISS_CMD_RETURN) {
    return;
  }

  const uint8_t port = (uint8_t)((type_byte >> 4) & 0x0Fu);
  const uint8_t cmd = (uint8_t)(type_byte & 0x0Fu);
  if (port != 0) {
    return;
  }

  const uint8_t* data = &_rx_buf[1];
  const uint16_t data_len = (uint16_t)(_rx_len - 1);

  switch (cmd) {
    case KISS_CMD_DATA:
      if (data_len > 0 && data_len <= KISS_MAX_PACKET_SIZE && !_has_pending_tx) {
        memcpy(_pending_tx, data, data_len);
        _pending_tx_len = data_len;
        _has_pending_tx = true;
      } else if (_has_pending_tx) {
        writeHardwareError(HW_ERR_TX_BUSY);
      }
      break;

    case KISS_CMD_TXDELAY:
      if (data_len >= 1) _txdelay = data[0];
      break;

    case KISS_CMD_PERSISTENCE:
      if (data_len >= 1) _persistence = data[0];
      break;

    case KISS_CMD_SLOTTIME:
      if (data_len >= 1) _slottime = data[0];
      break;

    case KISS_CMD_TXTAIL:
      if (data_len >= 1) _txtail = data[0];
      break;

    case KISS_CMD_FULLDUPLEX:
      if (data_len >= 1) _fullduplex = data[0];
      break;

    case KISS_CMD_SETHARDWARE:
      if (data_len >= 1) {
        handleHardwareCommand(data[0], data + 1, (uint16_t)(data_len - 1));
      }
      break;

    default:
      break;
  }
}

void TcpKissModem::handleHardwareCommand(uint8_t sub_cmd, const uint8_t* data, uint16_t len) {
  switch (sub_cmd) {
    case HW_CMD_GET_RANDOM:
      handleGetRandom(data, len);
      break;
    case HW_CMD_SET_RADIO:
      handleSetRadio(data, len);
      break;
    case HW_CMD_SET_TX_POWER:
      handleSetTxPower(data, len);
      break;
    case HW_CMD_GET_RADIO:
      handleGetRadio();
      break;
    case HW_CMD_GET_TX_POWER:
      handleGetTxPower();
      break;
    case HW_CMD_GET_CURRENT_RSSI:
      handleGetCurrentRssi();
      break;
    case HW_CMD_IS_CHANNEL_BUSY:
      handleIsChannelBusy();
      break;
    case HW_CMD_GET_AIRTIME:
      handleGetAirtime(data, len);
      break;
    case HW_CMD_GET_NOISE_FLOOR:
      handleGetNoiseFloor();
      break;
    case HW_CMD_GET_VERSION:
      handleGetVersion();
      break;
    case HW_CMD_GET_STATS:
      handleGetStats();
      break;
    case HW_CMD_GET_BATTERY:
      handleGetBattery();
      break;
    case HW_CMD_GET_MCU_TEMP:
      handleGetMCUTemp();
      break;
    case HW_CMD_GET_DEVICE_NAME:
      handleGetDeviceName();
      break;
    case HW_CMD_PING:
      handlePing();
      break;
    case HW_CMD_REBOOT:
      handleReboot();
      break;
    case HW_CMD_SET_SIGNAL_REPORT:
      handleSetSignalReport(data, len);
      break;
    case HW_CMD_GET_SIGNAL_REPORT:
      handleGetSignalReport();
      break;
    case HW_CMD_ENTER_OTA_DFU:
      handleEnterOtaDfu();
      break;
    case HW_CMD_GET_IDENTITY:
    case HW_CMD_VERIFY_SIGNATURE:
    case HW_CMD_SIGN_DATA:
    case HW_CMD_ENCRYPT_DATA:
    case HW_CMD_DECRYPT_DATA:
    case HW_CMD_KEY_EXCHANGE:
    case HW_CMD_HASH:
    case HW_CMD_GET_SENSORS:
      // Minimal modem: no on-node identity store, crypto offload, or I2C sensors.
      writeHardwareError(HW_ERR_NO_CALLBACK);
      break;
    default:
      writeHardwareError(HW_ERR_UNKNOWN_CMD);
      break;
  }
}

void TcpKissModem::processTx() {
  switch (_tx_state) {
    case TX_IDLE:
      if (_has_pending_tx) {
        _tx_timer = millis();
        _tx_state = _fullduplex ? TX_DELAY : TX_WAIT_CLEAR;
      }
      break;

    case TX_WAIT_CLEAR:
      if (!_radio.isReceiving()) {
        if (nextRandomByte() <= _persistence) {
          _tx_timer = millis();
          _tx_state = TX_DELAY;
        } else {
          _tx_timer = millis();
          _tx_state = TX_SLOT_WAIT;
        }
      } else {
        const uint32_t max_wait =
            (_radio.getEstAirtimeFor(KISS_MAX_PACKET_SIZE) * KISS_TX_TIMEOUT_NUM) / KISS_TX_TIMEOUT_DEN;
        if ((uint32_t)(millis() - _tx_timer) >= max_wait) {
          _tx_timer = millis();
          _tx_state = TX_DELAY;
        }
      }
      break;

    case TX_SLOT_WAIT:
      if ((uint32_t)(millis() - _tx_timer) >= (uint32_t)_slottime * 10u) {
        _tx_timer = millis();
        _tx_state = TX_WAIT_CLEAR;
      }
      break;

    case TX_DELAY:
      if ((uint32_t)(millis() - _tx_timer) >= (uint32_t)_txdelay * 10u) {
        if (_radio.startSendRaw(_pending_tx, _pending_tx_len)) {
          _tx_timer = millis();
          _tx_state = TX_SENDING;
        } else {
          setTxDonePending(0x00);
        }
      }
      break;

    case TX_SENDING:
      if (_radio.isSendComplete()) {
        _radio.onSendFinished();
        setTxDonePending(0x01);
      } else {
        const uint32_t tx_timeout =
            (_radio.getEstAirtimeFor(_pending_tx_len) * KISS_TX_TIMEOUT_NUM) / KISS_TX_TIMEOUT_DEN;
        if ((uint32_t)(millis() - _tx_timer) >= tx_timeout) {
          _radio.onSendFinished();
          setTxDonePending(0x00);
        }
      }
      break;

    case TX_DONE_PENDING:
      if (!_active_client) {
        // No host connected to receive TxDone; release radio TX slot.
        _tx_done_pending = false;
        _has_pending_tx = false;
        _tx_state = TX_IDLE;
      } else if (queuePendingTxDone()) {
        _has_pending_tx = false;
        _tx_state = TX_IDLE;
      }
      break;
  }
}

void TcpKissModem::onPacketReceived(int8_t snr_x4, int8_t rssi_dbm, const uint8_t* packet, uint16_t len) {
  if (!_active_client || len == 0 || len > KISS_MAX_PACKET_SIZE) {
    return;
  }
  if (queueFrame(KISS_CMD_DATA, packet, len) && _signal_report_enabled) {
    const uint8_t meta[2] = {(uint8_t)snr_x4, (uint8_t)rssi_dbm};
    queueHardwareFrame(HW_RESP_RX_META, meta, 2);
  }
}

void TcpKissModem::handleGetRandom(const uint8_t* data, uint16_t len) {
  if (len < 1) {
    writeHardwareError(HW_ERR_INVALID_LENGTH);
    return;
  }
  const uint8_t requested = data[0];
  if (requested < 1 || requested > 64) {
    writeHardwareError(HW_ERR_INVALID_PARAM);
    return;
  }
  uint8_t buf[64];
  for (uint8_t i = 0; i < requested; i++) {
    buf[i] = nextRandomByte();
  }
  queueHardwareFrame(HW_RESP(HW_CMD_GET_RANDOM), buf, requested);
}

void TcpKissModem::handleSetRadio(const uint8_t* data, uint16_t len) {
  if (len < 10) {
    writeHardwareError(HW_ERR_INVALID_LENGTH);
    return;
  }
  if (!_setRadioCallback) {
    writeHardwareError(HW_ERR_NO_CALLBACK);
    return;
  }

  uint32_t freq_hz = 0;
  uint32_t bw_hz = 0;
  memcpy(&freq_hz, data, 4);
  memcpy(&bw_hz, data + 4, 4);
  const uint8_t sf = data[8];
  const uint8_t cr = data[9];

  if (freq_hz < 150000000u || freq_hz > 960000000u ||
      bw_hz < 7800u || bw_hz > 500000u ||
      sf < 5 || sf > 12 ||
      cr < 5 || cr > 8) {
    writeHardwareError(HW_ERR_INVALID_PARAM);
    return;
  }

  _config.freq_hz = freq_hz;
  _config.bw_hz = bw_hz;
  _config.sf = sf;
  _config.cr = cr;

  _setRadioCallback((float)freq_hz / 1000000.0f, (float)bw_hz / 1000.0f, sf, cr);
  queueHardwareFrame(HW_RESP_OK, nullptr, 0);
}

void TcpKissModem::handleSetTxPower(const uint8_t* data, uint16_t len) {
  if (len < 1) {
    writeHardwareError(HW_ERR_INVALID_LENGTH);
    return;
  }
  if (!_setTxPowerCallback) {
    writeHardwareError(HW_ERR_NO_CALLBACK);
    return;
  }
  if (data[0] > 22) {
    writeHardwareError(HW_ERR_INVALID_PARAM);
    return;
  }

  _config.tx_power = data[0];
  _setTxPowerCallback(data[0]);
  queueHardwareFrame(HW_RESP_OK, nullptr, 0);
}

void TcpKissModem::handleGetRadio() {
  uint8_t buf[10];
  memcpy(buf, &_config.freq_hz, 4);
  memcpy(buf + 4, &_config.bw_hz, 4);
  buf[8] = _config.sf;
  buf[9] = _config.cr;
  queueHardwareFrame(HW_RESP(HW_CMD_GET_RADIO), buf, sizeof(buf));
}

void TcpKissModem::handleGetTxPower() {
  queueHardwareFrame(HW_RESP(HW_CMD_GET_TX_POWER), &_config.tx_power, 1);
}

void TcpKissModem::handleGetCurrentRssi() {
  if (!_getCurrentRssiCallback) {
    writeHardwareError(HW_ERR_NO_CALLBACK);
    return;
  }
  const int8_t rssi_byte = (int8_t)_getCurrentRssiCallback();
  queueHardwareFrame(HW_RESP(HW_CMD_GET_CURRENT_RSSI), (const uint8_t*)&rssi_byte, 1);
}

void TcpKissModem::handleIsChannelBusy() {
  const uint8_t busy = _radio.isReceiving() ? 0x01 : 0x00;
  queueHardwareFrame(HW_RESP(HW_CMD_IS_CHANNEL_BUSY), &busy, 1);
}

void TcpKissModem::handleGetAirtime(const uint8_t* data, uint16_t len) {
  if (len < 1) {
    writeHardwareError(HW_ERR_INVALID_LENGTH);
    return;
  }
  const uint32_t airtime = _radio.getEstAirtimeFor(data[0]);
  queueHardwareFrame(HW_RESP(HW_CMD_GET_AIRTIME), (const uint8_t*)&airtime, 4);
}

void TcpKissModem::handleGetNoiseFloor() {
  const int16_t noise_floor = (int16_t)_radio.getNoiseFloor();
  queueHardwareFrame(HW_RESP(HW_CMD_GET_NOISE_FLOOR), (const uint8_t*)&noise_floor, 2);
}

void TcpKissModem::handleGetVersion() {
  const uint8_t buf[2] = {KISS_FIRMWARE_VERSION, 0};
  queueHardwareFrame(HW_RESP(HW_CMD_GET_VERSION), buf, sizeof(buf));
}

void TcpKissModem::handleGetStats() {
  if (!_getStatsCallback) {
    writeHardwareError(HW_ERR_NO_CALLBACK);
    return;
  }
  uint32_t rx = 0, tx = 0, errors = 0;
  _getStatsCallback(&rx, &tx, &errors);
  uint8_t buf[12];
  memcpy(buf, &rx, 4);
  memcpy(buf + 4, &tx, 4);
  memcpy(buf + 8, &errors, 4);
  queueHardwareFrame(HW_RESP(HW_CMD_GET_STATS), buf, sizeof(buf));
}

void TcpKissModem::handleGetBattery() {
  const uint16_t mv = _board.getBattMilliVolts();
  queueHardwareFrame(HW_RESP(HW_CMD_GET_BATTERY), (const uint8_t*)&mv, 2);
}

void TcpKissModem::handleGetMCUTemp() {
  const float temp = _board.getMCUTemperature();
  if (isnan(temp)) {
    writeHardwareError(HW_ERR_NO_CALLBACK);
    return;
  }
  const int16_t temp_tenths = (int16_t)(temp * 10.0f);
  queueHardwareFrame(HW_RESP(HW_CMD_GET_MCU_TEMP), (const uint8_t*)&temp_tenths, 2);
}

void TcpKissModem::handleGetDeviceName() {
  const char* name = _board.getManufacturerName();
  queueHardwareFrame(HW_RESP(HW_CMD_GET_DEVICE_NAME), (const uint8_t*)name, (uint16_t)strlen(name));
}

void TcpKissModem::handlePing() {
  queueHardwareFrame(HW_RESP(HW_CMD_PING), nullptr, 0);
}

void TcpKissModem::handleReboot() {
  queueHardwareFrame(HW_RESP_OK, nullptr, 0);
  _pending_reboot = true;
}

void TcpKissModem::handleSetSignalReport(const uint8_t* data, uint16_t len) {
  if (len < 1) {
    writeHardwareError(HW_ERR_INVALID_LENGTH);
    return;
  }
  _signal_report_enabled = (data[0] != 0x00);
  const uint8_t val = _signal_report_enabled ? 0x01 : 0x00;
  queueHardwareFrame(HW_RESP(HW_CMD_GET_SIGNAL_REPORT), &val, 1);
}

void TcpKissModem::handleGetSignalReport() {
  const uint8_t val = _signal_report_enabled ? 0x01 : 0x00;
  queueHardwareFrame(HW_RESP(HW_CMD_GET_SIGNAL_REPORT), &val, 1);
}

void TcpKissModem::handleEnterOtaDfu() {
  queueHardwareFrame(HW_RESP_OK, nullptr, 0);
  _pending_ota_dfu = true;
}
