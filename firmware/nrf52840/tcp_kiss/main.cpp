#include <Arduino.h>
#include <SPI.h>
#include <RAK13800_W5100S.h>
#include <Dhcp.h>
#include <w5100.h>
#include <nrf.h>
#include <variant.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/CustomSX1262.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>

#include "MinimalRak4631Board.h"
#include "TcpKissModem.h"

// ----------------------------- Configuration ---------------------------------

#ifndef KISS_TCP_PORT
#define KISS_TCP_PORT 8001
#endif

#ifndef OTA_TCP_PORT
#define OTA_TCP_PORT 8002
#endif

#ifndef KISS_IDLE_TIMEOUT_MS
#define KISS_IDLE_TIMEOUT_MS 300000UL
#endif

#ifndef OTA_CLIENT_TIMEOUT_MS
#define OTA_CLIENT_TIMEOUT_MS 5000UL
#endif

#ifndef DHCP_TOTAL_TIMEOUT_MS
#define DHCP_TOTAL_TIMEOUT_MS 8000UL
#endif

#ifndef DHCP_RESPONSE_TIMEOUT_MS
#define DHCP_RESPONSE_TIMEOUT_MS 2000UL
#endif

#ifndef ETH_RETRY_INTERVAL_MS
#define ETH_RETRY_INTERVAL_MS 10000UL
#endif

#ifndef ETH_POLL_INTERVAL_MS
#define ETH_POLL_INTERVAL_MS 500UL
#endif

#ifndef HEALTH_CHECK_INTERVAL_MS
#define HEALTH_CHECK_INTERVAL_MS 1000UL
#endif

#ifndef NOISE_FLOOR_CALIB_INTERVAL_MS
#define NOISE_FLOOR_CALIB_INTERVAL_MS 2000UL
#endif

#ifndef AGC_RESET_INTERVAL_MS
#define AGC_RESET_INTERVAL_MS 30000UL
#endif

#ifndef DFU_MAGIC_OTA_RESET
#define DFU_MAGIC_OTA_RESET 0xB1u
#endif

#ifndef WDT_TIMEOUT_SECONDS
#define WDT_TIMEOUT_SECONDS 60UL
#endif

// ----------------------------- Hardware Objects ------------------------------

MinimalRak4631Board board;

// Dedicated SPIM3 peripheral for internal RAK4631 SX1262 LoRa pins
// (MISO=45, SCK=43, MOSI=44, NSS=42, DIO1=47, RESET=38, BUSY=46).
// variants/rak4631/variant.h sets SPI_INTERFACES_COUNT=1 (no SPI1 object);
// global SPI is never initialized so NRF_SPIM3 belongs exclusively to loraSpi.
SPIClass loraSpi(NRF_SPIM3, P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
CustomSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, loraSpi);
CustomSX1262Wrapper radio_driver(radio, board);

// Dedicated SPIM2 peripheral for RAK13800 W5100S on WisBlock IO header
// (MISO=29, SCK=3, MOSI=30, CS=WB_SPI_CS=26). Both NRFX_SPIM2 and NRFX_SPIM3
// are enabled in nrfx_config.h (SPIM0/SPIM1 are reserved for I2C TWIM0/TWIM1).
SPIClass ethSpi(NRF_SPIM2, WB_SPI_MISO, WB_SPI_CLK, WB_SPI_MOSI);

EthernetServer kissServer(KISS_TCP_PORT);
EthernetServer otaServer(OTA_TCP_PORT);
EthernetClient kissClient;
EthernetClient otaClient;

TcpKissModem modem(radio_driver, board);

// ----------------------------- Runtime State ---------------------------------

static uint8_t ethMac[6] = {0};
static bool ethReady = false;
static bool ethUsingDhcp = false;
static bool linkWasOn = false;
static bool linkLostPendingRecovery = false;
static bool serversStarted = false;

static bool kissClientActive = false;
static uint32_t kissClientLastRxMs = 0;

static bool otaClientActive = false;
static uint32_t otaClientConnectedMs = 0;
static char otaCmdBuf[32] = {0};
static size_t otaCmdLen = 0;

static uint32_t lastEthPollMs = 0;
static uint32_t lastEthRetryMs = 0;
static uint32_t lastHealthCheckMs = 0;
static uint32_t nextNoiseFloorCalibMs = 0;
static uint32_t nextAgcResetMs = 0;

static bool radioSpiHealthy = true;
static bool ethSpiHealthy = true;

// ----------------------------- Watchdog (NRF_WDT) ----------------------------

static void wdtInit() {
  if (!(NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)) {
    // Pause during SWD debug halt; keep running during CPU sleep/delay so
    // any firmware deadlock still recovers automatically.
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
                      (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = (WDT_TIMEOUT_SECONDS * 32768UL) - 1UL;
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
  }
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

static inline void wdtFeed() {
  if (NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  }
}

// ----------------------------- Helper Functions ------------------------------

static void deriveDeterministicMac(uint8_t mac[6]) {
  const uint32_t addr0 = NRF_FICR->DEVICEADDR[0];
  const uint32_t addr1 = NRF_FICR->DEVICEADDR[1];
  mac[0] = (uint8_t)((((addr1 >> 8) & 0xFFu) | 0x02u) & ~0x01u);  // Locally administered, unicast
  mac[1] = (uint8_t)(addr1 & 0xFFu);
  mac[2] = (uint8_t)((addr0 >> 24) & 0xFFu);
  mac[3] = (uint8_t)((addr0 >> 16) & 0xFFu);
  mac[4] = (uint8_t)((addr0 >> 8) & 0xFFu);
  mac[5] = (uint8_t)(addr0 & 0xFFu);
}

static const char* hardwareStatusStr(EthernetHardwareStatus st) {
  switch (st) {
    case EthernetNoHardware: return "EthernetNoHardware";
    case EthernetW5100:      return "W5100/W5100S";
    case EthernetW5100S:     return "W5100S";
    case EthernetW5200:      return "W5200";
    case EthernetW5500:      return "W5500";
    default:                 return "Unknown";
  }
}

static const char* linkStatusStr(EthernetLinkStatus st) {
  switch (st) {
    case LinkON:  return "LinkON";
    case LinkOFF: return "LinkOFF";
    default:      return "Unknown";
  }
}

static void printIp(const IPAddress& ip) {
  Serial.printf("%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

// Live SPI probe of W5100S VERSIONR register (0x0080 == 0x51).
static bool probeW5100sSpi() {
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    return false;
  }
  W5100.getSPI()->beginTransaction(SPI_ETHERNET_SETTINGS);
  const uint8_t ver = W5100.readVERSIONR_W5100s();
  W5100.getSPI()->endTransaction();
  return ver == 0x51;
}

// Direct BUSY-aware SX1262 GetStatus (0xC0) SPI transfer.
// RadioLib 7.7.1 SX126x::getStatus() passes numBytes=0 to SPIreadStream(),
// which copies 0 bytes and always returns 0x00; query 0xC0 directly on loraSpi.
static bool readSx1262StatusRaw(uint8_t* outStatus) {
  const uint32_t waitStart = millis();
  while (digitalRead(P_LORA_BUSY)) {
    if ((uint32_t)(millis() - waitStart) >= 50UL) {
      *outStatus = 0x00u;
      return false;
    }
    yield();
  }

  uint8_t tx[2] = {RADIOLIB_SX126X_CMD_GET_STATUS, RADIOLIB_SX126X_CMD_NOP};
  uint8_t rx[2] = {0x00u, 0x00u};
  loraSpi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(P_LORA_NSS, LOW);
  loraSpi.transfer(tx, rx, sizeof(tx));
  digitalWrite(P_LORA_NSS, HIGH);
  loraSpi.endTransaction();

  *outStatus = rx[1];
  return true;
}

// Live SPI probe of SX1262 GetStatus (chip mode bits [6:4] must be 2..6).
static bool probeSx1262Spi(uint8_t* outStatus = nullptr) {
  uint8_t st = 0;
  if (!readSx1262StatusRaw(&st)) {
    if (outStatus) {
      *outStatus = 0x00u;
    }
    return false;
  }
  if (outStatus) {
    *outStatus = st;
  }
  const uint8_t chipMode = (uint8_t)((st >> 4) & 0x07u);
  return (st != 0x00u && st != 0xFFu && chipMode >= 2u && chipMode <= 6u);
}

static bool initSx1262Radio() {
  loraSpi.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  loraSpi.begin();

  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
  int16_t status = radio.begin(
      LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
      RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo, false);
  Serial.printf("[RADIO] radio.begin(tcxo=1.8V) -> %d\n", (int)status);

  if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
    tcxo = 0.0f;
    status = radio.begin(
        LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo, false);
    Serial.printf("[RADIO] radio.begin(tcxo=0.0V fallback) -> %d\n", (int)status);
  }

  uint8_t rawStatus = 0;
  const bool statusOk = readSx1262StatusRaw(&rawStatus);
  Serial.printf("[RADIO] Raw GetStatus(0xC0)=0x%02X (mode=%u, busy=%d, ok=%u) DeviceErrors=0x%04X\n",
                (unsigned)rawStatus,
                (unsigned)((rawStatus >> 4) & 0x07u),
                (int)digitalRead(P_LORA_BUSY),
                statusOk ? 1u : 0u,
                (unsigned)radio.getDeviceErrors());

  if (status != RADIOLIB_ERR_NONE) {
    return false;
  }

  radio.setCRC(1);
#ifdef SX126X_CURRENT_LIMIT
  radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
#endif
#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio.setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif
#ifdef SX126X_RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(SX126X_RX_BOOSTED_GAIN);
#endif
  return true;
}

static void stopActiveClients(const char* reason) {
  if (kissClientActive || kissClient) {
    if (reason) {
      Serial.printf("[KISS] Closing client (%s)\n", reason);
    }
    kissClient.stop();
    kissClient = EthernetClient();
    kissClientActive = false;
    modem.resetSession();
  }
  if (otaClientActive || otaClient) {
    otaClient.stop();
    otaClient = EthernetClient();
    otaClientActive = false;
    otaCmdLen = 0;
  }
}

static void clearServerPortTable() {
  for (uint8_t i = 0; i < MAX_SOCK_NUM; i++) {
    EthernetServer::server_port[i] = 0;
  }
}

static void ethernetHardwareReset() {
  wdtFeed();
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);  // Enable 3V3_S power to RAK13800
  delay(50);
  pinMode(WB_IO3, OUTPUT);
  digitalWrite(WB_IO3, LOW);   // Assert W5100S hardware reset
  delay(100);
  digitalWrite(WB_IO3, HIGH);
  delay(100);
  wdtFeed();
}

static void startTcpServers() {
  clearServerPortTable();
  kissServer.begin();
  otaServer.begin();
  serversStarted = true;
  Serial.printf("[ETH] Listening on TCP port %u (KISS) and %u (OTA)\n",
                (unsigned)KISS_TCP_PORT, (unsigned)OTA_TCP_PORT);
}

static void ensureServersListening() {
  if (!ethReady) {
    return;
  }
  if (!kissServer) {
    kissServer.begin();
  }
  if (!otaServer) {
    otaServer.begin();
  }
}

static bool bringUpEthernet(bool doHardwareReset) {
  stopActiveClients("Ethernet reinit");
  serversStarted = false;
  ethReady = false;
  ethUsingDhcp = false;

  if (doHardwareReset) {
    ethernetHardwareReset();
  }

  ethSpi.begin();
  Ethernet.init(ethSpi, WB_SPI_CS);

  // If W5100.init() already ran once, its static flag skips TMSR/RMSR writes
  // after a subsequent WB_IO3 reset; restore default 2KBx4 socket buffers.
  if (Ethernet.hardwareStatus() != EthernetNoHardware) {
    W5100.getSPI()->beginTransaction(SPI_ETHERNET_SETTINGS);
    W5100.writeMR(0x00);
    W5100.writeTMSR(0x55);
    W5100.writeRMSR(0x55);
    W5100.getSPI()->endTransaction();
  }

  Serial.printf("[ETH] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                ethMac[0], ethMac[1], ethMac[2], ethMac[3], ethMac[4], ethMac[5]);

  wdtFeed();
  Serial.println("[ETH] Starting DHCP...");
  const int dhcpOk = Ethernet.begin(ethMac, DHCP_TOTAL_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS);
  wdtFeed();

  const EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  const EthernetLinkStatus link = Ethernet.linkStatus();
  Serial.printf("[ETH] Hardware: %s, Link: %s\n", hardwareStatusStr(hw), linkStatusStr(link));

  ethSpiHealthy = (hw != EthernetNoHardware) && probeW5100sSpi();
  if (!ethSpiHealthy) {
    Serial.println("[ETH] ERROR: W5100S not responding on SPI");
    return false;
  }

  if (dhcpOk == 1) {
    ethUsingDhcp = true;
    ethReady = true;
  } else {
    Serial.println("[ETH] DHCP timed out");
#if defined(ETHERNET_STATIC_IP)
    IPAddress staticIp(ETHERNET_STATIC_IP);
    Serial.print("[ETH] Falling back to static IP ");
    printIp(staticIp);
    Serial.println();
  #if defined(ETHERNET_STATIC_DNS) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET)
    Ethernet.begin(ethMac, staticIp, IPAddress(ETHERNET_STATIC_DNS),
                   IPAddress(ETHERNET_STATIC_GATEWAY), IPAddress(ETHERNET_STATIC_SUBNET));
  #elif defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET)
    IPAddress gw(ETHERNET_STATIC_GATEWAY);
    Ethernet.begin(ethMac, staticIp, gw, gw, IPAddress(ETHERNET_STATIC_SUBNET));
  #else
    Ethernet.begin(ethMac, staticIp);
  #endif
    ethUsingDhcp = false;
    ethReady = true;
#else
    return false;
#endif
  }

  // Bound TCP retransmission stall so flush() cannot exceed the WDT window.
  Ethernet.setRetransmissionTimeout(200);
  Ethernet.setRetransmissionCount(4);

  Serial.print("[ETH] IPv4: ");
  printIp(Ethernet.localIP());
  Serial.print("  Subnet: ");
  printIp(Ethernet.subnetMask());
  Serial.print("  Gateway: ");
  printIp(Ethernet.gatewayIP());
  Serial.println();

  linkWasOn = (Ethernet.linkStatus() == LinkON);
  linkLostPendingRecovery = false;
  startTcpServers();
  return true;
}

// ----------------------------- OTA & Reboot Triggers -------------------------

static void enterBleOtaDfu() {
  Serial.println("Entering BLE OTA DFU mode...");
  Serial.flush();

  stopActiveClients(nullptr);
  delay(20);

  // Feed watchdog one final time before resetting into the BLE DFU bootloader.
  wdtFeed();

  NRF_POWER->GPREGRET = DFU_MAGIC_OTA_RESET;
  __DSB();
  NVIC_SystemReset();
  while (true) {}
}

static void cleanReboot() {
  Serial.println("[SYS] Reboot requested via KISS; restarting...");
  Serial.flush();
  stopActiveClients(nullptr);
  delay(20);
  wdtFeed();
  NVIC_SystemReset();
  while (true) {}
}

// ----------------------------- KISS Radio Callbacks --------------------------

static void onSetRadio(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) {
  radio_driver.setParams(freq_mhz, bw_khz, sf, cr);
  const uint32_t freq_khz = (uint32_t)(freq_mhz * 1000.0f + 0.5f);
  const uint32_t bw_tenths_khz = (uint32_t)(bw_khz * 10.0f + 0.5f);
  Serial.printf("[RADIO] SetRadio: %lu.%03lu MHz, BW %lu.%lu kHz, SF%u, CR%u\n",
                (unsigned long)(freq_khz / 1000UL), (unsigned long)(freq_khz % 1000UL),
                (unsigned long)(bw_tenths_khz / 10UL), (unsigned long)(bw_tenths_khz % 10UL),
                (unsigned)sf, (unsigned)cr);
}

static void onSetTxPower(uint8_t power_dbm) {
  radio_driver.setTxPower((int8_t)power_dbm);
  Serial.printf("[RADIO] SetTxPower: %u dBm\n", (unsigned)power_dbm);
}

static float onGetCurrentRssi() {
  return radio_driver.getCurrentRSSI();
}

static void onGetStats(uint32_t* rx, uint32_t* tx, uint32_t* errors) {
  *rx = radio_driver.getPacketsRecv();
  *tx = radio_driver.getPacketsSent();
  *errors = radio_driver.getPacketsRecvErrors();
}

// ----------------------------- TCP Polling -----------------------------------

static void pollEthernetLinkAndDhcp(uint32_t now) {
  if (ethReady && ethUsingDhcp) {
    const int rc = Ethernet.maintain();
    if (rc == DHCP_CHECK_RENEW_OK || rc == DHCP_CHECK_REBIND_OK) {
      Serial.print("[ETH] DHCP lease renewed: ");
      printIp(Ethernet.localIP());
      Serial.println();
    } else if (rc == DHCP_CHECK_RENEW_FAIL || rc == DHCP_CHECK_REBIND_FAIL) {
      Serial.printf("[ETH] DHCP maintain failed (%d); will recover on next cycle\n", rc);
      ethReady = false;
      lastEthRetryMs = now;
    }
  }

  if ((uint32_t)(now - lastEthPollMs) < ETH_POLL_INTERVAL_MS) {
    return;
  }
  lastEthPollMs = now;

  // Check for hardware loss (cached or live SPI read).
  if (Ethernet.hardwareStatus() == EthernetNoHardware || !probeW5100sSpi()) {
    Serial.println("[ETH] W5100S hardware unresponsive; toggling WB_IO3 reset...");
    ethSpiHealthy = false;
    if (!bringUpEthernet(true)) {
      lastEthRetryMs = millis();
    }
    return;
  }
  ethSpiHealthy = true;

  const EthernetLinkStatus link = Ethernet.linkStatus();
  if (link == LinkOFF) {
    if (linkWasOn) {
      Serial.println("[ETH] Physical link down (LinkOFF)");
      stopActiveClients("link down");
      linkWasOn = false;
      linkLostPendingRecovery = true;
    }
  } else if (link == LinkON) {
    if (linkLostPendingRecovery) {
      Serial.println("[ETH] Physical link restored (LinkON); resetting W5100S and renewing network...");
      linkLostPendingRecovery = false;
      if (!bringUpEthernet(true)) {
        lastEthRetryMs = millis();
      }
      return;
    }
    linkWasOn = true;
  }

  // Retry initial bring-up if DHCP had timed out at boot and no static IP fallback is configured.
  if (!ethReady && link == LinkON && (uint32_t)(now - lastEthRetryMs) >= ETH_RETRY_INTERVAL_MS) {
    lastEthRetryMs = now;
    bringUpEthernet(true);
  }
}

static void pollKissClient(uint32_t now) {
  if (!ethReady) {
    return;
  }

  // Accept incoming connection; immediately evict any existing client.
  EthernetClient incoming = kissServer.accept();
  if (incoming) {
    const IPAddress rip = incoming.remoteIP();
    const uint16_t rport = incoming.remotePort();
    if (kissClientActive || kissClient) {
      Serial.println("[KISS] New client arrived; closing previous client");
      kissClient.stop();
      modem.resetSession();
    }
    kissClient = incoming;
    kissClient.setConnectionTimeout(250);
    kissClientActive = true;
    kissClientLastRxMs = now;
    Serial.printf("[KISS] Client connected: %u.%u.%u.%u:%u (sock %u)\n",
                  rip[0], rip[1], rip[2], rip[3], (unsigned)rport,
                  (unsigned)kissClient.getSocketNumber());
  }

  if (!kissClientActive) {
    ensureServersListening();
    return;
  }

  // Run non-blocking KISS RX/TX before checking CLOSE_WAIT so final bytes aren't lost.
  const bool gotBytes = modem.loop(&kissClient);
  if (gotBytes) {
    kissClientLastRxMs = now;
  }

  const uint8_t st = kissClient.status();
  const bool closing = (st == SnSR::CLOSED || st == SnSR::FIN_WAIT || st == SnSR::CLOSING ||
                        st == SnSR::TIME_WAIT || st == SnSR::LAST_ACK ||
                        (st == SnSR::CLOSE_WAIT && kissClient.available() == 0));

  if (closing || !kissClient.connected()) {
    Serial.printf("[KISS] Client disconnected (SnSR=0x%02X); reclaiming socket\n", (unsigned)st);
    kissClient.stop();
    kissClient = EthernetClient();
    kissClientActive = false;
    modem.resetSession();
    ensureServersListening();
    return;
  }

  if ((uint32_t)(now - kissClientLastRxMs) >= KISS_IDLE_TIMEOUT_MS) {
    Serial.printf("[KISS] Idle timeout (%lus); reclaiming half-open socket\n",
                  (unsigned long)(KISS_IDLE_TIMEOUT_MS / 1000UL));
    kissClient.stop();
    kissClient = EthernetClient();
    kissClientActive = false;
    modem.resetSession();
    ensureServersListening();
  }
}

static void pollOtaControl(uint32_t now) {
  if (!ethReady) {
    return;
  }

  EthernetClient incoming = otaServer.accept();
  if (incoming) {
    if (otaClientActive || otaClient) {
      otaClient.stop();
    }
    otaClient = incoming;
    otaClient.setConnectionTimeout(250);
    otaClientActive = true;
    otaClientConnectedMs = now;
    otaCmdLen = 0;
    const IPAddress rip = otaClient.remoteIP();
    Serial.printf("[OTA] Control client connected: %u.%u.%u.%u:%u\n",
                  rip[0], rip[1], rip[2], rip[3], (unsigned)otaClient.remotePort());
  }

  if (!otaClientActive) {
    return;
  }

  const int avail = otaClient.available();
  if (avail > 0) {
    uint8_t buf[32];
    const size_t toRead = (avail < (int)sizeof(buf)) ? (size_t)avail : sizeof(buf);
    const int n = otaClient.read(buf, toRead);
    for (int i = 0; i < n; i++) {
      const char c = (char)buf[i];
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        otaCmdBuf[otaCmdLen] = '\0';
        if (strcmp(otaCmdBuf, "START_OTA") == 0) {
          static const char kAck[] = "OK Entering BLE OTA DFU mode\n";
          otaClient.write((const uint8_t*)kAck, sizeof(kAck) - 1);
          otaClient.flush();
          delay(20);
          enterBleOtaDfu();
        } else {
          static const char kErr[] = "ERR Unknown command\n";
          otaClient.write((const uint8_t*)kErr, sizeof(kErr) - 1);
          otaClient.flush();
          otaClient.stop();
          otaClient = EthernetClient();
          otaClientActive = false;
          otaCmdLen = 0;
          ensureServersListening();
          return;
        }
      } else if (otaCmdLen + 1 < sizeof(otaCmdBuf)) {
        otaCmdBuf[otaCmdLen++] = c;
      } else {
        // Line too long; drop immediately to free W5100S socket.
        otaClient.stop();
        otaClient = EthernetClient();
        otaClientActive = false;
        otaCmdLen = 0;
        ensureServersListening();
        return;
      }
    }
  }

  const uint8_t st = otaClient.status();
  const bool closing = (st == SnSR::CLOSED || st == SnSR::FIN_WAIT || st == SnSR::CLOSING ||
                        st == SnSR::TIME_WAIT || st == SnSR::LAST_ACK ||
                        (st == SnSR::CLOSE_WAIT && otaClient.available() == 0));

  if (closing || !otaClient.connected() ||
      (uint32_t)(now - otaClientConnectedMs) >= OTA_CLIENT_TIMEOUT_MS) {
    otaClient.stop();
    otaClient = EthernetClient();
    otaClientActive = false;
    otaCmdLen = 0;
    ensureServersListening();
  }
}

// ----------------------------- Arduino Entry Points --------------------------

void setup() {
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && (uint32_t)(millis() - serialStart) < 2000UL) {
    delay(10);
  }

  const uint32_t resetReas = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = resetReas;  // Write-1-to-clear

  Serial.println();
  Serial.println("=== RAK4631 + RAK13800 Ethernet TCP KISS Modem ===");
  Serial.printf("[SYS] RESETREAS=0x%08lX  WDT=%lus\n",
                (unsigned long)resetReas, (unsigned long)WDT_TIMEOUT_SECONDS);

  wdtInit();
  board.begin();

  // Initialize SX1262 LoRa radio on dedicated loraSpi (NRF_SPIM3 -> internal LoRa pins).
  if (!initSx1262Radio()) {
    Serial.println("[RADIO] FATAL: SX1262 init failed; waiting for WDT reset");
    Serial.flush();
    while (true) {
      delay(100);
    }
  }
  radio_driver.begin();
  wdtFeed();

  const KissRadioConfig initialRadio = {
      (uint32_t)(LORA_FREQ * 1000000.0 + 0.5),
      (uint32_t)(LORA_BW * 1000.0 + 0.5),
      (uint8_t)LORA_SF,
      (uint8_t)LORA_CR,
      (uint8_t)LORA_TX_POWER
  };
  Serial.printf("[RADIO] SX1262 ready: %lu.%03lu MHz, BW %lu.%lu kHz, SF%u, CR%u, %u dBm\n",
                (unsigned long)(initialRadio.freq_hz / 1000000UL),
                (unsigned long)((initialRadio.freq_hz / 1000UL) % 1000UL),
                (unsigned long)(initialRadio.bw_hz / 1000UL),
                (unsigned long)((initialRadio.bw_hz % 1000UL) / 100UL),
                (unsigned)LORA_SF, (unsigned)LORA_CR, (unsigned)LORA_TX_POWER);

  modem.setRadioCallback(onSetRadio);
  modem.setTxPowerCallback(onSetTxPower);
  modem.setGetCurrentRssiCallback(onGetCurrentRssi);
  modem.setGetStatsCallback(onGetStats);
  modem.setRebootCallback(cleanReboot);
  modem.setEnterOtaCallback(enterBleOtaDfu);
  modem.begin(initialRadio, radio_driver.getRngSeed());

  // Derive deterministic MAC and initialize RAK13800 W5100S over NRF_SPIM2.
  deriveDeterministicMac(ethMac);
  if (!bringUpEthernet(true)) {
    lastEthRetryMs = millis();
  }

  const uint32_t now = millis();
  lastEthPollMs = now;
  lastHealthCheckMs = now;
  nextNoiseFloorCalibMs = now + NOISE_FLOOR_CALIB_INTERVAL_MS;
  nextAgcResetMs = now + AGC_RESET_INTERVAL_MS;
}

void loop() {
  const uint32_t now = millis();

  pollEthernetLinkAndDhcp(now);
  pollOtaControl(now);
  pollKissClient(now);

  // If no KISS client is connected, still advance any in-flight SX1262 TX state.
  if (!kissClientActive) {
    modem.loop(nullptr);
  }

  // Receive LoRa packets whenever SX1262 is not actively transmitting.
  if (!modem.isActuallyTransmitting() && !modem.isHostOutputBackedUp()) {
    if (!modem.isTxBusy() && (uint32_t)(now - nextAgcResetMs) >= AGC_RESET_INTERVAL_MS) {
      radio_driver.resetAGC();
      nextAgcResetMs = now;
    }

    uint8_t rx_buf[KISS_MAX_PACKET_SIZE + 1];
    const int rx_len = radio_driver.recvRaw(rx_buf, KISS_MAX_PACKET_SIZE);
    if (rx_len > 0 && kissClientActive) {
      const int8_t snr_x4 = (int8_t)(radio_driver.getLastSNR() * 4.0f);
      const int8_t rssi_dbm = (int8_t)radio_driver.getLastRSSI();
      modem.onPacketReceived(snr_x4, rssi_dbm, rx_buf, (uint16_t)rx_len);
    }
  }

  if ((uint32_t)(now - nextNoiseFloorCalibMs) >= NOISE_FLOOR_CALIB_INTERVAL_MS) {
    radio_driver.triggerNoiseFloorCalibrate(0);
    nextNoiseFloorCalibMs = now;
  }
  radio_driver.loop();

  // Periodically verify both SPI buses (SX1262 and W5100S) respond validly.
  if ((uint32_t)(now - lastHealthCheckMs) >= HEALTH_CHECK_INTERVAL_MS) {
    lastHealthCheckMs = now;
    if (!modem.isActuallyTransmitting()) {
      uint8_t rawStatus = 0;
      radioSpiHealthy = probeSx1262Spi(&rawStatus);
      if (!radioSpiHealthy) {
        Serial.printf("[WDT] SX1262 SPI unresponsive (GetStatus=0x%02X, busy=%d); withholding WDT feed\n",
                      (unsigned)rawStatus, (int)digitalRead(P_LORA_BUSY));
      }
    }
    ethSpiHealthy = probeW5100sSpi();
    if (!ethSpiHealthy) {
      Serial.println("[WDT] W5100S SPI unresponsive; withholding WDT feed");
    }
  }

  if (radioSpiHealthy && ethSpiHealthy) {
    wdtFeed();
  }

  delay(1);
}
