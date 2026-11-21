#pragma once

#include <Arduino.h>
#include <MeshCore.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <variant.h>

#define PIN_VBAT_READ 5
#define ADC_MULTIPLIER (3.0f * 1.73f * 1.187f * 1000.0f)

// Drive WB_IO2 (P1.02, Arduino pin 34) HIGH immediately after SystemInit so the
// WisBlock 3V3_S peripheral rail and RAK13800 PoE power path stay stable.
static void __attribute__((constructor(102))) rak4631_tcp_kiss_early_poe_power() {
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 2));
  nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 2));
}

// Minimal board implementation for RAK4631 KISS Ethernet modem:
// zero BLE stack, zero InternalFS, zero I2C/display/sensor drivers.
class MinimalRak4631Board : public mesh::MainBoard {
  static constexpr int BATTERY_SAMPLES = 8;

public:
  void begin() {
    // Enable internal nRF52840 DC/DC regulator (SoftDevice is not active).
    NRF_POWER->DCDCEN = 1;

    pinMode(PIN_VBAT_READ, INPUT);

    // Deassert SX1262 CS, power on SX126X_POWER_EN (37), and pulse P_LORA_RESET (38)
    // cleanly before SPI and RadioLib initialize the transceiver.
    pinMode(P_LORA_NSS, OUTPUT);
    digitalWrite(P_LORA_NSS, HIGH);
    pinMode(P_LORA_BUSY, INPUT);

    pinMode(SX126X_POWER_EN, OUTPUT);
    digitalWrite(SX126X_POWER_EN, HIGH);
    delay(20);

    pinMode(P_LORA_RESET, OUTPUT);
    digitalWrite(P_LORA_RESET, LOW);
    delay(10);
    digitalWrite(P_LORA_RESET, HIGH);
    delay(20);

    const uint32_t busyStart = millis();
    while (digitalRead(P_LORA_BUSY) && (uint32_t)(millis() - busyStart) < 50UL) {
      delay(1);
    }
  }

  uint16_t getBattMilliVolts() override {
    analogReadResolution(12);
    uint32_t raw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
      raw += (uint32_t)analogRead(PIN_VBAT_READ);
    }
    raw /= BATTERY_SAMPLES;
    return (uint16_t)((ADC_MULTIPLIER * raw) / 4096.0f);
  }

  float getMCUTemperature() override {
    NRF_TEMP->EVENTS_DATARDY = 0;
    NRF_TEMP->TASKS_START = 1;
    const uint32_t start = millis();
    while (NRF_TEMP->EVENTS_DATARDY == 0) {
      if ((uint32_t)(millis() - start) > 5) {
        NRF_TEMP->TASKS_STOP = 1;
        return NAN;
      }
    }
    NRF_TEMP->EVENTS_DATARDY = 0;
    const int32_t raw = NRF_TEMP->TEMP;
    NRF_TEMP->TASKS_STOP = 1;
    return (float)raw * 0.25f;
  }

  const char* getManufacturerName() const override {
    return "RAK 4631";
  }

  uint8_t getStartupReason() const override {
    return BD_STARTUP_NORMAL;
  }

  void reboot() override {
    if (NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk) {
      NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    }
    NVIC_SystemReset();
  }
};
