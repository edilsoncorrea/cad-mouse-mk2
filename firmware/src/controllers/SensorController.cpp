/**
 * SensorController — TLI493D-A1B6 (geração 1)
 *
 * Controla 3 sensores magnéticos Hall para o mouse 3D.
 * Os sensores são TLI493D-A1B6 (gen 1), NÃO A2B6 (gen 2).
 *
 * Diferenças principais em relação ao A2B6:
 *   - Mapa de endereços I2C completamente diferente (gen 1 vs gen 2)
 *   - setSensitivity() NÃO é suportado — opera apenas em full range
 *   - Fator de conversão magnético: 0.098 mT/LSB (full range fixo)
 *
 * Endereços I2C utilizados (grupo SDA-baixo, A4–A7):
 *   A4 = 0x3E (8-bit) / 0x1F (7-bit) — padrão ao ligar
 *   A5 = 0x36 (8-bit) / 0x1B (7-bit)
 *   A6 = 0x1E (8-bit) / 0x0F (7-bit)
 *   A7 = 0x16 (8-bit) / 0x0B (7-bit) — reservado
 *
 * Atribuição neste projeto:
 *   MAG1 (D10) → A6 (0x0F)
 *   MAG2 (D9)  → A5 (0x1B)
 *   MAG3 (D8)  → A4 (0x1F) — permanece no endereço padrão
 */

#include "controllers/SensorController.h"

#include "Config.h"

using namespace ifx::tlx493d;

SensorController::SensorController()
    : mag1Sensor_(Wire, TLx493D_IIC_ADDR_A4_e),
      mag2Sensor_(Wire, TLx493D_IIC_ADDR_A4_e),
      mag3Sensor_(Wire, TLx493D_IIC_ADDR_A4_e) {}

// PMOS high-side: gate LOW = ON (Vgs=-3.3V, conduz), gate HIGH = OFF (Vgs=0)
void SensorController::powerOff(int pin) { digitalWrite(pin, HIGH); }

void SensorController::powerOn(int pin) {
  digitalWrite(pin, LOW);
  delay(5);
}

void SensorController::begin() {

  pinMode(Config::PIN_MAG1_LS, OUTPUT);
  pinMode(Config::PIN_MAG2_LS, OUTPUT);
  pinMode(Config::PIN_MAG3_LS, OUTPUT);

  // PMOS high-side: pull-up externo no gate garante OFF no boot.
  // Force all OFF (HIGH) before sequential power-on for address assignment.

  powerOff(Config::PIN_MAG1_LS);
  powerOff(Config::PIN_MAG2_LS);
  powerOff(Config::PIN_MAG3_LS);
  delay(5);

  Wire.begin();
  Wire.setClock(400000);

  powerOn(Config::PIN_MAG1_LS);
  mag1Sensor_.begin(true, false, false, true);
#ifdef SENSOR_ADDR_RETRY
  if (!mag1Sensor_.setIICAddress(TLx493D_IIC_ADDR_A6_e)) {
    // Sensor may have been at a persisted address; begin() resets to A4.
    // Retry begin() + setIICAddress() on the default address.
    delay(50);
    mag1Sensor_.begin(true, false, false, true);
    mag1Sensor_.setIICAddress(TLx493D_IIC_ADDR_A6_e);
  }
#else
  mag1Sensor_.setIICAddress(TLx493D_IIC_ADDR_A6_e);  // A6: 0x1E (8-bit) / 0x0F (7-bit)
#endif
  delay(10);

  powerOn(Config::PIN_MAG2_LS);
  mag2Sensor_.begin(true, false, false, true);
#ifdef SENSOR_ADDR_RETRY
  if (!mag2Sensor_.setIICAddress(TLx493D_IIC_ADDR_A5_e)) {
    delay(50);
    mag2Sensor_.begin(true, false, false, true);
    mag2Sensor_.setIICAddress(TLx493D_IIC_ADDR_A5_e);
  }
#else
  mag2Sensor_.setIICAddress(TLx493D_IIC_ADDR_A5_e);  // A5: 0x36 (8-bit) / 0x1B (7-bit)
#endif
  delay(10);

  powerOn(Config::PIN_MAG3_LS);
  mag3Sensor_.begin(true, false, false, true);
#ifdef SENSOR_ADDR_RETRY
  // MAG3 stays at default A4, but begin() might fail if address was persisted.
  // The first begin() resets the address; if it failed, retry.
  // (No setIICAddress needed — MAG3 uses the default A4)
#endif
  // MAG3 permanece no endereço padrão A4: 0x3E (8-bit) / 0x1F (7-bit)
  delay(10);
}

void SensorController::readRaw(float out[9]) {
  double mag1x = 0, mag1y = 0, mag1z = 0, temp1 = 0;
  double mag2x = 0, mag2y = 0, mag2z = 0, temp2 = 0;
  double mag3x = 0, mag3y = 0, mag3z = 0, temp3 = 0;

  mag1Sensor_.getMagneticFieldAndTemperature(&mag1x, &mag1y, &mag1z, &temp1);
  mag2Sensor_.getMagneticFieldAndTemperature(&mag2x, &mag2y, &mag2z, &temp2);
  mag3Sensor_.getMagneticFieldAndTemperature(&mag3x, &mag3y, &mag3z, &temp3);

  // MAG1 = bottom, MAG2 = top left, MAG3 = top right.
  out[0] = mag1x;
  out[1] = mag1y;
  out[2] = mag1z;
  out[3] = mag2x;
  out[4] = mag2y;
  out[5] = mag2z;
  out[6] = mag3x;
  out[7] = mag3y;
  out[8] = mag3z;
}

void SensorController::beginCalibration() {
  calibrationActive_ = true;
  calibrationDone_ = false;
  calibrationSamples_ = 0;
  lastCalibrationSampleMs_ = 0;
  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] = 0.0;
  }
}

void SensorController::updateCalibration() {
  if (!calibrationActive_) {
    return;
  }

  const unsigned long now = millis();
  if (lastCalibrationSampleMs_ != 0 &&
      (now - lastCalibrationSampleMs_) < 10) {
    return;
  }
  lastCalibrationSampleMs_ = now;

  float raw[9] = {};
  readRaw(raw);

  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] += raw[i];
  }

  calibrationSamples_++;
  if (calibrationSamples_ < Config::ZERO_SAMPLES) {
    return;
  }

  for (int i = 0; i < 9; i++) {
    baseline_[i] = calibrationSum_[i] / Config::ZERO_SAMPLES;
  }

  calibrationActive_ = false;
  calibrationDone_ = true;
}

bool SensorController::calibrationDone() const { return calibrationDone_; }

const float* SensorController::baseline() const { return baseline_; }
