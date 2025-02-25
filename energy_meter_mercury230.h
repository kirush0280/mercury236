// Автор кода https://github.com/Brokly/ESPHome-Mercury230
// My telegram https://t.me/DieMetRik

#pragma once

#include <Arduino.h>
#include "esphome.h"
#include <stdarg.h>
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include <vector>

// Макросы из mercury230_proto.h
#define PASS_1 "\1\1\1\1\1\1"
#define PASS_2 "\2\2\2\2\2\2"
#define PACKET_MIN_DELAY 500
#define ABORT_RECIVE_TIME 50
#define MIN_SCAN_PERIOD 2000

namespace esphome {
namespace energy_meter_mercury230 {

using namespace esphome;
using sensor::Sensor;
using text_sensor::TextSensor;
using uart::UARTDevice;
using uart::UARTComponent;

// Объявляем типы и константы перед использованием в Meter
enum _replyReason : uint8_t {
  REP_OK = 0,              // Все нормально
  ERROR_COMMAND = 1,       // Недопустимая команда или параметр
  ERROR_HARDWARE = 2,      // Внутренняя ошибка счетчика
  ERROR_ACCESS_LEVEL = 3,  // Недостаточен уровень для удовлетворения запроса
  ERROR_CORE_TIME = 4,     // Внутренние часы счетчика уже корректировались в течение текущих суток
  ERROR_CONNECTION = 5,    // Не открыт канал связи
  ERROR_TIMEOUT = 6,       // Ошибка ответа, ошибка КС
  BUFFER_OVERFLOW = 7      // Переполнение буфера
};

enum _packetType : uint8_t {
  _OK = 0,      // Пакет проверки связи
  CONNECT = 1,  // Установка конекта
  CLOSE = 2,    // Закрытие конекта
  WRITE = 3,    // Запись
  READ = 4,     // Чтение параметров
  LIST = 5,     // Чтение журналов
  READ_PARAMS = 8 // Чтение доп параметров
};

enum _currentSend : uint8_t {
  NONE = 0,
  GET_TEST,
  GET_ACCESS,
  WRITE_TIME,
  CORE_TIME,
  GET_TIME,
  GET_POWER,
  GET_VOLTAGE,
  GET_CURRENT,
  GET_KOEF_POWER,
  GET_FREQ,
  GET_ANGLE_PH,
  GET_DISTORTION,
  GET_TEMP,
  GET_LINEAR_VOLTAGE,
  GET_VERS,
  GET_SER_NUM,
  GET_TIME_CODE,
  GET_CRC,
  GET_VALUE_A,
  GET_VALUE_B,
  GET_VALUE_C,
  GET_ADDR
};

struct _sBuff {
  uint8_t addr;       // адрес счетчика
  _packetType packType; // тип пакета
  uint8_t data[30];   // тело буфера
};

enum _reqType : uint8_t {
  PARAM_SER_NUM = 0,    // серийный номер и дату
  PARAM_VERS = 3,       // версия
  PARAM_UNO = 0x11,     // читаем один конкретный параметр
  PARAM_ALL_FULL = 0x14, // ответ по всем фазам, списком, без сокращения незначащих битов
  PARAM_ALL = 0x16,     // ответ по всем фазам, списком, в сокращенном формате
  PARAM_CRC = 0x26      // читаем CRC прибора
};

static const uint16_t crcTable[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// Определяем Meter после типов и констант
struct Meter {
  uint8_t addr;                     // Адрес счетчика
  std::string pass;                 // Пароль для подключения
  bool act_pass = false;            // Пароль указан пользователем
  bool pas_in_hex = false;          // Пароль в HEX-формате
  bool admin = false;               // Тип доступа (админ)
  Sensor *VoltA = nullptr;          // Сенсоры для напряжения, тока и т.д.
  Sensor *VoltB = nullptr;
  Sensor *VoltC = nullptr;
  Sensor *Amps = nullptr;
  Sensor *AmpA = nullptr;
  Sensor *AmpB = nullptr;
  Sensor *AmpC = nullptr;
  Sensor *Watts = nullptr;
  Sensor *WattA = nullptr;
  Sensor *WattB = nullptr;
  Sensor *WattC = nullptr;
  Sensor *RatioA = nullptr;
  Sensor *RatioB = nullptr;
  Sensor *RatioC = nullptr;
  Sensor *AngleA = nullptr;
  Sensor *AngleB = nullptr;
  Sensor *AngleC = nullptr;
  Sensor *Freq = nullptr;
  Sensor *ValueAA = nullptr;
  Sensor *ValueRA = nullptr;
  Sensor *ValueAB = nullptr;
  Sensor *ValueRB = nullptr;
  Sensor *ValueAC = nullptr;
  Sensor *ValueRC = nullptr;
  TextSensor *vers_string = nullptr;
  TextSensor *error_string = nullptr;
  TextSensor *sn_string = nullptr;
  TextSensor *fab_date_string = nullptr;
  bool cbPower = false;             // Флаги обратных вызовов
  bool cbVolt = false;
  bool cbCurrent = false;
  bool cbKoef = false;
  bool cbAngles = false;
  bool cbFreq = false;
  bool cbValuesA = false;
  bool cbValuesB = false;
  bool cbValuesC = false;
  uint8_t readBuff[32] = {0};       // Буфер входящих данных
  uint8_t fromReadArrow = 0;        // Указатель точки заполнения буфера приема
  uint8_t sendBuff[32] = {0};       // Буфер отправки
  _sBuff *sBuff = (_sBuff *)sendBuff; // Фантом буфера отправки
  uint8_t forSendSize = 0;          // Количество данных в буфере отправки
  uint8_t forSendArrow = 0;         // Указатель на очередной байт в буфере
  _currentSend forSenfType = NONE;  // Тип пакета в буфере отправки
  uint32_t scanPeriod = 0xFFFFFFFF; // Период сканирования
  uint32_t timeReadByte = 0;        // Время последнего получения байта
  uint32_t timeSendByte = 0;        // Время последней отправки байта
  uint16_t mainCRC = 0;             // CRC прибора
  bool procError = false;           // Ошибка во время цикла опроса
  bool waiteReply = false;          // Флаг ожидания ответа
  _replyReason lastError = REP_OK;  // Последний статус ответа
  uint32_t scanTimer = millis();    // Таймер периодов связи
  bool debugIn = false;             // Флаг отладки входных данных
  bool debugOut = false;            // Флаг отладки исходящих данных
  uint8_t inPacket[32] = {0};       // Буфер для отладки входящих пакетов
  uint8_t sizeInPacket = 0;         // Размер входящего пакета
  uint8_t outPacket[32] = {0};      // Буфер для отладки исходящих пакетов
  uint8_t sizeOutPacket = 0;        // Размер исходящего пакета
};

class Mercury : public PollingComponent {
 public:
  Mercury() {}

  void add_meter(uint8_t addr, const std::string &pass = "", bool pas_in_hex = false, bool admin = false) {
    Meter meter;
    meter.addr = addr;
    meter.pass = pass;
    meter.act_pass = !pass.empty();
    meter.pas_in_hex = pas_in_hex;
    meter.admin = admin;
    meters_.push_back(meter);
  }

  void set_VoltA(Sensor *sens) { if (!meters_.empty()) meters_.back().VoltA = sens; }
  void set_VoltB(Sensor *sens) { if (!meters_.empty()) meters_.back().VoltB = sens; }
  void set_VoltC(Sensor *sens) { if (!meters_.empty()) meters_.back().VoltC = sens; }
  void set_Amps(Sensor *sens) { if (!meters_.empty()) meters_.back().Amps = sens; }
  void set_AmpA(Sensor *sens) { if (!meters_.empty()) meters_.back().AmpA = sens; }
  void set_AmpB(Sensor *sens) { if (!meters_.empty()) meters_.back().AmpB = sens; }
  void set_AmpC(Sensor *sens) { if (!meters_.empty()) meters_.back().AmpC = sens; }
  void set_Watts(Sensor *sens) { if (!meters_.empty()) meters_.back().Watts = sens; }
  void set_WattA(Sensor *sens) { if (!meters_.empty()) meters_.back().WattA = sens; }
  void set_WattB(Sensor *sens) { if (!meters_.empty()) meters_.back().WattB = sens; }
  void set_WattC(Sensor *sens) { if (!meters_.empty()) meters_.back().WattC = sens; }
  void set_RatioA(Sensor *sens) { if (!meters_.empty()) meters_.back().RatioA = sens; }
  void set_RatioB(Sensor *sens) { if (!meters_.empty()) meters_.back().RatioB = sens; }
  void set_RatioC(Sensor *sens) { if (!meters_.empty()) meters_.back().RatioC = sens; }
  void set_AngleA(Sensor *sens) { if (!meters_.empty()) meters_.back().AngleA = sens; }
  void set_AngleB(Sensor *sens) { if (!meters_.empty()) meters_.back().AngleB = sens; }
  void set_AngleC(Sensor *sens) { if (!meters_.empty()) meters_.back().AngleC = sens; }
  void set_Freq(Sensor *sens) { if (!meters_.empty()) meters_.back().Freq = sens; }
  void set_ValueAA(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueAA = sens; }
  void set_ValueRA(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueRA = sens; }
  void set_ValueAB(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueAB = sens; }
  void set_ValueRB(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueRB = sens; }
  void set_ValueAC(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueAC = sens; }
  void set_ValueRC(Sensor *sens) { if (!meters_.empty()) meters_.back().ValueRC = sens; }
  void set_vers_string(TextSensor *sens) { if (!meters_.empty()) meters_.back().vers_string = sens; }
  void set_error_string(TextSensor *sens) { if (!meters_.empty()) meters_.back().error_string = sens; }
  void set_sn_string(TextSensor *sens) { if (!meters_.empty()) meters_.back().sn_string = sens; }
  void set_fab_date_string(TextSensor *sens) { if (!meters_.empty()) meters_.back().fab_date_string = sens; }

  void initUart(UARTComponent *parent) { my_serial = parent; }
  void set_active_pin(GPIOPin *pin) { led_active_pin = pin; if (pin) pin->setup(); }
  void set_update_interval(uint32_t update_interval) { update_interval_ = update_interval; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

 private:
  std::vector<Meter> meters_;  // Список счетчиков
  UARTComponent *my_serial = nullptr;
  GPIOPin *led_active_pin = nullptr;
  const uint32_t minUpdatePeriod = 5000;
  const char *const TAG = "Mercury";

  void _debugMsg(const String &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ...);
  void _debugPrintPacket(uint8_t *data, uint8_t size, bool in, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0);
};

// Функции после определения Meter
inline uint32_t dm32_3(uint8_t *d) { return (((((uint32_t)(d[0] & 0x3f)) << 8) + d[2]) << 8) + d[1]; }
inline uint32_t dm32_4(uint8_t *d) { return ((((uint32_t)d[1] << 8) + d[0]) << 16) | (((uint32_t)d[3] << 8) + d[2]); }

inline uint16_t stepCrc16mb(uint8_t in, bool start = false, uint16_t *crc = nullptr) {
  static uint16_t static_crc = 0xFFFF;
  if (!crc) crc = &static_crc;
  if (start) *crc = 0xFFFF;
  *crc = ((*crc >> 8) ^ crcTable[(*crc ^ in) & 0xFF]);
  return *crc;
}

inline void inDataReady(uint8_t size, uint8_t *buff, Meter *meter);

inline uint8_t crc16mb(uint8_t *s, uint8_t count, Meter *meter) {
  uint16_t *crc = (uint16_t *)(&s[count - 2]);
  *crc = 0xFFFF;
  for (int i = 0; i < count - 2; i++) {
    *crc = ((*crc >> 8) ^ crcTable[(*crc ^ s[i]) & 0xFF]);
  }
  if (meter->debugOut) inDataReady(count, s, meter);
  return count;
}

inline bool getPass(uint8_t *buff, Meter *meter) {
  if (!meter->act_pass) return false;
  if (meter->pas_in_hex) {
    for (uint8_t i = 0; i < 6; i++) {
      if (!((meter->pass[i] >= '0' && meter->pass[i] <= '9') ||
            (meter->pass[i] >= 'A' && meter->pass[i] <= 'F') ||
            (meter->pass[i] >= 'a' && meter->pass[i] <= 'f'))) {
        meter->pas_in_hex = false;
        ESP_LOGE("Mercury", "Password for meter %u is not in HEX format!", meter->addr);
        break;
      }
    }
  }
  if (meter->pas_in_hex) {
    for (uint8_t i = 0; i < 6; i++) {
      if (meter->pass[i] >= '0' && meter->pass[i] <= '9') buff[i] = meter->pass[i] - '0';
      else if (meter->pass[i] >= 'A' && meter->pass[i] <= 'F') buff[i] = meter->pass[i] - 'A' + 10;
      else if (meter->pass[i] >= 'a' && meter->pass[i] <= 'f') buff[i] = meter->pass[i] - 'a' + 10;
    }
  } else {
    for (uint8_t i = 0; i < 6; i++) buff[i] = meter->pass[i];
  }
  ESP_LOGD("Mercury", "Password for meter %u: %X,%X,%X,%X,%X,%X", meter->addr, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
  return true;
}

inline void sConnect(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = _OK;
  meter->forSendSize = crc16mb(meter->sendBuff, 4, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_TEST;
}

inline void sAccess(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = CONNECT;
  if (meter->admin) {
    meter->sBuff->data[0] = 2;
    if (meter->act_pass) getPass(meter->sBuff->data + 1, meter);
    else memcpy(meter->sBuff->data + 1, PASS_2, sizeof(PASS_2));
  } else {
    meter->sBuff->data[0] = 1;
    if (meter->act_pass) getPass(meter->sBuff->data + 1, meter);
    else memcpy(meter->sBuff->data + 1, PASS_1, sizeof(PASS_1));
  }
  meter->forSendSize = crc16mb(meter->sendBuff, 11, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_ACCESS;
}

inline void _getParam(uint8_t param, Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = READ_PARAMS;
  meter->sBuff->data[0] = (uint8_t)PARAM_ALL;
  meter->sBuff->data[1] = param;
  meter->forSendSize = crc16mb(meter->sendBuff, 6, meter);
  meter->forSendArrow = 0;
}

inline void sGetPower(Meter *meter) { _getParam(0, meter); meter->forSenfType = GET_POWER; }
inline void sGetVoltage(Meter *meter) { _getParam(0x11, meter); meter->forSenfType = GET_VOLTAGE; }
inline void sGetCurrent(Meter *meter) { _getParam(0x21, meter); meter->forSenfType = GET_CURRENT; }
inline void sGetKoefPower(Meter *meter) { _getParam(0x31, meter); meter->forSenfType = GET_KOEF_POWER; }
inline void sGetFreq(Meter *meter) { _getParam(0x40, meter); meter->forSenfType = GET_FREQ; }
inline void sGetAnglePh(Meter *meter) { _getParam(0x51, meter); meter->forSenfType = GET_ANGLE_PH; }

inline void sGetVers(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = READ_PARAMS;
  meter->sBuff->data[0] = 1;
  meter->forSendSize = crc16mb(meter->sendBuff, 5, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_VERS;
}

inline void sGetAddr(Meter *meter) {
  meter->sBuff->addr = 0;
  meter->sBuff->packType = READ_PARAMS;
  meter->sBuff->data[0] = 5;
  meter->forSendSize = crc16mb(meter->sendBuff, 5, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_ADDR;
}

inline void sGetValue_A(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = LIST;
  meter->sBuff->data[0] = 0;
  meter->sBuff->data[1] = 1;
  meter->forSendSize = crc16mb(meter->sendBuff, 6, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_VALUE_A;
}

inline void sGetValue_B(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = LIST;
  meter->sBuff->data[0] = 0;
  meter->sBuff->data[1] = 2;
  meter->forSendSize = crc16mb(meter->sendBuff, 6, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_VALUE_B;
}

inline void sGetValue_C(Meter *meter) {
  meter->sBuff->addr = meter->addr;
  meter->sBuff->packType = LIST;
  meter->sBuff->data[0] = 0;
  meter->sBuff->data[1] = 3;
  meter->forSendSize = crc16mb(meter->sendBuff, 6, meter);
  meter->forSendArrow = 0;
  meter->forSenfType = GET_VALUE_C;
}

inline _replyReason getLastError(Meter *meter) { return meter->lastError; }

inline const char *getStrError(_replyReason lastError, Meter *meter) {
  static char out[27] = {0};
  char *rep = out;
  if ((uint8_t)lastError & 0x80) {
    lastError = (_replyReason)((uint8_t)lastError & 0x7F);
    strcpy(rep, "Broadcast ");
    rep += 10;
  }
  if (lastError == REP_OK) strcpy(rep, "OK");
  else if (lastError == ERROR_COMMAND) strcpy(rep, "Command error");
  else if (lastError == ERROR_HARDWARE) strcpy(rep, "Hardware error");
  else if (lastError == ERROR_ACCESS_LEVEL) strcpy(rep, "Access deny");
  else if (lastError == ERROR_CORE_TIME) strcpy(rep, "Core time forbidden");
  else if (lastError == ERROR_CONNECTION) strcpy(rep, "Connection close");
  else if (lastError == ERROR_TIMEOUT) strcpy(rep, "Timeout error");
  else if (lastError == BUFFER_OVERFLOW) strcpy(rep, "Buffer overflow");
  else strcpy(rep, "Unexpected");
  return out;
}

inline void _cbPower(Meter *meter, float Psumm, float Pa, float Pb, float Pc) {
  static float _Psumm = -Psumm, _Pa = -Pa, _Pb = -Pb, _Pc = -Pc;
  if (meter->Watts && _Psumm != Psumm) { _Psumm = Psumm; meter->Watts->publish_state(Psumm); }
  if (meter->WattA && _Pa != Pa) { _Pa = Pa; meter->WattA->publish_state(Pa); }
  if (meter->WattB && _Pb != Pb) { _Pb = Pb; meter->WattB->publish_state(Pb); }
  if (meter->WattC && _Pc != Pc) { _Pc = Pc; meter->WattC->publish_state(Pc); }
}

inline void _cbVolt(Meter *meter, float Va, float Vb, float Vc) {
  static float _Va = -Va, _Vb = -Vb, _Vc = -Vc;
  if (meter->VoltA && _Va != Va) { _Va = Va; meter->VoltA->publish_state(Va); }
  if (meter->VoltB && _Vb != Vb) { _Vb = Vb; meter->VoltB->publish_state(Vb); }
  if (meter->VoltC && _Vc != Vc) { _Vc = Vc; meter->VoltC->publish_state(Vc); }
}

inline void _cbCurrent(Meter *meter, float Ca, float Cb, float Cc) {
  bool change_summ = false;
  static float _Ca = -Ca, _Cb = -Cb, _Cc = -Cc;
  if (meter->AmpA && _Ca != Ca) { _Ca = Ca; meter->AmpA->publish_state(Ca); change_summ = true; }
  if (meter->AmpB && _Cb != Cb) { _Cb = Cb; meter->AmpB->publish_state(Cb); change_summ = true; }
  if (meter->AmpC && _Cc != Cc) { _Cc = Cc; meter->AmpC->publish_state(Cc); change_summ = true; }
  if (meter->Amps && change_summ) meter->Amps->publish_state(Ca + Cb + Cc);
}

inline void _cbKoef(Meter *meter, float Ra, float Rb, float Rc) {
  static float _Ra = -Ra, _Rb = -Rb, _Rc = -Rc;
  if (meter->RatioA && _Ra != Ra) { _Ra = Ra; meter->RatioA->publish_state(Ra); }
  if (meter->RatioB && _Rb != Rb) { _Rb = Rb; meter->RatioB->publish_state(Rb); }
  if (meter->RatioC && _Rc != Rc) { _Rc = Rc; meter->RatioC->publish_state(Rc); }
}

inline void _cbAngles(Meter *meter, float Aa, float Ab, float Ac) {
  static float _Aa = -Aa, _Ab = -Ab, _Ac = -Ac;
  if (meter->AngleA && _Aa != Aa) { _Aa = Aa; meter->AngleA->publish_state(Aa); }
  if (meter->AngleB && _Ab != Ab) { _Ab = Ab; meter->AngleB->publish_state(Ab); }
  if (meter->AngleC && _Ac != Ac) { _Ac = Ac; meter->AngleC->publish_state(Ac); }
}

inline void _cbFreq(Meter *meter, float Fr) {
  static float _Fr = -Fr;
  if (meter->Freq && _Fr != Fr) { _Fr = Fr; meter->Freq->publish_state(Fr); }
}

inline void _cbValues_A(Meter *meter, float Aa, float Ar) {
  static float _Aa = -Aa, _Ar = -Ar;
  if (meter->ValueAA && _Aa != Aa) { _Aa = Aa; meter->ValueAA->publish_state(Aa); }
  if (meter->ValueRA && _Ar != Ar) { _Ar = Ar; meter->ValueRA->publish_state(Ar); }
}

inline void _cbValues_B(Meter *meter, float Aa, float Ar) {
  static float _Aa = -Aa, _Ar = -Ar;
  if (meter->ValueAB && _Aa != Aa) { _Aa = Aa; meter->ValueAB->publish_state(Aa); }
  if (meter->ValueRB && _Ar != Ar) { _Ar = Ar; meter->ValueRB->publish_state(Ar); }
}

inline void _cbValues_C(Meter *meter, float Aa, float Ar) {
  static float _Aa = -Aa, _Ar = -Ar;
  if (meter->ValueAC && _Aa != Aa) { _Aa = Aa; meter->ValueAC->publish_state(Aa); }
  if (meter->ValueRC && _Ar != Ar) { _Ar = Ar; meter->ValueRC->publish_state(Ar); }
}

inline void inDataReady(uint8_t size, uint8_t *buff, Meter *meter) {
  memcpy(meter->inPacket, buff, size);
  meter->sizeInPacket = size;
}

inline void outDataReady(uint8_t size, uint8_t *buff, Meter *meter) {
  memcpy(meter->outPacket, buff, size);
  meter->sizeOutPacket = size;
}

inline void parceInbound(Meter *meter) {
  if (meter->debugIn) outDataReady(meter->fromReadArrow, meter->readBuff, meter);
  if (meter->readBuff[0] == meter->addr || meter->readBuff[0] == 0) {
    if (meter->fromReadArrow == 4) {
      meter->lastError = (_replyReason)meter->readBuff[1];
      if (meter->lastError != REP_OK && meter->lastError != ERROR_CORE_TIME) meter->procError = true;
      if (meter->readBuff[0] == 0) meter->lastError = (_replyReason)(meter->readBuff[1] | 0x80);
      meter->fromReadArrow = 0;
    } else if (meter->fromReadArrow == 5) {
      if (meter->forSenfType == GET_ADDR) {
        meter->addr = meter->readBuff[2];
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
      }
    } else if (meter->fromReadArrow == 15) {
      if (meter->forSenfType == GET_POWER) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbPower(meter, (float)dm32_3(meter->readBuff + 1) / 100,
                 (float)dm32_3(meter->readBuff + 4) / 100,
                 (float)dm32_3(meter->readBuff + 7) / 100,
                 (float)dm32_3(meter->readBuff + 10) / 100);
      }
    } else if (meter->fromReadArrow == 12) {
      if (meter->forSenfType == GET_VOLTAGE) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbVolt(meter, (float)dm32_3(meter->readBuff + 1) / 100,
                (float)dm32_3(meter->readBuff + 4) / 100,
                (float)dm32_3(meter->readBuff + 7) / 100);
      } else if (meter->forSenfType == GET_KOEF_POWER) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbKoef(meter, (float)dm32_3(meter->readBuff + 1) / 100,
                (float)dm32_3(meter->readBuff + 4) / 100,
                (float)dm32_3(meter->readBuff + 7) / 100);
      } else if (meter->forSenfType == GET_ANGLE_PH) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbAngles(meter, (float)dm32_3(meter->readBuff + 1) / 100,
                  (float)dm32_3(meter->readBuff + 4) / 100,
                  (float)dm32_3(meter->readBuff + 7) / 100);
      } else if (meter->forSenfType == GET_CURRENT) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbCurrent(meter, (float)dm32_3(meter->readBuff + 1) / 1000,
                   (float)dm32_3(meter->readBuff + 4) / 1000,
                   (float)dm32_3(meter->readBuff + 7) / 1000);
      }
    } else if (meter->fromReadArrow == 6) {
      if (meter->forSenfType == GET_FREQ) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbFreq(meter, (float)dm32_3(meter->readBuff + 1) / 100);
      }
    } else if (meter->fromReadArrow == 19) {
      if (meter->forSenfType == GET_VERS) {
        char temp[15] = {0};
        if (meter->sn_string) {
          snprintf(temp, sizeof(temp) - 1, "%02d%02d%02d%02d", meter->readBuff[1], meter->readBuff[2], meter->readBuff[3], meter->readBuff[4]);
          meter->sn_string->publish_state(temp);
        }
        if (meter->vers_string) {
          snprintf(temp, sizeof(temp) - 1, "%d.%02d.%02d", meter->readBuff[8], meter->readBuff[9], meter->readBuff[10]);
          meter->vers_string->publish_state(temp);
        }
        if (meter->fab_date_string) {
          snprintf(temp, sizeof(temp) - 1, "%d/%02d/%02d", meter->readBuff[5], meter->readBuff[6], meter->readBuff[7]);
          meter->fab_date_string->publish_state(temp);
        }
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
      } else if (meter->forSenfType == GET_VALUE_A) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbValues_A(meter, (float)dm32_4(meter->readBuff + 1) / 1000, (float)dm32_4(meter->readBuff + 9) / 1000);
      } else if (meter->forSenfType == GET_VALUE_B) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbValues_B(meter, (float)dm32_4(meter->readBuff + 1) / 1000, (float)dm32_4(meter->readBuff + 9) / 1000);
      } else if (meter->forSenfType == GET_VALUE_C) {
        meter->fromReadArrow = 0;
        meter->lastError = REP_OK;
        _cbValues_C(meter, (float)dm32_4(meter->readBuff + 1) / 1000, (float)dm32_4(meter->readBuff + 9) / 1000);
      }
    }
  } else {
    meter->fromReadArrow = 0;
    meter->lastError = REP_OK;
  }
}

inline void setupMerc(uint32_t _scanPeriod, Meter *meter) {
  meter->scanPeriod = _scanPeriod;
  meter->scanTimer = millis() - _scanPeriod;
}

inline void setUpdatePeriod(uint32_t period, Meter *meter) {
  if (period < MIN_SCAN_PERIOD) period = MIN_SCAN_PERIOD;
  meter->scanPeriod = period;
}

inline uint8_t availableMerc(Meter *meter) {
  uint32_t _now = millis();
  static uint8_t counter = 0;

  if (meter->waiteReply && _now - meter->timeReadByte > ABORT_RECIVE_TIME) {
    meter->waiteReply = false;
    if (counter > 5) meter->lastError = ERROR_TIMEOUT;
    meter->procError = true;
  }

  uint8_t ret = meter->forSendSize - meter->forSendArrow;
  if (ret) return ret;

  if (_now - meter->scanTimer >= meter->scanPeriod) {
    if (!meter->waiteReply && _now - meter->timeSendByte >= PACKET_MIN_DELAY) {
      meter->timeSendByte = _now;
      while (counter <= 12) {
        if (counter == 0) { sConnect(meter); break; }
        else if (counter == 1) { sAccess(meter); break; }
        else if (counter == 2 && meter->addr == 0) { sGetAddr(meter); counter = 13; meter->procError = true; break; }
        else if (counter == 3) { sGetVers(meter); break; }
        else if (counter == 4) { if (meter->cbValuesA) { sGetValue_A(meter); break; } }
        else if (counter == 5) { if (meter->cbValuesB) { sGetValue_B(meter); break; } }
        else if (counter == 6) { if (meter->cbValuesC) { sGetValue_C(meter); break; } }
        else if (counter == 7) { if (meter->cbPower) { sGetPower(meter); break; } }
        else if (counter == 8) { if (meter->cbVolt) { sGetVoltage(meter); break; } }
        else if (counter == 9) { if (meter->cbCurrent) { sGetCurrent(meter); break; } }
        else if (counter == 10) { if (meter->cbKoef) { sGetKoefPower(meter); break; } }
        else if (counter == 11) { if (meter->cbAngles) { sGetAnglePh(meter); break; } }
        else if (counter == 12) { if (meter->cbFreq) { sGetFreq(meter); break; } }
        counter++;
      }
      if (counter++ > 12) {
        if (_now - meter->scanTimer > 2 * meter->scanPeriod) meter->scanTimer = _now;
        else meter->scanTimer += meter->scanPeriod;
        if (meter->procError) {
          meter->procError = false;
          counter = 0;
        } else {
          counter = 3;
        }
      }
    }
  } else {
    meter->timeReadByte = _now;
  }
  return meter->forSendSize - meter->forSendArrow;
}

inline uint8_t *getBuffForMerc(Meter *meter) {
  uint8_t *ret = &(meter->sendBuff[meter->forSendArrow]);
  meter->forSendArrow = 0;
  meter->forSendSize = 0;
  meter->waiteReply = true;
  return ret;
}

inline _replyReason getFromMerc(uint8_t d, Meter *meter) {
  uint32_t _now = millis();
  if (_now - meter->timeReadByte > ABORT_RECIVE_TIME) {
    meter->fromReadArrow = 0;
  }
  if (meter->fromReadArrow == sizeof(meter->readBuff)) {
    meter->procError = true;
    meter->lastError = BUFFER_OVERFLOW;
    meter->fromReadArrow = 0;
  }
  meter->waiteReply = false;
  meter->timeReadByte = _now;
  meter->readBuff[meter->fromReadArrow++] = d;

  static uint16_t crc = 0;
  if (meter->fromReadArrow == 1) {
    stepCrc16mb(d, true, &crc);
  } else if (meter->fromReadArrow < 3) {
    stepCrc16mb(d, false, &crc);
  } else {
    if (stepCrc16mb(d, false, &crc) == 0) {
      parceInbound(meter);
      availableMerc(meter);
    }
  }
  return meter->lastError;
}

void Mercury::setup() {
  if (update_interval_ < 5000) {
    update_interval_ = 30000;
  }

  if (led_active_pin) {
    led_active_pin->pin_mode(gpio::FLAG_OUTPUT);
    led_active_pin->digital_write(false);
  }

  for (auto &meter : meters_) {
    meter.cbPower = (meter.Watts || meter.WattA || meter.WattB || meter.WattC);
    meter.cbVolt = (meter.VoltA || meter.VoltB || meter.VoltC);
    meter.cbCurrent = (meter.Amps || meter.AmpA || meter.AmpB || meter.AmpC);
    meter.cbKoef = (meter.RatioA || meter.RatioB || meter.RatioC);
    meter.cbAngles = (meter.AngleA || meter.AngleB || meter.AngleC);
    meter.cbFreq = (meter.Freq);
    meter.cbValuesA = (meter.ValueAA || meter.ValueRA);
    meter.cbValuesB = (meter.ValueAB || meter.ValueRB);
    meter.cbValuesC = (meter.ValueAC || meter.ValueRC);
    setupMerc(minUpdatePeriod, &meter); // Инициализация каждого счетчика
  }

  for (auto &meter : meters_) {
    meter.debugIn = true;
    meter.debugOut = true;
  }
}

void Mercury::loop() {
  if (!my_serial) return;

  static size_t current_meter_idx = 0;
  if (meters_.empty()) return;

  Meter &current_meter = meters_[current_meter_idx]; // Исправлено: используем ссылку вместо копии

  // Чтение данных от счетчика
  if (my_serial->available()) {
    uint8_t data;
    my_serial->read_byte(&data);
    getFromMerc(data, &current_meter); // Передаем указатель
    if (led_active_pin) led_active_pin->digital_write(false);
  }

  // Отправка данных счетчику
  uint8_t data_size = availableMerc(&current_meter); // Передаем указатель
  if (data_size) {
    uint8_t *buff = getBuffForMerc(&current_meter); // Передаем указатель
    my_serial->write_array(buff, data_size);
    if (led_active_pin) led_active_pin->digital_write(true);
  }

  // Отладка
  if (current_meter.sizeOutPacket) {
    _debugPrintPacket(current_meter.outPacket, current_meter.sizeOutPacket, false);
    current_meter.sizeOutPacket = 0;
  }
  if (current_meter.sizeInPacket) {
    _debugPrintPacket(current_meter.inPacket, current_meter.sizeInPacket, true);
    current_meter.sizeInPacket = 0;
  }

  // Переключение на следующий счетчик
  current_meter_idx = (current_meter_idx + 1) % meters_.size();
}

void Mercury::update() {
  for (auto &meter : meters_) {
    static uint32_t upd_int = update_interval_;
    static uint32_t upd_period = upd_int - 1;
    if (upd_int != upd_period) {
      upd_period = upd_int;
      if (upd_period < minUpdatePeriod) upd_period = minUpdatePeriod;
      setUpdatePeriod(upd_period, &meter);
      _debugMsg(F("Core scan period %u"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, upd_period);
      upd_period = upd_int;
    }

    static _replyReason oldError = ERROR_CORE_TIME;
    _replyReason currentError = getLastError(&meter);
    if (oldError != currentError) {
      oldError = currentError;
      if (meter.error_string) {
        meter.error_string->publish_state(getStrError(oldError, &meter));
      }
      if (currentError == REP_OK) {
        _debugMsg(F("No errors for meter %u!"), ESPHOME_LOG_LEVEL_INFO, __LINE__, meter.addr);
      } else {
        _debugMsg(F("Error for meter %u: %s"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, meter.addr, getStrError(currentError, &meter));
      }
    }
  }
}

void Mercury::dump_config() {
  ESP_LOGCONFIG(TAG, "Mercury:");
  ESP_LOGCONFIG(TAG, "UART Bus:");
  ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", my_serial->get_rx_buffer_size());
  ESP_LOGCONFIG(TAG, "  Baud Rate: %u baud", my_serial->get_baud_rate());
  ESP_LOGCONFIG(TAG, "  Update interval: %u sec", (update_interval_ / 1000));
  for (const auto &meter : meters_) {
    ESP_LOGCONFIG(TAG, "Meter at address %u:", meter.addr);
    LOG_SENSOR("  ", "Voltage phase A", meter.VoltA);
    LOG_SENSOR("  ", "Voltage phase B", meter.VoltB);
    LOG_SENSOR("  ", "Voltage phase C", meter.VoltC);
    LOG_SENSOR("  ", "Amperage Summ", meter.Amps);
    LOG_SENSOR("  ", "Amperage phase A", meter.AmpA);
    LOG_SENSOR("  ", "Amperage phase B", meter.AmpB);
    LOG_SENSOR("  ", "Amperage phase C", meter.AmpC);
    LOG_SENSOR("  ", "Watts All", meter.Watts);
    LOG_SENSOR("  ", "Watts phase A", meter.WattA);
    LOG_SENSOR("  ", "Watts phase B", meter.WattB);
    LOG_SENSOR("  ", "Watts phase C", meter.WattC);
    LOG_SENSOR("  ", "Ratio phase A", meter.RatioA);
    LOG_SENSOR("  ", "Ratio phase B", meter.RatioB);
    LOG_SENSOR("  ", "Ratio phase C", meter.RatioC);
    LOG_SENSOR("  ", "Phase shift AB", meter.AngleA);
    LOG_SENSOR("  ", "Phase shift BC", meter.AngleB);
    LOG_SENSOR("  ", "Phase shift CA", meter.AngleC);
    LOG_SENSOR("  ", "Frequency", meter.Freq);
    LOG_SENSOR("  ", "Values ActiveA+", meter.ValueAA);
    LOG_SENSOR("  ", "Values ReactiveA+", meter.ValueRA);
    LOG_SENSOR("  ", "Values ActiveB+", meter.ValueAB);
    LOG_SENSOR("  ", "Values ReactiveB+", meter.ValueRB);
    LOG_SENSOR("  ", "Values ActiveC+", meter.ValueAC);
    LOG_SENSOR("  ", "Values ReactiveC+", meter.ValueRC);
    LOG_TEXT_SENSOR("  ", "Date of Manufacture", meter.fab_date_string);
    LOG_TEXT_SENSOR("  ", "Serial Number", meter.sn_string);
    LOG_TEXT_SENSOR("  ", "Version", meter.vers_string);
    LOG_TEXT_SENSOR("  ", "Last Error", meter.error_string);
    if (meter.act_pass) {
      uint8_t buff[6] = {0};
      getPass(buff, const_cast<Meter*>(&meter)); // Используем const_cast для константного объекта
      ESP_LOGCONFIG("  ", "Password: %X,%X,%X,%X,%X,%X", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
    }
    if (meter.admin) {
      ESP_LOGCONFIG("  ", "Access level: ADMIN");
    }
  }
  LOG_PIN("Active pin: ", led_active_pin);
}

void Mercury::_debugMsg(const String &msg, uint8_t dbgLevel, unsigned int line, ...) {
  if (dbgLevel < ESPHOME_LOG_LEVEL_NONE) dbgLevel = ESPHOME_LOG_LEVEL_NONE;
  if (dbgLevel > ESPHOME_LOG_LEVEL_VERY_VERBOSE) dbgLevel = ESPHOME_LOG_LEVEL_VERY_VERBOSE;
  if (line == 0) line = __LINE__;
  va_list vl;
  va_start(vl, line);
  esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
  va_end(vl);
}

void Mercury::_debugPrintPacket(uint8_t *data, uint8_t size, bool in, uint8_t dbgLevel, unsigned int line) {
  String st = "";
  char textBuf[11];
  sprintf(textBuf, "%010u", millis());
  st += textBuf + String(": ");
  st += in ? "[<=] " : "[=>] ";
  for (uint8_t i = 0; i < size; i++) {
    sprintf(textBuf, "%02X", data[i]);
    st += textBuf;
    if (i < size - 3) st += " ";
    else if (i == size - 3) st += " (";
    else if (i == size - 1) st += ")";
  }
  if (line == 0) line = __LINE__;
  _debugMsg(st, dbgLevel, line);
}

}  // namespace energy_meter_mercury230
}  // namespace esphome
