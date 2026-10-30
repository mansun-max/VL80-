//Универсальный контроллер ModBus с PWM выходами (16+1? входов)
// Фьюзы: Low- FF, High-DF, Extended-FD
#include <EEPROM.h>

// Определение констант Modbus
#define DEFAULT_MODBUS_ADDRESS 0x01
#define DEFAULT_BAUD_RATE 9600
#define MODBUS_READ_DISCRETE_INPUTS 0x02
#define MODBUS_READ_HOLDING_REGISTERS 0x03
#define MODBUS_READ_INPUT_REGISTERS 0x04
#define MODBUS_WRITE_SINGLE_COIL 0x05
#define MODBUS_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_WRITE_MULTIPLE_REGISTERS 0x10

// Адреса в EEPROM для хранения настроек
#define EEPROM_ADDRESS_ADDR 0x00    // Адрес Modbus (1 байт)
#define EEPROM_BAUD_ADDR 0x01       // Скорость (4 байта - uint32_t)
#define EEPROM_MAGIC_ADDR 0x05      // Адрес магического числа для проверки (2 байта)
#define EEPROM_MAGIC_VALUE 0x55AA   // Магическое число

// Поддерживаемые скорости
const uint32_t SUPPORTED_BAUD_RATES[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
const uint8_t NUM_SUPPORTED_BAUD_RATES = 8;

// Конфигурация пинов
#define FIRST_INPUT_PIN 3      // Arduino pin 3 соответствует Discrete Input 0
#define LAST_INPUT_PIN 13      // Arduino pin 13 соответствует Discrete Input 10
#define NUM_INPUT_PINS 11      // Количество входных пинов (3-13 включительно)
#define FIRST_DISCRETE_INPUT 0 // Начальный адрес Discrete Inputs

// Конфигурация аналоговых пинов как дискретных входов
#define FIRST_ANALOG_PIN A0    // Первый аналоговый пин
#define LAST_ANALOG_PIN A5     // Последний аналоговый пин
#define NUM_ANALOG_PINS 6      // Количество аналоговых пинов (A0-A5)
#define FIRST_ANALOG_DISCRETE_INPUT 11 // Начальный адрес для аналоговых входов (биты 11-16)

// Управление MAX485
#define MAX485_RE_DE_PIN 2     // Пин для управления RE/DE MAX485
#define MAX485_RECEIVE_MODE LOW    // Режим приема
#define MAX485_TRANSMIT_MODE HIGH  // Режим передачи

// Адреса регистров Holding Registers
#define REG_MODBUS_ADDRESS 0    // Регистр 0: Адрес Modbus (1-247)
#define REG_BAUD_RATE_INDEX 1   // Регистр 1: Индекс скорости (0-7)
#define REG_BAUD_RATE_VALUE 2   // Регистр 2: Значение скорости (только чтение)
#define REG_DEVICE_STATUS 3     // Регистр 3: Статус устройства
#define REG_FIRMWARE_VERSION 4  // Регистр 4: Версия прошивки
#define NUM_HOLDING_REGISTERS 5 // Всего 5 регистров

// Регистры для управления выводами (пины 3..13)
#define REG_OUTPUT_PINS_START 100 // Стартовый адрес регистров для выходов
#define NUM_OUTPUT_REGISTERS 11   // 11 регистров (пин 3..13)

// Буферы для Modbus 
#define MODBUS_RX_BUFFER_SIZE 64
#define MODBUS_TX_BUFFER_SIZE 64
uint8_t modbusRxBuffer[MODBUS_RX_BUFFER_SIZE];
uint8_t modbusTxBuffer[MODBUS_TX_BUFFER_SIZE];
uint8_t rxIndex = 0;

// Переменные для оптимизации скорости
volatile bool frameReady = false;
volatile uint8_t rxBuffer[MODBUS_RX_BUFFER_SIZE];
volatile uint8_t rxBufferIndex = 0;
volatile unsigned long lastRxTime = 0;

// Текущие настройки устройства
uint8_t currentModbusAddress;
uint8_t newModbusAddress;
uint32_t currentBaudRate;
uint8_t currentBaudRateIndex;

// Временные интервалы Modbus (в микросекундах)
unsigned long modbusInterFrameDelayUs;
unsigned long modbusInterCharTimeoutUs;

// Версия прошивки
const uint16_t FIRMWARE_VERSION = 0x0301; // Версия 3.01

// Структура для настроек в EEPROM
struct DeviceSettings {
  uint8_t modbusAddress;
  uint32_t baudRate;
  uint16_t magic;
};

// Предварительное вычисление значений для оптимизации
uint16_t discreteInputValues = 0;
unsigned long lastPinReadTime = 0;
#define PIN_READ_INTERVAL 10 // Чтение пинов каждые 10 мс

// Буфер для выходных значений (пины 3..13)
uint16_t outputPinValues[NUM_OUTPUT_REGISTERS] = {0};

// Прототипы функций
inline void processModbusFrame();
inline void sendResponse(uint8_t* data, uint8_t length);
inline uint16_t calculateCRC(uint8_t *data, uint8_t length);
void applyOutputPins();

void setup() {
  // Инициализация EEPROM
  EEPROM.begin();
  
  // Загрузка настроек из EEPROM
  loadSettingsFromEEPROM();
  
  // Настройка пина управления MAX485
  pinMode(MAX485_RE_DE_PIN, OUTPUT);
  digitalWrite(MAX485_RE_DE_PIN, MAX485_RECEIVE_MODE);
  
  // Настройка последовательного порта с загруженной скоростью
  Serial.begin(currentBaudRate);
  
  // Пересчет таймингов для текущей скорости
  calculateModbusTimings();
  
  // Настройка пинов 3-13 как входы с подтяжкой
  for (int pin = FIRST_INPUT_PIN; pin <= LAST_INPUT_PIN; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }
  
  // Настройка аналоговых пинов A0-A5 как дискретные входы с подтяжкой
  for (int pin = FIRST_ANALOG_PIN; pin <= LAST_ANALOG_PIN; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }
  
  // Инициализация переменных
  rxIndex = 0;
  frameReady = false;
  lastRxTime = micros();
  
  // Первоначальное чтение пинов
  readInputPins();
}

void loop() {
  // Быстрая проверка приема данных Modbus
  checkModbusFrameFast();
  
  // Периодическое чтение пинов
  if (millis() - lastPinReadTime >= PIN_READ_INTERVAL) {
    discreteInputValues = readInputPins();
    lastPinReadTime = millis();
  }
  
  // Обработка готового кадра
  if (frameReady) {
    processModbusFrame();
    frameReady = false;
  }
}

// Проверка приема данных Modbus 
void checkModbusFrameFast() {
  while (Serial.available()) {
    uint8_t data = Serial.read();
    unsigned long currentTime = micros();
    
    // Проверка межсимвольного тайм-аута
    if (rxIndex > 0 && (currentTime - lastRxTime > modbusInterCharTimeoutUs)) {
      rxIndex = 0;
    }
    
    lastRxTime = currentTime;
    
    if (rxIndex < MODBUS_RX_BUFFER_SIZE) {
      modbusRxBuffer[rxIndex++] = data;
    }
  }
  
  // Проверка межкадрового интервала
  if (rxIndex > 0 && (micros() - lastRxTime > modbusInterFrameDelayUs)) {
    frameReady = true;
  }
}

// Загрузка настроек из EEPROM
void loadSettingsFromEEPROM() {
  DeviceSettings settings;
  
  // Чтение структуры из EEPROM
  EEPROM.get(EEPROM_ADDRESS_ADDR, settings);
  
  // Проверка магического числа
  if (settings.magic != EEPROM_MAGIC_VALUE) {
    // Нет сохраненных настроек, используем значения по умолчанию
    currentModbusAddress = DEFAULT_MODBUS_ADDRESS;
    currentBaudRate = DEFAULT_BAUD_RATE;
    currentBaudRateIndex = getBaudRateIndex(DEFAULT_BAUD_RATE);
    
    // Сохраняем настройки по умолчанию
    saveSettingsToEEPROM();
  } else {
    // Используем сохраненные настройки
    currentModbusAddress = settings.modbusAddress;
    currentBaudRate = settings.baudRate;
    currentBaudRateIndex = getBaudRateIndex(currentBaudRate);
    
    // Проверка корректности скорости
    if (currentBaudRateIndex == 0xFF) {
      currentBaudRate = DEFAULT_BAUD_RATE;
      currentBaudRateIndex = getBaudRateIndex(DEFAULT_BAUD_RATE);
      saveSettingsToEEPROM();
    }
  }
  newModbusAddress = currentModbusAddress; 
}

// Получение индекса скорости по значению
inline uint8_t getBaudRateIndex(uint32_t baudRate) {
  for (uint8_t i = 0; i < NUM_SUPPORTED_BAUD_RATES; i++) {
    if (baudRate == SUPPORTED_BAUD_RATES[i]) {
      return i;
    }
  }
  return 0xFF; // Неподдерживаемая скорость
}

// Сохранение настроек в EEPROM
void saveSettingsToEEPROM() {
  DeviceSettings settings;
  settings.modbusAddress = currentModbusAddress;
  settings.baudRate = currentBaudRate;
  settings.magic = EEPROM_MAGIC_VALUE;
  
  EEPROM.put(EEPROM_ADDRESS_ADDR, settings);
}

// Применение новых настроек
void applyNewSettings() {
  // Останавливаем Serial
  Serial.end();
  // Короткая пауза
  delay(1);
  currentModbusAddress = newModbusAddress;
  // Перезапускаем Serial с новой скоростью
  Serial.begin(currentBaudRate);
  
  // Пересчет временных интервалов 
  calculateModbusTimings();
  
  // Сохраняем в EEPROM
  saveSettingsToEEPROM();
}

// Расчет временных интервалов Modbus для текущей скорости
void calculateModbusTimings() {
  // Время одного символа в микросекундах = 11 * 1000000 / baudRate
  uint32_t charTimeUs = (11UL * 1000000UL) / currentBaudRate;
  
  modbusInterFrameDelayUs = (charTimeUs * 35UL) / 10UL; // 3.5 символа
  modbusInterCharTimeoutUs = (charTimeUs * 15UL) / 10UL; // 1.5 символа
}

// Чтение дискретных входов (пины 3-13 и A0-A5)
inline uint16_t readInputPins() {
  uint8_t portD = PIND;  // PORTD - пины 0-7 (Arduino 0-7)
  uint8_t portB = PINB;  // PORTB - пины 8-13 (Arduino 8-13)
  
  uint16_t result = 0;
  // Чтение цифровых пинов 3-13 (биты 0-10)
  result = (portD >> 3) & 0x0F;  // Биты 3-6 → биты 0-3
  result |= ((portD >> 3) & 0x10);  // Бит 7 → бит 4
  result |= ((portB & 0x3F) << 5);  // Биты 0-5 → биты 5-10
  
  // Чтение аналоговых пинов A0-A5 как дискретные входы (биты 11-16)
  // A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19
  for (uint8_t i = 0; i < NUM_ANALOG_PINS; i++) {
    if (digitalRead(FIRST_ANALOG_PIN + i) == HIGH) {
      result |= (1 << (FIRST_ANALOG_DISCRETE_INPUT + i));
    }
  }
  
  return result;
}

// Применение выходных значений на пины (пины 3..13)
void applyOutputPins() {
  for (uint8_t i = 0; i < NUM_OUTPUT_REGISTERS; i++) {
    uint8_t pin = FIRST_INPUT_PIN + i; // 3..13
    uint16_t value = outputPinValues[i];
    
    if (value == 0) {
      // Если значение 0 - переключаем пин обратно во вход
      pinMode(pin, INPUT_PULLUP);
    } else {
      // Если значение не 0 - настраиваем как выход
      pinMode(pin, OUTPUT);
      
      // Пины с поддержкой PWM: 3,5,6,9,10,11
      if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11) {
        // Ограничиваем значение 0-255 для analogWrite
        uint8_t pwmValue = (value > 255) ? 255 : (uint8_t)value;
        analogWrite(pin, pwmValue);
      } else {
        // Для остальных пинов (4,7,8,12,13) - дискретный выход
        // любое ненулевое значение = HIGH
        digitalWrite(pin, HIGH);
      }
    }
  }
}

// Обработка Modbus кадра
inline void processModbusFrame() {
  uint8_t frameSize = rxIndex;
  
  if (frameSize < 8) {
    rxIndex = 0;
    return;
  }
  
  // Проверка адреса устройства
  if (modbusRxBuffer[0] != currentModbusAddress) {
    rxIndex = 0;
    return;
  }
  
  // Проверка CRC
  uint16_t crc = calculateCRC(modbusRxBuffer, frameSize - 2);
  uint16_t receivedCRC = (modbusRxBuffer[frameSize - 1] << 8) | modbusRxBuffer[frameSize - 2];
  
  if (crc != receivedCRC) {
    rxIndex = 0;
    return;
  }
  
  uint8_t functionCode = modbusRxBuffer[1];
  
  // определение функции
  switch (functionCode) {
    case MODBUS_READ_DISCRETE_INPUTS:
      processReadDiscreteInputs();
      break;
      
    case MODBUS_READ_HOLDING_REGISTERS:
      processReadHoldingRegisters();
      break;
      
    case MODBUS_READ_INPUT_REGISTERS:
      processReadInputRegisters();
      break;
      
    case MODBUS_WRITE_SINGLE_REGISTER:
      processWriteSingleRegister();
      break;
      
    case MODBUS_WRITE_MULTIPLE_REGISTERS:
      processWriteMultipleRegisters();
      break;
      
    case MODBUS_WRITE_SINGLE_COIL:
      sendExceptionResponse(MODBUS_WRITE_SINGLE_COIL, 0x01);
      break;
      
    default:
      sendExceptionResponse(functionCode, 0x01);
      break;
  }
  
  rxIndex = 0;
}

// ===== ФУНКЦИЯ 0x03: READ HOLDING REGISTERS =====
inline void processReadHoldingRegisters() {
  uint16_t startAddr = (modbusRxBuffer[2] << 8) | modbusRxBuffer[3];
  uint16_t regCount = (modbusRxBuffer[4] << 8) | modbusRxBuffer[5];
  
  // Проверка запроса с учетом выходных регистров
  if (regCount == 0 || regCount > 125) {
    sendExceptionResponse(MODBUS_READ_HOLDING_REGISTERS, 0x03);
    return;
  }
  
  uint16_t totalRegs = NUM_HOLDING_REGISTERS + NUM_OUTPUT_REGISTERS;
  if (startAddr >= totalRegs || (startAddr + regCount) > totalRegs) {
    sendExceptionResponse(MODBUS_READ_HOLDING_REGISTERS, 0x02);
    return;
  }
  
  uint8_t txIndex = 0;
  modbusTxBuffer[txIndex++] = currentModbusAddress;
  modbusTxBuffer[txIndex++] = MODBUS_READ_HOLDING_REGISTERS;
  modbusTxBuffer[txIndex++] = regCount * 2;
  
  for (uint16_t i = 0; i < regCount; i++) {
    uint16_t regAddr = startAddr + i;
    uint16_t regValue = 0;
    
    if (regAddr < NUM_HOLDING_REGISTERS) {
      // Системные регистры
      switch (regAddr) {
        case REG_MODBUS_ADDRESS:
          regValue = currentModbusAddress;
          break;
        case REG_BAUD_RATE_INDEX:
          regValue = currentBaudRateIndex;
          break;
        case REG_BAUD_RATE_VALUE:
          regValue = currentBaudRate / 100;
          break;
        case REG_DEVICE_STATUS:
          regValue = 0x0001;
          break;
        case REG_FIRMWARE_VERSION:
          regValue = FIRMWARE_VERSION;
          break;
      }
    } else {
      // Регистры выходов
      uint8_t outputIdx = regAddr - REG_OUTPUT_PINS_START;
      if (outputIdx < NUM_OUTPUT_REGISTERS) {
        regValue = outputPinValues[outputIdx];
      }
    }
    
    modbusTxBuffer[txIndex++] = (regValue >> 8) & 0xFF;
    modbusTxBuffer[txIndex++] = regValue & 0xFF;
  }
  
  uint16_t crc = calculateCRC(modbusTxBuffer, txIndex);
  modbusTxBuffer[txIndex++] = crc & 0xFF;
  modbusTxBuffer[txIndex++] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, txIndex);
}

// ===== ФУНКЦИЯ 0x06: WRITE SINGLE REGISTER =====
inline void processWriteSingleRegister() {
  uint16_t regAddr = (modbusRxBuffer[2] << 8) | modbusRxBuffer[3];
  uint16_t regValue = (modbusRxBuffer[4] << 8) | modbusRxBuffer[5];
  
  // Проверка на регистры выходов
  if (regAddr >= REG_OUTPUT_PINS_START && regAddr < REG_OUTPUT_PINS_START + NUM_OUTPUT_REGISTERS) {
    uint8_t outputIdx = regAddr - REG_OUTPUT_PINS_START;
    outputPinValues[outputIdx] = regValue;
    applyOutputPins();
    
    // Эхо-ответ
    modbusTxBuffer[0] = currentModbusAddress;
    modbusTxBuffer[1] = MODBUS_WRITE_SINGLE_REGISTER;
    modbusTxBuffer[2] = (regAddr >> 8) & 0xFF;
    modbusTxBuffer[3] = regAddr & 0xFF;
    modbusTxBuffer[4] = (regValue >> 8) & 0xFF;
    modbusTxBuffer[5] = regValue & 0xFF;
    
    uint16_t crc = calculateCRC(modbusTxBuffer, 6);
    modbusTxBuffer[6] = crc & 0xFF;
    modbusTxBuffer[7] = (crc >> 8) & 0xFF;
    
    sendResponse(modbusTxBuffer, 8);
    return;
  }
  
  // Системные регистры (только 0 и 1)
  if (regAddr >= NUM_HOLDING_REGISTERS) {
    sendExceptionResponse(MODBUS_WRITE_SINGLE_REGISTER, 0x02);
    return;
  }
  
  bool settingChanged = false;
  
  switch (regAddr) {
    case REG_MODBUS_ADDRESS:
      if (regValue >= 1 && regValue <= 247) {
        newModbusAddress = regValue;
        settingChanged = true;
      }
      break;
      
    case REG_BAUD_RATE_INDEX:
      if (regValue < NUM_SUPPORTED_BAUD_RATES) {
        currentBaudRateIndex = regValue;
        currentBaudRate = SUPPORTED_BAUD_RATES[regValue];
        settingChanged = true;
      }
      break;
      
    default:
      sendExceptionResponse(MODBUS_WRITE_SINGLE_REGISTER, 0x02);
      return;
  }
  
  if (settingChanged) {
    modbusTxBuffer[0] = currentModbusAddress;
    modbusTxBuffer[1] = MODBUS_WRITE_SINGLE_REGISTER;
    modbusTxBuffer[2] = (regAddr >> 8) & 0xFF;
    modbusTxBuffer[3] = regAddr & 0xFF;
    modbusTxBuffer[4] = (regValue >> 8) & 0xFF;
    modbusTxBuffer[5] = regValue & 0xFF;
    
    uint16_t crc = calculateCRC(modbusTxBuffer, 6);
    modbusTxBuffer[6] = crc & 0xFF;
    modbusTxBuffer[7] = (crc >> 8) & 0xFF;
    
    sendResponse(modbusTxBuffer, 8);
    applyNewSettings();
    return;
  }
  
  // Эхо-ответ для неизмененных значений
  modbusTxBuffer[0] = currentModbusAddress;
  modbusTxBuffer[1] = MODBUS_WRITE_SINGLE_REGISTER;
  modbusTxBuffer[2] = (regAddr >> 8) & 0xFF;
  modbusTxBuffer[3] = regAddr & 0xFF;
  modbusTxBuffer[4] = (regValue >> 8) & 0xFF;
  modbusTxBuffer[5] = regValue & 0xFF;
  
  uint16_t crc = calculateCRC(modbusTxBuffer, 6);
  modbusTxBuffer[6] = crc & 0xFF;
  modbusTxBuffer[7] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, 8);
}

// ===== ФУНКЦИЯ 0x10: WRITE MULTIPLE REGISTERS =====
inline void processWriteMultipleRegisters() {
  uint16_t startAddr = (modbusRxBuffer[2] << 8) | modbusRxBuffer[3];
  uint16_t regCount = (modbusRxBuffer[4] << 8) | modbusRxBuffer[5];
  uint8_t byteCount = modbusRxBuffer[6];
  
  // Проверка количества регистров
  if (regCount == 0 || regCount > 0x007B || byteCount != regCount * 2) {
    sendExceptionResponse(MODBUS_WRITE_MULTIPLE_REGISTERS, 0x03);
    return;
  }
  
  uint16_t totalRegs = NUM_HOLDING_REGISTERS + NUM_OUTPUT_REGISTERS;
  if (startAddr >= totalRegs || (startAddr + regCount) > totalRegs) {
    sendExceptionResponse(MODBUS_WRITE_MULTIPLE_REGISTERS, 0x02);
    return;
  }
  
  bool settingsChanged = false;
  bool outputsChanged = false;
  
  // Обработка данных
  for (uint16_t i = 0; i < regCount; i++) {
    uint16_t regAddr = startAddr + i;
    uint16_t regValue = (modbusRxBuffer[7 + i * 2] << 8) | modbusRxBuffer[8 + i * 2];
    
    if (regAddr < NUM_HOLDING_REGISTERS) {
      // Системные регистры
      switch (regAddr) {
        case REG_MODBUS_ADDRESS:
          if (regValue >= 1 && regValue <= 247) {
            newModbusAddress = regValue;
            settingsChanged = true;
          }
          break;
        case REG_BAUD_RATE_INDEX:
          if (regValue < NUM_SUPPORTED_BAUD_RATES) {
            currentBaudRateIndex = regValue;
            currentBaudRate = SUPPORTED_BAUD_RATES[regValue];
            settingsChanged = true;
          }
          break;
        // Остальные системные регистры только для чтения
        default:
          sendExceptionResponse(MODBUS_WRITE_MULTIPLE_REGISTERS, 0x02);
          return;
      }
    } else {
      // Регистры выходов
      uint8_t outputIdx = regAddr - REG_OUTPUT_PINS_START;
      if (outputIdx < NUM_OUTPUT_REGISTERS) {
        outputPinValues[outputIdx] = regValue;
        outputsChanged = true;
      }
    }
  }
  
  // Применяем выходы
  if (outputsChanged) {
    applyOutputPins();
  }
  
  // Отправляем успешный ответ (эхо)
  modbusTxBuffer[0] = currentModbusAddress;
  modbusTxBuffer[1] = MODBUS_WRITE_MULTIPLE_REGISTERS;
  modbusTxBuffer[2] = (startAddr >> 8) & 0xFF;
  modbusTxBuffer[3] = startAddr & 0xFF;
  modbusTxBuffer[4] = (regCount >> 8) & 0xFF;
  modbusTxBuffer[5] = regCount & 0xFF;
  
  uint16_t crc = calculateCRC(modbusTxBuffer, 6);
  modbusTxBuffer[6] = crc & 0xFF;
  modbusTxBuffer[7] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, 8);
  
  // Применяем настройки после отправки ответа
  if (settingsChanged) {
    applyNewSettings();
  }
}

// ===== ФУНКЦИЯ 0x02: READ DISCRETE INPUTS =====
inline void processReadDiscreteInputs() {
  uint16_t startAddr = (modbusRxBuffer[2] << 8) | modbusRxBuffer[3];
  uint16_t inputCount = (modbusRxBuffer[4] << 8) | modbusRxBuffer[5];
  
  // Общее количество дискретных входов (цифровые 11 + аналоговые 6)
  const uint16_t TOTAL_DISCRETE_INPUTS = NUM_INPUT_PINS + NUM_ANALOG_PINS;
  
  // Быстрая проверка
  if (inputCount == 0 || inputCount > 2000 ||
      startAddr < FIRST_DISCRETE_INPUT || 
      (startAddr + inputCount) > (FIRST_DISCRETE_INPUT + TOTAL_DISCRETE_INPUTS)) {
    sendExceptionResponse(MODBUS_READ_DISCRETE_INPUTS, 
                         (inputCount == 0 || inputCount > 2000) ? 0x03 : 0x02);
    return;
  }
  
  // Используем предварительно прочитанные значения
  uint16_t pinValues = discreteInputValues;
  
  uint8_t byteCount = (inputCount + 7) / 8;
  uint8_t txIndex = 0;
  
  modbusTxBuffer[txIndex++] = currentModbusAddress;
  modbusTxBuffer[txIndex++] = MODBUS_READ_DISCRETE_INPUTS;
  modbusTxBuffer[txIndex++] = byteCount;
  
  // Упаковка битов
  for (uint8_t byteIndex = 0; byteIndex < byteCount; byteIndex++) {
    uint8_t byteValue = 0;
    
    for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
      uint16_t currentInput = startAddr + (byteIndex * 8) + bitIndex;
      
      if (currentInput < startAddr + inputCount) {
        if (pinValues & (1 << currentInput)) {
          byteValue |= (1 << bitIndex);
        }
      }
    }
    
    modbusTxBuffer[txIndex++] = byteValue;
  }
  
  uint16_t crc = calculateCRC(modbusTxBuffer, txIndex);
  modbusTxBuffer[txIndex++] = crc & 0xFF;
  modbusTxBuffer[txIndex++] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, txIndex);
}

// ===== ФУНКЦИЯ 0x04: READ INPUT REGISTERS =====
inline void processReadInputRegisters() {
  uint16_t startAddr = (modbusRxBuffer[2] << 8) | modbusRxBuffer[3];
  uint16_t regCount = (modbusRxBuffer[4] << 8) | modbusRxBuffer[5];
  
  // Быстрая проверка
  if (regCount == 0 || regCount > 125 || startAddr != 0 || regCount > 1) {
    sendExceptionResponse(MODBUS_READ_INPUT_REGISTERS, 
                         (regCount == 0 || regCount > 125) ? 0x03 : 0x02);
    return;
  }
  
  // Используем предварительно прочитанные значения
  uint16_t pinValues = discreteInputValues;
  
  modbusTxBuffer[0] = currentModbusAddress;
  modbusTxBuffer[1] = MODBUS_READ_INPUT_REGISTERS;
  modbusTxBuffer[2] = 0x02;
  modbusTxBuffer[3] = (pinValues >> 8) & 0xFF;
  modbusTxBuffer[4] = pinValues & 0xFF;
  
  uint16_t crc = calculateCRC(modbusTxBuffer, 5);
  modbusTxBuffer[5] = crc & 0xFF;
  modbusTxBuffer[6] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, 7);
}

// Отправка исключения
inline void sendExceptionResponse(uint8_t functionCode, uint8_t exceptionCode) {
  modbusTxBuffer[0] = currentModbusAddress;
  modbusTxBuffer[1] = functionCode | 0x80;
  modbusTxBuffer[2] = exceptionCode;
  
  uint16_t crc = calculateCRC(modbusTxBuffer, 3);
  modbusTxBuffer[3] = crc & 0xFF;
  modbusTxBuffer[4] = (crc >> 8) & 0xFF;
  
  sendResponse(modbusTxBuffer, 5);
}

// Быстрая отправка ответа с минимальными задержками
inline void sendResponse(uint8_t* data, uint8_t length) {
  // Переключение в режим передачи
  digitalWrite(MAX485_RE_DE_PIN, MAX485_TRANSMIT_MODE);
  
  // Короткая пауза для стабилизации переключения MAX485
  delayMicroseconds(10);
  
  // Отправка данных
  Serial.write(data, length);
  Serial.flush();
  
  // Переключение обратно в режим приема
  digitalWrite(MAX485_RE_DE_PIN, MAX485_RECEIVE_MODE);
  
  // Минимальная пауза для завершения передачи (2 символа)
  delayMicroseconds((2000000UL / currentBaudRate) + 10);
}

// Оптимизированный расчет CRC16 Modbus
inline uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  
  while (length--) {
    crc ^= *data++;
    
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}
