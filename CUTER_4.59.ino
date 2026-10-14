/*
 * CUTER 4.54 - Станок для резки проводов
 * Управление шаговым двигателем NEMA 17 через A4988
 * и сервоприводом MG996R для режущего механизма
 *
 * Copyright (C)2025 Alexey Lukin
 *
 * Проект использует библиотеки:
 * - LiquidCrystal_I2C для дисплея 1602
 * - Keypad для клавиатуры 4x4
 * - Servo для управления сервоприводом
 * - Custom StepperDriver для управления шаговым двигателем
 */
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Servo.h>
#include "A4988.h"
#include <string.h>

// ==================================== КОНСТАНТЫ ВРЕМЕНИ КОМПИЛЯЦИИ ==================================== //

// Адреса EEPROM для хранения настроек
constexpr uint8_t SET_KEY_EEPROM = 46;
constexpr uint8_t SET_KEY_ADDR_EEPROM = 0;
constexpr uint8_t UP_ANGLE_ADDR_EEPROM = 4;
constexpr uint8_t DOWN_ANGLE_ADDR_EEPROM = 8;
constexpr uint8_t SERVO_MOVE_TIME_ADDR_EEPROM = 12;
constexpr uint8_t MICROSTEPS_ADDR_EEPROM = 16;
constexpr uint8_t DIAMETER_PULLEY_ADDR_EEPROM = 20;

// Настройки сервопривода по умолчанию
constexpr uint8_t UP_ANGLE_DEFAULT = 124;         // Угол открытия сервы по умолчанию
constexpr uint8_t DOWN_ANGLE_DEFAULT = 148;       // Угол закрытия сервы по умолчанию
constexpr uint16_t SERVO_MOVE_TIME_DEFAULT = 1850; // Время движения сервы по умолчанию
constexpr uint8_t SERVO_ANGLE_MIN = 0;            // Минимальный угол сервы
constexpr uint8_t SERVO_ANGLE_MAX = 180;          // Максимальный угол сервы
constexpr uint16_t SERVO_MOVE_TIME_MIN = 1000;    // Минимальное время движения сервы
constexpr uint16_t SERVO_MOVE_TIME_MAX = 2500;    // Максимальное время движения сервы
constexpr uint8_t SERVO_PIN = 10;                 // Пин подключения сервы

// Настройки шагового двигателя по умолчанию
constexpr float DIAMETER_PULLEY_DEFAULT = 20.65f; // Диаметр шкива по умолчанию (мм)
constexpr float DIAMETER_PULLEY_MIN = 5.0f;       // Минимальный диаметр шкива
constexpr float DIAMETER_PULLEY_MAX = 70.0f;      // Максимальный диаметр шкива
constexpr uint16_t STEPS_PER_ROUND = 200;         // Шагов на оборот двигателя (1.8°)

constexpr uint8_t MICROSTEPS_DEFAULT = 8;         // Микрошаги по умолчанию
constexpr uint8_t MICROSTEPS_MIN = 1;             // Минимальные микрошаги
constexpr uint8_t MICROSTEPS_MAX = 16;            // Максимальные микрошаги (для A4988)
constexpr uint16_t REVOLUTIONS_PER_MINUTE = 200;  // Оборотов в минуту

// Пины подключения драйвера шагового двигателя A4988
constexpr uint8_t STEP_PIN = 7;                   // Пин STEP драйвера
constexpr uint8_t DIR_PIN = 8;                    // Пин DIR драйвера  
constexpr uint8_t ENABLE_PIN = 5;                 // Пин ENABLE драйвера
constexpr uint8_t MS1_PIN = 4;                    // Пин MS1 драйвера
constexpr uint8_t MS2_PIN = 3;                    // Пин MS2 драйвера
constexpr uint8_t MS3_PIN = 2;                    // Пин MS3 драйвера

// Настройки дисплея LCD 1602 I2C
constexpr uint8_t LCD_ADDR_I2C = 0x27;            // Адрес I2C дисплея
constexpr uint8_t LCD_COLS = 16;                  // Количество столбцов дисплея
constexpr uint8_t LCD_ROWS = 2;                   // Количество строк дисплея
constexpr uint8_t CLEAR_SCREEN = 2;               // Флаг очистки всего экрана
constexpr uint8_t CLEAR_ROW = 1;                  // Флаг очистки строки

// Настройки клавиатуры 4x4
constexpr uint8_t KPD_COLS = 4;                   // Количество столбцов клавиатуры
constexpr uint8_t KPD_ROWS = 4;                   // Количество строк клавиатуры

// Пины подключения клавиатуры
constexpr uint8_t KPD_COL_PINS[KPD_COLS] = {17, 16, 15, 14}; // Столбцы (аноды)
constexpr uint8_t KPD_ROW_PINS[KPD_ROWS] = {9, 13, 11, 12};  // Строки (катоды)

// Матрица клавиш клавиатуры
constexpr char KPD_KEYS[KPD_ROWS][KPD_COLS] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '.', '0', '#', 'D' }
};

// Ограничения ввода
constexpr uint32_t LENGTH_MAX = 14000;            // Максимальная длина провода (мм)
constexpr uint32_t QUANTITY_MAX = 500;            // Максимальное количество отрезков

// ==================================== КАСТОМНЫЕ СИМВОЛЫ ДЛЯ КИРИЛЛИЦЫ ==================================== //

// Создание пользовательских символов для отображения русских букв
constexpr uint8_t CUSTOM_CHARS[7][8] = {
    {0b11111,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b00000}, // Г \1
    {0b00111,0b01001,0b01001,0b01001,0b01001,0b01001,0b11111,0b10001}, // Д \2
    {0b10001,0b10001,0b10011,0b10101,0b11001,0b10001,0b10001,0b00000}, // И \3
    {0b00011,0b00101,0b01001,0b01001,0b01001,0b01001,0b10001,0b00000}, // Л \4
    {0b11111,0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b00000}, // П \5
    {0b10001,0b10001,0b10001,0b01111,0b00001,0b00001,0b00001,0b00000}, // Ч \6
    {0b10101,0b10101,0b10101,0b10101,0b10101,0b10101,0b11111,0b00000}, // Ш \7
};

// Текстовые константы с использованием кастомных символов
constexpr char _LOGO[] = "HAPE3A\4O\6KA 4.59";        // Логотип программы
constexpr char _SET[] = "HACTPO\3K\3";                // Пункт меню "Настройки"
constexpr char _WORK[] = "              >";           // Пункт меню "Работа"
constexpr char _DIAMETER_PULLEY[] = "\2\3AMETP \7K\3BA"; // Настройка диаметра шкива
constexpr char _DOWN_ANGLE[] = "H\3>KH\3\3 Y\1O\4";   // Настройка угла закрытия
constexpr char _UP_ANGLE[] = "BEPXH\3\3 Y\1O\4";      // Настройка угла открытия
constexpr char _SERVO_MOVE_TIME[] = "CEPBA 0-180";    // Настройка времени сервы
constexpr char _MICROSTEPS[] = "M\3KPO\7A\1\3";       // Настройка микрошагов
constexpr char _EXIT[] = "BbIXO\2";                   // Выход из меню
constexpr char _LENGTH[] = "\2\4\3HA \5POBO\2A";      // Запрос длины провода
constexpr char _QUANTITY[] = "CKO\4bKO \7TYK";        // Запрос количества отрезков
constexpr char _CONFIRM[] = "<OTMEHA | CTAPT>";       // Подтверждение действий
constexpr char _SHT[] = "\7T";                        // Единица "штуки"
constexpr char _MM[] = "MM";                          // Единица "миллиметры"
constexpr char _PAUSE[] = "     \5AY3A!";             // Сообщение "ПАУЗА"

// ==================================== ОБЪЯВЛЕНИЕ ОБЪЕКТОВ ==================================== //

// Объекты периферии
Servo snippers;                                     // Объект сервопривода
LiquidCrystal_I2C lcd(LCD_ADDR_I2C, LCD_COLS, LCD_ROWS); // Объект дисплея

// Инициализация клавиатуры
Keypad kpd = Keypad(
    makeKeymap(KPD_KEYS), 
    const_cast<uint8_t*>(KPD_ROW_PINS), 
    const_cast<uint8_t*>(KPD_COL_PINS), 
    KPD_ROWS, 
    KPD_COLS
);

// Драйвер шагового двигателя A4988
A4988 stepper(STEPS_PER_ROUND, DIR_PIN, STEP_PIN, ENABLE_PIN, MS1_PIN, MS2_PIN, MS3_PIN);

// ==================================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==================================== //

// Переменные настроек
uint32_t wireLength = 0;                            // Длина провода для резки
uint32_t wireQuantity = 0;                          // Количество отрезков
uint16_t servoMoveTime = 0;                         // Время движения сервопривода
uint8_t downAngle = 0;                              // Угол закрытия сервы
uint8_t upAngle = 0;                                // Угол открытия сервы
uint8_t microSteps = 0;                             // Уровень микрошагов
float diameterPulley = 0.0f;                        // Диаметр шкива (мм)
bool fSet = true;                                   // Флаг доступа к настройкам

// Расчетные параметры
float stepPerMM = 0.0f;                             // Шагов на миллиметр
uint16_t servoDelay = 0;                            // Задержка для сервопривода

// Буферы для ввода/вывода
char inputBuffer[17] = "";                          // Буфер для ввода данных
char displayBuffer[17] = "";                        // Буфер для вывода на дисплей

// ==================================== ПЕРЕЧИСЛЕНИЯ И СТРУКТУРЫ ==================================== //

// Режимы работы программы
enum class Mode : uint8_t {
    CHOOSE_SET,          // Выбор режима (настройки/работа)
    SETTINGS,            // Меню настроек
    INPUT_LENGTH,        // Ввод длины провода
    INPUT_QUANTITY,      // Ввод количества отрезков
    CONFIRM,             // Подтверждение действий
    CUTTING              // Процесс резки
};

Mode currentMode = Mode::CHOOSE_SET;                // Текущий режим работы

// ==================================== ПРОТОТИПЫ ФУНКЦИЙ ==================================== //

// Функции работы с дисплеем
void printLCD(const char* str, uint8_t col, uint8_t row, uint8_t option = 0);
void initLCD();

// Функции работы с EEPROM
template<typename T>
void safeEEPROMWrite(uint8_t eepromAddr, const T& value);
void exchangeDataEEPROM();
void calculateParam();

// Функции ввода данных
char* inputValue(const char* str, uint8_t dCol, uint8_t dRow);
template<typename T>
T chooseValue(const char* prompt, T minValue, T maxValue);
template<typename T>
T chooseSetValue(const char* prompt, T minValue, T maxValue, uint8_t eepromAddr);

// Функции меню и навигации
void showMenu(const char** menuOptions, uint8_t menuSize, const char* title, Mode parentMode);
void handleMenuSelection(const char* selectedOption, Mode parentMode);

// Функции управления оборудованием
void changingTheWire(char key);
void initStepper();
void initServo();
bool activatingServo();
bool activatingStepper(int32_t stepsToTake);

// ==================================== РЕАЛИЗАЦИЯ ФУНКЦИЙ ==================================== //

/*
 * Функция вывода текста на дисплей
 * str - строка для вывода, col - столбец, row - строка, option - опция очистки
 */
void printLCD(const char* str, uint8_t col, uint8_t row, uint8_t option) {
    if (option == CLEAR_SCREEN) {
        lcd.clear(); // Очистка всего экрана
    }
    
    lcd.setCursor(col, row); // Установка курсора
    
    // Копирование строки в буфер для безопасного вывода
    strncpy(displayBuffer, str, sizeof(displayBuffer) - 1);
    displayBuffer[sizeof(displayBuffer) - 1] = '\0';
    
    lcd.print(displayBuffer); // Вывод строки

    // Заполнение остатка строки пробелами при необходимости
    if (option == CLEAR_ROW || strlen(str) < LCD_COLS) {
        uint8_t currentLength = strlen(str);
        for (uint8_t i = currentLength; i < LCD_COLS; i++) {
            lcd.print(" ");
        }
    }
}

/*
 * Безопасная запись данных в EEPROM
 * eepromAddr - адрес в EEPROM, value - значение для записи
 */
template<typename T>
void safeEEPROMWrite(uint8_t eepromAddr, const T& value) {
    T currentValue;
    EEPROM.get(eepromAddr, currentValue);
    
    if (currentValue != value) {
        EEPROM.put(eepromAddr, value);
    }
}

/*
 * Расчет статичных параметров на основе текущих настроек
 */
void calculateParam() {
    // Расчет шагов на миллиметр: (шаги/оборот * микрошаги) / (π * диаметр)
    stepPerMM = (STEPS_PER_ROUND * microSteps) / (3.1415926535f * diameterPulley);
    
    // Расчет времени задержки сервы пропорционально углу поворота
    servoDelay = servoMoveTime * abs(upAngle - downAngle) / 180;
}

/*
 * Обмен данными с EEPROM - чтение настроек при запуске
 */
void exchangeDataEEPROM() {
    // Проверка наличия сохраненных настроек по ключу
    if (EEPROM.read(SET_KEY_ADDR_EEPROM) != SET_KEY_EEPROM) {
        // Инициализация EEPROM значениями по умолчанию
        safeEEPROMWrite(SET_KEY_ADDR_EEPROM, SET_KEY_EEPROM);
        safeEEPROMWrite(DIAMETER_PULLEY_ADDR_EEPROM, DIAMETER_PULLEY_DEFAULT);
        safeEEPROMWrite(DOWN_ANGLE_ADDR_EEPROM, DOWN_ANGLE_DEFAULT);
        safeEEPROMWrite(UP_ANGLE_ADDR_EEPROM, UP_ANGLE_DEFAULT);
        safeEEPROMWrite(SERVO_MOVE_TIME_ADDR_EEPROM, SERVO_MOVE_TIME_DEFAULT);
        safeEEPROMWrite(MICROSTEPS_ADDR_EEPROM, MICROSTEPS_DEFAULT);
    }
    
    // Чтение настроек из EEPROM
    EEPROM.get(DIAMETER_PULLEY_ADDR_EEPROM, diameterPulley);
    EEPROM.get(DOWN_ANGLE_ADDR_EEPROM, downAngle);
    EEPROM.get(UP_ANGLE_ADDR_EEPROM, upAngle);
    EEPROM.get(SERVO_MOVE_TIME_ADDR_EEPROM, servoMoveTime);
    EEPROM.get(MICROSTEPS_ADDR_EEPROM, microSteps);
    
    // Пересчет параметров на основе загруженных настроек
    calculateParam();
}

/*
 * Функция ввода числового значения с клавиатуры
 * str - начальное значение, dCol - столбец, dRow - строка
 * возвращает указатель на буфер с введенным значением
 */
char* inputValue(const char* str, uint8_t dCol, uint8_t dRow) {
    strcpy(inputBuffer, str); // Копирование начального значения
    
    printLCD(inputBuffer, dCol, dRow); // Отображение начального значения
    
    while (true) {
        char key = kpd.getKey(); // Ожидание нажатия клавиши
        
        if (key == NO_KEY) {
            continue; // Пропуск если клавиша не нажата
        }
        
        // Управление шаговым двигателем при ручном вводе
        changingTheWire(key);
        
        // Обработка цифрового ввода
        if (key >= '0' && key <= '9' && strlen(inputBuffer) < 7) {
            strncat(inputBuffer, &key, 1); // Добавление цифры
        } 
        // Обработка десятичной точки (только в режиме настроек)
        else if (key == '.' && fSet && strchr(inputBuffer, '.') == NULL && strlen(inputBuffer) < 7) {
            strncat(inputBuffer, &key, 1); // Добавление точки
        } 
        // Обработка удаления символа (клавиша C)
        else if (key == 'C' && strlen(inputBuffer) > 0) {
            inputBuffer[strlen(inputBuffer) - 1] = '\0'; // Удаление последнего символа
        } 
        // Обработка подтверждения ввода (клавиша D)
        else if (key == 'D') {
            // Удаление точки если она последняя
            if (strlen(inputBuffer) > 0 && inputBuffer[strlen(inputBuffer) - 1] == '.') {
                inputBuffer[strlen(inputBuffer) - 1] = '\0';
            }
            break; // Завершение ввода
        }
        
        // Обновление отображения при изменении ввода
        if (key != NO_KEY) {
            printLCD(inputBuffer, dCol, dRow);
        }
    }
    
    return inputBuffer;
}

/*
 * Функция выбора числового значения с ограничениями
 * prompt - приглашение, minValue - минимум, maxValue - максимум
 */
template<typename T>
T chooseValue(const char* prompt, T minValue, T maxValue) {
    printLCD(prompt, 0, 0, CLEAR_SCREEN);
    
    char* input = inputValue("", 0, 1);
    T value = static_cast<T>(atof(input)); // Преобразование ввода в число
    
    // Ограничение значения диапазоном
    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;
    
    return value;
}

/*
 * Функция выбора и сохранения значения в EEPROM
 * prompt - приглашение, min/max - ограничения, eepromAddr - адрес сохранения
 */
template<typename T>
T chooseSetValue(const char* prompt, T minValue, T maxValue, uint8_t eepromAddr) {
    T value;
    
    printLCD(prompt, 0, 0, CLEAR_SCREEN);
    
    // Чтение текущего значения из EEPROM
    EEPROM.get(eepromAddr, value);
    
    // Специальная обработка для разных типов данных
    if (strcmp(prompt, _DIAMETER_PULLEY) == 0) {
        // Для диаметра шкива используем формат с плавающей точкой
        char floatStr[10];
        dtostrf(value, 7, 2, floatStr);
        strcpy(inputBuffer, inputValue(floatStr, 0, 1));
        value = constrain(static_cast<T>(atof(inputBuffer)), minValue, maxValue);
    } else {
        // Для целочисленных значений
        char intStr[10];
        itoa(value, intStr, 10);
        strcpy(inputBuffer, inputValue(intStr, 0, 1));
        value = constrain(static_cast<T>(atoi(inputBuffer)), minValue, maxValue);
        
        // Особенная обработка для микрошагов (выравнивание по степеням двойки)
        if (strcmp(prompt, _MICROSTEPS) == 0) {
            uint16_t logVal = static_cast<uint16_t>(log(value) / log(2));
            value = 1 << logVal; // Выравнивание до ближайшей степени двойки
            stepper.setMicrostep(value); // Немедленное применение к драйверу
        }
    }
    
    // Сохранение нового значения в EEPROM
    safeEEPROMWrite(eepromAddr, value);
    calculateParam(); // Пересчет производных параметров
    
    return value;
}

/*
 * Универсальная функция отображения меню
 * menuOptions - массив пунктов, menuSize - размер, title - заголовок
 */
void showMenu(const char** menuOptions, uint8_t menuSize, const char* title, Mode parentMode) {
    static uint8_t selectedOption = 0;    // Индекс выбранного пункта
    static bool needsUpdate = true;       // Флаг необходимости обновления дисплея
    
    // Первичный вывод или обновление при изменении
    if (needsUpdate) {
        printLCD(title, 0, 0, CLEAR_SCREEN);
        printLCD(":", strlen(title) + 1, 0);
        printLCD(menuOptions[selectedOption], 0, 1);
        printLCD(">", 15, 1);
        needsUpdate = false;
    }
    
    char key = kpd.getKey();
    
    // Управление двигателем при навигации
    changingTheWire(key);
    
    // Обработка навигации по меню
    if (key == '#') {
        selectedOption = (selectedOption + 1) % menuSize;
        needsUpdate = true;
    }
    
    // Обработка выбора пункта меню
    if (key == 'D') {
        handleMenuSelection(menuOptions[selectedOption], parentMode);
        needsUpdate = true;
    }
}

/*
 * Функция управления шаговым двигателем при ручном вводе
 * key - нажатая клавиша (A/B для движения, отпускание для остановки)
 */
void changingTheWire(char key) {
    static bool rotationStepper = false; // Флаг вращения двигателя
    static int16_t direction = 0;        // Направление вращения
    
    // Обработка нажатия кнопок ручного управления
    if (key == 'A' || key == 'B') {
        if (kpd.getState() == PRESSED || kpd.getState() == HOLD) {
            direction = (key == 'A') ? -1 : 1;
            rotationStepper = true;
        }
    } 
    // Остановка при отпускании кнопки
    else if (kpd.getState() == RELEASED) {
        rotationStepper = false;
        direction = 0;
    }
    
    // Управление двигателем в зависимости от состояния
    if (rotationStepper && direction != 0) {
        // Временное увеличение микрошагов для плавного ручного управления
        stepper.setMicrostep(16);
        activatingStepper(direction * 10); // Малые шаги для точного позиционирования
    } else {
        stepper.stop(); // Остановка двигателя
    }
}

/*
 * Обработка выбора пункта меню
 * selectedOption - выбранный пункт, parentMode - родительский режим
 */
void handleMenuSelection(const char* selectedOption, Mode parentMode) {
    // Обработка основных пунктов меню
    if (strcmp(selectedOption, _EXIT) == 0) {
        currentMode = parentMode; // Возврат к родительскому меню
    } else if (strcmp(selectedOption, _SET) == 0) {
        currentMode = Mode::SETTINGS; // Переход в настройки
    } else if (strcmp(selectedOption, _WORK) == 0) {
        fSet = false; // Отключение режима настроек
        currentMode = Mode::INPUT_LENGTH; // Начало рабочего процесса
    } 
    // Обработка пунктов настроек
    else if (strcmp(selectedOption, _DIAMETER_PULLEY) == 0) {
        diameterPulley = chooseSetValue<float>(_DIAMETER_PULLEY, DIAMETER_PULLEY_MIN, DIAMETER_PULLEY_MAX, DIAMETER_PULLEY_ADDR_EEPROM);
    } else if (strcmp(selectedOption, _DOWN_ANGLE) == 0) {
        downAngle = chooseSetValue<uint8_t>(_DOWN_ANGLE, upAngle + 1, SERVO_ANGLE_MAX, DOWN_ANGLE_ADDR_EEPROM);
    } else if (strcmp(selectedOption, _UP_ANGLE) == 0) {
        upAngle = chooseSetValue<uint8_t>(_UP_ANGLE, SERVO_ANGLE_MIN, downAngle - 1, UP_ANGLE_ADDR_EEPROM);
    } else if (strcmp(selectedOption, _SERVO_MOVE_TIME) == 0) {
        servoMoveTime = chooseSetValue<uint16_t>(_SERVO_MOVE_TIME, SERVO_MOVE_TIME_MIN, SERVO_MOVE_TIME_MAX, SERVO_MOVE_TIME_ADDR_EEPROM);
    } else if (strcmp(selectedOption, _MICROSTEPS) == 0) {
        microSteps = chooseSetValue<uint16_t>(_MICROSTEPS, MICROSTEPS_MIN, MICROSTEPS_MAX, MICROSTEPS_ADDR_EEPROM);
    }
}

/*
 * Управление сервоприводом для резки провода
 * возвращает true если серва все еще движется
 */
bool activatingServo() {
    static uint32_t lastMoveTime = 0;     // Время начала движения
    static bool isMoving = false;         // Флаг активности движения
    static uint8_t currentState = 0;      // Текущее состояние (0-вверх, 1-вниз)
    
    if (!isMoving) {
        // Начало нового цикла движения
        isMoving = true;
        currentState = 1; // Начинаем с движения вниз
        lastMoveTime = millis();
        snippers.write(downAngle); // Движение вниз для резки
    } else {
        uint32_t currentTime = millis();
        
        // Проверка времени для смены состояния
        if (currentTime - lastMoveTime >= servoDelay) {
            if (currentState == 1) {
                // Переход к движению вверх после резки
                snippers.write(upAngle);
                currentState = 0;
                lastMoveTime = currentTime;
            } else {
                // Завершение цикла движения
                isMoving = false;
            }
        }
    }
    
    return isMoving;
}

/*
 * Управление шаговым двигателем
 * stepsToTake - количество шагов для выполнения
 * возвращает true если двигатель все еще движется
 */
bool activatingStepper(int32_t stepsToTake) {
    static bool isRunning = false; // Флаг активности двигателя
    
    if (!isRunning && stepsToTake != 0) {
        // Начало нового движения
        isRunning = true;
        stepper.startMove(stepsToTake);
    } 
    // Проверка завершения движения
    else if (stepper.nextAction() <= 0) {
        if (stepper.getStepsRemaining() == 0) {
            isRunning = false;
        }
    }
    
    return isRunning;
}

/*
 * Инициализация дисплея LCD 1602 I2C
 */
void initLCD() {
    lcd.init();         // Инициализация дисплея
    lcd.backlight();    // Включение подсветки
    
    // Создание пользовательских символов для русского текста
    for (uint8_t i = 0; i < 7; i++) {
        lcd.createChar(i + 1, const_cast<uint8_t*>(CUSTOM_CHARS[i]));
    }
}

/*
 * Инициализация шагового двигателя и драйвера A4988
 */
void initStepper() {
    stepper.begin(REVOLUTIONS_PER_MINUTE);    // Инициализация с базовой скоростью
    stepper.setMicrostep(microSteps);         // Установка микрошагов
    stepper.setEnableActiveState(LOW);        // Активный уровень пина ENABLE
    stepper.enable();                         // Включение драйвера
    stepper.setSpeedProfile(BasicStepperDriver::Mode::LINEAR_SPEED); // Профиль с ускорением
}

/*
 * Инициализация сервопривода
 */
void initServo() {
    snippers.attach(SERVO_PIN);   // Подключение к пину
    snippers.write(upAngle);      // Установка в начальное положение
    delay(500);                   // Задержка для стабилизации
}

/*
 * Функция setup() - инициализация системы
 */
void setup() {
    // Инициализация периферии
    exchangeDataEEPROM(); // Загрузка настроек из EEPROM
    initLCD();            // Инициализация дисплея
    initStepper();        // Инициализация шагового двигателя
    initServo();          // Инициализация сервопривода
    
    // Приветственное сообщение
    printLCD(_LOGO, 0, 0, CLEAR_SCREEN);
    printLCD("3A\1PY\6EHA", 4, 1);
    delay(2000); // Задержка для отображения приветствия
}

/*
 * Главный цикл программы - управление состояниями
 */
void loop() {
    // Основной конечный автомат программы
    switch (currentMode) {
        case Mode::CHOOSE_SET: {
            // Главное меню выбора режима
            const char* mainMenuOptions[] = { _WORK, _SET };
            showMenu(mainMenuOptions, 2, _LOGO, Mode::CHOOSE_SET);
            break;
        }
        
        case Mode::SETTINGS: {
            // Меню настроек
            const char* settingsMenuOptions[] = { 
                _EXIT, _DIAMETER_PULLEY, _DOWN_ANGLE, _UP_ANGLE, 
                _SERVO_MOVE_TIME, _MICROSTEPS 
            };
            showMenu(settingsMenuOptions, 6, _SET, Mode::CHOOSE_SET);
            break;
        }
        
        case Mode::INPUT_LENGTH: {
            // Ввод длины провода
            wireLength = chooseValue<uint32_t>(_LENGTH, 1, LENGTH_MAX);
            currentMode = Mode::INPUT_QUANTITY;
            break;
        }
        
        case Mode::INPUT_QUANTITY: {
            // Ввод количества отрезков
            wireQuantity = chooseValue<uint32_t>(_QUANTITY, 1, QUANTITY_MAX);
            currentMode = Mode::CONFIRM;
            break;
        }
        
        case Mode::CONFIRM: {
            // Подтверждение параметров перед началом резки
            char confirmStr[17];
            snprintf(confirmStr, sizeof(confirmStr), "%lu%s x %lu%s", 
                     wireQuantity, _SHT, wireLength, _MM);
            printLCD(confirmStr, 0, 0, CLEAR_SCREEN);
            printLCD(_CONFIRM, 0, 1);
            
            while (true) {
                char key = kpd.getKey();
                changingTheWire(key); // Ручное управление во время подтверждения
                
                if (key == 'C') {
                    // Отмена - сброс параметров и возврат к вводу длины
                    wireLength = 0;
                    wireQuantity = 0;
                    currentMode = Mode::INPUT_LENGTH;
                    return;
                } else if (key == 'D') {
                    // Подтверждение - тестовое движение сервы
                    while (activatingServo()) {
                        // Ожидание завершения тестового цикла
                    }
                    break;
                }
            }
            
            currentMode = Mode::CUTTING; // Переход к режиму резки
            break;
        }
        
        case Mode::CUTTING: {
            // Основной режим резки провода
            static bool isStepperRunning = false;
            static bool isServoRunning = false;
            static int32_t stepsToTake = 0;
            static bool isPaused = false;
            static uint16_t cuttingCounter = 0;
            
            // Инициализация параметров резки при первом входе
            if (cuttingCounter == 0 && !isStepperRunning && !isServoRunning) {
                stepsToTake = static_cast<int32_t>(wireLength * stepPerMM + 0.5f);
                
                char statusStr[17];
                snprintf(statusStr, sizeof(statusStr), "%lu%s x %u%s", 
                         wireQuantity, _SHT, cuttingCounter, _SHT);
                printLCD(statusStr, 0, 1, CLEAR_ROW);
                
                stepper.setMicrostep(microSteps);
                isStepperRunning = true;
            }
            
            // Управление шаговым двигателем (если не пауза и серва не активна)
            if (!isPaused && !isServoRunning) {
                isStepperRunning = activatingStepper(stepsToTake);
            }
            
            // Обработка клавиатуры для управления паузой
            // ИСПРАВЛЕНИЕ: добавляем проверку на неактивность сервопривода
            char key = kpd.getKey();
            if (key != NO_KEY && !isPaused && !isServoRunning) {
                stepper.pause();
                isPaused = true;
                printLCD(_PAUSE, 0, 0, CLEAR_SCREEN);
            } else if (key == 'D' && isPaused && !isServoRunning) {
                // Выход из паузы только если сервопривод не активен
                if (stepsToTake > 0) {
                    stepper.resume();
                    isPaused = false;
                    isStepperRunning = true;
                    
                    // Восстановление отображения параметров
                    char statusStr[17];
                    snprintf(statusStr, sizeof(statusStr), "%lu%s x %lu%s", 
                             wireQuantity, _SHT, wireLength, _MM);
                    printLCD(statusStr, 0, 0, CLEAR_SCREEN);
                    
                    snprintf(statusStr, sizeof(statusStr), "%lu%s x %u%s", 
                             wireQuantity, _SHT, cuttingCounter, _SHT);
                    printLCD(statusStr, 0, 1);
                }
            }
            
            // Запуск сервопривода после завершения движения двигателя
            if (!isStepperRunning && !isPaused) {
                isServoRunning = activatingServo();
            }
            
            // Обработка завершения цикла резки
            if (!isStepperRunning && !isServoRunning && !isPaused) {
                cuttingCounter++;
                
                // Обновление счетчика на дисплее
                char statusStr[17];
                snprintf(statusStr, sizeof(statusStr), "%lu%s x %u%s", 
                         wireQuantity, _SHT, cuttingCounter, _SHT);
                printLCD(statusStr, 0, 1);
                
                // Проверка завершения всей работы
                if (cuttingCounter >= wireQuantity) {
                    cuttingCounter = 0;
                    currentMode = Mode::CONFIRM; // Возврат к подтверждению для повторения
                } else {
                    isStepperRunning = true; // Продолжение резки
                }
            }
            break;
        }
    }
}