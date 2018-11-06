import smbus as I2C
import RPi.GPIO as GPIO
import time
from enum import IntEnum   # для создания нумерованных списков
import math
import threading
import warnings


class _I2c:
    """Общий служебный класс, с помощью которого реализована работа с I2C"""
    def __init__(self):
        self._bus = I2C.SMBus(1)

    def readRaw(self, addr: int, cmd: int, len: int):
        """
        Чтение "сырых" данных из i2c
        :param addr: адрес устройства
        :param cmd: код комманды
        :param len: сколько байт считать
        :return: считанные данные
        """
        return self._bus.read_i2c_block_data(addr, cmd, len)

    def readU8(self, addr: int, register: int):
        """
        Чтение unsigned byte из i2c.
        :param addr: адрес устройства
        :param register: регистр для чтения
        :return: считанные данные
        """
        return self._bus.read_byte_data(addr, register) & 0xFF

    def writeByte(self, addr: int, value: int):
        """
        Отправка одного байта данных в шину i2c.
        :param addr: адрес устройства
        :param value: значение для отправки
        """
        return self._bus.write_byte(addr, value)

    def writeByteData(self, addr: int, register: int, value: int):
        """
        Запись одного байта данных в заданный регистр устройства.
        :param addr: адрес устройства
        :param register: регистр для записи
        :param value: значение для записи
        """
        value = value & 0xFF
        self._bus.write_byte_data(addr, register, value)

    def writeList(self, addr: int, register: int, data: list):
        """
        Запись списка байтов в заданный регистр устройства.
        :param addr: адрес устройства
        :param register: регистр для записи
        :param data: список данных
        """
        for i in range(len(data)):
            self._bus.write_byte_data(addr, register, data[i])


class Battery(threading.Thread):
    """Класс для получения информации от одноканального АЦП MCP3221."""
    def __init__(self, vRef=3.3, gain=7.66):
        """
        Конструктор класса.
        :param vRef: опорное напряжение (относительно которого происходит измерение)
        :param gain: коэффициент делителя напряжения (если он есть)
        """
        self._addr = 0x4D
        self._vRef = vRef
        self._gain = gain
        self._i2c = _I2c()
        threading.Thread.__init__(self, daemon=True)
        self.__exit = False  # флаг завершения тредов
        self._filteredVoltage = 0   # отфильтрованное значение напряжения
        self._K = 0.1   # коэффициент фильтрации

    def run(self):
        """Метод для threading. Запуск вычислений в отдельном потоке."""
        while not self.__exit:    # 20 раз в секунду опрашивает АЦП, фильтрует значение
            self._filteredVoltage = self._filteredVoltage * (1 - self._K) + self.getVoltageInstant() * self._K
            time.sleep(0.05)

    def _readRaw(self):
        """Чтение cырых показаний с АЦП - просто 2 байта."""
        reading = self._i2c.readRaw(self._addr, 0x00, 2)
        return (reading[0] << 8) + reading[1]

    def _readConverted(self):
        """Преобразование к напряжению, относительно опорного (после предделителя)"""
        voltage = (self._readRaw() / 4095) * self._vRef  # 4095 - число разрядов АЦП
        return voltage

    def getVoltageInstant(self):
        """Возвращает моментальное значение напряжения аккумулятора с АЦП."""
        battery = self._readConverted() * self._gain
        return round(battery, 2)

    def stop(self):
        """Остановка вычислений в отдельном потоке."""
        self.__exit = True

    def getVoltageFiltered(self):
        """Возвращает отфильтрованное значение напряжения."""
        return round(self._filteredVoltage, 2)

    def calibrate(self, exactVoltage: float):
        """Подгонка коэффициента делитея напряжения."""
        value = 0
        for i in range(100):
            value += self._readConverted()
            time.sleep(0.01)
        value /= 100
        self._gain = exactVoltage/value
    # TODO: возможно сделать калибровку более точной (но вроде как без нее все работает и так)


# Регистры для работы с PCA9685
_PCA9685_ADDRESS = 0x40
_MODE1 = 0x00
_MODE2 = 0x01
_SUBADR1 = 0x02
_SUBADR2 = 0x03
_SUBADR3 = 0x04
_PRESCALE = 0xFE
_LED0_ON_L = 0x06
_LED0_ON_H = 0x07
_LED0_OFF_L = 0x08
_LED0_OFF_H = 0x09
_ALL_LED_ON_L = 0xFA
_ALL_LED_ON_H = 0xFB
_ALL_LED_OFF_L = 0xFC
_ALL_LED_OFF_H = 0xFD

# Биты для работы с PCA9685:
_RESTART = 0x80     # при чтении возвращает свое состояние, при записи - разрешает или запрещает перезагрузку
_SLEEP = 0x10       # режим энергосбережения (выключен внутренний осциллятор)
_ALLCALL = 0x01     # PCA9685 будет отвечать на запрос всех устройств на шине
_INVRT = 0x10       # инверсный или неинверсный выход сигнала на микросхеме
_OUTDRV = 0x04      # способ подключения светодиодов (см. даташит, нам это вроде не надо)

'''
################################  ВНИМАНИЕ  ########################################
#############  Я ПОКА НЕ ЗНАЮ КАК СДЕЛАТЬ БЕЗ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ  ###############
################################  МНЕ ЖАЛЬ  ########################################
'''
_pwmIsInited = False    # глобальный флаг, по которому будем отслеживать, нужна ли микросхеме новая инициализация
_pwmList = {}    # глобальный словарь, который содержит номер канала и выставленный режим


class _PwmMode(IntEnum):    # список режимов работы
    servo90 = 90            # серва 90 градусов
    servo120 = 120          # серва 120 градусов
    servo180 = 180          # серва 180 градусов
    servo270 = 270          # серва 270 градусов
    forwardMotor = 100      # мотор без реверса
    reverseMotor = 4        # мотор с реверсом
    onOff = 5               # вкл/выкл пина


class PwmFreq(IntEnum):     # список возможных частот работы
    H50 = 50                # 50 Гц
    H125 = 125              # 125 Гц
    H250 = 250              # 250 Гц


_global_freq = None  # глобальная переменная, содержащая информацию о текущей частоте работы микросхемы


class PwmBase:
    """Базовый класс для управления драйвером ШИМ (PCA9685)"""
    def __init__(self, channel: int, mode, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала устройства
        :param mode: режим работы (какое устройство подключается)
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmIsInited, _global_freq
        self._i2c = _I2c()  # объект для общения с i2c шиной
        if (channel > 15) or (channel < 0):
            raise ValueError("Channel number must be from 0 to 15 (inclusive).")
        self._channel = channel
        self._mode = mode
        self._extended = extended
        self._value = 0     # значение, которе установлено на канале

        if not isinstance(freq, PwmFreq):
            raise ValueError("freq must be set as PwmFreq.H* !!")
        else:
            if _global_freq is None:    # если частота еще не была задана - задаем
                self._freq = freq
                _global_freq = freq
            elif _global_freq == freq:  # если была задана такой же - задаем
                self._freq = freq
            else:                       # если была задана другой - ругаемся
                warnings.warn("Frequency was already set! Current frequency is: {} Hz".format(int(_global_freq)))
                self._freq = _global_freq

        # при соответствующей частоте получаем:
        # 4096 - весь период, в зависимости от частоты это может быть 20, 8, 4 мс при 50, 125, 250 Гц соответственно
        self._parrot_ms = int(4096*self._freq/1000)
        self._min = self._parrot_ms     # минимальное значение = 1 мс
        self._max = self._parrot_ms*2   # максимальное значение = 2 мс
        self._range = self._max - self._min     # диапазон от min до max, нужен для вычислений
        self._wideMin = int(self._min/2)        # при расширенном диапазоне минимум = 0.5 мс
        self._wideMax = self._wideMin*5         # при расширенном диапазоне максимум = 2.5 мс
        self._wideRange = self._wideMax - self._wideMin     # аналогично, но тут расширенный диапазон
        if not _pwmIsInited:    # если микросхема еще не была инициализирована
            self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE2, _OUTDRV)
            self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE1, _ALLCALL)
            time.sleep(0.005)
            mode1 = self._i2c.readU8(_PCA9685_ADDRESS, _MODE1)  # читаем установленный режим
            mode1 = mode1 & ~_SLEEP  # будим
            self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE1, mode1)
            time.sleep(0.005)
            self._setPwmFreq(self._freq)    # устанавливаем частоту сигнала
            _pwmIsInited = True     # поднимаем флаг, что микросхема инициализирована

    def _setPwmFreq(self, freqHz: PwmFreq):
        """
        Установка частоты ШИМ сигнала.
        :param freqHz: Частота ШИМ сигнала (Гц)
        """
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= freqHz
        prescaleval -= 1
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self._i2c.readU8(_PCA9685_ADDRESS, _MODE1)    # смотрим какой режим был у микросхемы
        newmode = (oldmode & 0x7F) | 0x10   # отключаем внутреннее тактирование, чтобы внести изменения
        self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE1, newmode)
        self._i2c.writeByteData(_PCA9685_ADDRESS, _PRESCALE, prescale)  # изменяем частоту
        self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE1, oldmode)  # включаем тактирование обратно
        time.sleep(0.005)   # ждем пока оно включится
        # разрешаем микросхеме отвечать на subaddress 1
        self._i2c.writeByteData(_PCA9685_ADDRESS, _MODE1, oldmode | 0x08)

    def _setPwm(self, value: int):
        """
        Установка длительности импульса ШИМ для канала.
        :param value: Длительность (в попугаях микросхемы. 205 "попугаев" ~ 1000 мкс)
        """
        self._i2c.writeByteData(_PCA9685_ADDRESS, _LED0_ON_L + 4 * self._channel, 0 & 0xFF)   # момент включения в цикле
        self._i2c.writeByteData(_PCA9685_ADDRESS, _LED0_ON_H + 4 * self._channel, 0 >> 8)
        self._i2c.writeByteData(_PCA9685_ADDRESS, _LED0_OFF_L + 4 * self._channel, value & 0xFF)  # момент выключения в цикле
        self._i2c.writeByteData(_PCA9685_ADDRESS, _LED0_OFF_H + 4 * self._channel, value >> 8)

    def setMcs(self, value: int):
        """
        Установка длительности импульса ШИМ в мкс
        :param value: Длительность импульса в мкс
        """
        max_mcs = 1/self._freq  # максимальая длительность импульса в зависимости от частоты (в секундах)
        max_mcs *= 1000000      # максимальная длительность импульса в микросекунднах

        if value > max_mcs:     # обрезаем диапазон - от 0 до максимального
            value = max_mcs
        if value < 0:
            value = 0
        self._value = value     # запоминаем значение до преобразований
        value /= 1000           # приводим мкс к мс
        value *= self._parrot_ms    # приводим мс к попугаям которые затем задаются на ШИМ
        if value > 4095:            # обрезаем максимальное значение, чтобы микросхема не сходила с ума
            value = 4095
        self._setPwm(int(value))

    def getMcs(self):
        """Возвращает текущее значение длительности импульса ШИМ, выставленное на канале (в мкс)."""
        reading_H = self._i2c.readU8(_PCA9685_ADDRESS, _LED0_OFF_H + 4 * self._channel)
        reading_L = self._i2c.readU8(_PCA9685_ADDRESS, _LED0_OFF_L + 4 * self._channel)
        result = (reading_H << 8) + reading_L
        return int((result / self._parrot_ms) * 1000)

    def getValue(self):
        """Возвращает последнее значение, установленное на канале."""
        return self._value

    def setValue(self, value: int):  # устанавливаем значение
        """
        Установка значения для канала
        :param value: значение зависит от режима работы канала (угол, скорость и т.п.)
        """
        if self._mode == _PwmMode.onOff:   # если режим вкл/выкл (ему неважен расширенный диапазон)
            if value < 0:
                raise ValueError("Value must be True or False for On/Off mode")
            self._value = value     # запоминаем какое значение мы задаем (до всех преобразований)
            if value is True:   # если надо включить (True) - зажигаем полностью
                value = 4095
            else:               # иначе выключаем
                value = 0
        else:
            if self._extended is False:     # если диапазон не расширенный
                if self._mode == _PwmMode.reverseMotor:  # если говорим о моторе с реверсом
                    if value < -100:    # обрезаем диапазон
                        value = -100
                    if value > 100:
                        value = 100
                    self._value = value     # запоминаем какое значение мы задаем (до всех преобразований)
                    value += 100    # сдвигаем диапазон -100-100 -> 0-200
                    value *= self._range/200    # чуть изменяем 0-200 -> 0-range
                    value += self._min          # сдвигаем 0-range -> min-max
                else:
                    if value < 0:   # обрезаем крайние значения
                        value = 0
                    if value > self._mode.value:
                        value = self._mode.value
                    self._value = value     # запоминаем какое значение мы задаем (до всех преобразований)
                    value *= self._range/self._mode.value   # изменяем диапазон 0-mode -> 0-range
                    value += self._min      # сдвигаем диапазон 0-range -> min-max
            else:   # если диапазон расширенный
                if self._mode == _PwmMode.reverseMotor:  # если говорим о моторе с реверсом
                    if value < -100:    # обрезаем диапазон
                        value = -100
                    if value > 100:
                        value = 100
                    self._value = value  # запоминаем какое значение мы задаем (до всех преобразований)
                    value += 100    # сдвигаем диапазон -100-100 -> 0-200
                    value *= self._wideRange/200    # чуть изменяем 0-200 -> 0-range
                    value += self._wideMin      # сдвигаем 0-range -> min-max
                else:
                    if value < 0:   # обрезаем крайние значения
                        value = 0
                    if value > self._mode.value:
                        value = self._mode.value
                    self._value = value  # запоминаем какое значение мы задаем (до всех преобразований)
                    value *= self._wideRange/self._mode.value   # изменяем диапазон 0-mode -> 0-range
                    value += self._wideMin      # сдвигаем диапазон 0-range -> min-max
        self._setPwm(int(value))  # устанавливаем значение


'''
Классы для управления переферией. Параметры - номер канала, частота и является ли диапазон расширенным
'''


class Servo90(PwmBase):
    """Класс для управления сервой 90 град"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmList
        mode = _PwmMode.servo90
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo90, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo120(PwmBase):
    """Класс для управления сервой 120 град"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmList
        mode = _PwmMode.servo120
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode  # отмечаем, что канал занят
            super(Servo120, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo180(PwmBase):
    """Класс для управления сервой 180 град"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmList
        mode = _PwmMode.servo180
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo180, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo270(PwmBase):
    """Класс для управления сервой 270 град"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmList
        mode = _PwmMode.servo270
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo270, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


class ForwardMotor(PwmBase):
    """Класс для управления мотором с одним направлением"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        if 0 <= channel < 12:
            warnings.warn("Better use channels 12-15. Be sure that driver does not return voltage.")
        global _pwmList
        mode = _PwmMode.forwardMotor
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(ForwardMotor, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


class ReverseMotor(PwmBase):
    """Класс для управления мотором с реверсом"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        if 0 <= channel < 12:
            warnings.warn("Better use channels 12-15. Be sure that driver does not return voltage.")
        global _pwmList
        mode = _PwmMode.reverseMotor
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode
            super(ReverseMotor, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")

class Switch(PwmBase):
    """Класс реализующий только логические 0 и 1 на канале"""
    def __init__(self, channel, freq=PwmFreq.H50, extended=False):
        """
        Конструктор класса
        :param channel: номер канала
        :param freq: частота работы
        :param extended: флаг расширенного режима работы (0.5 - 2.5 мс, вместо 1 - 2 мс)
        """
        global _pwmList
        mode = _PwmMode.onOff
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode
            super(Switch, self).__init__(channel, mode, freq, extended)
        else:
            raise ValueError("This channel is already used!")


'''
Классы для работы с дисплеем.
'''
# Регистры для работы с SSD1306
_SSD1306_I2C_ADDRESS = 0x3C    # 011110+SA0+RW - 0x3C or 0x3D
_SSD1306_SETCONTRAST = 0x81
_SSD1306_DISPLAYALLON_RESUME = 0xA4
_SSD1306_DISPLAYALLON = 0xA5
_SSD1306_NORMALDISPLAY = 0xA6
_SSD1306_INVERTDISPLAY = 0xA7
_SSD1306_DISPLAYOFF = 0xAE
_SSD1306_DISPLAYON = 0xAF
_SSD1306_SETDISPLAYOFFSET = 0xD3
_SSD1306_SETCOMPINS = 0xDA
_SSD1306_SETVCOMDETECT = 0xDB
_SSD1306_SETDISPLAYCLOCKDIV = 0xD5
_SSD1306_SETPRECHARGE = 0xD9
_SSD1306_SETMULTIPLEX = 0xA8
_SSD1306_SETLOWCOLUMN = 0x00
_SSD1306_SETHIGHCOLUMN = 0x10
_SSD1306_SETSTARTLINE = 0x40
_SSD1306_MEMORYMODE = 0x20
_SSD1306_COLUMNADDR = 0x21
_SSD1306_PAGEADDR = 0x22
_SSD1306_COMSCANINC = 0xC0
_SSD1306_COMSCANDEC = 0xC8
_SSD1306_SEGREMAP = 0xA0
_SSD1306_CHARGEPUMP = 0x8D
_SSD1306_EXTERNALVCC = 0x1
_SSD1306_SWITCHCAPVCC = 0x2
# Константы для работы с прокруткой дисплея
_SSD1306_ACTIVATE_SCROLL = 0x2F
_SSD1306_DEACTIVATE_SCROLL = 0x2E
_SSD1306_SET_VERTICAL_SCROLL_AREA = 0xA3
_SSD1306_RIGHT_HORIZONTAL_SCROLL = 0x26
_SSD1306_LEFT_HORIZONTAL_SCROLL = 0x27
_SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29
_SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL = 0x2A


class _SSD1306Base(object):
    """Базовый класс для работы с OLED дисплеями на базе SSD1306"""
    def __init__(self, width, height):
        """
        Конструктор класса
        :param width: ширина дисплея, в пикселях
        :param height: высота дисплея, в пикселях
        """
        self._width = width  # ширина и высота дисплея
        self._height = height
        self._pages = height//8     # строки дисплея
        self._buffer = [0]*(width*self._pages)  # буффер изображения (из нулей)
        self._i2c = _I2c()

    def _initialize(self):
        """Инициализация дисплея"""
        raise NotImplementedError

    def _command(self, c: int):
        """Отправка байта команды дисплею"""
        control = 0x00
        self._i2c.writeByteData(_SSD1306_I2C_ADDRESS, control, c)

    def _data(self, c: int):  # Отправка байта данных дисплею
        """Отправка байта данных дисплею"""
        control = 0x40
        self._i2c.writeByteData(_SSD1306_I2C_ADDRESS, control, c)

    def getSize(self):
        """
        Возвращает ширину и высоту дисплея
        :return: width, height
        """
        return self._width, self._height

    def begin(self, vccstate=_SSD1306_SWITCHCAPVCC):
        """Включение дисплея"""
        self._vccstate = vccstate
        self._initialize()
        self._command(_SSD1306_DISPLAYON)

    def display(self):
        """Вывод программного буфера дисплея на физическое устройство"""
        self._command(_SSD1306_COLUMNADDR)  # задаем нумерацию столбцов
        self._command(0)                    # Начало столбцов (0 = сброс)
        self._command(self._width - 1)       # адрес последнего столбца
        self._command(_SSD1306_PAGEADDR)    # задаем адрес страниц (строк)
        self._command(0)                    # Начало строк (0 = сброс)
        self._command(self._pages - 1)      # адрес последней строки

        for i in range(0, len(self._buffer), 16):   # Выводим буффер данных
            control = 0x40
            self._i2c.writeList(_SSD1306_I2C_ADDRESS, control, self._buffer[i:i + 16])

    def image(self, image):
        """
        Вывод картинки, созданной с помощью библиотеки PIL
        :param image: картинка должна быть в режиме mode = 1 и совпадать по размеру с дисплеем
        """
        if image.mode != '1':
            raise ValueError('image must be in mode 1.')
        imWidth, imHeight = image.size
        if imWidth != self._width or imHeight != self._height:
            raise ValueError('image must be same dimensions as display ({0}x{1})'.format(self._width, self._height))
        pix = image.load()  # выгружаем пиксели из картинки
        # проходим через память чтобы записать картинку в буффер
        index = 0
        for page in range(self._pages):
            # идем по оси x (колонны)
            for x in range(self._width):
                bits = 0
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:    # быстрее чем range
                    bits = bits << 1
                    bits |= 0 if pix[(x, page*8 + 7 - bit)] == 0 else 1
                # обновляем буффер и увеличиваем счетчик
                self._buffer[index] = bits
                index += 1

    def clear(self):
        """Очистка буффера изображения"""
        self._buffer = [0]*(self._width * self._pages)

    def setBrightness(self, contrast: int):    # установка яркости дисплея от 0 до 255
        """
        Установка яркости дисплея
        :param contrast: значение от 0 до 255
        """
        if contrast < 0 or contrast > 255:
            raise ValueError('Contrast must be value from 0 to 255 (inclusive).')
        self._command(_SSD1306_SETCONTRAST)
        self._command(contrast)

    # ИМХО - бесполезная функция, когда есть предыдущая
    def _Dim(self, dim: bool):
        """
        Подстройка значения яркости.
        :param dim: True - значение в зависимости от источника питания. False - минимальная яркость
        :return:
        """
        contrast = 0
        if not dim:
            if self._vccstate == _SSD1306_EXTERNALVCC:
                contrast = 0x9F
            else:
                contrast = 0xCF
        self.setBrightness(contrast)


class SSD1306_128_64(_SSD1306Base):
    """Класс для дисплея 128x64 pix"""
    def __init__(self):
        # вызываем конструктор класса
        super(SSD1306_128_64, self).__init__(128, 64)

    def _initialize(self):
        """Инициализация для дисплея размером 128x64"""
        self._command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._command(0x80)                          # предлагаемоое соотношение 0x80
        self._command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._command(0x3F)
        self._command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._command(0x0)                           # без отступов
        self._command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x10)
        else:
            self._command(0x14)
        self._command(_SSD1306_MEMORYMODE)           # 0x20
        self._command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._command(_SSD1306_SEGREMAP | 0x1)
        self._command(_SSD1306_COMSCANDEC)
        self._command(_SSD1306_SETCOMPINS)           # 0xDA
        self._command(0x12)
        self._command(_SSD1306_SETCONTRAST)          # 0x81
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x9F)
        else:
            self._command(0xCF)
        self._command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x22)
        else:
            self._command(0xF1)
        self._command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._command(0x40)
        self._command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._command(_SSD1306_NORMALDISPLAY)        # 0xA6


class SSD1306_128_32(_SSD1306Base):  # класс для дисплея 128*32 pix
    """Класс для дисплея 128x32 pix"""
    def __init__(self):
        # Вызываем конструктор класса
        super(SSD1306_128_32, self).__init__(128, 32)

    def _initialize(self):
        """Инициализация для дисплея размером 128x32"""
        self._command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._command(0x80)                          # предлагаемоое соотношение 0x80
        self._command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._command(0x1F)
        self._command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._command(0x0)                           # без отступов
        self._command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x10)
        else:
            self._command(0x14)
        self._command(_SSD1306_MEMORYMODE)           # 0x20
        self._command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._command(_SSD1306_SEGREMAP | 0x1)
        self._command(_SSD1306_COMSCANDEC)
        self._command(_SSD1306_SETCOMPINS)           # 0xDA
        self._command(0x02)
        self._command(_SSD1306_SETCONTRAST)          # 0x81
        self._command(0x8F)
        self._command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x22)
        else:
            self._command(0xF1)
        self._command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._command(0x40)
        self._command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._command(_SSD1306_NORMALDISPLAY)        # 0xA6


class SSD1306_96_16(_SSD1306Base):
    """Класс для дисплея 96x16 pix"""
    def __init__(self):
        # Вызываем конструктор класса
        super(SSD1306_96_16, self).__init__(96, 16)

    def _initialize(self):
        """Инициализация для дисплея размером 96x16"""
        self._command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._command(0x60)                          # предлагаемоое соотношение 0x60
        self._command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._command(0x0F)
        self._command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._command(0x0)                           # без отступов
        self._command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x10)
        else:
            self._command(0x14)
        self._command(_SSD1306_MEMORYMODE)           # 0x20
        self._command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._command(_SSD1306_SEGREMAP | 0x1)
        self._command(_SSD1306_COMSCANDEC)
        self._command(_SSD1306_SETCOMPINS)           # 0xDA
        self._command(0x02)
        self._command(_SSD1306_SETCONTRAST)          # 0x81
        self._command(0x8F)
        self._command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._command(0x22)
        else:
            self._command(0xF1)
        self._command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._command(0x40)
        self._command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._command(_SSD1306_NORMALDISPLAY)        # 0xA6


'''
Класс для работы с кнопкой и светодиодом.
При создании класса инициализируются пины.
'''
_chanButton = 20    # пины по умолчанию, возможно в будущем будут меняться
_chanLed = 21


class Gpio:
    """Класс для работы с кнопкой и светодиодом"""
    def __init__(self):   # флаг, по которому будем очищать (или нет) GPIO
        GPIO.setwarnings(False)  # очищаем, если кто-то еще использовал GPIO воизбежание ошибок
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_chanButton, GPIO.IN, pull_up_down = GPIO.PUD_OFF)
        GPIO.setup(_chanLed, GPIO.OUT, initial=GPIO.LOW)

    def buttonAddEvent(self, foo):
        """
        Добавление функции, которая срабатывает при нажатии на кнопку.
        :param foo: Передаваемая функция, обязательно должна иметь один аргумент, который ей передаст GPIO (см. пример)
        """
        if foo is not None and callable(foo):
            GPIO.add_event_detect(_chanButton, GPIO.FALLING, callback = foo, bouncetime = 200)
        else:
            raise TypeError("Parameter must be callable function!")

    def ledSet(self, value: bool):
        """
        Включение/выключение светодиода.
        :param value: True или False - соответственно вкл и выкл.
        :return:
        """
        GPIO.output(_chanLed, value)

    def ledToggle(self):
        """Переключение состояния светодиода"""
        GPIO.output(_chanLed, not GPIO.input(_chanLed))

    def cleanUp(self):
        """Очистка GPIO при закрытии программы"""
        GPIO.cleanup()
