import smbus as I2C
import RPi.GPIO as GPIO
import time
from enum import IntEnum   # для создания нумерованных списков
import math
import threading
import warnings

###
'''
Общий служебный класс, с помощью которого реализована работа с I2C
'''
###


class _I2c:
    def __init__(self):
        self._bus = I2C.SMBus(1)

    def ReadRaw(self, addr, cmd, len):    # чтение "сырых" данных из i2c
        return self._bus.read_i2c_block_data(addr, cmd, len)

    def ReadU8(self, addr, register):    # чтение unsigned byte
        return self._bus.read_byte_data(addr, register) & 0xFF

    def WriteByte(self, addr, value):   # (Writebyte)отправка байта в шину
        return self._bus.write_byte(addr, value)

    def WriteByteData(self, addr, register, value):    # (Write8)запись 8-битного значения в заданный регистр
        value = value & 0xFF
        self._bus.write_byte_data(addr, register, value)

    def WriteList(self, addr, register, data):  # запись списка байтов в заданный регистр
        for i in range(len(data)):
            self._bus.write_byte_data(addr, register, data[i])


###
'''
Класс для получения информации от одноканального АЦП MCP3221.
При инициализации задаются значения: vRef - опорное напряжение (относительно которого происходит измерение),
gain - коэффициент делителя напряжения (если он есть.
Методы:
Read - читает информацию из шины I2C (2 байта измерения);
GetVoltage - Вызывает метод Read, преобразует полученное значение в напряжение исходя из заданного опорного напряжения;
GetBattery - Вызывает метод GetVoltage, домножает полученное напряжение на коэффициент делителя напряжения.
'''
###


class Battery(threading.Thread):
    def __init__(self, vRef=3.3, gain=7.66):
        self._addr = 0x4D
        self._vRef = vRef
        self._gain = gain
        self._i2c = _I2c()
        threading.Thread.__init__(self, daemon=True)
        self._exit = False  # флаг завершения тредов
        self._filteredVoltage = 0   # отфильтрованное значение напряжения
        self._K = 0.1   # коэффициент фильтрации

    def run(self):
        while not self._exit:    # 20 раз в секунду опрашивает АЦП, фильтрует значение
            self._filteredVoltage = self._filteredVoltage * (1 - self._K) + self.GetVoltageInstant() * self._K
            time.sleep(0.05)

    def _ReadRaw(self):    # чтение показаний АЦП
        reading = self._i2c.ReadRaw(self._addr, 0x00, 2)
        return (reading[0] << 8) + reading[1]

    def _ReadConverted(self):   # преобразование к напряжению, относительно опорного (после предделителя)
        voltage = (self._ReadRaw() / 4095) * self._vRef  # 4095 - число разрядов АЦП
        return voltage

    def GetVoltageInstant(self):  # возвращает моментальное значение напряжения аккумулятора с АЦП (до предделителя)
        battery = self._ReadConverted() * self._gain
        return round(battery, 2)

    def stop(self):     # останавливает треды
        self._exit = True

    def GetVoltageFiltered(self):   # возвращаяет отфильтрованное значение напряжения
        return round(self._filteredVoltage, 2)

    def Calibrate(self, exactVoltage):  # подгоняет коэффциент делителя напряжения
        value = 0
        for i in range(100):
            value += self._ReadConverted()
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

# при частоте ШИМ 50 Гц (20 мс) получаем
_parrot_ms = 205    # коэффициент преобразования 205 попугаев ~ 1 мс
_min = 205  # 1 мс (~ 4096/20)
_max = 410  # 2 мс (~ 4096*2/20)
_range = _max - _min  # диапазон от min до max, нужен для вычислений
# _wideMin = 164  # 0.8 мс (~ _min*0.8)
# _wideMax = 451  # 2.2 мс (~ _max*1.1)
# _wideRange = _wideMax - _wideMin    # аналогично, но тут расширенный диапазон
_wideMin = 103     # 0.5 мс (~ _min*0.5)   # тестово
_wideMax = 513    # 2.5 мс (~_max*1.25)
_wideRange = _wideMax - _wideMin

###
'''
Базовый класс для управления драйвером ШИМ.
Параметры при инициализации - номер канала, режим работы канала и флаг, нужен ли расширенный диапазон (800 - 2200 мкс)
'''
###
'''
################################  ВНИМАНИЕ  ########################################
#############  Я ПОКА НЕ ЗНАЮ КАК СДЕЛАТЬ БЕЗ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ  ###############
################################  МНЕ ЖАЛЬ  ########################################
'''
_pwmIsInited = False    # глобальный флаг, по которому будем отслеживать, нужна ли микросхеме новая инициализация
_pwmList= {}    # глобальный словарь, который содержит номер канала и выставленный режим


class _PwmMode(IntEnum):    # список режимов работы
    servo90 = 90            # серва 90 градусов
    servo120 = 120          # серва 120 градусов
    servo180 = 180          # серва 180 градусов
    servo270 = 270          # серва 270 градусов
    forwardMotor = 100      # мотор без реверса
    reverseMotor = 4        # мотор с реверсом
    onOff = 5               # вкл/выкл пина


class PwmBase:
    def __init__(self, channel, mode, extended=False):
        global _pwmIsInited
        self._i2c = _I2c()  # объект для общения с i2c шиной
        if (channel > 15) or (channel < 0):
            raise ValueError("Channel number must be from 0 to 15 (inclusive).")
        self._channel = channel
        self._mode = mode
        self._extended = extended
        self._value = 0     # значение, которе установлено на канале
        self._valueParrot = 0   # значение шим, которое установленно на канале в попугаях, понятных микросхеме
        if not _pwmIsInited:    # если микросхема еще не была инициализирована
            self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE2, _OUTDRV)
            self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE1, _ALLCALL)
            time.sleep(0.005)
            mode1 = self._i2c.ReadU8(_PCA9685_ADDRESS, _MODE1)  # читаем установленный режим
            mode1 = mode1 & ~_SLEEP  # будим
            self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE1, mode1)
            time.sleep(0.005)
            self._SetPwmFreq(50)    # устанавливаем частоту сигнала 50 Гц
            _pwmIsInited = True     # поднимаем флаг, что микросхема инициализирована

    def _SetPwmFreq(self, freqHz):  # устанавливает частоту ШИМ сигнала в Гц
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= freqHz
        prescaleval -= 1
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self._i2c.ReadU8(_PCA9685_ADDRESS, _MODE1)    # смотрим какой режим был у микросхемы
        newmode = (oldmode & 0x7F) | 0x10   # отключаем внутреннее тактирование, чтобы внести изменения
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE1, newmode)
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _PRESCALE, prescale)  # изменяем частоту
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE1, oldmode)  # включаем тактирование обратно
        time.sleep(0.005)   # ждем пока оно включится
        # разрешаем микросхеме отвечать на subaddress 1
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _MODE1, oldmode | 0x08)

    def _SetPwm(self, value):   # установка значения на канал
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _LED0_ON_L + 4 * self._channel, 0 & 0xFF)   # момент включения в цикле
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _LED0_ON_H + 4 * self._channel, 0 >> 8)
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _LED0_OFF_L + 4 * self._channel, value & 0xFF)  # момент выключения в цикле
        self._i2c.WriteByteData(_PCA9685_ADDRESS, _LED0_OFF_H + 4 * self._channel, value >> 8)

    def SetMcs(self, value):    # установка значения на канал в мкс
        if value > 20000:       # обрезаем диапазон - от 20 мс до 0 мс
            value = 20000
        if value < 0:
            value = 0
        self._value = value     # запоминаем значение до преобразований
        value /= 1000           # приводим мкс к мс
        value *= _parrot_ms     # приводим мс к попугаям которые затем задаются на ШИМ
        if value > 4095:        # обрезаем максимальное значение, чтобы микросхема не сходила с ума
            value = 4095
        self._valueParrot = value   # запоминаем значение в попугаях, чтобы затем выводить его в мс на канале
        self._SetPwm(int(value))

    def GetMcs(self):   # возвращает текущее значение длительности импульса, выставленное на канале, в мкс
        # значение 205 примерно соответствует 1 мс, при частоте 50 Гц
        return int((self._valueParrot / _parrot_ms)*1000)

    def GetValue(self):     # возвращает значение, установленное на канале
        return self._value

    def SetValue(self, value):  # устанавливаем значение
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
                    value *= _range/200    # чуть изменяем 0-200 -> 0-range
                    value += _min    # сдвигаем 0-range -> min-max
                else:
                    if value < 0:   # обрезаем крайние значения
                        value = 0
                    if value > self._mode.value:
                        value = self._mode.value
                    self._value = value  # запоминаем какое значение мы задаем (до всех преобразований)
                    value *= _range/self._mode.value   # изменяем диапазон 0-mode -> 0-range
                    value += _min    # сдвигаем диапазон 0-range -> min-max
            else:   # если диапазон расширенный
                if self._mode == _PwmMode.reverseMotor:  # если говорим о моторе с реверсом
                    if value < -100:    # обрезаем диапазон
                        value = -100
                    if value > 100:
                        value = 100
                    self._value = value  # запоминаем какое значение мы задаем (до всех преобразований)
                    value += 100    # сдвигаем диапазон -100-100 -> 0-200
                    value *= _wideRange/200    # чуть изменяем 0-200 -> 0-range
                    value += _wideMin    # сдвигаем 0-range -> min-max
                    # value *= _expWideRange/200    # чуть изменяем 0-200 -> 0-range
                    # value += _expWideMin    # сдвигаем 0-range -> min-max
                else:
                    if value < 0:   # обрезаем крайние значения
                        value = 0
                    if value > self._mode.value:
                        value = self._mode.value
                    self._value = value  # запоминаем какое значение мы задаем (до всех преобразований)
                    value *= _wideRange/self._mode.value   # изменяем диапазон 0-mode -> 0-range
                    value += _wideMin    # сдвигаем диапазон 0-range -> min-max
                    # value *= _expWideRange/self._mode.value   # изменяем диапазон 0-mode -> 0-range
                    # value += _expWideMin    # сдвигаем диапазон 0-range -> min-max
        self._valueParrot = value   # запоминаем значение в попугаях, чтобы возвращать его в мс на канале
        self._SetPwm(int(value))  # устанавливаем значение


'''
Классы для управления переферией. Параметры - номер канала и является ли диапазон расширенным.
'''


class Servo90(PwmBase):     # Класс для управления сервой 90 град
    def __init__(self, channel, extended=False):
        global _pwmList
        mode = _PwmMode.servo90
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo90, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo120(PwmBase):
    def __init__(self, channel, extended=False):
        global _pwmList
        mode = _PwmMode.servo120
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode  # отмечаем, что канал занят
            super(Servo120, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo180(PwmBase):    # класс для управления сервой 180 град
    def __init__(self, channel, extended=False):
        global _pwmList
        mode = _PwmMode.servo180
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo180, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")


class Servo270(PwmBase):    # класс для управления сервой 270 град
    def __init__(self, channel, extended=False):
        global _pwmList
        mode = _PwmMode.servo270
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(Servo270, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")


class ForwardMotor(PwmBase):    # класс для управления мотором с одним направлением
    def __init__(self, channel, extended=False):
        if 0 <= channel < 12:
            warnings.warn("Better use channels 12-15. Be sure that driver does not return voltage.")
        global _pwmList
        mode = _PwmMode.forwardMotor
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode    # отмечаем, что канал занят
            super(ForwardMotor, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")


class ReverseMotor(PwmBase):    # класс для управления мотором с реверсом
    def __init__(self, channel, extended=False):
        if 0 <= channel < 12:
            warnings.warn("Better use channels 12-15. Be sure that driver does not return voltage.")
        global _pwmList
        mode = _PwmMode.reverseMotor
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode
            super(ReverseMotor, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")

class Switch(PwmBase):    # класс, реализующий возможность включать/выключать канал
    def __init__(self, channel, extended=False):
        global _pwmList
        mode = _PwmMode.onOff
        if _pwmList.get(channel) is None:
            _pwmList[channel] = mode
            super(Switch, self).__init__(channel, mode, extended)
        else:
            raise ValueError("This channel is already used!")

###
'''
Классы для работы с дисплеем.
'''
###
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


class SSD1306Base(object):  # Базовый класс для работы с OLED дисплеями на базе SSD1306
    def __init__(self, width, height):
        self.width = width  # ширина и высота дисплея
        self.height = height
        self._pages = height//8     # строки дисплея
        self._buffer = [0]*(width*self._pages)  # буффер изображения (из нулей)
        self._i2c = _I2c()

    def _Initialize(self):
        raise NotImplementedError

    def _Command(self, c):  # Отправка байта команды дисплею
        control = 0x00
        self._i2c.WriteByteData(_SSD1306_I2C_ADDRESS, control, c)

    def _Data(self, c):  # Отправка байта данных дисплею
        control = 0x40
        self._i2c.WriteByteData(_SSD1306_I2C_ADDRESS, control, c)

    def Begin(self, vccstate=_SSD1306_SWITCHCAPVCC):    # инициализация дисплея
        self._vccstate = vccstate
        self._Initialize()
        self._Command(_SSD1306_DISPLAYON)

    def Display(self):  # вывод программного буффера дисплея на физическое устройство
        self._Command(_SSD1306_COLUMNADDR)   # задаем нумерацию столбцов
        self._Command(0)                     # Начало столбцов (0 = сброс)
        self._Command(self.width - 1)          # адрес последнего столбца
        self._Command(_SSD1306_PAGEADDR)     # задаем адрес страниц (строк)
        self._Command(0)                     # Начало строк (0 = сброс)
        self._Command(self._pages - 1)         # адрес последней строки
        # Выводим буффер данных
        for i in range(0, len(self._buffer), 16):
            control = 0x40
            self._i2c.WriteList(_SSD1306_I2C_ADDRESS, control, self._buffer[i:i+16])

    def Image(self, image):     # выводит картинку созданную при помощи библиотеки PIL
        # картинка должна быть в режиме mode = 1 и совпадать по размеру с дисплеем
        if image.mode != '1':
            raise ValueError('Image must be in mode 1.')
        imWidth, imHeight = image.size
        if imWidth != self.width or imHeight != self.height:
            raise ValueError('Image must be same dimensions as display ({0}x{1})'.format(self.width, self.height))
        pix = image.load()  # выгружаем пиксели из картинки
        # проходим через память чтобы записать картинку в буффер
        index = 0
        for page in range(self._pages):
            # идем по оси x (колонны)
            for x in range(self.width):
                bits = 0
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:    # быстрее чем range
                    bits = bits << 1
                    bits |= 0 if pix[(x, page*8 + 7 - bit)] == 0 else 1
                # обновляем буффер и увеличиваем счетчик
                self._buffer[index] = bits
                index += 1

    def Clear(self):    # очищает буффер изображения
        self._buffer = [0]*(self.width*self._pages)

    def SetBrightness(self, contrast):    # установка яркости дисплея от 0 до 255
        if contrast < 0 or contrast > 255:
            raise ValueError('Contrast must be value from 0 to 255 (inclusive).')
        self._Command(_SSD1306_SETCONTRAST)
        self._Command(contrast)

    # Подстраивает значение яркости. входное значение True или False.
    # Если True - задает значение в зависимости от источника питания (внешний, или от шины)
    # Если False - опускает до нуля
    # ИМХО - бесполезная функция, когда есть предыдущая
    def _Dim(self, dim):
        contrast = 0
        if not dim:
            if self._vccstate == _SSD1306_EXTERNALVCC:
                contrast = 0x9F
            else:
                contrast = 0xCF
        self.SetBrightness(contrast)


class SSD1306_128_64(SSD1306Base):  # класс для дисплея 128*64 pix
    def __init__(self):
        # вызываем конструктор класса
        super(SSD1306_128_64, self).__init__(128, 64)

    def _Initialize(self):
        # инициализация конкретно для размера 128x64
        self._Command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._Command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._Command(0x80)                          # предлагаемоое соотношение 0x80
        self._Command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._Command(0x3F)
        self._Command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._Command(0x0)                           # без отступов
        self._Command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._Command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x10)
        else:
            self._Command(0x14)
        self._Command(_SSD1306_MEMORYMODE)           # 0x20
        self._Command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._Command(_SSD1306_SEGREMAP | 0x1)
        self._Command(_SSD1306_COMSCANDEC)
        self._Command(_SSD1306_SETCOMPINS)           # 0xDA
        self._Command(0x12)
        self._Command(_SSD1306_SETCONTRAST)          # 0x81
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x9F)
        else:
            self._Command(0xCF)
        self._Command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x22)
        else:
            self._Command(0xF1)
        self._Command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._Command(0x40)
        self._Command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._Command(_SSD1306_NORMALDISPLAY)        # 0xA6


class SSD1306_128_32(SSD1306Base):  # класс для дисплея 128*32 pix
    def __init__(self):
        # Вызываем конструктор класса
        super(SSD1306_128_32, self).__init__(128, 32)

    def _Initialize(self):
        # Инициализация конкретно для размера 128x32 pix
        self._Command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._Command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._Command(0x80)                          # предлагаемоое соотношение 0x80
        self._Command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._Command(0x1F)
        self._Command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._Command(0x0)                           # без отступов
        self._Command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._Command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x10)
        else:
            self._Command(0x14)
        self._Command(_SSD1306_MEMORYMODE)           # 0x20
        self._Command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._Command(_SSD1306_SEGREMAP | 0x1)
        self._Command(_SSD1306_COMSCANDEC)
        self._Command(_SSD1306_SETCOMPINS)           # 0xDA
        self._Command(0x02)
        self._Command(_SSD1306_SETCONTRAST)          # 0x81
        self._Command(0x8F)
        self._Command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x22)
        else:
            self._Command(0xF1)
        self._Command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._Command(0x40)
        self._Command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._Command(_SSD1306_NORMALDISPLAY)        # 0xA6


class SSD1306_96_16(SSD1306Base):
    def __init__(self):
        # Вызываем конструктор класса
        super(SSD1306_96_16, self).__init__(96, 16)

    def _Initialize(self):
        # Инициализация конкретно для размера 96x16 pix
        self._Command(_SSD1306_DISPLAYOFF)           # 0xAE
        self._Command(_SSD1306_SETDISPLAYCLOCKDIV)   # 0xD5
        self._Command(0x60)                          # предлагаемоое соотношение 0x60
        self._Command(_SSD1306_SETMULTIPLEX)         # 0xA8
        self._Command(0x0F)
        self._Command(_SSD1306_SETDISPLAYOFFSET)     # 0xD3
        self._Command(0x0)                           # без отступов
        self._Command(_SSD1306_SETSTARTLINE | 0x0)   # начинаем строки с 0
        self._Command(_SSD1306_CHARGEPUMP)           # 0x8D
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x10)
        else:
            self._Command(0x14)
        self._Command(_SSD1306_MEMORYMODE)           # 0x20
        self._Command(0x00)                          # иначе работает неправильно (0x0 act like ks0108)
        self._Command(_SSD1306_SEGREMAP | 0x1)
        self._Command(_SSD1306_COMSCANDEC)
        self._Command(_SSD1306_SETCOMPINS)           # 0xDA
        self._Command(0x02)
        self._Command(_SSD1306_SETCONTRAST)          # 0x81
        self._Command(0x8F)
        self._Command(_SSD1306_SETPRECHARGE)         # 0xd9
        if self._vccstate == _SSD1306_EXTERNALVCC:
            self._Command(0x22)
        else:
            self._Command(0xF1)
        self._Command(_SSD1306_SETVCOMDETECT)        # 0xDB
        self._Command(0x40)
        self._Command(_SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self._Command(_SSD1306_NORMALDISPLAY)        # 0xA6


###
'''
Класс для работы с кнопкой и светодиодом.
При создании класса инициализируются пины.
'''
###
_chanButton = 20
_chanLed = 21


class Gpio:
    def __init__(self):   # флаг, по которому будем очищать (или нет) GPIO
        GPIO.setwarnings(False)  # очищаем, если кто-то еще использовал GPIO воизбежание ошибок
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_chanButton, GPIO.IN, pull_up_down = GPIO.PUD_OFF)
        GPIO.setup(_chanLed, GPIO.OUT, initial=GPIO.LOW)

    # добавление функции, которая срабатывает при нажатии на кнопку, у функции обязательно должен быть один аргумент, который ему передает GPIO (см. пример)
    def ButtonAddEvent(self, foo):
        if foo is not None:
            GPIO.add_event_detect(_chanButton, GPIO.FALLING, callback = foo, bouncetime = 200)

    def LedSet(self, value):    # включает или выключает светодиод в зависимости от заданного значения
        GPIO.output(_chanLed, value)

    def LedToggle(self):    # переключает состояние светодиода
        GPIO.output(_chanLed, not GPIO.input(_chanLed))

    def CleanUp(self):
        GPIO.cleanup()
