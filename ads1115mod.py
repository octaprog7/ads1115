# micropython
# mail: goctaprog@gmail.com
# MIT license
# import struct

from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import DeviceEx, Iterator, check_value, get_error_str   # all_none
from sensor_pack_2.adcmod import ADC, adc_init_props    # , raw_value_ex
import micropython
from micropython import const
from collections import namedtuple
from sensor_pack_2.bitfield import bit_field_info
from sensor_pack_2.bitfield import BitFields
from sensor_pack_2.regmod import RegistryRW

# Register address pointer
# 00: Conversion register
# 01: Config register
# 10: Lo threshold register
# 11: Hi threshold register

_mask_in_mux = const(0b111_0000_0000_0000)
_mask_gain_amp = const(0b1110_0000_0000)
_mask_data_rate = const(0b1110_0000)
_mask_comp_queue = const(0b11)

_2pwr15 = const(2 ** 15)
_low_val = const(_2pwr15)
_max_val = const(_2pwr15 - 1)

'''
# параметры компаратора АЦП
comparator_props = namedtuple("comparator_props", "mode polarity latch queue")
# основные параметры АЦП
common_props = namedtuple("common_props", "operational_status in_mux_config gain_amplifier operating_mode data_rate")
'''

# свойства компаратора: mode:bool, polarity:bool, latch:bool, queue:int 0..3,
# lo_threshold:int 16 bit, hi_threshold:int 16 bit
# mode: False - обычный компаратор (по умолчанию); True - оконный компаратор
# polarity: False - активный низкий уровень (по умолчанию); True - активный высокий уровень
# latch: False - Компаратор без фиксации. Вывод ALERT/RDY не фиксируется при его включении (по умолчанию); True -
# Защелкивающийся компаратор. Вывод ALERT/RDY остается зафиксированным до тех пор, пока данные преобразования не будут
# прочитаны ведущим.
#  queue: 00 Утвердить после одного преобразования
#         01 Утвердить после двух преобразований
#         10 Утвердить после четырех преобразований
#         11 Отключить компаратор и установите вывод ALERT/RDY в состояние высокого импеданса (по умолчанию).
# lo_threshold, hi_threshold: значения верхнего и нижнего порогов, используемые компаратором, сохраняются в двух
# 16-битных регистрах в формате дополнения до двух. Компаратор реализован как цифровой компаратор; поэтому значения в
# этих регистрах должны обновляться при каждом изменении настроек PGA.
#
specific_props = namedtuple("specific_props", "mode polarity latch queue lo_threshold hi_threshold")
# цена младшего разряда АЦП в зависимости от коэффициента усиления(gain_amp)
#       0,          1,      2,      3,          4,          5
_lsb = 187.5E-6, 125E-6, 62.5E-6, 31.25E-6, 15.625E-6, 7.8125E-6
# действительные коэффициенты усиления в зависимости от поля PGA[2:0] регистра CONFIG
_natural_gains = 3.0, 2.0, 1.0, 1/2, 1/4, 1/8


def _get_lsb(gain_amp: int) -> float:
    """Возвращает цену младшего разряда в зависимости от настройки входного делителя напряжения.
    gain_amp = 0..5"""
    print(f"DBG: lsb: {_lsb[gain_amp]}; {gain_amp}")
    return _lsb[gain_amp]


def _get_correct_gain(gain: int) -> int:
    """Проверяет усиление на правильность и возвращает правильное значение в диапазоне 0..5 включительно.
    gain - сырое/raw значение для записи в регистр"""
    check_value(gain, range(8), f"Неверное значение усиления: {gain}")
    # print(f"DBG:_get_correct_gain: {gain}")
    if gain > 5:
        return 5
    return gain


_model_1115 = 'ads1115'


def get_init_props(model: str) -> adc_init_props:
    """Возвращает параметры для инициализации АЦП в виде именованного кортежа по имени модели АЦП."""
    if _model_1115 == model.lower():
        return adc_init_props(reference_voltage=2.048, max_resolution=16, channels=4,
                              differential_channels=4, differential_mode=True)
    raise ValueError(f"Неизвестная модель АЦП!")


class Ads1115(DeviceEx, ADC, Iterator):
    """АЦП на шине I2C от TI.
    Low-Power, I2C, 860-SPS, 16-Bit ADC with internal reference, oscillator, and programmable comparator"""

    _config_reg_ads1115 = (bit_field_info(name='OS', position=range(15, 16), valid_values=None),    # Operational status or single-shot conversion start
                           bit_field_info(name='MUX', position=range(12, 15), valid_values=None),   # Input multiplexer configuration
                           bit_field_info(name='PGA', position=range(9, 12), valid_values=range(6)),    # Programmable gain amplifier configuration
                           bit_field_info(name='MODE', position=range(8, 9), valid_values=None),        # Device operating mode
                           bit_field_info(name='DR', position=range(5, 8), valid_values=None),          # Data rate
                           bit_field_info(name='COMP_MODE', position=range(4, 5), valid_values=None),   # Comparator mode
                           bit_field_info(name='COMP_POL', position=range(3, 4), valid_values=None),    # Comparator polarity
                           bit_field_info(name='COMP_LAT', position=range(2, 3), valid_values=None),    # Latching comparator
                           bit_field_info(name='COMP_QUE', position=range(0, 2), valid_values=None)     # Comparator queue and disable
                           )

    def __init__(self, adapter: bus_service.BusAdapter, address=0x48):
        check_value(address, range(0x48, 0x4C), f"Неверное значение адреса I2C устройства: 0x{address:x}")
        DeviceEx.__init__(self, adapter, address, True)
        ADC.__init__(self, get_init_props(_model_1115), model=_model_1115)
        # регистр настройки
        self._config_reg = RegistryRW(device=self, address=0x01,
                                      fields=BitFields(Ads1115._config_reg_ads1115), byte_len=None)
        self._differential_mode = True      # дифференциальный АЦП. для get_lsb
        # буфер на 4 байта
        self._buf_4 = bytearray((0 for _ in range(4)))
        # При записи: 0: Нет эффекта; 1: Запустить одиночное преобразование (в состоянии пониж. энергопотребл)
        # При чтении: 0: устройство выполняет преобразование; 1: устройство НЕ выполняет преобразование.
        self._operational_status = None
        # настройки компаратора
        self._comparator_mode = None
        self._comparator_polarity = None
        self._comparator_latch = None
        self._comparator_queue = None
        # Внимание, важный вызов(!)
        # читаю config АЦП и обновляю поля класса
        _raw_cfg = self.get_raw_config()
        # print(f"DBG: get_raw_config(): 0x{_raw_cfg:x}")
        self.raw_config_to_adc_properties(_raw_cfg)

    @staticmethod
    def get_raw_mux_cfg(ch: int, df: bool) -> int:
        """Возвращает сырое значение для мультиплексора входов (MUX)"""
        # _ch = self.check_channel_number(ch, df)
        if df:
            return ch      # каналы 0..3 дифференциальные
        return 4 + ch      # каналы 4..7 обычные

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """возвращает время преобразования в [мкc] аналогового значения в цифровое"""
        return 1 + int(1_000_000 / self.sample_rate)

    def gain_raw_to_real(self, gain_raw: int) -> float:
        """Преобразует 'сырое' значение усиления в 'настоящее'"""
        # self._natural_gain = _natural_gains[raw_gain]
        return 1 / _natural_gains[gain_raw]

#    @staticmethod
#    def get_correct_gain(self, gain_raw: int) -> int:
#        """Проверяет усиление на правильность и возвращает правильное значение в диапазоне 0..5 включительно.
#        Вычисляет действительный коэффициент усиления в self._natural_gain.
#        gain - сырое/raw значение для записи в регистр"""
#        check_value(gain_raw, range(8), f"Неверное значение усиления: {gain_raw}")
#        # присваиваю действительное значение усиления входного аналогового сигнала
#        # self._real_gain = self.gain_raw_to_real(gain)     # qqq
#        if gain_raw in range(6):
#            return gain_raw
#        return 5

    def check_gain_raw(self, gain_raw: int) -> int:
        """Проверяет сырое усиление на правильность. В случае ошибки выброси исключение!
        Возвращает gain_raw в случае успеха! Для переопределения в классе-наследнике."""
        r6 = range(6)
        return check_value(gain_raw, r6, get_error_str("gain_raw", gain_raw, r6))

    def check_data_rate_raw(self, data_rate_raw: int) -> int:
        """Проверяет сырое data_rate на правильность. В случае ошибки выброси исключение!
        Возвращает data_rate_raw в случае успеха!"""
        r8 = range(8)
        return check_value(data_rate_raw, r8, get_error_str("data_rate_raw", data_rate_raw, r8))

#    def check_params(self, data_rate_raw: int, gain_raw: int):
#        r6, r8 = range(6), range(8)
#        # check_value(in_mux_conf, r8, get_error_str("in_mux_conf", in_mux_conf, r8))
#        check_value(gain_raw, r6, get_error_str("gain_raw", gain_raw, r6))
#        check_value(data_rate_raw, r8, get_error_str("data_rate_raw", data_rate_raw, r8))

    def _adc_props_to_raw(self,
                          # При записи: 0: Нет эффекта; 1: Запустить одиночное преобразование (в состоянии пониж. энергопотребл)
                          # При чтении: 0: устройство выполняет преобразование; 1: устройство НЕ выполняет преобразование.
                          operational_status: [bool, None] = None,  # bit 15, config register
                          # Эти биты настраивают входной мультиплексор. Эти биты не выполняют никакой функции в ADS1113/ADS1114.
                          # 0 - измеряет напряжение между выводами Ain0(+) и Ain1(-)    (по умолчанию, дифференциальный вход)
                          # 1 - измеряет напряжение между выводами Ain0(+) и Ain3(-)    (дифференциальный вход)
                          # 2 - измеряет напряжение между выводами Ain1(+) и Ain3(-)    (дифференциальный вход)
                          # 3 - измеряет напряжение между выводами Ain2(+) и Ain3(-)    (дифференциальный вход)
                          # 4 - измеряет напряжение между выводами Ain0(+) и GND(-)
                          # 5 - измеряет напряжение между выводами Ain1(+) и GND(-)
                          # 6 - измеряет напряжение между выводами Ain2(+) и GND(-)
                          # 7 - измеряет напряжение между выводами Ain3(+) и GND(-)
                          in_mux_config: [int, None] = None,  # bit 14..12, config register, только ADS1115;
                          # Конфигурация усилителя с программируемым усилением.
                          # Не подавайте на аналоговые входы устройства напряжение более VDD+0,3 В!!!
                          # Эти биты устанавливают 'полный диапазон шкалы' (FSR) усилителя с программируемым усилением.
                          # Эти биты не выполняют никакой функции в ADS1113.
                          # 0 - FSR = +/- 6.144 Вольта (полный диапазон масштабирования АЦП)
                          # 1 - FSR = +/- 4.096 Вольта (полный диапазон масштабирования АЦП)
                          # 2 - FSR = +/- 2.048 Вольта (по умолчанию)
                          # 3 - FSR = +/- 1.024 Вольта
                          # 4 - FSR = +/- 0.512 Вольта
                          # 5 - FSR = +/- 0.256 Вольта
                          gain_amplifier: [int, None] = None,  # bit 11..9, config register. усиление raw
                          # 0 - Режим непрерывного преобразования;
                          # 1 - Режим одиночного преобразования или состояние отключения питания (по умолчанию)
                          operating_mode: [bool, None] = None,  # bit 8, config register, текущий режим работы
                          # bit 7..5, кол-во отсчетов АЦП в секунду (ОВС/samples per second)
                          # 0 - 8 отчетов в секунду (ОВС)
                          # 1 - 16 ОВС
                          # 2 - 32 ОВС
                          # 3 - 64 ОВС
                          # 4 - 128 ОВС
                          # 5 - 250 ОВС
                          # 6 - 475 ОВС
                          # 7 - 860 ОВС
                          data_rate: [int, None] = None,    # bit 7..5, config register
                          # 0: Традиционный компаратор (по умолчанию).
                          # 1: Оконный компаратор.
                          comparator_mode: [bool, None] = None,  # bit 4, config register, режим сравнения, только для ADS1114 и ADS1115!
                          # bit 3, режим сравнения, только для ADS1114 и ADS1115!
                          # 0: активный сигнал "0" (по умолчанию).
                          # 1: активный сигнал "1".
                          comparator_polarity: [bool, None] = None,     # config register
                          # 0: Компаратор без фиксации. Вывод ALERT/RDY не фиксируется при подаче сигнала (по умолчанию).
                          # 1: Фиксирующийся компаратор. Вывод ALERT/RDY остается зафиксированным до тех пор, пока данные
                          # преобразования не будут прочитаны ведущим или соответствующий ответ на предупреждение SMBus не будет
                          # отправлен ведущим.
                          comparator_latch: [bool, None] = None,  # bit 2, config register, только для ADS1114 и ADS1115!
                          # Эти биты выполняют две функции. Если установлено значение 0b11, компаратор отключается, а вывод
                          # ALERT/RDY устанавливается в состояние с высоким импедансом. При установке любого другого значения
                          # вывод ALERT/RDY и функция компаратора активируются, а установленное значение определяет количество
                          # последовательных преобразований, превышающих верхний или нижний порог, необходимый для установки
                          # вывода ALERT/RDY.
                          # 0: установка после одного преобразования.
                          # 1: установка после двух преобразований.
                          # 2: установка после четырех преобразований.
                          # 3: отключает компаратор и установит вывод ALERT/RDY в высокое сопротивление (по умолчанию).
                          comparator_queue: [int, None] = None,  # bit 1..0, config register, только для ADS1114 и ADS1115!
                          ) -> int:
        """Возвращает 'сырое' значение настроек АЦП/датчика для записи в регистр CONF/config register.
        Регистр Config. (P[1:0] = 0x1) [reset value = 0x8583]"""
        val = self.get_raw_config()  # читаю регистр CONF. Обязательная первая строка метода!!!
        reg = self._config_reg
        # изменяю настройки. Ниже пишете свой код!
        if operational_status is not None:
            reg['OS'] = operational_status
        if in_mux_config is not None:
            reg['MUX'] = in_mux_config
        if gain_amplifier is not None:
            reg['PGA'] = self.check_gain_raw(gain_amplifier)
        if operating_mode is not None:
            reg['MODE'] = operating_mode
        if data_rate is not None:
            reg['DR'] = data_rate
        if comparator_mode is not None:
            reg['COMP_MODE'] = comparator_mode
        if comparator_polarity is not None:
            reg['COMP_POL'] = comparator_polarity
        if comparator_latch is not None:
            reg['COMP_LAT'] = comparator_latch
        if comparator_queue is not None:
            reg['COMP_QUE'] = comparator_queue
        # возвращаю измененное, в соотв с входными параметрами, 'сырое' значение для записи в регистр CONF
        return reg.value

    def get_raw_config(self) -> int:
        """Возвращает текущие настройки датчика из регистров(конфигурации) датчика в виде числа.
        Чтение регистра CONF, адрес 0x01, 16 бит"""
        # return self.unpack(fmt_char="H", source=self.read_reg(reg_addr=0x01, bytes_count=2))[0]
        return int(self._config_reg)    # неявный вызов метода RegistryRO.read()

    def set_raw_config(self, value: int):
        self._config_reg.write(value)
        # self.write_reg(reg_addr=0x01, value=value, bytes_count=2)

    def raw_config_to_adc_properties(self, raw_config: int):
        """Возвращает текущие настройки датчика из числа в поля класса."""
        def mux_raw_cfg_to_channel(mux_raw_cfg: int):
            """Преобразует сырою информацию о канале АЦП в обработанную. А именно, номер канала и является ли этот канал
            дифференциальным"""
            check_value(mux_raw_cfg, range(8), f"Неверный значение MUX[2:0]: {mux_raw_cfg}")
            self._is_diff_channel = mux_raw_cfg < 4   # У ADS1115 первые 4 канала дифференциальные!
            if mux_raw_cfg in range(4):     # 0..3
                self._curr_channel = mux_raw_cfg
            else:   # >= 4..7
                self._curr_channel = mux_raw_cfg - 4
        # поехали
        reg = self._config_reg
        # это лишний вызов, поскольку метод get_raw_config вызывает метод RegistryRO.read(), но
        # для ясности кода присваиваю явно
        reg.value = raw_config
        # обновление значений полей экземпляра класса
        self._operational_status = reg['OS']
        mux_raw_cfg_to_channel(reg['MUX'])
        self._curr_raw_gain = _get_correct_gain(reg['PGA'])
        self._single_shot_mode = reg['MODE']
        self._curr_raw_data_rate = reg['DR']
        self._curr_resolution = self.init_props.max_resolution        # число бит в отсчете не изменяется у этого АЦП
        # comparator
        self._comparator_mode = reg['COMP_MODE']
        self._comparator_polarity = reg['COMP_POL']
        self._comparator_latch = reg['COMP_LAT']
        self._comparator_queue = reg['COMP_QUE']

    def adc_properties_to_raw_config(self) -> int:
        """Преобразует свойства АЦП из полей класса в 'сырую' конфигурацию АЦП.
        adc_properties -> raw_config"""

        return self._adc_props_to_raw(
            operational_status=self._single_shot_mode,
            in_mux_config=Ads1115.get_raw_mux_cfg(self._curr_channel, self._is_diff_channel),
            gain_amplifier=self.current_raw_gain,
            operating_mode=self._single_shot_mode,
            data_rate=self.current_sample_rate,
            comparator_mode=None,
            comparator_polarity=None,
            comparator_latch=None,
            comparator_queue=None
        )

    def get_raw_value(self) -> int:
        """Возвращает 'сырое' значение отсчета АЦП."""
        return self.unpack(fmt_char="h", source=self.read_reg(reg_addr=0x00, bytes_count=2))[0]

    def _get_thresholds(self) -> tuple:
        """Возвращает пороговые значения компаратора в виде кортежа: (высокий порог, низкий порог)"""
        buf = self.read_buf_from_mem(0x02, self._buf_4)
        # print(f"DBG: {buf}")
        return self.unpack(fmt_char="hh", source=buf)

    # def _set_thresholds(self, hi_val: int, lo_val: int):
    #    rng = range(-32768, 32768)
    #    str_error = "Входной параметр вне диапазона!"
    #    check_value(hi_val, rng, str_error)
    #    check_value(lo_val, rng, str_error)
    #    # в буфере пара значений
    #    buf = self.pack("hh", lo_val, hi_val)
    #    self.write_buf_to_mem(0x02, buf)

    # @property
    # def thresholds(self) -> tuple:
    #    return self._get_thresholds()

    # @thresholds.setter
    # def thresholds(self, value: tuple[int, int]):
    #    self._set_thresholds(value[0], value[1])

    def get_specific_props(self):
        """Возвращает свойства компаратора"""
        thr = self._get_thresholds()
        return specific_props(mode=self._comparator_mode, polarity=self._comparator_polarity,
                              latch=self._comparator_latch, queue=self._comparator_queue,
                              lo_threshold=thr[0], hi_threshold=thr[1])

    def get_resolution(self, raw_data_rate: int) -> int:
        """Возвращает кол-во бит в отсчете АЦП в зависимости от частоты взятия отсчетов (сырое значение!).
        Переопределить в классе - наследнике!"""
        # у данного АЦП разрешение не изменяется!
        return 16

    def raw_sample_rate_to_real(self, raw_sample_rate: int) -> float:
        """Преобразует сырое значение частоты преобразования в [Гц].
        Переопределить в классе - наследнике!"""
        sps = 8, 16, 32, 64, 128, 250, 475, 860
        return sps[raw_sample_rate]

    # Iterator
    def __iter__(self):
        return self

    def __next__(self) -> [int, None]:
        if not self._single_shot_mode:
            # режим непрерывного преобразования!
            return self.value
        return None
