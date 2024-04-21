import sys
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
import ads1115mod
import time


def decode_comp_props(source: ads1115mod.comparator_props):
    """Выводит в stdout свойства компаратора АЦП"""
    print(f"mode: {source.mode}")
    print(f"polarity: {source.polarity}")
    print(f"latch: {source.latch}")
    print(f"queue: {source.queue}")


def get_input_leg_names(in_mux_config: int) -> tuple[str, str]:
    """возвращает кортеж имен входных выводов микросхемы"""
    if 0 == in_mux_config:
        return "AIN0", "AIN1"
    if 1 == in_mux_config:
        return "AIN0", "AIN3"
    if 2 == in_mux_config:
        return "AIN1", "AIN3"
    if 3 == in_mux_config:
        return "AIN2", "AIN3"
    if 4 == in_mux_config:
        return "AIN0", "GND"
    if 5 == in_mux_config:
        return "AIN1", "GND"
    if 6 == in_mux_config:
        return "AIN2", "GND"
    if 7 == in_mux_config:
        return "AIN3", "GND"


def get_full_scale_range(gain_amp: int) -> float:
    """возвращает диапазон полной шкалы в Вольтах"""
    _fsr = 6.144, 4.096, 2.048, 1.024, 0.512, 0.256
    return _fsr[gain_amp]


def decode_common_props(source: ads1115mod.common_props):
    """Выводит в stdout основные свойства АЦП"""
    if not source.operational_status:
        print("operational status: устройство выполняет преобразование")
    else:
        print("operational status: устройство не выполняет преобразование")
    legs = get_input_leg_names(source.in_mux_config)
    print(f"in mux config: positive leg: {legs[0]}; negative leg: {legs[1]}")
    print(f"gain amplifier +/-: {get_full_scale_range(source.gain_amplifier)} [Вольт]")
    if not source.operating_mode:
        print("operating mode: режим непрерывного преобразования")
    else:
        print("operating mode: режим одиночного преобразования или состояние отключения питания")
    if source.data_rate < 5:
        print(f"data rate: {8 * 2 ** source.data_rate} отсчетов в секунду(!)")
    else:
        tmp = 250, 475, 860
        print(f"data rate: {tmp[source.data_rate - 5]} отсчетов в секунду(!)")


if __name__ == '__main__':
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)

    sensor = ads1115mod.Ads1115(adapter)
    print("---Основные настройки датчика---")
    gp = sensor.get_general_props()
    print(gp)
    print(16 * "--")
    print("---Информация о текущем канале АЦП---")
    ch = sensor.channel
    print(ch)
    print(16 * "--")
    print("---Основные 'сырые' настройки датчика---")
    gp = sensor.get_general_raw_props()
    print(gp)
    print(16 * "--")
    #
    print("---Одиночный режим измерения---")
    my_gain = 2
    sensor.start_measurement(single_shot=True, data_rate_raw=2, gain_raw=my_gain, channel=0, differential_channel=False)
    print("---Основные 'сырые' настройки датчика---")
    gp = sensor.get_general_raw_props()
    print(gp)
    print(16 * "--")
    td = sensor.get_conversion_cycle_time()
    print(f"Время преобразования [мкс]: {td}")
    for _ in range(10):
        time.sleep_us(td)
        # print(f"value: {sensor.value}; raw: {sensor.get_value(raw=True)}")
        print(f"value: {sensor.value}")
        sensor.start_measurement(single_shot=True, data_rate_raw=2, gain_raw=my_gain, channel=0, differential_channel=False)

    print("Определение 'зашкаливания' за предел измерения АЦП")
    print(16 * "--")
    sensor.start_measurement(single_shot=True, data_rate_raw=2, gain_raw=my_gain, channel=0, differential_channel=False)
    td = sensor.get_conversion_cycle_time()
    for _ in range(33):
        time.sleep_us(td)
        ex = sensor.get_raw_value_ex()
        voltage = sensor.raw_value_to_real(ex.value)
        print(f"get_raw_value_ex: {ex}; voltage: {voltage} Вольт")
        sensor.start_measurement(single_shot=True, data_rate_raw=2, gain_raw=my_gain, channel=0, differential_channel=False)

    print(16 * "--")
    print("Автоматический режим измерений АЦП")
    print(16 * "--")
    sensor.start_measurement(single_shot=False, data_rate_raw=0, gain_raw=my_gain, channel=0, differential_channel=False)
    td = sensor.get_conversion_cycle_time()
    time.sleep_us(td)
    _max = 3000
    for counter, voltage in enumerate(sensor):
        print(f"Напряжение: {voltage} Вольт")
        if counter > _max:
            sys.exit(0)
        time.sleep_us(td)
