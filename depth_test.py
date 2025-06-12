from machine import Timer, Pin, ADC, PWM
from sys import exit
from time import sleep

WATER_DENSITY = 1000 # kg/m³
GRAVITY = 9.81 # m/s²
CALLBACK_TIME = 100 # ms
MIN_PRESSURE_VOLTAGE = 0.5 # V
MAX_PRESSURE_VOLTAGE = 4.5 # V
MAX_PRESSURE_RATING = 206843 # Pa
ATMOSPHERE_PRESSURE = 101325 # Pa

class PressureSensor:
    def __init__(self):
        self.sensor = ADC(Pin(3))
        self.sensor.atten(ADC.ATTN_11DB)
    
    def read_voltage(self):
        return max(0.5, self.sensor.read_uv() / 1e6)

    def read_pressure(self):
        voltage = self.read_voltage()
        return MAX_PRESSURE_RATING * (voltage - MIN_PRESSURE_VOLTAGE) / (MAX_PRESSURE_VOLTAGE - MIN_PRESSURE_VOLTAGE)
    
    def calculate_depth(self):
        pressure = max(0, self.read_pressure() - ATMOSPHERE_PRESSURE)
        depth = pressure / (WATER_DENSITY * GRAVITY)
        return depth
    
    def test(self):
        voltage = self.read_voltage()
        pressure = self.read_pressure()
        depth = self.calculate_depth()
        print(f"Voltage: {voltage:.2f}V, Pressure: {pressure:.0f}Pa, Depth: {depth:.2f}m")

try:
    print("Starting sensor")
    sensor = PressureSensor()
    while True:
        sleep(1)
except KeyboardInterrupt:
    print("Keyboard Interrupt - stopping sensor")
    pressure_sensor.stop()
    exit()
except Exception as exp:
    print(f"Error: {exp}")
    exit()