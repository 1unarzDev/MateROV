from machine import Timer, Pin, PWM, ADC
from sys import exit
from time import time, sleep
from collections import deque
import ubluetooth 
from micropython import const # Optimize constants at runtime

# Bluetooth
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_PERIPHERAL_DISCONNECT = const(8)

# General
LOG_TIME = const(1000) # ms
SERVO_PIN = const(2)
PRESSURE_SENSOR_PIN = const(3)

# Servo
SERVO_STOPPED = const(1.5) # ms
SERVO_MOVE_RANGE = const(0.5) # ms
SERVO_SLEEP = const(50) # ms

# Pressures sensor (depth)
WATER_DENSITY = const(1000) # kg/m³
GRAVITY = const(9.81) # m/s²
MIN_PRESSURE_VOLTAGE = const(0.5) # V
MAX_PRESSURE_VOLTAGE = const(4.5) # V
MAX_PRESSURE_RATING = const(206843) # Pa
ATMOSPHERE_PRESSURE = const(101325) # Pa

# Syringe
SECONDS_PER_ML = const(0.2)

# PID
PID_CALLBACK_TIME = const(100) # ms
FLOAT_VOLUME = const(600) # cm^3
FLOAT_MASS = const(550) # g
DIVE_TIME = const(45000) # ms
DESIRED_DEPTH = const(2.5) # m

# Continuous rotation servo which takes in input speeds (pulse widths in ms) rather than positional inputs
class Servo:
    def __init__(self):
        self.pwm = PWM(Pin(SERVO_PIN))
        self.pwm.freq(50)

    def set_duty_ms(self, ms):
        self.pwm.duty_ns(int(ms * 1e6))
        sleep(SERVO_SLEEP * 1e-3)
        
    def stop(self):
        self.set_duty_ms(SERVO_STOPPED)

    # -1 is backward the fastest and 1 is forward the fastest
    def move(self, speed):
        duty_ms = (SERVO_STOPPED + max(-1, min(1, speed * SERVO_MOVE_RANGE)))
        self.set_duty_ms(duty_ms)
        
class Syringe:
    def __init__(self):
        self.servo = Servo()
        self.timer = Timer(0) # Only one syringe allowed
        
    def move(self,ml):
        self.servo.move(-ml / abs(ml)) # Move in appropriate direction
        wait_time = abs(ml) * self.SECONDS_PER_ML * 1000
        self.timer.init(period=wait_time, mode=Timer.ONE_SHOT, callback=lambda t: self.servo.stop())

class PressureSensor:
    def __init__(self):
        self.sensor = ADC(Pin(PRESSURE_SENSOR_PIN))
        self.sensor.atten(ADC.ATTN_11DB)
    
    def read_voltage(self):
        return max(0.5, self.sensor.read_uv() / 1e6)

    def read_pressure(self):
        voltage = self.read_voltage()
        return MAX_PRESSURE_RATING * (voltage - MIN_PRESSURE_VOLTAGE) / (MAX_PRESSURE_VOLTAGE - MIN_PRESSURE_VOLTAGE)
    
    def read_depth(self):
        pressure = max(0, self.read_pressure() - ATMOSPHERE_PRESSURE)
        depth = pressure / (WATER_DENSITY * GRAVITY)
        return depth

class PID:
    def __init__(self, pressure_sensor, syringe, kp, ki, kd, ke, integral_bound):
        self.desired_depth = DESIRED_DEPTH
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ke = ke
        self.integral_bound = integral_bound
        self.pressure_sensor = pressure_sensor
        self.syringe = syringe
        self.prev_t = time()
        self.integral = 0
        self.dt = 0
        self.change_ml = 0
    
    # Assumes water density of 1 g/cm^3 or 1 g/mL and uses some fancy logic to do with water displacement relative to float volume and mass
    def error(self):
        m = FLOAT_VOLUME - FLOAT_MASS + self.ke * DESIRED_DEPTH
        return m * (DESIRED_DEPTH - self.pressure_sensor.read_depth()) 

    def compute(self, error):
        self.t = time()
        self.dt = self.t - self.prev_t
        self.prev_t = self.t

        self.integral += error * self.dt
        self.integral = max(-self.integral_bound, min(self.integral, self.integral_bound))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        ml = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        return ml
    
    def run(self):
        self.change_ml = self.compute(self.error())
        self.syringe.move(change_ml)

class Bluetooth:
    def __init__(self, name, pressure_sensor, syringe):   
        self.name = name.encode('UTF-8')
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.connHandle = 0
        self.queue = deque((), DIVE_TIME / LOG_TIME * 2)
        self.connected = False
        self.disconnect()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        self.pressure_sensor = pressure_sensor
        self.syringe = syringe
        self.pid = None
        self.timer = Timer(2)
        self.timer.init(period=LOG_TIME, mode=Timer.PERIODIC, callback=lambda t: self.send_data())

    def connect(self, data):
        self.connHandle = data[0]
        self.connected = True
        print("Connected")

    def disconnect(self):        
        self.connected = False
        print("Disconnected")
         
    def send_queued(self):
        while self.connected:
            try:
                data = self.queue.popleft()
                print("Sending: " + data)
                self.ble.gatts_write(self.tx, data + '\n',True)
            except:
                return
     
    def send(self, data):
        try:
            print("Queueing: " + data)
            self.queue.append(data)
            self.send_queued()
        except:
            return
        
    # Message in format [time, voltage, pressure, depth, error, change_ml]
    def send_data(self):
        voltage = self.pressure_sensor.read_voltage()
        pressure = self.pressure_sensor.read_pressure()
        depth = self.pressure_sensor.read_depth()
        data = f"D,{time()},{voltage},{pressure},{depth}" 
        if(self.pid != None):
            data += f",{self.pid.error},{self.pid.change_ml}"
        self.send(data)
        
    # First tries to reach the approximate depth by moving the syringe by a fixed amount then runs PID for the time that the diver must stay in place
    def dive(self, dive_milliliters, kp, ki, kd, ke, integral_bound):
        self.send(f"Diving for: {dive_milliliters} milliliters")
        self.syringe.move_syringe(dive_milliliters)
        self.pid = PID(self.pressure_sensor, self.syringe, kp, ki, kd, ke, integral_bound)
        
        self.dive_milliliters = dive_milliliters
        self.dive_start_time = time.ticks_ms()

        self.dive_timer = Timer(1)
        self.dive_timer.init(period=PID_CALLBACK_TIME, mode=Timer.PERIODIC, callback=self.pid_run)
    
    def pid_run(self):
        elapsed = time.ticks_diff(time.ticks_ms(), self.dive_start_time)
        if elapsed < DIVE_TIME:
            self.pid.run()
        else:
            self.dive_timer.deinit()
            self.syringe.move_syringe(-self.dive_milliliters)
            self.pid = None

    # Parse request as request and a single argument
    def parse_request(self, message):
        request = None
        argument = None
        splits = message.split()
        if len(splits) > 0:
            request = splits[0]
            if len(splits) > 1:
                argument = splits[1]
        return request, argument
    
    def ble_irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self.connect(data)
        
        elif event == _IRQ_CENTRAL_DISCONNECT:
            # Central disconnected
            self.advertiser()
            self.disconnect()
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Peripheral disconnect
            self.advertiser()
            self.disconnect()
        elif event == _IRQ_GATTS_WRITE:
            # New message received
            buffer = self.ble.gatts_read(self.rx)
            message = buffer.decode('UTF-8').strip()
            print(message)
            command,argument = self.parse_request(message)
               
            if command == 'd':
                if(self.pid == None):
                    self.dive(*list(map(int, argument.split(",")[:6])))
                
            if command == 'f':
                self.syringe.move_syringe(int(argument))
                
            if command == 'b':
                self.syringe.move_syringe(-int(argument))
           
    def register(self):        
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
            
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_RX = (ubluetooth.UUID(RX_UUID), ubluetooth.FLAG_WRITE)
        BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_NOTIFY)
            
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        self.ble.gap_advertise(100, bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name)

class Float:
    def __init__(self):
        self.pressure_sensor = PressureSensor()
        self.syringe = Syringe()
        self.bluetooth = Bluetooth("DemonDiver", self.pressure_sensor, self.syringe)

try:
    print("Starting Float")
    float = Float()
    while True:
        sleep(1)
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    exit()
except Exception as exp:
    print(exp)