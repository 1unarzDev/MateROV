from machine import Timer, Pin, PWM, ADC
from sys import exit
from time import sleep, ticks_ms, ticks_diff
from collections import deque
import ubluetooth 
from micropython import const # Optimize constants at runtime
import struct

# Bluetooth
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
CMD_DIVE = const(0x01)
CMD_MOVE_FORWARD = const(0x02)
CMD_MOVE_BACKWARD = const(0x03)
DATA_PRESSURE = const(0x50)
DATA_PID = const(0x44)     

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
PID_CALLBACK_TIME = const(10) # ms
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
        self.active = False
        self.end_time = 0

    def move(self, ml):
        direction = -ml / abs(ml)
        self.servo.move(direction)
        wait_ms = int(abs(ml) * SECONDS_PER_ML * 1000)
        self.end_time = ticks_ms() + wait_ms
        self.active = True

    def update(self):
        if self.active and ticks_diff(ticks_ms(), self.end_time) >= 0:
            self.servo.stop()
            self.active = False

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
        self.prev_t = ticks_ms()
        self.prev_error = 0
        self.integral = 0
        self.dt = 0
        self.change_ml = 0
    
    # Assumes water density of 1 g/cm^3 or 1 g/mL and uses some fancy logic to do with water displacement relative to float volume and mass
    def error(self):
        m = FLOAT_VOLUME - FLOAT_MASS + self.ke * DESIRED_DEPTH
        return m * (DESIRED_DEPTH - self.pressure_sensor.read_depth()) 

    def compute(self, error):
        self.t = ticks_ms()
        self.dt = ticks_diff(self.t, self.prev_t) / 1000
        self.prev_t = self.t

        self.integral += error * self.dt
        self.integral = max(-self.integral_bound, min(self.integral, self.integral_bound))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        ml = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return ml
    
    def run(self):
        self.change_ml = self.compute(self.error())
        self.syringe.move(change_ml)

class Bluetooth:
    def __init__(self, name, pressure_sensor, syringe):   
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.connHandle = 0
        self.queue = deque((), int(DIVE_TIME / LOG_TIME * 2))
        self.connected = False
        self.disconnect()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        self.pressure_sensor = pressure_sensor
        self.syringe = syringe
        self.pid = None
        self.timer = Timer(0)
        self.timer.init(period=LOG_TIME, mode=Timer.PERIODIC, callback=lambda t: self.send_data())
        self.dive_timer = Timer(1)
        self.dive_timer.init(period=PID_CALLBACK_TIME, mode=Timer.PERIODIC, callback=lambda t: self.pid_run())
        self.elapsed = 0

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
                self.ble.gatts_write(self.tx, data, True)
            except:
                return
     
    def send(self, data):
        try:
            print("Queueing: " + data)
            self.queue.append(data)
            self.send_queued()
        except:
            return

    def send_pressure_data(self):
        voltage = self.pressure_sensor.read_voltage()
        pressure = self.pressure_sensor.read_pressure()
        depth = self.pressure_sensor.read_depth()
        
        data = struct.pack('>BHfff', 
                          DATA_PRESSURE,
                          self.elapsed,
                          voltage,      
                          pressure,     
                          depth)        
        self.send(data)

    def send_pid_data(self):
        error = self.pid.error()
        moved_ml = self.pid.change_ml
        
        data = struct.pack('>BHff', 
                          DATA_PID,    
                          self.elapsed,
                          error,       
                          moved_ml)    
        self.send(data)  
        
    def send_data(self):
        self.send_pressure_data()
        if self.pid is not None:
            self.send_pid_data()
        
    # First tries to reach the approximate depth by moving the syringe by a fixed amount then runs PID for the time that the diver must stay in place
    def dive(self, dive_milliliters, kp, ki, kd, ke, integral_bound):
        self.send(f"Diving for: {dive_milliliters} milliliters")
        self.syringe.move(dive_milliliters)
        self.pid = PID(self.pressure_sensor, self.syringe, kp, ki, kd, ke, integral_bound)
        
        self.dive_milliliters = dive_milliliters
        self.dive_start_time = ticks_ms()
    
    def pid_run(self):
        self.syringe.update() 
        if self.pid is not None:
            self.elapsed = ticks_diff(ticks_ms(), self.dive_start_time)
            if self.elapsed < DIVE_TIME:
                self.pid.run()
            else:
                self.syringe.move(-self.dive_milliliters)
                self.pid = None

    def parse_request(self, buffer):
        if len(buffer) < 1:
            return None, None
            
        cmd_id = buffer[0]
        
        if cmd_id == CMD_DIVE and len(buffer) >= 8:
            try:
                _, dive_ml, kp_raw, ki_raw, kd_raw, ke, integral_bound = struct.unpack('>BBHHHBb', buffer[:8])
                kp = kp_raw / 1000.0
                ki = ki_raw / 1000.0  
                kd = kd_raw / 1000.0
                return 'd', [dive_ml, kp, ki, kd, ke, integral_bound]
            except:
                return None, None
                
        elif cmd_id == CMD_MOVE_FORWARD and len(buffer) >= 2:
            try:
                _, amount = struct.unpack('>BB', buffer[:2])
                return 'f', amount
            except:
                return None, None
                
        elif cmd_id == CMD_MOVE_BACKWARD and len(buffer) >= 2:
            try:
                _, amount = struct.unpack('>BB', buffer[:2])
                return 'b', amount
            except:
                return None, None
        
        return None, None
    
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
            buffer = self.ble.gatts_read(self.rx)
            command, argument = self.parse_request(buffer)
               
            if command == 'd':
                if(self.pid == None):
                    self.dive(*argument)
            elif command == 'f':
                self.syringe.move(argument)
            elif command == 'b':
                self.syringe.move(-argument)
           
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
        self.ble.gap_advertise(100, bytearray(b'\x02\x01\x02') + bytearray([len(self.name) + 1, 0x09]) + self.name.encode('UTF-8'))

class Float:
    def __init__(self):
        self.pressure_sensor = PressureSensor()
        self.syringe = Syringe()
        self.bluetooth = Bluetooth("DemonDiver", self.pressure_sensor, self.syringe)

try:
    print("Starting Float")
    float = Float()
    while True:
        print(".")
        sleep(1)
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    exit()
except Exception as exp:
    print(exp)