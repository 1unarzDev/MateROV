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
CMD_STOP = const(0x04)
DATA_PRESSURE = const(0x50)
DATA_PID = const(0x44)     

# General
LOG_TIME = const(1000) # ms
SAFE_MODE_PIN = Pin(10, Pin.IN, Pin.PULL_UP)
SERVO_PIN = const(2)
PRESSURE_SENSOR_PIN = const(3)

# Servo
SERVO_STOPPED = const(1.5) # ms
SERVO_MOVE_RANGE = const(0.5) # ms
EPSILON = const(3) # mL

# Pressures sensor (depth)
WATER_DENSITY = const(1000) # kg/m³
GRAVITY = const(9.81) # m/s²
MIN_PRESSURE_VOLTAGE = const(0.5) # V
MAX_PRESSURE_VOLTAGE = const(4.5) # V
MAX_PRESSURE_RATING = const(206843) # Pa
ATMOSPHERE_PRESSURE = const(101325) # Pa

# Syringe
SECONDS_PER_ML = const(0.85)

# PID
PID_ENABLED = const(False)
PID_CALLBACK_TIME = const(10) # ms
DIVE_TIME = const(45000) # ms (45000 for task)
DESIRED_DEPTH = const(2.5) # m (2.5 for task)

# Continuous rotation servo which takes in input speeds (pulse widths in ms) rather than positional inputs
class Servo:
    def __init__(self):
        self.pwm = PWM(Pin(SERVO_PIN))
        self.pwm.freq(50)

    def set_duty_ms(self, ms):
        self.pwm.duty_ns(int(ms * 1e6))
        
    def stop(self):
        self.set_duty_ms(SERVO_STOPPED)

    # 1 is backward the fastest and -1 is forward the fastest
    def move(self, speed):
        duty_ms = (SERVO_STOPPED + max(-0.5, min(0.5, speed * SERVO_MOVE_RANGE)))
        self.set_duty_ms(-duty_ms)
        
class Syringe:
    def __init__(self):
        self.servo = Servo()
        self.pos = 0
        self.goal_pos = 0
        self.dt = 0
        self.prev_t = time_ms()
        self.end_time = 0
        self.movement = 0

    def move(self, ml):
        self.goal_pos = ml        
        
    def stop(self):
        self.servo.stop()

    def update(self):
        self.t = time_ms()
        self.dt = t - prev_t / 1000.0
        self.prev_t = t
        
        self.pos += movement * SECONDS_PER_ML * dt

        if(abs(self.goal_pos - self.pos) <= EPSILON):
            self.stop()
        else:
            self.movement = abs(self.goal_pos - self.pos) / (self.goa_pos - self.pos)
            self.servo.move(movement)

class PressureSensor:
    def __init__(self):
        self.sensor = ADC(Pin(PRESSURE_SENSOR_PIN))
        self.sensor.atten(ADC.ATTN_11DB)
    
    def read_voltage(self):
        return max(0.5, self.sensor.read_uv() / 1e6)
    
    def read_depth(self):
        voltage = self.read_voltage()
        return voltage * 6.953 - 3.107 # Linear regression since it is directly proportional

    # Work backward from depth because we don't have a regression for it
    def read_pressure(self):
        depth = self.read_depth()    
        pressure = depth * GRAVITY  * WATER_DENSITY + ATMOSPHERE_PRESSURE
        return pressure

class PID:
    def __init__(self, pressure_sensor, syringe, kp, ki, kd, equilibrium, integral_bound):
        self.desired_depth = DESIRED_DEPTH
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.equlibrium = equilibrium
        self.integral_bound = integral_bound
        self.pressure_sensor = pressure_sensor
        self.syringe = syringe
        self.prev_t = ticks_ms()
        self.prev_error = DESIRED_DEPTH
        self.integral = 0
        self.dt = 0
        self.change_ml = 0
    
    def error(self):
        return DESIRED_DEPTH - self.pressure_sensor.read_depth() 

    def compute(self, error):
        self.t = ticks_ms()
        self.dt = ticks_diff(self.t, self.prev_t) / 1000.0
        self.prev_t = self.t

        self.integral += error * self.dt
        self.integral = max(-self.integral_bound, min(self.integral, self.integral_bound))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        ml = self.equlibrium + (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return ml
    
    def run(self):
        self.change_ml = self.compute(self.error())
        if(PID_ENABLED):
            self.syringe.move(self.change_ml)

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
                self.ble.gatts_write(self.tx, data, True)
            except:
                return
     
    def send(self, data):
        try:
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
                          ticks_ms(),
                          voltage,      
                          pressure,     
                          depth)        
        self.send(data)

    def send_pid_data(self):
        error = self.pid.error()
        moved_ml = self.pid.change_ml
        
        data = struct.pack('>BHff', 
                          DATA_PID,    
                          ticks_ms(),
                          error,       
                          moved_ml)  
        self.send(data)  
        
    def send_data(self):
        if self.connected:
            self.send_pressure_data()
            if self.pid is not None:
                self.send_pid_data()
        
    # First tries to reach the approximate depth by moving the syringe by a fixed amount then runs PID for the time that the diver must stay in place
    def dive(self, dive_milliliters, kp, ki, kd, equilibrium, integral_bound):
        self.syringe.move(dive_milliliters)
        self.pid = PID(self.pressure_sensor, self.syringe, kp, ki, kd, equilibrium, integral_bound)
        
        self.dive_milliliters = dive_milliliters
        self.dive_start_time = ticks_ms()
    
    def pid_run(self):
        self.syringe.update() 
        if self.pid is not None:
            self.elapsed = ticks_ms() - self.dive_start_time
            if self.elapsed < DIVE_TIME:
                self.pid.run()
            else:
                self.syringe.move(0)
                self.pid = None

    def parse_request(self, buffer):
        if len(buffer) < 1:
            return None, None

        cmd_id = buffer[0]
        
        if cmd_id == CMD_DIVE and len(buffer) >= 10:
            try:
                _, dive_ml, kp_raw, ki_raw, kd_raw, ke, integral_bound = struct.unpack('>BBHHHBb', buffer[:10])
                kp = kp_raw / 1000.0
                ki = ki_raw / 1000.0  
                kd = kd_raw / 1000.0
                return 'd', [dive_ml, kp, ki, kd, ke, integral_bound]
            except:
                print("Failed to unpack dive")
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
        
        elif cmd_id == CMD_STOP:
            try:
                return 's', 0;
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
            print(f"Received data: {command}, {argument}")
               
            if command == 'd':
                if(self.pid == None):
                    self.dive(*argument)
            elif command == 'f':
                self.syringe.move(argument)
            elif command == 'b':
                self.syringe.move(-argument)
            elif command == 's':
                self.pid = None
                self.syringe.stop()
           
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

class Diver:
    def __init__(self):
        self.pressure_sensor = PressureSensor()
        self.syringe = Syringe()
        self.bluetooth = Bluetooth("DemonDiver", self.pressure_sensor, self.syringe)

try:
    if SAFE_MODE_PIN.value() == 1:
        print("Starting Diver")
        diver = Diver()
        while True:
            print(".")
            sleep(1)
    else:
        print("Safety Pin Activated, Quitting...")
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    exit()
except Exception as exp:
    print(exp)