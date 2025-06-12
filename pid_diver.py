from machine import Timer, RTC, Pin, PWM, ADC
from sys import exit
from time import time, sleep
from collections import deque
import ubluetooth 
from micropython import const

# Micropython const function optimizes code at runtime
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
CALLBACK_TIME = const(1000) # ms
SERVO_STOPPED = const(1.5) # ms
SERVO_MOVE_RANGE = const(0.5) # ms
SERVO_SLEEP = const(50) # ms
WATER_DENSITY = const(1000) # kg/m³
GRAVITY = const(9.81) # m/s²
CALLBACK_TIME = const(100) # ms
MIN_PRESSURE_VOLTAGE = const(0.5) # V
MAX_PRESSURE_VOLTAGE = const(4.5) # V
MAX_PRESSURE_RATING = const(206843) # Pa
ATMOSPHERE_PRESSURE = const(101325) # Pa
SECONDS_PER_ML = const(0.2)

# Continuous rotation servo which takes in input speeds (pulse widths in ms) rather than positional inputs
class Servo():
    def __init__(self):
        self.pwm = PWM(Pin(2))
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
        
class Syringe():
    def __init__(self):
        self.servo=Servo()
        
    def move_syringe(self,ml):
        self.servo.move(ml / abs(ml)) # Move in appropriate direction
        sleep(ml*self.SECONDS_PER_ML)
        self.servo.stop()

class PressureSensor:
    def __init__(self):
        self.sensor = ADC(Pin(3))
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

class PID():
    # TODO: Tune the PID parameters in the controller
    def __init__(self, pressure_sensor, desired_depth, syringe, dive_time=20, kp=0.8, ki=0.02, kd=0.1, integral_bound=3):
        self.time = dive_time
        self.time_left = self.time
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_bound = integral_bound
        self.pressure_sensor = pressure_sensor
        self.desired_depth = desired_depth
        self.syringe = syringe
        self.prev_t = time()
        self.dt = 0
        
    def depth_to_ml(ml):
        return ml * AVERAGE_CROSS_SECTIONAL_AREA * 1e6 

    def compute(self, error):
        self.t = time()
        self.dt = self.t - self.prev_t
        self.prev_t = self.t

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_bound, self.integral_bound)  
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        ml = (self.Kp * error) + (self.Kd * derivative) + (self.Ki * self.integral)
        return depth_to_ml(ml) 
    
    def run():
        while(self.time_left > 0):
            # TODO: Find the average intial error of the depth (e.g., it says that it is 5m before entering the water, then adjust for it)
            error = self.pressure_sensor.read()/GRAVITY - self.desired_depth
            self.syringe.move_syringe(compute(error))
            self.time_left -= self.dt

class BLE():
    def __init__(self, name):   
        self.name = name.encode('UTF-8')
        self.syringe = Syringe()
        
        # Setup the bluetooth
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.timer = Timer(0)
        self.connHandle = 0
        self.queue = deque((), 100)
        self.connected = False
        self.disconnect()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        self.pressure_sensor = PressureSensor()
        self.timer.init(period=CALLBACK_TIME, mode=Timer.PERIODIC, callback=lambda t: self.send_data())

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
        
    # Message in format [time, kpa, millivolts, depth]
    def send_data(self):
        millivolts = self.pressure_sensor.read_millivolts()
        kpa = self.pressure_sensor.read()
        data = f"D,{time()},{kpa},{millivolts},{kpa/GRAVITY}" # Dividing killopascals by gravity tells us the depth in water 
        self.send(data)

    def set_utc(self, argument):
        try:
            print(argument)
            if argument != None:
                splits = argument.split(':')
                if len(splits) == 3:
                    hours = int(splits[0])
                    minutes = int(splits[1])
                    seconds = int(splits[2])
                    RTC().datetime((2023,5,6,1,hours,minutes,seconds,0))
                    print(rtc.datetime())
        except:
            return
        
    # First tries to reach the approximate depth by moving the syringe by a fixed amount then runs PID for the time that the diver must stay in place
    def dive(self, dive_milliliters, time, kp, ki, kd, integral_bound, desired_depth):
        self.send(f"Diving for: {dive_milliliters} milliliters")
        self.syringe.move_syringe(-dive_milliliters)
        self.pid = PID(self.pressure_sensor, desired_depth, self.syringe, time, kp, ki, kd, integral_bound)
        self.pid.run()
        self.syringe.move_syringe(dive_milliliters + DIVE_SURFACE_BUFFER)

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
            if (command == 'time') or (command == 't'):
               self.set_utc(argument)
               self.send("Time Set")
               
            if (command == 'dive') or (command == 'd'):
                self.dive(*list(map(int, argument.split(",")[:7])))
                
            if (command == 'forward') or (command == 'f'):
                self.syringe.move_syringe(int(argument))
                
            if (command == 'backward') or (command == 'b'):
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
    
    def run(self):
        ...

# Runtime loop
while True:
    try:
        ble = BLE("DemonDiver")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        exit()
    except Exception as exp:
        print(exp)