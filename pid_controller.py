import tkinter
from threading import Thread
import queue
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import asyncio
import com
import struct
import os
from datetime import datetime

KP = 0.8
KI = 0.01
KD = 0.4
EQULIBRIUM = 40
INTEGRAL_BOUND = 100
MOVE_INCREMENT = 5
DIVE_ML = 40

root = tkinter.Tk()
root.wm_title("Diver Controller")

inbound_queue = queue.Queue()

fig = Figure(figsize=(5, 4), dpi=100)
base_time = None
depths = []
proportionals = []
integrals = []
derivatives = []
ax = None
diver_com = com.DiverCom("DemonDiver")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()

if not os.path.exists("logs"):
    os.makedirs("logs")

def get_next_log_filename():
    log_number = 1
    while os.path.exists(f"logs/diverLog_{log_number:03d}.txt"):
        log_number += 1
    return f"logs/diverLog_{log_number:03d}.txt"

current_log_file = get_next_log_filename()

with open(current_log_file, "w") as f:
    f.write(f"Diver Log Session Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"Parameters - KP: {KP}, KI: {KI}, KD: {KD}, Equilibrium: {EQULIBRIUM}, Integral Bound: {INTEGRAL_BOUND}\n")
    f.write("-" * 80 + "\n")

def ticks_diff(end, start):
    return (end - start) & 0xFFFFFFFF

def update_line():
    global ax
    if ax is not None:
        fig.delaxes(ax)
    ax = fig.add_subplot()
    ax.plot(list(row[0] for row in depths), list(row[1] for row in depths), label="Depth (m)")
    if proportionals:
        ax.plot(list(row[0] for row in proportionals), list(row[1] for row in proportionals), label="Proportional")
    if integrals:
        ax.plot(list(row[0] for row in integrals), list(row[1] for row in integrals), label="Integral")
    if derivatives:
        ax.plot(list(row[0] for row in derivatives), list(row[1] for row in derivatives), label="Derivative")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Values")
    ax.legend()
    canvas.draw()

def parse_binary_data(data):
    if len(data) < 1:
        return None
        
    packet_type = data[0]
    
    if packet_type == 0x50:
        if len(data) >= 15:
            try:
                _, timestamp, voltage, pressure, depth = struct.unpack('>BHfff', data[:15])
                return {
                    'type': 'pressure',
                    'timestamp': timestamp,
                    'voltage': voltage,
                    'pressure': pressure,
                    'depth': depth
                }
            except struct.error:
                pass
                
    elif packet_type == 0x44:  
        if len(data) >= 15:
            try:
                _, timestamp, proportional, integral, derivative = struct.unpack('>BHfff', data[:15])
                return {
                    'type': 'pid',
                    'timestamp': timestamp,
                    'proportional': proportional,
                    'integral': integral,
                    'derivative': derivative,
                }
            except struct.error:
                pass
    
    return None

def handle_pressure_data(data):
    global base_time
    
    t = int(data['timestamp'])
    voltage = data['voltage']
    pressure = data['pressure']
    depth = data['depth']
    
    if base_time is None:
        base_time = t
    
    curr_time = ticks_diff(t, base_time) / 1000

    point = [curr_time, depth]
    depths.append(point)
    
    log_msg = f"Pressure - Time: {t}, Voltage: {voltage:.2f} V, Pressure: {pressure:.1f} Pa, Depth: {depth:.2f} m"
    print(log_msg)
    with open(current_log_file, "a") as f:
        f.write(log_msg + "\n")

def handle_pid_data(data):
    global base_time

    t = int(data['timestamp'])
    proportional = data['proportional']
    integral = data['integral']
    derivative = data['derivative']
    
    if base_time is None:
        base_time = t

    curr_time = ticks_diff(t, base_time) / 1000

    point = [curr_time, proportional]
    proportionals.append(point)

    point = [curr_time, integral]
    integrals.append(point)

    point = [curr_time, derivative]
    derivatives.append(point)

    log_msg = f"PID - Time: {t}, P: {proportional:.2f}, I: {integral:.2f}, D: {derivative:.2f}"
    print(log_msg)
    with open(current_log_file, "a") as f:
        f.write(log_msg + "\n")

def handle_data_message(msg):
    parsed_data = parse_binary_data(msg)
    
    if parsed_data:
        if parsed_data['type'] == 'pressure':
            handle_pressure_data(parsed_data)
        elif parsed_data['type'] == 'pid':
            handle_pid_data(parsed_data)
        
        update_line()

def handle_message():
    try:
        msg = inbound_queue.get(False)
        handle_data_message(msg)
    except queue.Empty:
        pass
    root.after(100, handle_message)
    
def dive():
    data = struct.pack('>BBHHHBb', 
                      0x01,  # Command ID for dive
                      DIVE_ML,
                      int(KP * 1000),
                      int(KI * 1000), 
                      int(KD * 1000),
                      EQULIBRIUM,
                      INTEGRAL_BOUND)
    diver_com.send(data)

def move(direction, amount):
    cmd = 0x02 if direction == 0 else 0x03
    data = struct.pack('>BB', cmd, int(abs(amount)))
    diver_com.send(data)

def forward():
    move(0, MOVE_INCREMENT)

def backward():
    move(1, MOVE_INCREMENT)

def stop():
    data = struct.pack('>B', 0x04)
    diver_com.send(data)

def reset_graph():
    global base_time, depths, times, proportionals, integrals, derivatives
    base_time = None
    depths = []
    times = []
    proportionals = []
    integrals = []
    derivatives = []
    update_line()
    print("Graph Reset")

button_forward = tkinter.Button(master=root, text=f"Down {MOVE_INCREMENT} mL", command=forward)
button_backward = tkinter.Button(master=root, text=f"Up {MOVE_INCREMENT} mL", command=backward)
button_stop = tkinter.Button(master=root, text=f"Stop", command=stop)
button_quit = tkinter.Button(master=root, text="Quit", command=root.destroy)
button_dive = tkinter.Button(master=root, text="Dive", command=dive)
button_reset = tkinter.Button(master=root, text="Reset Graph", command=reset_graph)

label_connected = tkinter.Label(master=root, text="Not Connected", bg="red", fg="white")
label_message = tkinter.Label(master=root, width=40)

label_message.pack(side=tkinter.BOTTOM, fill=tkinter.X)
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=True)
button_quit.pack(side=tkinter.RIGHT)
button_dive.pack(side=tkinter.RIGHT)
button_reset.pack(side=tkinter.LEFT)
label_connected.pack(side=tkinter.LEFT)
button_forward.pack(side=tkinter.RIGHT)
button_backward.pack(side=tkinter.RIGHT)
button_stop.pack(side=tkinter.RIGHT)

def com_connected():
    label_connected.config(text="Connected", bg="green", fg="white")

def com_received(msg):
    inbound_queue.put(msg)

def com_disconnected():
    label_connected.config(text="Not Connected", bg="red", fg="white")

async def start_com():
    try:
        diver_com.on_connect = com_connected
        diver_com.on_received = com_received
        diver_com.on_disconnect = com_disconnected
        await diver_com.run()
    except Exception as exp:
        print(exp)

Thread(target=asyncio.run, args=(start_com(),), daemon=True).start()

root.after(100, handle_message)
tkinter.mainloop()