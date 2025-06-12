import tkinter
from threading import Thread
import queue
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import asyncio
import com
import struct

KP = 0.8
KI = 0.02
KD = 0.1
KE = 5
INTEGRAL_BOUND = 3
MOVE_INCREMENT = 5
DIVE_ML = 5

root = tkinter.Tk()
root.wm_title("Diver Controller")

inbound_queue = queue.Queue()

fig = Figure(figsize=(5, 4), dpi=100)
times = []
base_time = None
depths = []
errors = []
move_mls = []
ax = None
diver_com = com.DiverCom("DemonDiver")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()

def update_line():
    global ax
    if ax is not None:
        fig.delaxes(ax)
    ax = fig.add_subplot()
    ax.plot(times, depths, label="Depth (m)")
    if errors:
        ax.plot(times[:len(errors)], errors, label="Error")
    if move_mls:
        ax.plot(times[:len(move_mls)], move_mls, label="Move mL")
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
        if len(data) >= 11:
            try:
                _, timestamp, error, moved_ml = struct.unpack('>BHff', data[:11])
                return {
                    'type': 'pid',
                    'timestamp': timestamp,
                    'error': error,
                    'moved_ml': moved_ml
                }
            except struct.error:
                pass
    
    return None

def handle_pressure_data(data):
    global base_time
    
    latest_pressure_data = data
    t = int(data['timestamp'])
    voltage = data['voltage']
    pressure = data['pressure']
    depth = data['depth']
    
    if base_time is None:
        base_time = t
    
    times.append((t - base_time) / 1000)
    depths.append(depth)
    
    log_msg = f"Pressure - Time: {t}, Voltage: {voltage:.2f} V, Pressure: {pressure:.1f} Pa, Depth: {depth:.2f} m"
    print(log_msg)
    with open("diverLog.txt", "a") as f:
        f.write(log_msg + "\n")

def handle_pid_data(data):
    t = int(data['timestamp'])
    error = data['error']
    moved_ml = data['moved_ml']
    
    if len(errors) < len(times):
        errors.extend([0] * (len(times) - len(errors)))
    if len(move_mls) < len(times):
        move_mls.extend([0] * (len(times) - len(move_mls)))
    
    if len(errors) == len(times):
        errors.append(error)
        move_mls.append(moved_ml)
    else:
        errors[-1] = error
        move_mls[-1] = moved_ml
    
    log_msg = f"PID - Time: {t}, Error: {error:.3f}, Moved: {moved_ml:.3f} mL"
    print(log_msg)
    with open("diverLog.txt", "a") as f:
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
                      KE,
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

def reset_graph():
    global base_time, depths, times, errors, move_mls
    base_time = None
    depths = []
    times = []
    errors = []
    move_mls = []
    update_line()
    print("Graph Reset")

button_forward = tkinter.Button(master=root, text=f"Down {MOVE_INCREMENT} mL", command=forward)
button_backward = tkinter.Button(master=root, text=f"Up {MOVE_INCREMENT} mL", command=backward)
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