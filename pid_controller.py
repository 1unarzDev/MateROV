import tkinter
from threading import Thread
import queue
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import asyncio
from random import uniform
import com

# TODO: Find good pid parameter values
TURN_RATIO = 23/50 # Turn ratio from testing
DIVE_TIME = 20
KP = 0.8
KI = 0.02
KD = 0.1
INTEGRAL_BOUND = 3
MOVE_INCREMENT = 5 # Defines move command movement
COMPANY_NUMBER = 2
DESIRED_DEPTH = 2.5 # TODO: Find the desired depth in meters

root = tkinter.Tk()
root.wm_title("Diver Controller")

inbound_queue = queue.Queue()

fig = Figure(figsize=(5, 4), dpi=100)
times = []
base_time = None
depths = []
syringe_volumes = []
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
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Depth (m) / Volume (mL)")
    canvas.draw()

# Message in format [time, kpa, millivolts, depth]
def handle_data_message(msg):
    # See send_data() in the BLE class of pid_diver.py 
    global base_time
    
    data = msg.split(",")
    # First element in array is the command "D"
    if len(data) < 4:
        return
    t = int(data[1])
    kpa = float(data[2])
    millivolts = float(data[3])
    depth = float(data[4])

    parsed_msg = f"Time: {t}, Kpa: {kpa}, Millivolts: {millivolts}, Depth: {depth}" 
    print(parsed_msg)

    # Log data
    with open("diverLog.txt", "a") as f:
        f.write(parsed_msg)

    # Update time from diver
    if base_time is None:
        base_time = t
    times.append(t - base_time)

    # Update depth data from pressure sensor
    depths.append(depth)

    update_line()

def handle_message():
    try:
        msg = inbound_queue.get(False)
        label_message.config(text=msg)
        if msg.startswith("D"):
            handle_data_message(msg)
    except queue.Empty:
        pass
    root.after(100, handle_message)

def dive():
    diver_com.send(f"d {30/TURN_RATIO},{DIVE_TIME},{KP},{KI},{KD},{INTEGRAL_BOUND},{DESIRED_DEPTH}") # Must maintain this exact format or match diver pid 

def forward():
    diver_com.send(f"f {5/TURN_RATIO}")

def backward():
    diver_com.send(f"b {5/TURN_RATIO}")

def reset_graph():
    global base_time, depths, times
    base_time = None
    depths = []
    times = []
    update_line()
    print("Graph Reset")

# Basic syringe movement commands
button_forward = tkinter.Button(master=root, text=f"Down {MOVE_INCREMENT} mL", command=forward)
button_backward = tkinter.Button(master=root, text=f"Up {MOVE_INCREMENT} mL", command=backward)

# UI Components
button_quit = tkinter.Button(master=root, text="Quit", command=root.destroy)
button_dive = tkinter.Button(master=root, text="Dive", command=dive)
button_reset = tkinter.Button(master=root, text="Reset Graph", command=reset_graph)

label_connected = tkinter.Label(master=root, text="Not Connected", bg="red", fg="white")
label_message = tkinter.Label(master=root, width=20)

# Layout Adjustments
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
def received(msg):
    inbound_queue.put(msg)
def com_disconnected():
    label_connected.config(text="Not Connected", bg="red", fg="white")

async def start_com():
    try:
        diver_com.onConnect = com_connected
        diver_com.onReceive = received
        diver_com.onDisconnect = com_disconnected
        await diver_com.run()
    except Exception as exp:
        print(exp)

Thread(target=asyncio.run, args=(start_com(),), daemon=True).start()

root.after(100, handle_message)
tkinter.mainloop()