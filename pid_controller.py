import tkinter
from threading import Thread
import queue
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import asyncio
import com

KP = 0.8
KI = 0.02
KD = 0.1
KE = 5
INTEGRAL_BOUND = 3
MOVE_INCREMENT = 5
DIVE_ML = 40

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

def handle_data_message(msg):
    global base_time
    data = msg.split(",")
    if len(data) < 5:
        return
    t = int(data[1])
    voltage = float(data[2])
    pressure = float(data[3])
    depth = float(data[4])
    error = float(data[5]) if len(data) > 5 else 0
    move_ml = float(data[6]) if len(data) > 6 else 0

    parsed_msg = f"Time: {t}, Voltage: {voltage:.2f} V, Pressure: {pressure:.1f} Pa, Depth: {depth:.2f} m, Error: {error:.3f}, Move: {move_ml:.3f}" 
    print(parsed_msg)

    with open("diverLog.txt", "a") as f:
        f.write(parsed_msg + "\n")

    if base_time is None:
        base_time = t
    times.append((t - base_time) / 1000)
    depths.append(depth)
    errors.append(error)
    move_mls.append(move_ml)

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
    diver_com.send(f"d {DIVE_ML},{KP},{KI},{KD},{KE},{INTEGRAL_BOUND}")

def forward():
    diver_com.send(f"f {MOVE_INCREMENT}")

def backward():
    diver_com.send(f"b {MOVE_INCREMENT}")

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