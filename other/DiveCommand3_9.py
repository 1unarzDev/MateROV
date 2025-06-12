import tkinter
from threading import Thread
import queue
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import asyncio
from random import uniform
import DiverCom

root = tkinter.Tk()
root.wm_title("Dive Command")

inboundQueue = queue.Queue()

fig = Figure(figsize=(5, 4), dpi=100)
times = []
baseTime = None
depths = []
syringe_volumes = []
ax = None
diverCom = DiverCom.DiverCom("DemonDiver")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()

def update_line():
    global ax
    if ax is not None:
        fig.delaxes(ax)
    ax = fig.add_subplot()
    ax.plot(times, depths, label='Depth (m)')
    ax.plot(times, syringe_volumes, label='Syringe Volume (mL)')
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Depth (m) / Volume (mL)")
    canvas.draw()

def handle_data_message(msg):
    global baseTime
    with open("diverLog.txt", "a") as f:
        f.write(msg)
    data = msg.split(",")
    if len(data) < 4:
        return
    t = int(data[1])
    if baseTime is None:
        baseTime = t
    times.append(t - baseTime)
    milliVolts = float(data[3])
    depth = (milliVolts * 0.006953 - 3.107)
    if depth < 0.3:
        depth = 0.3 + uniform(-0.01, 0.01)
    depths.append(depth)
    if len(data) >= 7:
        syringe_vol = float(data[6])
    else:
        syringe_vol = 0
    syringe_volumes.append(syringe_vol)
    update_line()

def handle_message():
    try:
        msg = inboundQueue.get(False)
        label_message.config(text=msg)
        if msg.startswith("D"):
            handle_data_message(msg)
    except queue.Empty:
        pass
    root.after(100, handle_message)

def dive():
    diverCom.send("d 15")

def forward_5():
    diverCom.send("f 5")   # Sends the "forward 5 mL" command

def backward_5():
    diverCom.send("b 5")   # Sends the "backward 5 mL" command

def calibrate_zero():
    diverCom.send("z 0")

def resetGraph():
    global baseTime, depths, times
    baseTime = None
    depths = []
    times = []
    syringe_volumes = []
    update_line()
    print("Graph Reset")

# New Buttons for syringe movement
button_forward = tkinter.Button(master=root, text="Down 5 mL", command=forward_5)
button_backward = tkinter.Button(master=root, text="Up 5 mL", command=backward_5)

# UI Components

button_quit = tkinter.Button(master=root, text="Quit", command=root.destroy)
button_dive = tkinter.Button(master=root, text="Dive (15s)", command=dive)
button_calibrate = tkinter.Button(master=root, text="Calibrate Zero", command=calibrate_zero)
button_reset = tkinter.Button(master=root, text="Reset Graph", command=resetGraph)

label_connected = tkinter.Label(master=root, text="Not Connected", bg="red", fg="white")
label_message = tkinter.Label(master=root, width=20)

# Layout Adjustments
label_message.pack(side=tkinter.BOTTOM, fill=tkinter.X)
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=True)
button_quit.pack(side=tkinter.RIGHT)
button_dive.pack(side=tkinter.RIGHT)
button_calibrate.pack(side=tkinter.RIGHT)
button_reset.pack(side=tkinter.LEFT)
label_connected.pack(side=tkinter.LEFT)
button_forward.pack(side=tkinter.RIGHT)
button_backward.pack(side=tkinter.RIGHT)

def com_connected():
    label_connected.config(text="Connected", bg="green", fg="white")
def received(msg):
    inboundQueue.put(msg)
def com_disconnected():
    label_connected.config(text="Not Connected", bg="red", fg="white")

async def start_com():
    try:
        diverCom.onConnect = com_connected
        diverCom.onReceive = received
        diverCom.onDisconnect = com_disconnected
        await diverCom.run()
    except Exception as exp:
        print(exp)

Thread(target=asyncio.run, args=(start_com(),), daemon=True).start()

root.after(100, handle_message)
tkinter.mainloop()
