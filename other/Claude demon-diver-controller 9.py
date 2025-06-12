import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import asyncio
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import time
import re
import threading
import queue
import bleak
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("DemonDiverController")

# UUIDs for Nordic UART Service
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to this characteristic
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Receive notifications from this characteristic

# Maximum history to keep in graphs (seconds)
MAX_HISTORY = 300  # 5 minutes

class BLEController:
    def __init__(self, data_callback, connection_callback, message_callback):
        self.client = None
        self.device_name = "DemonDiver"
        self.connected = False
        self.connecting = False
        self.data_callback = data_callback
        self.connection_callback = connection_callback
        self.message_callback = message_callback
        self.disconnect_event = asyncio.Event()
        self.should_reconnect = True
        self.manual_disconnect = False  # Flag for manual disconnection
        self.loop = None
        self.rx_char = None
        
    def set_event_loop(self, loop):
        """Set the asyncio event loop for this controller"""
        self.loop = loop
        
    def set_device_name(self, name):
        """Set the target device name"""
        self.device_name = name
        
    async def start_scan(self):
        """Start scanning for BLE devices - with manual disconnect handling"""
        self.should_reconnect = True
        self.manual_disconnect = False  # Reset manual disconnect flag when starting scan
        
        while self.should_reconnect:
            try:
                # Skip if already connected
                if self.connected:
                    await asyncio.sleep(1)
                    continue
                
                # Skip if already trying to connect
                if self.connecting:
                    await asyncio.sleep(1)
                    continue
                
                self.connecting = True
                self.connection_callback(f"Searching for {self.device_name}...")
                
                # Simple direct approach - find device by name
                device = await BleakScanner.find_device_by_name(self.device_name)
                
                if device:
                    self.connection_callback(f"Found {device.name}, connecting...")
                    async with BleakClient(device, disconnected_callback=self.handle_disconnect) as self.client:
                        await self.client.start_notify(UART_TX_CHAR_UUID, self.notification_handler)
                        self.connected = True
                        self.connecting = False
                        self.connection_callback(f"Connected to {device.name}")
                        
                        # Get the UART service and characteristic
                        nus = self.client.services.get_service(UART_SERVICE_UUID)
                        self.rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)
                        
                        # Wait until disconnected
                        self.disconnect_event.clear()
                        await self.disconnect_event.wait()
                        
                        # If this was a manual disconnect, exit the loop
                        if self.manual_disconnect:
                            logger.info("Manual disconnect detected, stopping reconnection")
                            break
                else:
                    self.connection_callback(f"Device '{self.device_name}' not found. Retrying...")
                    self.connecting = False
                    await asyncio.sleep(5)  # Wait 5 seconds before retrying
                    
            except Exception as e:
                logger.error(f"Connection error: {e}")
                self.connection_callback(f"Connection error: {str(e)}")
                self.connected = False
                self.connecting = False
                await asyncio.sleep(5)  # Wait before retrying
                
        # Reset flags when exiting the reconnection loop
        self.connecting = False
        self.connected = False
        logger.info("Exited reconnection loop")

    def handle_disconnect(self, client):
        """Handle disconnection event"""
        logger.info("Disconnected from device")
        self.connected = False
        self.connecting = False
        
        # Only show "Reconnecting" message if not manually disconnected
        if not self.manual_disconnect:
            self.connection_callback("Disconnected. Reconnecting...")
        else:
            self.connection_callback("Disconnected.")
            
        self.disconnect_event.set()
        
    def notification_handler(self, sender, data):
        """Handle incoming notifications from the BLE device"""
        try:
            message = data.decode('utf-8').strip()
            logger.debug(f"Received: {message}")
            
            # Parse the message based on its prefix
            if message.startswith('D,'):  # Data message
                self.parse_data_message(message)
            elif message.startswith('M,'):  # User message
                user_msg = message[2:]  # Remove 'M,' prefix
                self.message_callback("MESSAGE", user_msg)
            elif message.startswith('L,'):  # Log message
                parts = message[2:].split(',', 2)
                if len(parts) >= 3:
                    timestamp, level, log_msg = parts
                    self.message_callback("LOG", f"[{level}] {log_msg}")
                else:
                    self.message_callback("LOG", message[2:])
            else:
                self.message_callback("UNKNOWN", message)
                
        except Exception as e:
            logger.error(f"Error processing notification: {e}")

    def parse_data_message(self, message):
        """Parse data messages from the device (depth, pressure, etc.)"""
        try:
            # Expected format: D,timestamp,company,pressure_kpa,millivolts,depth_m,volume_ml
            parts = message.split(',')
            if len(parts) >= 7:
                timestamp = float(parts[1])
                company = int(parts[2])
                pressure_kpa = float(parts[3])
                millivolts = float(parts[4])
                depth_m = float(parts[5])
                volume_ml = float(parts[6])
                
                # Create data dict and call the callback
                data = {
                    'timestamp': timestamp,
                    'company': company,
                    'pressure_kpa': pressure_kpa,
                    'millivolts': millivolts,
                    'depth_m': depth_m,
                    'volume_ml': volume_ml,
                    'system_time': time.time()
                }
                
                # Call the callback directly
                if self.data_callback:
                    self.data_callback(data)
                
        except Exception as e:
            logger.error(f"Error parsing data message: {e}, message: {message}")

    async def send_command(self, command):
        """Send a command to the device"""
        if not self.connected:
            self.message_callback("ERROR", "Not connected to device")
            return False
        
        try:
            logger.info(f"Sending command: {command}")
            # Add newline to ensure proper command processing on device
            command_bytes = (command + '\n').encode('utf-8')
            await self.client.write_gatt_char(self.rx_char, command_bytes)
            return True
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            self.message_callback("ERROR", f"Failed to send command: {str(e)}")
            return False

    def disconnect(self):
        """Disconnect from the device - with manual flag"""
        logger.info("Manual disconnect requested")
        self.should_reconnect = False
        self.manual_disconnect = True  # Set the manual disconnect flag
        
        if self.connected:
            # Signal disconnect event to exit the connection context
            self.disconnect_event.set()
            self.connection_callback("Disconnecting...")
        else:
            # If we're not connected, just update the UI
            self.connection_callback("Disconnected")


class DataManager:
    """Manages data storage and retrieval for plotting"""
    def __init__(self, max_history=MAX_HISTORY):
        self.max_history = max_history
        self.reset()
    
    def reset(self):
        """Reset all data series"""
        self.timestamps = []
        self.system_times = []
        self.depths = []
        self.volumes = []
        self.pressures = []
        self.millivolts = []
    
    def add_data_point(self, data):
        """Add a new data point to all series"""
        # Add the new data
        self.timestamps.append(data['timestamp'])
        self.system_times.append(data['system_time'])
        self.depths.append(data['depth_m'])
        self.volumes.append(data['volume_ml'])
        self.pressures.append(data['pressure_kpa'])
        self.millivolts.append(data['millivolts'])
        
        # Trim old data beyond max_history
        if len(self.system_times) > 1:
            current_time = self.system_times[-1]
            cutoff_time = current_time - self.max_history
            
            # Find the index to cut at
            cut_index = 0
            for i, t in enumerate(self.system_times):
                if t >= cutoff_time:
                    cut_index = i
                    break
            
            # Only trim if we have more than one point after trimming
            if cut_index > 0 and len(self.system_times) - cut_index > 1:
                self.timestamps = self.timestamps[cut_index:]
                self.system_times = self.system_times[cut_index:]
                self.depths = self.depths[cut_index:]
                self.volumes = self.volumes[cut_index:]
                self.pressures = self.pressures[cut_index:]
                self.millivolts = self.millivolts[cut_index:]
    
    def get_relative_times(self):
        """Get times relative to the first data point"""
        if not self.system_times:
            return []
        
        reference_time = self.system_times[0]
        return [t - reference_time for t in self.system_times]


class DemonDiverGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("DemonDiver Controller")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Data manager for storing and retrieving time series data
        self.data_manager = DataManager()
        
        # BLE controller
        self.ble = BLEController(
            data_callback=self.on_data_received,
            connection_callback=self.on_connection_status,
            message_callback=self.on_message_received
        )
        
        # Create main frame
        self.main_frame = ttk.Frame(root, padding="10")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create connection status frame
        self.status_frame = ttk.LabelFrame(self.main_frame, text="Connection Status", padding="5")
        self.status_frame.pack(fill=tk.X, pady=5)
        
        self.status_label = ttk.Label(self.status_frame, text="Disconnected")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.connect_button = ttk.Button(self.status_frame, text="Connect", command=self.toggle_connect)
        self.connect_button.pack(side=tk.RIGHT, padx=5)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create Dashboard tab
        self.dashboard_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.dashboard_frame, text="Dashboard")
        
        # Create Controls tab
        self.controls_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.controls_frame, text="Controls")
        
        # Create Settings tab
        self.settings_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.settings_frame, text="Settings")
        
        # Create Logs tab
        self.logs_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.logs_frame, text="Logs")
        
        # Set up the dashboard
        self.setup_dashboard()
        
        # Set up the controls
        self.setup_controls()
        
        # Set up the settings
        self.setup_settings()
        
        # Set up the logs
        self.setup_logs()
        
        # Create asyncio event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.ble.set_event_loop(self.loop)
        self.loop_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.loop_thread.start()

        # Set up periodic UI updates for plots
        self.root.after(100, self.update_ui)
    
    def setup_dashboard(self):
        """Set up the dashboard with real-time graphs"""
        # Create a frame for current values
        current_values_frame = ttk.LabelFrame(self.dashboard_frame, text="Current Values", padding="10")
        current_values_frame.pack(fill=tk.X, pady=5)
        
        # Create labels for current values
        ttk.Label(current_values_frame, text="Depth:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.current_depth_label = ttk.Label(current_values_frame, text="-- m")
        self.current_depth_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(current_values_frame, text="Volume:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.current_volume_label = ttk.Label(current_values_frame, text="-- mL")
        self.current_volume_label.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(current_values_frame, text="Pressure:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.current_pressure_label = ttk.Label(current_values_frame, text="-- kPa")
        self.current_pressure_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(current_values_frame, text="Voltage:").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.current_voltage_label = ttk.Label(current_values_frame, text="-- mV")
        self.current_voltage_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)
        
        # Create a frame for combined graph
        graph_frame = ttk.Frame(self.dashboard_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create the figure and axes for combined depth and volume graph
        self.fig = Figure(figsize=(10, 8))
        self.ax1 = self.fig.add_subplot(111)
        
        # Create a second y-axis that shares the same x-axis
        self.ax2 = self.ax1.twinx()
        
        # Set up the depth axis (left)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Depth (m)', color='blue')
        self.ax1.tick_params(axis='y', labelcolor='blue')
        self.ax1.grid(True)
        self.ax1.invert_yaxis()  # Invert y-axis for depth (positive down)
        
        # Set up the volume axis (right)
        self.ax2.set_ylabel('Volume (mL)', color='red')
        self.ax2.tick_params(axis='y', labelcolor='red')
        
        # Set the title
        self.fig.suptitle('Depth and Plunger Volume vs. Time', fontsize=14)
        
        # Create the canvas for matplotlib
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Create the initial empty plots
        self.depth_line, = self.ax1.plot([], [], 'b-', label='Depth')
        self.volume_line, = self.ax2.plot([], [], 'r-', label='Volume')
        
        # Create the hover target depth lines (will be updated later)
        self.hover_depth = 2.5  # Default value
        self.hover_range = 0.5
        # Target depth line (dashed)
        self.target_depth_line = self.ax1.axhline(y=self.hover_depth, color='blue', linestyle='--', linewidth=2, alpha=0.7, label='Target Depth')
        # Upper and lower bounds (dotted)
        self.upper_bound_line = self.ax1.axhline(y=self.hover_depth - self.hover_range, color='blue', linestyle=':', alpha=0.5)
        self.lower_bound_line = self.ax1.axhline(y=self.hover_depth + self.hover_range, color='blue', linestyle=':', alpha=0.5)
        
        # Add a single legend for all lines
        lines = [self.depth_line, self.volume_line, self.target_depth_line]
        labels = [line.get_label() for line in lines]
        self.ax1.legend(lines, labels, loc='upper right')
        
        # Add tight layout to ensure proper spacing
        self.fig.tight_layout(rect=[0, 0, 1, 0.95])  # Leave space for the title

    def update_hover_lines(self):
        """Update the hover depth target lines based on control tab input"""
        try:
            # Get the target depth from the control tab input field
            new_depth = float(self.hover_depth_entry.get())
            if new_depth <= 0:
                messagebox.showerror("Invalid Depth", "Hover depth must be greater than 0")
                return
                
            self.hover_depth = new_depth
            
            # Update the lines
            self.target_depth_line.set_ydata([self.hover_depth, self.hover_depth])
            self.upper_bound_line.set_ydata([self.hover_depth - self.hover_range, self.hover_depth - self.hover_range])
            self.lower_bound_line.set_ydata([self.hover_depth + self.hover_range, self.hover_depth + self.hover_range])
            
            # Redraw the canvas
            self.canvas.draw()
            
            self.add_log("SYSTEM", f"Updated target hover depth to {self.hover_depth}m (range: {self.hover_depth - self.hover_range}m to {self.hover_depth + self.hover_range}m)")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for hover depth")


    #  Handle device name changesAdd
    def set_device_name(self):
        """Set the target device name"""
        name = self.device_name_entry.get().strip()
        if not name:
            messagebox.showerror("Error", "Device name cannot be empty")
            return
        
        self.ble.set_device_name(name)
        self.add_log("SYSTEM", f"Device name set to: {name}")
        messagebox.showinfo("Device Name Changed", 
                       f"Device name set to: {name}\n\nThe application will look for this device name on the next connection attempt.")

            
    def setup_controls(self):
        """Set up the controls interface"""
        # Create a frame for basic commands
        basic_cmd_frame = ttk.LabelFrame(self.controls_frame, text="Basic Commands", padding="10")
        basic_cmd_frame.pack(fill=tk.X, pady=5)
        
        # Add buttons for basic commands
        ttk.Button(basic_cmd_frame, text="Zero Calibrate", 
                command=lambda: self.send_command("zero")).pack(side=tk.LEFT, padx=5)
        ttk.Button(basic_cmd_frame, text="Reset Syringe", 
                command=lambda: self.send_command("reset")).pack(side=tk.LEFT, padx=5)
        ttk.Button(basic_cmd_frame, text="Get Status", 
                command=lambda: self.send_command("status")).pack(side=tk.LEFT, padx=5)
        ttk.Button(basic_cmd_frame, text="Get PID Parameters", 
                command=lambda: self.send_command("getpid")).pack(side=tk.LEFT, padx=5)
        
        # Create a frame for manual movement
        manual_frame = ttk.LabelFrame(self.controls_frame, text="Manual Control", padding="10")
        manual_frame.pack(fill=tk.X, pady=5)
        
        # Volume entry for forward/backward
        ttk.Label(manual_frame, text="Volume (mL):").grid(row=0, column=0, padx=5, pady=5)
        self.volume_entry = ttk.Entry(manual_frame, width=10)
        self.volume_entry.grid(row=0, column=1, padx=5, pady=5)
        self.volume_entry.insert(0, "10")
        
        # Buttons for forward/backward
        ttk.Button(manual_frame, text="Forward (Push Water Out)", 
                command=self.send_forward_command).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(manual_frame, text="Backward (Pull Water In)", 
                command=self.send_backward_command).grid(row=0, column=3, padx=5, pady=5)
        
        # Create a frame for dive commands
        dive_frame = ttk.LabelFrame(self.controls_frame, text="Dive Commands", padding="10")
        dive_frame.pack(fill=tk.X, pady=5)
        
        # Simple dive command
        ttk.Label(dive_frame, text="Hold Duration (s):").grid(row=0, column=0, padx=5, pady=5)
        self.dive_duration_entry = ttk.Entry(dive_frame, width=10)
        self.dive_duration_entry.grid(row=0, column=1, padx=5, pady=5)
        self.dive_duration_entry.insert(0, "30")
        
        ttk.Button(dive_frame, text="Start Dive", 
                command=self.send_dive_command).grid(row=0, column=2, padx=5, pady=5)
        
        # Hover command
        ttk.Label(dive_frame, text="Target Depth (m):").grid(row=1, column=0, padx=5, pady=5)
        self.hover_depth_entry = ttk.Entry(dive_frame, width=10)
        self.hover_depth_entry.grid(row=1, column=1, padx=5, pady=5)
        self.hover_depth_entry.insert(0, "2.5")
        
        # Add a button to update graph reference lines
        ttk.Button(dive_frame, text="Update Graph Lines", 
                command=self.update_hover_lines).grid(row=1, column=2, padx=5, pady=5)
        
        ttk.Label(dive_frame, text="Timeout (s):").grid(row=2, column=0, padx=5, pady=5)
        self.hover_timeout_entry = ttk.Entry(dive_frame, width=10)
        self.hover_timeout_entry.grid(row=2, column=1, padx=5, pady=5)
        self.hover_timeout_entry.insert(0, "120")
        
        ttk.Label(dive_frame, text="Target Range Time (s):").grid(row=2, column=2, padx=5, pady=5)
        self.hover_range_time_entry = ttk.Entry(dive_frame, width=10)
        self.hover_range_time_entry.grid(row=2, column=3, padx=5, pady=5)
        self.hover_range_time_entry.insert(0, "45")
        
        ttk.Button(dive_frame, text="Start Hover Mission", 
                command=self.send_hover_command).grid(row=2, column=4, padx=5, pady=5)
        
        # Create a frame for custom commands
        custom_frame = ttk.LabelFrame(self.controls_frame, text="Custom Command", padding="10")
        custom_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(custom_frame, text="Command:").grid(row=0, column=0, padx=5, pady=5)
        self.custom_command_entry = ttk.Entry(custom_frame, width=50)
        self.custom_command_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(custom_frame, text="Send", 
                command=self.send_custom_command).grid(row=0, column=2, padx=5, pady=5)
        

    def setup_settings(self):
        """Set up the settings interface"""
        # Create a frame for connection settings
        connection_frame = ttk.LabelFrame(self.settings_frame, text="Connection Settings", padding="10")
        connection_frame.pack(fill=tk.X, pady=5)
        
        # Device name setting
        ttk.Label(connection_frame, text="Device Name:").grid(row=0, column=0, padx=5, pady=5)
        self.device_name_entry = ttk.Entry(connection_frame, width=20)
        self.device_name_entry.grid(row=0, column=1, padx=5, pady=5)
        self.device_name_entry.insert(0, "DemonDiver")
        
        ttk.Button(connection_frame, text="Set Device Name", 
                command=self.set_device_name).grid(row=0, column=2, padx=5, pady=5)
        
        # Create a frame for PID settings
        pid_frame = ttk.LabelFrame(self.settings_frame, text="PID Parameters", padding="10")
        pid_frame.pack(fill=tk.X, pady=5)

        
        # PID parameters
        ttk.Label(pid_frame, text="KP:").grid(row=0, column=0, padx=5, pady=5)
        self.kp_entry = ttk.Entry(pid_frame, width=10)
        self.kp_entry.grid(row=0, column=1, padx=5, pady=5)
        self.kp_entry.insert(0, "0.5")
        
        ttk.Label(pid_frame, text="KI:").grid(row=0, column=2, padx=5, pady=5)
        self.ki_entry = ttk.Entry(pid_frame, width=10)
        self.ki_entry.grid(row=0, column=3, padx=5, pady=5)
        self.ki_entry.insert(0, "0.1")
        
        ttk.Label(pid_frame, text="KD:").grid(row=0, column=4, padx=5, pady=5)
        self.kd_entry = ttk.Entry(pid_frame, width=10)
        self.kd_entry.grid(row=0, column=5, padx=5, pady=5)
        self.kd_entry.insert(0, "0.2")
        
        ttk.Label(pid_frame, text="Default Timeout (s):").grid(row=0, column=6, padx=5, pady=5)
        self.timeout_entry = ttk.Entry(pid_frame, width=10)
        self.timeout_entry.grid(row=0, column=7, padx=5, pady=5)
        self.timeout_entry.insert(0, "120")
        
        ttk.Button(pid_frame, text="Set PID Parameters", 
                  command=self.send_pid_command).grid(row=0, column=8, padx=5, pady=5)
        
        # Create a frame for calibration
        calibration_frame = ttk.LabelFrame(self.settings_frame, text="Calibration", padding="10")
        calibration_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(calibration_frame, text="Number of Turns:").grid(row=0, column=0, padx=5, pady=5)
        self.cal_turns_entry = ttk.Entry(calibration_frame, width=10)
        self.cal_turns_entry.grid(row=0, column=1, padx=5, pady=5)
        self.cal_turns_entry.insert(0, "5")
        
        ttk.Button(calibration_frame, text="Calibrate Timing", 
                  command=self.send_calibrate_command).grid(row=0, column=2, padx=5, pady=5)
        
        # Create a frame for system settings
        system_frame = ttk.LabelFrame(self.settings_frame, text="System Settings", padding="10")
        system_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(system_frame, text="Set Time (HH:MM:SS):").grid(row=0, column=0, padx=5, pady=5)
        self.time_entry = ttk.Entry(system_frame, width=10)
        self.time_entry.grid(row=0, column=1, padx=5, pady=5)
        current_time = time.strftime("%H:%M:%S", time.localtime())
        self.time_entry.insert(0, current_time)
        
        ttk.Button(system_frame, text="Set Device Time", 
                  command=self.send_time_command).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(system_frame, text="Company Number:").grid(row=0, column=3, padx=5, pady=5)
        self.company_entry = ttk.Entry(system_frame, width=5)
        self.company_entry.grid(row=0, column=4, padx=5, pady=5)
        self.company_entry.insert(0, "2")
        
        ttk.Button(system_frame, text="Set Company", 
                  command=self.send_company_command).grid(row=0, column=5, padx=5, pady=5)
        
        # Create a frame for graph settings
        graph_frame = ttk.LabelFrame(self.settings_frame, text="Graph Settings", padding="10")
        graph_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(graph_frame, text="History Length (s):").grid(row=0, column=0, padx=5, pady=5)
        self.history_entry = ttk.Entry(graph_frame, width=10)
        self.history_entry.grid(row=0, column=1, padx=5, pady=5)
        self.history_entry.insert(0, str(MAX_HISTORY))
        
        ttk.Button(graph_frame, text="Apply", 
                  command=self.apply_graph_settings).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Button(graph_frame, text="Clear Data", 
                  command=self.clear_graph_data).grid(row=0, column=3, padx=5, pady=5)
    
    def setup_logs(self):
        """Set up the logs interface"""
        # Create a text widget for logs
        self.log_text = scrolledtext.ScrolledText(self.logs_frame, height=30)
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create a frame for buttons
        button_frame = ttk.Frame(self.logs_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="Clear Logs", 
                  command=self.clear_logs).pack(side=tk.RIGHT, padx=5)
        
        # Configure tags for different message types
        self.log_text.tag_configure("ERROR", foreground="red")
        self.log_text.tag_configure("WARNING", foreground="orange")
        self.log_text.tag_configure("INFO", foreground="blue")
        self.log_text.tag_configure("MESSAGE", foreground="green")
        self.log_text.tag_configure("SYSTEM", foreground="purple")
    
    def run_event_loop(self):
        """Run the asyncio event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        
        # Instead of directly running start_scan, we'll create a task for it
        # This allows us to cancel and restart it later
        self.scan_task = None
        self.start_connection_task()
        
        # Run the event loop indefinitely
        self.loop.run_forever()

    def start_connection_task(self):
        """Start or restart the connection task"""
        # Cancel existing task if any
        if self.scan_task:
            self.scan_task.cancel()
            
        # Create a new task
        self.scan_task = self.loop.create_task(self.ble.start_scan())
    
    def toggle_connect(self):
        """Connect or disconnect from the device with consistent threading"""
        if self.ble.connected or self.ble.connecting:
            # Disconnect - stop any reconnection and set manual disconnect flag
            self.ble.should_reconnect = False
            self.ble.manual_disconnect = True
            self.ble.disconnect()
            self.connect_button.config(text="Connect")
            self.status_label.config(text="Disconnected")
            self.add_log("SYSTEM", "Manual disconnect requested")
        else:
            # Connect - using the same method consistently
            self.ble.should_reconnect = True
            self.ble.manual_disconnect = False  # Clear manual disconnect flag
            
            # Use the loop thread to restart the connection task
            self.loop.call_soon_threadsafe(self.start_connection_task)
            
            self.connect_button.config(text="Disconnect")
            self.status_label.config(text="Connecting...")
            self.add_log("SYSTEM", "Connecting to device...")

    
    def send_command(self, command):
        """Send a command to the device"""
        if not self.ble.connected:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        future = asyncio.run_coroutine_threadsafe(self.ble.send_command(command), self.loop)
        try:
            result = future.result(1)  # Wait up to 1 second for the result
            if result:
                self.add_log("SYSTEM", f"Sent command: {command}")
        except Exception as e:
            self.add_log("ERROR", f"Failed to send command: {str(e)}")
    
    def send_forward_command(self):
        """Send forward command with volume from entry field"""
        try:
            volume = float(self.volume_entry.get())
            if volume <= 0:
                messagebox.showerror("Error", "Volume must be greater than 0")
                return
            self.send_command(f"forward {volume}")
        except ValueError:
            messagebox.showerror("Error", "Invalid volume value")
    
    def send_backward_command(self):
        """Send backward command with volume from entry field"""
        try:
            volume = float(self.volume_entry.get())
            if volume <= 0:
                messagebox.showerror("Error", "Volume must be greater than 0")
                return
            self.send_command(f"backward {volume}")
        except ValueError:
            messagebox.showerror("Error", "Invalid volume value")
    
    def send_dive_command(self):
        """Send dive command with duration from entry field"""
        try:
            duration = int(self.dive_duration_entry.get())
            if duration <= 0:
                messagebox.showerror("Error", "Duration must be greater than 0")
                return
            self.send_command(f"dive {duration}")
        except ValueError:
            messagebox.showerror("Error", "Invalid duration value")
    
    def send_hover_command(self):
        """Send hover command with parameters from entry fields"""
        try:
            depth = float(self.hover_depth_entry.get())
            timeout = int(self.hover_timeout_entry.get())
            range_time = int(self.hover_range_time_entry.get())
            
            if depth <= 0:
                messagebox.showerror("Error", "Depth must be greater than 0")
                return
            if timeout <= 0:
                messagebox.showerror("Error", "Timeout must be greater than 0")
                return
            if range_time <= 0:
                messagebox.showerror("Error", "Target range time must be greater than 0")
                return
            
            # First update the graph lines to match the hover depth
            self.update_hover_lines()
                
            # Then send the hover command
            self.send_command(f"hover {depth} {timeout} {range_time}")
        except ValueError:
            messagebox.showerror("Error", "Invalid parameter values")
    
    def send_pid_command(self):
        """Send PID parameters command"""
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            timeout = int(self.timeout_entry.get())
            
            self.send_command(f"pid {kp} {ki} {kd} {timeout}")
        except ValueError:
            messagebox.showerror("Error", "Invalid PID parameter values")
    
    def send_calibrate_command(self):
        """Send calibration command"""
        try:
            turns = int(self.cal_turns_entry.get())
            if turns <= 0:
                messagebox.showerror("Error", "Number of turns must be greater than 0")
                return
                
            if messagebox.askyesno("Confirm Calibration", 
                                  f"Start calibration with {turns} turns? This may take some time."):
                self.send_command(f"calibrate {turns}")
        except ValueError:
            messagebox.showerror("Error", "Invalid number of turns")
    
    def send_time_command(self):
        """Send time setting command"""
        time_str = self.time_entry.get()
        # Verify time format HH:MM:SS
        if not re.match(r'^\d{1,2}:\d{1,2}:\d{1,2}$', time_str):
         messagebox.showerror("Error", "Invalid time format. Use HH:MM:SS")
        return
        
        self.send_command(f"time {time_str}")
    
    def send_company_command(self):
        """Send company setting command"""
        try:
            company = int(self.company_entry.get())
            if company < 0:
                messagebox.showerror("Error", "Company number must be a positive integer")
                return
                
            self.send_command(f"company {company}")
        except ValueError:
            messagebox.showerror("Error", "Invalid company number")
    
    def send_custom_command(self):
        """Send custom command from entry field"""
        command = self.custom_command_entry.get().strip()
        if not command:
            messagebox.showerror("Error", "Command cannot be empty")
            return
            
        self.send_command(command)
    
    def apply_graph_settings(self):
        """Apply graph settings"""
        try:
            new_history = int(self.history_entry.get())
            if new_history <= 0:
                messagebox.showerror("Error", "History length must be greater than 0")
                return
                
            self.data_manager.max_history = new_history
            self.add_log("SYSTEM", f"Graph history length set to {new_history} seconds")
        except ValueError:
            messagebox.showerror("Error", "Invalid history length")
    
    def clear_graph_data(self):
        """Clear all graph data"""
        if messagebox.askyesno("Confirm Clear", "Clear all graph data?"):
            self.data_manager.reset()
            self.update_plots()
            self.add_log("SYSTEM", "Graph data cleared")
    
    def clear_logs(self):
        """Clear the log window"""
        self.log_text.delete(1.0, tk.END)
        self.add_log("SYSTEM", "Logs cleared")
    
    def add_log(self, level, message):
        """Add a message to the log window"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_message, level)
        self.log_text.see(tk.END)  # Auto-scroll to the end
    
    def on_data_received(self, data):
        """Handle received data from the device"""
        # Add data to the data manager
        self.data_manager.add_data_point(data)
            
        # Update current value displays
        self.current_depth_label.config(text=f"{data['depth_m']:.2f} m")
        self.current_volume_label.config(text=f"{data['volume_ml']:.1f} mL")
        self.current_pressure_label.config(text=f"{data['pressure_kpa']:.2f} kPa")
        self.current_voltage_label.config(text=f"{data['millivolts']:.1f} mV")
    
        # Add debug log occasionally (not for every data point to avoid flooding)
        # Use data timestamp as a simple way to throttle log messages
        timestamp = data['timestamp']
        if int(timestamp) % 10 == 0:  # Only log every ~10 seconds
            self.add_log("DATA", f"Depth={data['depth_m']:.2f}m, Volume={data['volume_ml']:.1f}mL")

    def on_connection_status(self, status):
        """Handle connection status updates - simple approach"""
        self.status_label.config(text=status)
        
        # Update button based on status
        if "Connected" in status:
            self.connect_button.config(text="Disconnect")
        elif "Disconnected" in status:
            self.connect_button.config(text="Connect")
        
        # Log the status
        self.add_log("SYSTEM", status)
    
    def on_message_received(self, level, message):
        """Handle received messages from the device"""
        self.add_log(level, message)
    
    
    # Update the update_ui method to add debug feedback
    def update_ui(self):
        """Periodic UI update function"""
        # Update plots if there's data
        if self.data_manager.timestamps:
            try:
                self.update_plots()
            except Exception as e:
                self.add_log("ERROR", f"Error updating plots: {str(e)}")
    
    # Schedule the next update
        self.root.after(500, self.update_ui)
    
    def on_close(self):
        """Handle window close event"""
        # Stop the BLE controller
        self.ble.disconnect()
        
        # Stop the event loop
        self.loop.call_soon_threadsafe(self.loop.stop)
        
        # Close the window
        self.root.destroy()

    def update_plots(self):
        """Update the plot data"""
        # Get relative times for x-axis
        times = self.data_manager.get_relative_times()
        
        if not times:
            return  # No data to plot
        
        # Update the depth plot (left axis)
        self.depth_line.set_data(times, self.data_manager.depths)
        
        # Update the volume plot (right axis)
        self.volume_line.set_data(times, self.data_manager.volumes)
        
        # Adjust x-axis limits to show all data
        x_min = min(times) if times else 0
        x_max = max(times) if times else 10
        x_padding = max(0.1, (x_max - x_min) * 0.05)  # 5% padding or at least 0.1
        self.ax1.set_xlim(x_min - x_padding, x_max + x_padding)
        
        # Adjust y-axis limits for depth plot (left axis)
        if self.data_manager.depths:
            y1_min = min(self.data_manager.depths)
            y1_max = max(self.data_manager.depths)
            
            # Ensure hover lines are visible by including them in the y-axis limits calculation
            hover_min = self.hover_depth - self.hover_range
            hover_max = self.hover_depth + self.hover_range
            
            # Take the min/max of data and hover lines
            y1_min = min(y1_min, hover_min - 0.5)  # Include some padding
            y1_max = max(y1_max, hover_max + 0.5)  # Include some padding
            
            y1_padding = max(0.1, (y1_max - y1_min) * 0.1)  # 10% padding or at least 0.1
            
            # Add a bit more padding for inverted axis
            self.ax1.set_ylim(
                y1_max + y1_padding,  # Top limit (remember axis is inverted)
                max(0, y1_min - y1_padding)  # Bottom limit, ensure we don't go below 0
            )
        
        # Adjust y-axis limits for volume plot (right axis)
        if self.data_manager.volumes:
            y2_min = min(self.data_manager.volumes)
            y2_max = max(self.data_manager.volumes)
            y2_padding = max(0.1, (y2_max - y2_min) * 0.1)  # 10% padding or at least 0.1
            
            # Ensure we never go below 0 for volume
            self.ax2.set_ylim(
                max(0, y2_min - y2_padding),
                y2_max + y2_padding
            )
        
        # Redraw the canvas
        self.canvas.draw()


    # Modify the update_ui method to include a periodic data check
    def update_ui(self):
        """Periodic UI update function"""
        # Update plots if there's data
        if self.data_manager.timestamps:
            try:
                self.update_plots()
            except Exception as e:
                self.add_log("ERROR", f"Error updating plots: {str(e)}")
        
        # Check for data reception timeouts
        if self.ble.connected:
            current_time = time.time()
            if not hasattr(self, 'last_data_time'):
                self.last_data_time = current_time
            elif self.data_manager.timestamps:
                # We have received data before, check if it's still coming
                last_system_time = self.data_manager.system_times[-1] if self.data_manager.system_times else 0
                if current_time - last_system_time > 10:  # No data for 10 seconds
                    # Only log this warning periodically
                    if current_time - self.last_data_time > 30:  # Log every 30 seconds
                        self.add_log("WARNING", "No data received for 10+ seconds while connected")
                        self.last_data_time = current_time
        
        # Schedule the next update
        self.root.after(500, self.update_ui)

def main():
    """Main function to start the application"""
    root = tk.Tk()
    app = DemonDiverGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()