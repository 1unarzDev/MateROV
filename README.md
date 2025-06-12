# FLOAT

I'm honestly being a bit lazy here by not explaining the process for Windows, but you pretty much only need to do the following:
- Replace pip3 with pip
- Ensure you have a valid python installation with pip installed and environment variables added (little checkmark in Python install)
- Search for the device port in your device manager and serial devices (likely COM4)  

### Tutorial

1. Install needed tools
- Text editor such as [VSCode](https://code.visualstudio.com/download)
- Chip images [download link](https://micropython.org/download/ESP32_GENERIC_C3/)
- ESP tool for flashing and Adafruit-ampy for uploads
```bash
pip3 install esptool adafruit-ampy
```

2. Find the chip port (search through recent connects and port names)
```bash
sudo dmesg | grep -i usb | tail -20
```

3. Flash the chip
```bash
esptool.py --chip esp32c3 --port <serial_port> erase_flash # Can leave out port if unknown and it might find it for you
esptool.py --port <serial_port> write_flash 0x0 <chip_image_directory>
```

4. Upload and run your code with serial output
```bash
ampy --port <serial_port> put <script_name>.py
ampy --port <serial_port> run <script_name>.py
```