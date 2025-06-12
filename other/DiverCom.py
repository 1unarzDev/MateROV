import asyncio
from queue import Queue
from bleak import BleakClient, BleakScanner

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

def NoOp(*arg):
    return


class DiverCom():
    def __init__(self, name):
        self.diverName = name
        self.queue = Queue()
        self.onConnect = NoOp
        self.onDisconnect = NoOp
        self.onReceive = NoOp
        self.rx_char = None
        self.client = None
        self.device = None
        self.connected = False

    def handle_disconnect(self, _: BleakClient):
        self.onDisconnect()
        self.connected = False

    def handle_rx(self, BleakGATTCharacteristic, data: bytearray):
        message = data.decode('utf-8')
        print("Received: " + message)
        self.onReceive(message)

    def send(self, string_utf8):
        self.queue.put(string_utf8)

    async def send_message(self):
        if not self.queue.empty():
            string_utf8 = self.queue.get()
            s = bytearray(string_utf8, "utf-8")
            await self.client.write_gatt_char(self.rx_char, s, response=True)
        else:
            await asyncio.sleep(0.5)  #queue empty wait to try again.

    async def run(self):
        while True:
            try:
                try:
                    self.device = await BleakScanner.find_device_by_name(self.diverName)
                except Exception as exp:
                    print(exp)
                    break
                
                if self.device:
                    async with BleakClient(self.device, disconnected_callback=self.handle_disconnect) as self.client:
                        await self.client.start_notify(UART_TX_CHAR_UUID, self.handle_rx)
                        self.connected = True
                        self.onConnect()
                        
                        nus = self.client.services.get_service(UART_SERVICE_UUID)
                        self.rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)
                        
                        while self.connected:
                            await self.send_message()

            except Exception as exp:
                print(exp)
              #  pass

'''
def Main():
    diver = DiverCom("DemonDiver")
    asyncio.run(diver.run())
                      
Main()
'''