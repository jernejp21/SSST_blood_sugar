import asyncio
import struct
from bleak import BleakScanner
from bleak import BleakClient
from bleak import uuids
import matplotlib.pyplot as plt
import numpy as np

MODEL_DIS_UUID = "00002a24-0000-1000-8000-00805f9b34fb"
MANUF_DIS_UUID = "00002a29-0000-1000-8000-00805f9b34fb"
FW_DIS_UUID = "00002a26-0000-1000-8000-00805f9b34fb"
SSST_UUID = "0179bbd1-5351-48b5-bf6d-2167639bc867"

start_command = bytearray([0xAA, 0x11])
stop_command = bytearray([0xAA, 0x12])

whole_data = np.array([])
model_number = ""

def notification_handler(sender, data):
    global whole_data

    if(len(data)>2):
        for i in range(2, len(data)+2, 2):
            temp, = struct.unpack('>H', data[i-2:i])
            whole_data = np.append(whole_data, temp)

async def main():
    global model_number
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name is not None:
            if "Vital" in d.name:
                print(d.name)
                address = d.address

                client = BleakClient(address)
                try:
                    await client.connect()
                    model_number = await client.read_gatt_char(MANUF_DIS_UUID)
                    print("Model Number: {0}".format("".join(map(chr, model_number))))
                    fw_number = await client.read_gatt_char(FW_DIS_UUID)
                    print("FW Number: {0}".format("".join(map(chr, fw_number))))
                    svcs = await client.get_services()
                    
                    await client.start_notify(SSST_UUID, notification_handler)
                    await client.write_gatt_char(SSST_UUID, start_command)

                    await asyncio.sleep(3.0)

                    await client.write_gatt_char(SSST_UUID, stop_command)
                    await client.stop_notify(SSST_UUID)

                    print("Services:")
                    for service in svcs:
                        print(service)
                except Exception as e:
                    print(e)
                finally:
                    await client.disconnect()

asyncio.run(main())

model_number = model_number.decode()
freq = int(model_number.split(" ")[1]) # "Azurtest 100 Hz".split(" ") -> ['Azurtest', '100', 'Hz']
t = np.linspace(0, len(whole_data)/freq, len(whole_data))

plt.plot(t, whole_data)
plt.show()