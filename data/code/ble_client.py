import asyncio
import time
import struct
from bleak import BleakScanner,BleakClient
from ctypes import *
from mems import *

BUFFER_SIZE = 5        #5s
STEP_SIZE = 1          #1s
FRAME_RATE = 30        #30 frames per second
ble_client = None
ble_device = None
last_timestamp = MEMSDataProcess.instance().get_timestamp()
cur_timestamp = last_timestamp
frames_cnt = 0


def data_process(data):
    """
    sorted_data = sorted(data)
    Q1=np.quantile(sorted_data,0.25,interpolation='lower')#下四分位数
    Q3=np.quantile(sorted_data,0.75,interpolation='higher')#上四分位数
    IQR=Q3-Q1#四分位距
    upper_bound = Q3 + 1.5 * IQR
    lower_bound = Q1 - 1.5 * IQR
    for index, iter in enumerate(data):
        if iter > upper_bound:
            data[index] = upper_bound
        if iter < lower_bound:
            data[index] = lower_bound
    """
    #data = MEMSDataProcess.instance().mean_align(data)
    data = MEMSDataProcess.instance().med_filter(data, 25)
    #data = MEMSDataProcess.instance().band_pass_filter(data, 25, 8, 10, 3)
    #data = MEMSDataProcess.instance().low_pass_filter(data, 25, 0.8, 5)
    #data = MEMSDataProcess.instance().sg_filter(data, 51, 4)
    return data

mems_buffer = MEMSDataBuffer(MEMSDataType.MEMS_VECTOR, 20, 1, 25, data_process, True)

async def scan():
    global ble_device
    devices = await BleakScanner.discover()
    for device in devices:
        if "Nordic_Glass" in str(device):
            ble_device = device
            break
    if ble_device == None:
        print("didn't find the device")
        exit(0)
    else:
        print("find the device")
        print(ble_device, ble_device.rssi)

async def run():
    global ble_client
    ble_client = BleakClient(ble_device)
    connect_result = await ble_client.connect()
    if connect_result == False:
        print("connect fail")
        exit(0)
    else:
        print("connect success")
        
    #character = await ble_client.read_gatt_char("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
    services = await ble_client.get_services()
    _ = await ble_client.start_notify(services.characteristics[10], notify_handler)
    while True:
        await asyncio.sleep(1)

#An easy notify function, just print the recieve data
def notify_handler(sender, data):
    global frames_cnt
    global cur_timestamp
    global last_timestamp
    data_x, data_y, data_z, frame_end = struct.unpack('>hhhb', data)
    if frame_end == 0:
        frames_cnt += 1
        cur_timestamp = MEMSDataProcess.instance().get_timestamp()
        if cur_timestamp - last_timestamp >= 1000:
            freq = frames_cnt * 1000.0 / (cur_timestamp - last_timestamp)
            last_timestamp = cur_timestamp
            frames_cnt = 0
            print("freq:" + str(freq) + "Hz")
        #print(data_x, data_y, data_z)
        mems_buffer.append(mems_buffer.calculate_vector_sum(data_x, data_y, data_z))
    else:
        print("frame crash")


if __name__ == "__main__":
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(scan())

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())