import canopen
import can
import time
import _thread
  

network = canopen.Network()
network.connect(channel=0, bustype='canalystii', bitrate=500000)

# 抓取
data1 = [0x00, 0x6F, 0x00, 0x00, 0x3F, 0x2F, 0x00, 0x00]
# 放置
data2 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

def control(): 
    for i in range(5):
        message = can.Message(arbitration_id=0x100, data=data1, is_extended_id=False)
        time.sleep(0.1)
        network.bus.send(message)
        time.sleep(0.1)
        network.bus.send(message)
        time.sleep(1)
        message = can.Message(arbitration_id=0x100, data=data2, is_extended_id=False)
        network.bus.send(message)        
        time.sleep(1)

_thread.start_new_thread(control, ())

while True:
    time.sleep(0.2)
    data = network.bus.recv(1)
    print(data)


