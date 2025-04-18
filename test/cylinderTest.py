import canopen
import can
import time
import _thread
  

network = canopen.Network()
network.connect(channel=0, bustype='canalystii', bitrate=500000)

data1 = [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
data2 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# bus = can.interface.Bus(channel=0, bustype='canalystii', bitrate=500000)
def control(): 
    for i in range(10):
        message = can.Message(arbitration_id=0x101, data=data1, is_extended_id=False)
        network.bus.send(message)
        time.sleep(3)
        # message = can.Message(arbitration_id=0x201, data=data2, is_extended_id=False)
        # network.bus.send(message)
        # time.sleep(1)
        message = can.Message(arbitration_id=0x101, data=data2, is_extended_id=False)
        network.bus.send(message)        
        time.sleep(3)
        # message = can.Message(arbitration_id=0x201, data=data2, is_extended_id=False)
        # network.bus.send(message)
        # time.sleep(1)
#     print(msg)
_thread.start_new_thread(control, ())


while True:
    time.sleep(1)
    # message = can.Message(arbitration_id=0x101, data=data1, is_extended_id=False)
    # network.bus.send(message)
    # data = network.bus.recv(1)
    # print(data)


