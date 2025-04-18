import canopen
import can
import time
import _thread
  

network = canopen.Network()
network.connect(channel=0, bustype='canalystii', bitrate=500000)


dataGripper = [0x00, 0x6F, 0x00, 0x00, 0x3F, 0x2F, 0x00, 0x00]
dataOpen = [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
dataOff = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

message0 = can.Message(arbitration_id=0x100, data=dataGripper, is_extended_id=False)
message1 = can.Message(arbitration_id=0x100, data=dataOff, is_extended_id=False)

message2 = can.Message(arbitration_id=0x101, data=dataOpen, is_extended_id=False)
message3 = can.Message(arbitration_id=0x101, data=dataOff, is_extended_id=False)


while True:
    # network.bus.send(message0)  
    # time.sleep(0.5)
    network.bus.send(message2)
    time.sleep(3)
    # network.bus.send(message1)
    # time.sleep(0.5)
    network.bus.send(message3)
    time.sleep(3)
    
    


