#!/usr/bin/env python3
import can


filters = [
    # note that the Wiferion manual lists addresses with 'XX'.
    # these addresses have been changed to '0B' to align with the Wiferon being configured for address 11. 
    {"can_id":0x18FF50E5 , "can_mask": 0xFFFFFFF, "extended": True},  # Table 12: MOB_STATUS_CHARGER (TX 1s)
    # {"can_id":0x1806E5F4 , "can_mask": 0xFFFFFFF, "extended": True},  # Table 13: BMS_CHARGER_CONTROL - Charging parameters (RX 1s)
    # {"can_id":0x1806E6F4 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 16: BMS_DISABLE_CHARGING – Disable the charging process (RX 1s)
    # {"can_id":0x1FFD0B01 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 18: MOB_SN (TX 1s)
    # {"can_id":0x1FFD0B02 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 19: MOB_TEMP (TX 1s)
    # {"can_id":0x1FFD0B03 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 20: MOB_TEMP_2 (TX 1s)
    # {"can_id":0x1FFD0B04 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 21: MOB_ERROR – Error and warning bits. (TX 0.1s)
    # {"can_id":0x1FFD0B06 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 22: MOB_STAT_SN (TX 1s)
    # {"can_id":0x1FFD0B07 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 23: MOB_SW – Software version of the mobile electronics (TX 1s)
    # {"can_id":0x1FFD0B09 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 24: MOB_CONFIG (TX 1s)
    # {"can_id":0x1FFD0B10 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 25: MOB_STAT_STATUS (TX 1s)
    # {"can_id":0x1FFD0B11 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 26: MOB_STAT_SW – Software version of the stationary electronics (TX 1s)
    # {"can_id":0x1FFD0B13 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 27: MOB_STAT_TEMP (TX 1s)
    # {"can_id":0x1FFD0B14 , "can_mask": 0xFFFFFFF, "extended": True}, # Table 28: MOB_STAT_TEMP_2 (TX 1s)
]

# can_bus = can.ThreadSafeBus(interface='socketcan', channel='can0', can_filters=filters) # with kernel filtering
can_bus = can.ThreadSafeBus(interface='socketcan', channel='can0') # no kernel filtering

while True:
    received_message = can_bus.recv()
    received_message_ID = (((received_message.__str__()).split('ID: ', 1)[1])[0:8])
    # print(received_message_ID)
    if (received_message_ID == '18ff50e5'):
        print (received_message)
        print (received_message.data)

can_bus.shutdown()



# for msg in can_bus.recv():
#     print(msg.data)

# note that 25.5 V = \x00\xff
#   ff = 255
#   maybe this means 25.5 V
# 25.6 V = \x01\x00
#   so 25.5 + 0.1
# 28.8 = \x01\x1f
# 28.9 and greater causes a weird result. I assume this is related to an error state / overvoltage 