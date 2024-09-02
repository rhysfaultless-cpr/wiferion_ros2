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
    try:
        if (received_message_ID == '18ff50e5'):
            charge_voltage_integer = (received_message.data[0]*256) + (received_message.data[1])
            charge_voltage_float_whole = int(str(charge_voltage_integer)[0:2])
            charge_voltage_float_fractional = int(str(charge_voltage_integer)[2])
            charge_voltage_float = charge_voltage_float_whole + charge_voltage_float_fractional/10
            # charge_voltage_float


            charge_current_integer = (received_message.data[2]*256) + (received_message.data[3])
            #print(charge_current_integer)
            if (charge_current_integer < 10):
                charge_current_float_whole = int(('00'+str(charge_current_integer))[0:2])
                charge_current_float_fractional = int(('00'+str(charge_current_integer))[2])
            elif (charge_current_integer < 100):
                charge_current_float_whole = int(('0'+str(charge_current_integer))[0:2])
                charge_current_float_fractional = int(('0'+str(charge_current_integer))[2])
            else:
                charge_current_float_whole = int(str(charge_current_integer)[0:2])
                charge_current_float_fractional = int(str(charge_current_integer)[2])
            charge_current_float = charge_current_float_whole + charge_current_float_fractional/10
            # charge_current_float
            # note that this has not been tested
            # note that there may be issues related to signed, as the charger reports a magnitude, redardless if it is charging, or consuming.
            # this program may need some logic to note the charger's status, and then specify a current vector, + / -
    except:
        pass

can_bus.shutdown()
