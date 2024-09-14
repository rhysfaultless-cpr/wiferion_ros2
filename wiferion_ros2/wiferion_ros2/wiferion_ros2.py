#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Rhys Faultless <rfaultless@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class WiferionCharger(Node):
    def __init__(self):
        super().__init__('WiferionCharger')
        self.declare_parameter('can_device', 'can0')
        self.can_device = self.get_parameter('can_device').value

        self.can_bus = can.ThreadSafeBus(interface='socketcan', channel='can0')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.charger_publisher_voltage = None
        self.charger_publisher_voltage = self.create_publisher(Float32, 'wiferion_charger_voltage', 10)
        self.charger_publisher_current = None
        self.charger_publisher_current = self.create_publisher(Float32, 'wiferion_charger_current', 10)
        self.charger_publisher_status = None
        self.charger_publisher_status = self.create_publisher(String, 'wiferion_charger_status', 10)
        

    def timer_callback(self):
        try:
            while True:
                # while loop, to wait for a CAN message that meets the message ID '18ff50e5'
                received_message = self.can_bus.recv()
                received_message_ID = (((received_message.__str__()).split('ID: ', 1)[1])[0:8])

                if (received_message_ID == '18ff50e5'):
                    # voltage reading and conversion
                    charge_voltage_integer = (received_message.data[0]*256) + (received_message.data[1])
                    charge_voltage_float_whole = int(str(charge_voltage_integer)[0:2])
                    charge_voltage_float_fractional = int(str(charge_voltage_integer)[2])
                    charge_voltage_float = charge_voltage_float_whole + charge_voltage_float_fractional/10
                    
                    # voltage publsihing ROS
                    msg_voltage = Float32()
                    msg_voltage.data = charge_voltage_float
                    self.charger_publisher_voltage.publish(msg_voltage)

                    # current reading and conversion
                    charge_current_integer = (received_message.data[2]*256) + (received_message.data[3])
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
                    
                    # current publsihing ROS
                    msg_current = Float32()
                    msg_current.data = charge_current_float
                    self.charger_publisher_current.publish(msg_current)
                    
                    # status reading and conversion
                    # charge_status_string values are per the Wiferion manual
                    charge_status_integer = received_message.data[7]
                    if (charge_status_integer == 0):
                        charge_status_string = 'Idle'
                    elif (charge_status_integer == 1):
                        charge_status_string = 'Charging'
                    elif (charge_status_integer == 2):
                        charge_status_string = 'Charging constant current'
                    elif (charge_status_integer == 3):
                        charge_status_string = 'Charging constant voltage'
                    elif (charge_status_integer == 4):
                        charge_status_string = 'Charging pre-charge'
                    elif (charge_status_integer == 5):
                        charge_status_string = 'Charging second constant voltage'
                    elif (charge_status_integer == 6):
                        charge_status_string = 'Battery full'
                    elif (charge_status_integer == 7):
                        charge_status_string = 'Stopped'
                    else:
                        charge_status_string = 'Error'

                    # status publishing ROS
                    msg_status = String()
                    msg_status.data = charge_status_string
                    self.charger_publisher_status.publish(msg_status)

                    # stop looping, as a relevant CAN message was received
                    break

                else:
                    # pass, so the while loop can continue searching for a relevant CAN message
                    pass
        except:
            pass
    
    def shutdown(self):
        # Close the CAN bus when the node is shutting down
        self.can_send_bus.shutdown()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    wiferion_charger = WiferionCharger()
    try:
        rclpy.spin(wiferion_charger)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node
        wiferion_charger.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
