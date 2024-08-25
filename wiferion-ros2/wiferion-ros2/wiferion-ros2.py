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
import queue
import threading
import rclpy
from rclpy.node import Node

class ChargerNode:
    def __init__(self, can_data):
        super().__init__('wiferion_ros2_node')
        self.declare_parameter('can_device', 'can0')
        self.can_device = self.get_parameter('can_device').value
        self.get_logger().info("Using can_device: %s" % self.can_device)

        self.declare_parameter('pc_address', 0x40)
        self.pc_address = self.get_parameter('pc_address').value
        self.get_logger().info("Using PC address: 0x{:x}".format(self.pc_address))

        self.declare_parameter('charger_address', 0x0a) #todo
        self.charger_address = self.get_parameter('charger_address').value
        self.get_logger().info("Using charger address: 0x{:x}".format(self.charger_address))

        self.default_priority = 0x18
        self.can_rx_queue = queue.Queue()

        self.can_bus = can.interface.Bus(self.can_device, bustype='socketcan')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Start a thread for receiving CAN messages
        self.can_recv_thread = threading.Thread(target=self.can_recv, daemon=True)
        self.can_recv_thread.start()

        self.can_process_thread = threading.Thread(target=self.consume_can_rx_queue, daemon=True)
        self.can_process_thread.start()

        self.queue_soc_of_total_voltage_current = queue.SimpleQueue()
        self.queue_charge_discharge_mos_status = queue.SimpleQueue()
        self.queue_status_information_i = queue.SimpleQueue()
        self.queue_battery_failure_status = queue.SimpleQueue()

    def can_recv(self):
        while rclpy.ok():
            self.can_rx_queue.put(self.can_bus.recv())


# wip
# reference link:
#   https://gitlab.clearpathrobotics.com/a300/valence_bmu/-/blob/main/valence_bmu/driver.py?ref_type=heads
