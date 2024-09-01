# wiferion-ros2

## setup Wiferion etaLink 1000
- Using the BlueConfig application
  - Change the default CAN address to 11.
  - Change the default CAN bit rate from 500 kB to 250 kB.

## hardware
* Wiferion etaLink 1000 mobile unit, powered with a 20 - 29 V supply _( instead of a 24 V LFP battery )_.
* PEAK USB-CAN adapter.
* 2X terminating resistors at 120 Ω, M-F DE-9.
* Custom adapter cable, from an M12-5-A connctor, to a DE-9 connector

## setup socketcan in Ubuntu
```
sudo ip link set can0 up type can bitrate 250000
sudo ip link set can0 up
```

Then confirm that the interface is working by using canutils:
```
candump can0
```

You should receive data if the Wiferion is on, and the CAN network is configured correctly.
Here is some example output data received from the intial testing:

```bash
can0  1FFD0B00   [8]  03 00 32 4F 00 36 2D 3F
can0  1FFD0B0F   [8]  00 00 00 00 00 28 FF D9
can0  1FFD0B0C   [8]  00 13 00 2C 76 33 00 17
can0  1FFD0B05   [8]  B1 88 00 00 00 00 00 00
can0  1FFD0B04   [8]  00 00 01 00 00 00 00 00
can0  1FFD0B00   [8]  03 00 32 4E 00 35 2D 3F
can0  1FFD0B0F   [8]  00 00 00 00 00 28 FF D9
can0  1FFD0B0C   [8]  00 13 00 2C 76 33 00 16
can0  1FFD0B05   [8]  B2 08 00 00 00 00 00 00
can0  1FFD0B04   [8]  00 00 01 00 00 00 00 00
can0  1FFD0B00   [8]  03 00 32 4E 00 35 2D 40
can0  1FFD0B00   [8]  03 00 32 4F 00 36 2D 3F
can0  1FFD0B0F   [8]  00 00 00 00 00 28 FF D9
can0  1FFD0B0C   [8]  00 13 00 2C 76 33 00 17
can0  1FFD0B05   [8]  B1 88 00 00 00 00 00 00
can0  1FFD0B04   [8]  00 00 01 00 00 00 00 00
can0  1FFD0B00   [8]  03 00 32 4E 00 35 2D 3F
can0  1FFD0B0F   [8]  00 00 00 00 00 28 FF D9
Can0  1FFD0B0C   [8]  00 13 00 2C 76 33 00 16
can0  1FFD0B05   [8]  B2 08 00 00 00 00 00 00
can0  1FFD0B04   [8]  00 00 01 00 00 00 00 00
can0  1FFD0B00   [8]  03 00 32 4E 00 35 2D 40
```


## CAN filtering in Linux kernel

The python-can library has a filtering function.
This allows you to define what CAN messages are allowed to pass from the kernel level into Python.
This enable-list should improve performance, as the Python program only sees messages related to the Wiferion wireless charger.

```python
filters = [
    {"can_id":0x8FF50E5 , "can_mask": 0xFFFFFFF, "extended": True},
]
```

## References

- https://github.com/hardbyte/python-can/blob/main/examples/vcan_filtered.py
- https://stackoverflow.com/questions/66484457/python-can-j1939-filter-mask
- https://python-can.readthedocs.io/en/stable/bus.html#can.BusABC.set_filters