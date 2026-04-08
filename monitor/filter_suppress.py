from platformio.public import DeviceMonitorFilterBase


class SuppressTerminal(DeviceMonitorFilterBase):
    NAME = "suppress"

    def rx(self, text):
        return ""  # eat all device output — log2file already saved it

    def tx(self, text):
        return text  # still pass keyboard input through to device
