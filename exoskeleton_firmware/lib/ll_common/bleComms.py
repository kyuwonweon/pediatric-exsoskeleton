# This scrip handles BLE communications between the pyboard and a mobile phone application made in Unity 3D
#
#
#
# Author: Levi Hargrove
# Jan 20, 2020
#

import time

import bluetooth                            # type: ignore
import micropython                          # type: ignore

from . import ble_advertising

print('IMPORTING BLECOMMS')

# from ble_advertising import advertising_payload

_IRQ_CENTRAL_CONNECT = micropython.const(1)
_IRQ_CENTRAL_DISCONNECT = micropython.const(2)
_IRQ_GATTS_WRITE = micropython.const(3)
_IRQ_GATTS_READ_REQUEST = micropython.const(4)
_IRQ_SCAN_RESULT = micropython.const(5)
_IRQ_SCAN_DONE = micropython.const(6)
_IRQ_PERIPHERAL_CONNECT = micropython.const(7)
_IRQ_PERIPHERAL_DISCONNECT = micropython.const(8)
_IRQ_GATTC_SERVICE_RESULT = micropython.const(9)
_IRQ_GATTC_SERVICE_DONE = micropython.const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = micropython.const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = micropython.const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = micropython.const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = micropython.const(14)
_IRQ_GATTC_READ_RESULT = micropython.const(15)
_IRQ_GATTC_READ_DONE = micropython.const(16)
_IRQ_GATTC_WRITE_DONE = micropython.const(17)
_IRQ_GATTC_NOTIFY = micropython.const(18)
_IRQ_GATTC_INDICATE = micropython.const(19)


# code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]

    def new_func(*args, **kwargs):
        t = time.ticks_us()
        result = f(*args, **kwargs)
        print('execution time: ', time.ticks_diff(time.ticks_us(), t), ' us')
        return result
    return new_func


_serviceUUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_uartReadCharacteristicUUID  = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_uartWriteCharacteristicUUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

_READ_CHAR = (_uartReadCharacteristicUUID, bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY,)
_WRITE_CHAR = (_uartWriteCharacteristicUUID, bluetooth.FLAG_WRITE | bluetooth.FLAG_NOTIFY,)
_CONTROLLER_SERVICE = (_serviceUUID, (_READ_CHAR, _WRITE_CHAR),)


class BLEComms:
    # Initialization function. Set up variables, register callbacks, etc.

    def __init__(self, ble, name='BIONIC_LEG'):
        self._ble = ble
        self._conn_handle = None
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._txhandle, self._rxhandle),) = self._ble.gatts_register_services((_CONTROLLER_SERVICE,))
        self._connections = set()
        self._payload = ble_advertising.advertising_payload(name=name, services=[_serviceUUID])
        self._name = name
        print('Bluetooth Name:                      ', self._name)
        self._adv()
        self._connected = False
        self._new_message = False
        self._message = bytearray(32)
        self._send_controller_status = False
        self._exceptions = 0

    def adv_encode(self, adv_type, value):
        tmp = bytes((len(value) + 1, adv_type,)) + value
        return tmp

    def adv_encode_name(self, name):
        tmp = self.adv_encode(0x09, name.encode())
        return tmp

    def checkMessageFlag(self):
        return self._new_message

    def getMessage(self):
        self._new_message = False
        return self._message

    def sendMessage(self, mess):

        try:
            self._ble.gatts_notify(64, self._txhandle, mess)
            # self._ble.gatts_write(self._txhandle,mess)
            # self._ble.gatts_notify(64, self._txhandle)
            self._exceptions = 0
        except Exception as e:
            print("Got exception" + str(self._exceptions) + str(e))
            self._exceptions = self._exceptions + 1
            if self._exceptions > 20:
                self.disconnectBLE()

    def setSendControllerStatus(self, status_val):
        self._send_controller_status = status_val

    def checkSendControllerStatus(self):
        return self._send_controller_status

    def disconnectBLE(self):
        for conn_handle in self._connections:
            try:
                self._ble.gap_disconnect(conn_handle)
            except Exception as e:
                print("Disconnect exception:  " + str(e))

        self._send_controller_status = False

        time.sleep_ms(1)
        self._connections.clear()
        self._connected = False
        self._adv()

    def checkConnectionStatus(self):
        return self._connected

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            self._conn_handle, _, _, = data
            self._connections.add(self._conn_handle)
            self._connected = True
            self._send_controller_status = True
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conn_handle, _, _, = data

            # self._connections.remove(self._conn_handle)
            for conn_handle in self._connections:
                self._ble.gap_disconnect(conn_handle)
            self._connections.clear()
            self._connected = False
            self._send_controller_status = False
            print("Central Disconnect!")
            # Start advertising again to allow a new connection.
            self._adv()
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            print("PERIPHERAL Disconnected")

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle, = data
            if conn_handle in self._connections:
                self._new_message = True
                self._message = self._ble.gatts_read(self._rxhandle)

    def _adv(self, interval_us=500000):
        try:
            self._ble.gap_advertise(100, self.adv_encode(0x01, b'\x06') + self.adv_encode(0x03, b'\x0d\x18') + self.adv_encode(0x19, b'\xc1\x03') + self.adv_encode_name(self._name))
        except Exception as e:
            print("Advertising excepetion:   " + str(e))
