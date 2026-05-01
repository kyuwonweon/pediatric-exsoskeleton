import pyb  # type: ignore
from lib import config
from lib.ll_common import mode_switch
import micropython
import struct
#
# CAN Streamer for Debugging
#
# This script reads all traffic on a CAN bus and streams each message to REPL.
# Useful for debugging and monitoring CAN bus communications.
#
# Usage: Run this script to start monitoring all CAN traffic
# Press Ctrl+C to stop monitoring
#

class CANCommunication:
    def __init__(self):
        self.can_id = 1
        self.baudrate = 1_000_000
        self.sample_point = 50
        self.brs_baudrate = 1_000_000
        self.auto_restart = True
        
        # Initialize CAN bus (not in loopback mode for real monitoring)
        self.can = pyb.CAN(
            self.can_id,
            pyb.CAN.NORMAL,
            baudrate=self.baudrate,
            sample_point=self.sample_point,
            brs_baudrate=self.brs_baudrate,
            auto_restart=self.auto_restart
        )


#TODO: This maybe should have a different name
class FSR:
    def __init__(
            self, can: CANCommunication, fifoID= 0, bank=0):
        """Initialize CAN streamer

        Args:
            can_id: CAN peripheral ID (usually 1)
            baudrate: CAN bus speed in bits/second (default 1Mbps)
        """
        self.sync_id = 0x80
        self.arduino_ids = (0x481, 0x482, 0x483, 0x484)
        self.id_to_index={msg_id: i*4 for i, msg_id in enumerate(self.arduino_ids)}
        self.sensor_data = [0] * 16


        self.can = can
        self.fifoID = fifoID
        self.bank = bank
        self.config = config
        self.selector = mode_switch.selector  # defined in boot.py
        self.device_parameters = (
            self.config.DeviceParameters(self.selector.device_id)
        )

        self._update_FSR_ref = self._update_FSR
        self.busy = False


    def setup(self):
        """Setup CAN interface for monitoring all messages"""
        try:

            # Accept only Arduino messages
            # self.can.setfilter(0, pyb.CAN.MASK, 0, (0, 0))
            self.can.setfilter(self.bank, pyb.CAN.MASK, self.fifoID, (0x480, 0x7F8))

            self.run = False
            self.timer_can = pyb.Timer(
                self.device_parameters.TIMER_FSR['ID'],
                freq=self.device_parameters.TIMER_FSR['FREQ'],
                callback=self._update_callback
            )
            return True

        except Exception as e:
            print(f"Error setting up CAN interface: {e}")
            return False

    def send_sync(self):
        try:
            payload = bytes([0] * 8)  # 8-byte message of zeros
            self.can.send(payload, self.sync_id, timeout=0, extframe=False)
        except OSError:
            pass


    def get_value(self):
        return self.sensor_data

    def enable(self):
        """Enable CAN streaming"""
        self.run = True

    def _update_FSR(self, _):
        if not self.can:
            return
        try:
            self.send_sync()
            while self.can.any(self.fifoID):
                msg = self.can.recv(self.fifoID)
                if not msg:
                    continue
                msg_id, _, _, _, data = msg
                if msg_id in self.arduino_ids:
                    # print(msg_id)
                    base = self.id_to_index[msg_id]
                    # Manual unpack to avoid struct
                    self.sensor_data[base] = data[0] | (data[1] << 8)
                    self.sensor_data[base + 1] = data[2] | (data[3] << 8)
                    self.sensor_data[base + 2] = data[4] | (data[5] << 8)
                    self.sensor_data[base + 3] = data[6] | (data[7] << 8)
        except OSError as err:
            pass

    def _update_callback(self, _):
        if self.run and not self.busy:
            self.busy = True
            try:
                micropython.schedule(self._update_FSR_ref, 0)
                self.busy = False
            except RuntimeError:
                self.busy = False



class LoadCell:
    def __init__(
            self, 
            can:CANCommunication, fifoID =0, bank = 0):
        """Initialize CAN streamer

        Args:
            can_id: CAN peripheral ID (usually 1)
            baudrate: CAN bus speed in bits/second (default 1Mbps)
        """

        self.can = can
        self.fifoID = fifoID
        self.bank = bank
        self.load_cell_Fz = 0.0
        self.load_cell_My =0.0

        self.config = config
        self.selector = mode_switch.selector  # defined in boot.py
        self.device_parameters = (
            self.config.DeviceParameters(self.selector.device_id)
        )

        self._update_loadcell_ref = self._update_loadcell
        self.busy = False


    def setup(self):
        """Setup CAN interface for monitoring all messages"""
        try:

            # Configure filter to accept ALL messages
            # Using MASK mode with mask=0 accepts all message IDs
            self.can.setfilter(self.bank, pyb.CAN.MASK, self.fifoID, (0x1E8, 0x7FF))

            self.run = False
            self.timer_can = pyb.Timer(
                self.device_parameters.TIMER_LOADCELL['ID'],
                freq=self.device_parameters.TIMER_LOADCELL['FREQ'],
                callback=self._update_callback
            )

            return True

        except Exception as e:
            print(f"Error setting up CAN interface: {e}")
            return False

    def get_value(self):
        return self.load_cell_Fz, self.load_cell_My

    def enable(self):
        """Enable CAN streaming"""
        self.run = True

    def _update_loadcell(self, _):
        if not self.can:
            print("CAN not initialized. Call setup() first.")
            self.busy = False
            return
        try:
            msg = self.can.recv(self.fifoID, timeout=0)
            if msg and len(msg) == 5:
                values = [b for b in msg[4]]
                if values[2] == 3:  # Fz
                    val = float((values[3] << 8 | values[4]) - 0x7FFF)
                    self.load_cell_Fz = val * 500 / 6553.5
                elif values[2] == 5:  # My
                    val = float((values[3] << 8 | values[4]) - 0x7FFF)
                    self.load_cell_My = val * 40 / 6553.5
        except OSError:
            pass
        finally:
            self.busy = False

    def _update_callback(self, _):
        if self.run and not self.busy:
            self.busy = True
            try:
                micropython.schedule(self._update_loadcell_ref, 0)
                self.busy=  False
            except RuntimeError:
                self.busy = False




