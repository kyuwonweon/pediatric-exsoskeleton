"""
Force Torque Sensor (FTS) MicroPython Driver

Measures vertical force (Fz) and moment about the y-axis (My) using two hall
effect sensors (DRV5055-Q1) and an ADS1292 dual-channel 24-bit ADC. The sensor
uses a dual cantilever beam design where both beams deflect in the same
direction under vertical load, and in opposite directions under moment load.

Data Flow (Polynomial Calibration):
    Hall Effect Sensors (DRV5055-Q1)
             |
             v
    ADS1292 ADC (24-bit voltage)
             |
             v
    Voltage Offset: v1 = V_ch1 - V0_ch1, v2 = V_ch2 - V0_ch2
             |
             v
    Polynomial Evaluation:
        Fz = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5
        My = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5
             |
             v
    Force (N) and Moment (Nm)

Features:
    - Dual-channel 24-bit ADC (ADS1292)
    - Polynomial calibration (degree 1 or 2) on voltage offsets
    - Voltage zero-point (V0) captured in unloaded state
    - Hall sensor linearization available for diagnostics (get_magnetic_field)
    - ISR-based continuous data streaming
    - Configurable sample rate (125-8000 SPS)

Calibration coefficient order: [v1, v2, v1², v2², v1*v2, intercept]
For degree-1 calibration, set quadratic/cross-term coefficients to 0.

Example usage:
    from lib.ll_common import fts

    sensor = fts.FTS(
        spi_object=spi_bus,
        cs_pin_id='PE3',
        drdy_pin_id='PE4',
        hall_sensitivity=0.1,  # 100 mV/mT (DRV5055-A1)
        verbose=True
    )
    sensor.start_conversions()

    # Zero calibration (MUST be done in unloaded state)
    sensor.zero_calibration()  # Captures V0 reference voltages

    # Set polynomial calibration coefficients from calibration notebook
    # Order: [v1, v2, v1², v2², v1*v2, intercept]
    sensor.set_calibration(
        fz_coeffs=[26414.53, 28074.53, 83392.24, 54010.54, 177390.16, -8.76],
        my_coeffs=[558.55, -220.67, -5325.28, -9601.91, -18511.90, 0.52]
    )

    # Or set V0 directly from calibration notebook
    sensor.v0[0] = 0.663443
    sensor.v0[1] = 0.643838

    # Read calibrated force/moment
    sensor.get_force_moment()
    print(f'Fz: {sensor.fz:.3f} N, My: {sensor.my:.3f} Nm')

Reference Documents:
    - DRV5055-Q1 Hall Effect Sensor: @reference/drv5055-q1-1.pdf (section 6.3)
    - ADS1292 ADC: @reference/ads1292.pdf
"""

import pyb  # type: ignore
import micropython  # type: ignore
import array
from micropython import const  # type: ignore

from . import nvic

# ============================================================================
# ADS1292 REGISTER ADDRESSES
# ============================================================================
_REG_ID          = const(0x00)
_REG_CONFIG1     = const(0x01)
_REG_CONFIG2     = const(0x02)
_REG_LOFF        = const(0x03)
_REG_CH1SET      = const(0x04)
_REG_CH2SET      = const(0x05)
_REG_RLD_SENS    = const(0x06)
_REG_LOFF_SENS   = const(0x07)
_REG_LOFF_STAT   = const(0x08)
_REG_RESP1       = const(0x09)
_REG_RESP2       = const(0x0A)
_REG_GPIO        = const(0x0B)

# ============================================================================
# ADS1292 SPI COMMANDS
# ============================================================================
_CMD_WAKEUP      = const(0x02)
_CMD_STANDBY     = const(0x04)
_CMD_RESET       = const(0x06)
_CMD_START       = const(0x08)
_CMD_STOP        = const(0x0A)
_CMD_RDATAC      = const(0x10)
_CMD_SDATAC      = const(0x11)
_CMD_RDATA       = const(0x12)

# ============================================================================
# PGA GAIN SETTINGS FOR CHnSET[6:4]
# ============================================================================
_GAIN_1   = const(0x10)
_GAIN_2   = const(0x20)
_GAIN_3   = const(0x30)
_GAIN_4   = const(0x40)
_GAIN_6   = const(0x00)
_GAIN_8   = const(0x50)
_GAIN_12  = const(0x60)

# ============================================================================
# BUFFER SIZES
# ============================================================================
_STATUS_BYTES    = const(3)   # 24-bit status word
_CHANNEL_BYTES   = const(3)   # 24-bit per channel
_SAMPLE_SIZE     = const(9)   # Status (3) + CH1 (3) + CH2 (3)

# ============================================================================
# DRV5055-Q1 HALL SENSOR DEFAULTS
# ============================================================================
# Sensitivity values at VCC=5V (scale proportionally for other voltages)
DRV5055_SENSITIVITY_A1 = 0.100  # 100 mV/mT (highest sensitivity, +/-21 mT range)
DRV5055_SENSITIVITY_A2 = 0.050  # 50 mV/mT (+/-42 mT range)
DRV5055_SENSITIVITY_A3 = 0.025  # 25 mV/mT (+/-85 mT range)
DRV5055_SENSITIVITY_A4 = 0.0125  # 12.5 mV/mT (+/-169 mT range)

# Default quiescent voltage (VCC/2)
DRV5055_QUIESCENT_3V3 = 1.65  # VCC/2 at 3.3V supply
DRV5055_QUIESCENT_5V0 = 2.50  # VCC/2 at 5V supply


# ============================================================================
# FTS CLASS
# ============================================================================
class FTS:
    """
    Force Torque Sensor Driver

    Combines ADS1292 ADC with polynomial calibration on voltage offsets
    for force/moment measurement.

    Attributes:
        ch1_voltage (float): Raw voltage from channel 1 (V)
        ch2_voltage (float): Raw voltage from channel 2 (V)
        ch1_raw (array): Raw 24-bit ADC count for channel 1
        ch2_raw (array): Raw 24-bit ADC count for channel 2
        ch1_field (float): Linearized magnetic field from channel 1 (mT)
        ch2_field (float): Linearized magnetic field from channel 2 (mT)
        v0 (array): Voltage zero-point [ch1, ch2] (V)
        cal_fz (array): Fz polynomial coefficients [v1, v2, v1², v2², v1*v2, intercept]
        cal_my (array): My polynomial coefficients [v1, v2, v1², v2², v1*v2, intercept]
        fz (float): Calibrated vertical force (N)
        my (float): Calibrated moment about y-axis (Nm)
        is_new (bool): True if data is new since last read
    """

    def __init__(self,
                 spi_object,
                 cs_pin_id,
                 drdy_pin_id,
                 reset_pin_id=None,
                 start_pin_id=None,
                 sample_rate=500,
                 vref_4v=False,
                 pga_gain=1,
                 cal_fz=None,
                 cal_my=None,
                 hall_sensitivity=None,
                 hall_quiescent=None,
                 verbose=False,
                 interrupt_priority=6,
                 transient_reject_threshold=0):
        """
        Initialize FTS driver.

        Parameters:
            spi_object: Pre-configured pyb.SPI or upySPI instance
            cs_pin_id: Chip select pin (e.g., 'PE3')
            drdy_pin_id: Data ready interrupt pin (e.g., 'PE4')
            reset_pin_id: Optional RESET pin (e.g., 'PE5')
            start_pin_id: Optional START pin (None = use START command)
            sample_rate: Data rate in SPS (125, 250, 500, 1000, 2000, 4000, 8000)
            vref_4v: If True, use 4.033V reference; if False, use 2.42V
            pga_gain: PGA gain setting (1, 2, 3, 4, 6, 8, 12)
            cal_fz: Fz polynomial coefficients [v1, v2, v1², v2², v1*v2, intercept].
                     Default: all zeros (uncalibrated)
            cal_my: My polynomial coefficients [v1, v2, v1², v2², v1*v2, intercept].
                     Default: all zeros (uncalibrated)
            hall_sensitivity: Hall sensor sensitivity in V/mT.
                              Default: 0.1 V/mT (DRV5055-A1)
            hall_quiescent: Hall sensor quiescent voltage in V.
                            Default: 1.65V (VCC/2 at 3.3V supply)
            verbose: Enable debug printing
            interrupt_priority: NVIC interrupt priority for DRDY
            transient_reject_threshold: Maximum allowed change in raw ADC counts
                between consecutive valid samples. Samples exceeding this are
                rejected and the previous valid reading is held. Set to 0 to
                disable rate-of-change filtering. Status word validation is
                always active regardless of this setting.
        """
        self.verbose = verbose
        self.spi = spi_object
        self.sample_rate = sample_rate
        self.vref_4v = vref_4v
        self.pga_gain = pga_gain
        self.interrupt_priority = interrupt_priority
        self.transient_reject_threshold = transient_reject_threshold
        self._filter_armed = False
        self.rejected_count = 0

        if self.verbose:
            print(f'FTS: Initializing driver')
            print(f'FTS: Sample rate = {sample_rate} SPS')
            print(f'FTS: VREF = {"4.033V" if vref_4v else "2.42V"}')
            print(f'FTS: PGA gain = {pga_gain}')

        # ====================================================================
        # Calibration parameters
        # ====================================================================
        # Polynomial coefficients: [v1, v2, v1², v2², v1*v2, intercept]
        if cal_fz is None:
            self.cal_fz = array.array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.cal_fz = array.array('f', cal_fz)

        if cal_my is None:
            self.cal_my = array.array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.cal_my = array.array('f', cal_my)

        # Voltage zero-point (set via zero_calibration() or from calibration notebook)
        self.v0 = array.array('f', [0.0, 0.0])

        # Hall sensor parameters (used by get_magnetic_field() for diagnostics)
        if hall_sensitivity is None:
            self.hall_sensitivity = DRV5055_SENSITIVITY_A1
        else:
            self.hall_sensitivity = hall_sensitivity

        if hall_quiescent is None:
            self.hall_quiescent = DRV5055_QUIESCENT_3V3
        else:
            self.hall_quiescent = hall_quiescent

        if self.verbose:
            print(f'FTS: Hall sensitivity = {self.hall_sensitivity * 1000:.1f} mV/mT')
            print(f'FTS: Hall quiescent = {self.hall_quiescent:.3f} V')

        # ====================================================================
        # Configure GPIO pins
        # ====================================================================
        # Chip select (active low, push-pull output, initially high)
        self.cs_pin = pyb.Pin(cs_pin_id, pyb.Pin.OUT_PP, pull=pyb.Pin.PULL_UP)
        self.cs_pin.value(1)

        # RESET pin (optional)
        if reset_pin_id:
            if self.verbose:
                print(f'FTS: Using RESET pin for hardware reset')
            self.reset_pin = pyb.Pin(
                reset_pin_id,
                pyb.Pin.OUT_PP,
                pull=pyb.Pin.PULL_UP
            )
            self.reset_pin.value(1)
        else:
            if self.verbose:
                print(f'FTS: No RESET pin provided; skipping hardware reset')
            self.reset_pin = None

        # START pin (optional)
        if start_pin_id:
            self.start_pin = pyb.Pin(start_pin_id, pyb.Pin.OUT_PP)
            self.start_pin.value(0)
            self.use_start_pin = True
            if self.verbose:
                print(f'FTS: Using START pin for conversion control')
        else:
            self.use_start_pin = False
            if self.verbose:
                print(f'FTS: Using START command for conversion control')

        # ====================================================================
        # Pre-allocate buffers (ISR-safe, no heap allocation)
        # ====================================================================
        self.cmd_buffer = bytearray([0])
        self.sample_buffer = bytearray(_SAMPLE_SIZE)
        self.ch1_raw = array.array('i', [0])  # Signed 24-bit as 32-bit int
        self.ch2_raw = array.array('i', [0])
        self.status_word = bytearray(3)

        # Data ready flag
        self.data_ready = False

        # ====================================================================
        # Output values (native floats are ISR-safe)
        # ====================================================================
        # Raw ADC voltage (V)
        self.ch1_voltage = 0.0
        self.ch2_voltage = 0.0

        # Linearized magnetic field (mT)
        self.ch1_field = 0.0
        self.ch2_field = 0.0

        # Calibrated force/moment
        self.fz = 0.0  # Vertical force (N)
        self.my = 0.0  # Moment about y-axis (Nm)

        # Data freshness flag
        self.is_new = False

        # ====================================================================
        # Configure DRDY interrupt
        # ====================================================================
        self.drdy_pin = pyb.ExtInt(
            drdy_pin_id,
            pyb.ExtInt.IRQ_FALLING,
            pyb.Pin.PULL_UP,
            self.isr
        )
        self.drdy_pin.disable()

        nvic.nvic_set_prio(self.drdy_pin.line() + 6, interrupt_priority)

        # ====================================================================
        # Initialize ADS1292 device
        # ====================================================================
        if reset_pin_id:
            if self.verbose:
                print('FTS: Performing hardware reset...')
            self.hard_reset()

        if self.verbose:
            print('FTS: Configuring ADS1292 registers...')
        self.configure_device()

        if self.verbose:
            print('FTS: Initialization complete')

    # ========================================================================
    # CALIBRATION METHODS
    # ========================================================================

    def set_calibration(self, fz_coeffs, my_coeffs):
        """
        Update polynomial calibration coefficients.

        Coefficient order: [v1, v2, v1², v2², v1*v2, intercept]
        For degree-1 calibration, set quadratic/cross-term entries to 0.

        Parameters:
            fz_coeffs: 6-element list/array of Fz polynomial coefficients
            my_coeffs: 6-element list/array of My polynomial coefficients
        """
        for i in range(6):
            self.cal_fz[i] = fz_coeffs[i]
            self.cal_my[i] = my_coeffs[i]

        if self.verbose:
            print(f'FTS: Calibration updated')
            print(f'FTS: cal_fz = [{self.cal_fz[0]:.4f}, {self.cal_fz[1]:.4f}, '
                  f'{self.cal_fz[2]:.4f}, {self.cal_fz[3]:.4f}, '
                  f'{self.cal_fz[4]:.4f}, {self.cal_fz[5]:.4f}]')
            print(f'FTS: cal_my = [{self.cal_my[0]:.4f}, {self.cal_my[1]:.4f}, '
                  f'{self.cal_my[2]:.4f}, {self.cal_my[3]:.4f}, '
                  f'{self.cal_my[4]:.4f}, {self.cal_my[5]:.4f}]')

    def set_hall_params(self, sensitivity=None, quiescent=None):
        """
        Update hall sensor parameters (used by get_magnetic_field() for diagnostics).

        Parameters:
            sensitivity: Hall sensor sensitivity in V/mT (e.g., 0.1 for DRV5055-A1)
            quiescent: Hall sensor quiescent voltage in V (nominally VCC/2)
        """
        if sensitivity is not None:
            self.hall_sensitivity = sensitivity
        if quiescent is not None:
            self.hall_quiescent = quiescent

        if self.verbose:
            print(f'FTS: Hall params updated')
            print(f'FTS: Sensitivity = {self.hall_sensitivity * 1000:.1f} mV/mT')
            print(f'FTS: Quiescent = {self.hall_quiescent:.3f} V')

    def zero_calibration(self):
        """
        Capture current voltage as zero-point reference (V0).

        Call this function when the sensor is in the unloaded state (no force applied).
        This sets the reference voltage for each channel, used to compute voltage
        offsets for the polynomial calibration.

        The sensor should be running (start_conversions() called) before zeroing.
        """
        self.get_raw_voltage()

        self.v0[0] = self.ch1_voltage
        self.v0[1] = self.ch2_voltage

        if self.verbose:
            print(f'FTS: Zero calibration complete')
            print(f'FTS: V0[CH1] = {self.v0[0]:.6f} V')
            print(f'FTS: V0[CH2] = {self.v0[1]:.6f} V')

    # ========================================================================
    # DATA ACCESS METHODS
    # ========================================================================

    def get_raw_voltage(self):
        """
        Get latest raw ADC voltages (non-blocking).

        Converts raw 24-bit ADC counts to voltage using the configured
        VREF and PGA gain settings.

        Results:
            self.ch1_voltage: Channel 1 voltage in volts
            self.ch2_voltage: Channel 2 voltage in volts
            self.is_new: True if data is new since last call
        """
        self.is_new = self.data_ready
        self.data_ready = False

        # Calculate VREF based on configuration
        vref = 4.033 if self.vref_4v else 2.42

        # ADC full scale = +/-VREF / gain
        # LSB resolution: (2 * VREF/gain) / 2^24
        lsb = (2.0 * vref / self.pga_gain) / 16777216.0

        self.ch1_voltage = self.ch1_raw[0] * lsb
        self.ch2_voltage = self.ch2_raw[0] * lsb

    @micropython.native
    def get_magnetic_field(self):
        """
        Get linearized magnetic field readings (non-blocking).

        Applies hall effect sensor linearization per DRV5055-Q1 datasheet:
            B = (V_OUT - V_Q) / Sensitivity

        Results:
            self.ch1_field: Channel 1 magnetic field in mT
            self.ch2_field: Channel 2 magnetic field in mT
            self.ch1_voltage: Channel 1 raw voltage (also updated)
            self.ch2_voltage: Channel 2 raw voltage (also updated)
            self.is_new: True if data is new since last call
        """
        # First get raw voltage
        self.get_raw_voltage()

        # Linearize: B = (V - Vq) / Sensitivity
        # Result is in mT (since sensitivity is in V/mT)
        self.ch1_field = (self.ch1_voltage - self.hall_quiescent) / self.hall_sensitivity
        self.ch2_field = (self.ch2_voltage - self.hall_quiescent) / self.hall_sensitivity

    @micropython.native
    def get_force_moment(self):
        """
        Get calibrated force and moment readings (non-blocking).

        Computes voltage offsets from V0 and evaluates the polynomial:
            v1 = V_ch1 - V0_ch1
            v2 = V_ch2 - V0_ch2
            Fz = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5
            My = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5

        Results:
            self.fz: Vertical force in N
            self.my: Moment about y-axis in Nm
            self.ch1_voltage, self.ch2_voltage: Raw voltages (also updated)
            self.is_new: True if data is new since last call
        """
        self.get_raw_voltage()

        v1 = self.ch1_voltage - self.v0[0]
        v2 = self.ch2_voltage - self.v0[1]
        v1_sq = v1 * v1
        v2_sq = v2 * v2
        v1v2 = v1 * v2

        self.fz = (self.cal_fz[0] * v1 + self.cal_fz[1] * v2 +
                   self.cal_fz[2] * v1_sq + self.cal_fz[3] * v2_sq +
                   self.cal_fz[4] * v1v2 + self.cal_fz[5])
        self.my = (self.cal_my[0] * v1 + self.cal_my[1] * v2 +
                   self.cal_my[2] * v1_sq + self.cal_my[3] * v2_sq +
                   self.cal_my[4] * v1v2 + self.cal_my[5])

    def get_channel_data(self):
        """
        Get latest raw channel data (non-blocking, ISR-safe).

        Results are stored in pre-allocated attributes:
            self.ch1_raw[0]: Raw 24-bit signed integer for channel 1
            self.ch2_raw[0]: Raw 24-bit signed integer for channel 2
            self.is_new: True if data is new since last call
        """
        self.is_new = self.data_ready
        self.data_ready = False

    def get_channel_voltage(self, gain=None):
        """
        Get raw ADC voltage (backward compatible with ADS1292 API).

        Parameters:
            gain: PGA gain (if None, uses configured pga_gain)

        Results:
            self.ch1_voltage: Channel 1 voltage in volts
            self.ch2_voltage: Channel 2 voltage in volts
            self.is_new: True if data is new since last call
        """
        if gain is not None:
            original_gain = self.pga_gain
            self.pga_gain = gain
            self.get_raw_voltage()
            self.pga_gain = original_gain
        else:
            self.get_raw_voltage()

    def get_status(self):
        """
        Get ADS1292 status word (24 bits).

        Format: (1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13 '0's)

        Returns:
            bytearray of 3 bytes (STATUS[23:0])
        """
        return self.status_word

    # ========================================================================
    # ADS1292 RESET AND CONFIGURATION
    # ========================================================================

    def hard_reset(self):
        """
        Hardware reset via PWDN/RESET pin.

        Performs power-on reset sequence per ADS1292 datasheet flowchart.
        """
        if self.verbose:
            print('FTS: Asserting hardware reset')

        self.reset_pin.value(0)
        pyb.udelay(100)  # >2 tCLK cycles at 512kHz
        self.reset_pin.value(1)

        pyb.delay(2)  # Wait for power-on reset

        if self.verbose:
            print('FTS: Reset complete, device ready')

    def configure_device(self):
        """
        Configure ADS1292 registers for continuous conversion mode.
        """
        # Exit RDATAC mode to write registers
        self.send_command(_CMD_SDATAC)
        pyb.delay(1)

        if self.verbose:
            print('FTS: Device in SDATAC mode, ready for register writes')

        # CONFIG2: Enable internal reference buffer
        config2_val = 0x80 | 0x20  # Bit 7 and bit 5 set
        if self.vref_4v:
            config2_val |= 0x10
        self.write_register(_REG_CONFIG2, config2_val)

        pyb.delay(5)  # Wait for reference to settle

        if self.verbose:
            print('FTS: Internal reference enabled and settled')

        # CONFIG1: Set data rate
        dr_bits = self._get_dr_bits(self.sample_rate)
        self.write_register(_REG_CONFIG1, dr_bits)

        # Map PGA gain to register value
        gain_map = {
            1: _GAIN_1, 2: _GAIN_2, 3: _GAIN_3, 4: _GAIN_4,
            6: _GAIN_6, 8: _GAIN_8, 12: _GAIN_12
        }
        gain_bits = gain_map.get(self.pga_gain, _GAIN_1)

        # CH1SET and CH2SET: Set gain, normal input
        self.write_register(_REG_CH1SET, gain_bits)
        self.write_register(_REG_CH2SET, gain_bits)

        # RLD_SENS: Disable right leg drive
        self.write_register(_REG_RLD_SENS, 0x00)

        # LOFF_SENS: Disable lead-off detection
        self.write_register(_REG_LOFF_SENS, 0x00)

        # RESP1: Required 0x02 for ADS1292
        self.write_register(_REG_RESP1, 0x02)

        # RESP2: Internal RLDREF
        self.write_register(_REG_RESP2, 0x03)

        if self.verbose:
            print(f'FTS: Configured for {self.sample_rate} SPS, gain={self.pga_gain}')

    def _get_dr_bits(self, sample_rate):
        """Convert sample rate to DR[2:0] register bits."""
        rate_map = {
            125:  0b000,
            250:  0b001,
            500:  0b010,
            1000: 0b011,
            2000: 0b100,
            4000: 0b101,
            8000: 0b110,
        }
        if sample_rate not in rate_map:
            raise ValueError(f'Invalid sample rate: {sample_rate}. '
                           f'Must be one of {list(rate_map.keys())}')
        return rate_map[sample_rate]

    # ========================================================================
    # SPI COMMUNICATION
    # ========================================================================

    def send_command(self, cmd):
        """Send single-byte SPI command."""
        cmd_buffer = self.cmd_buffer
        cmd_buffer[0] = cmd

        self.cs_pin.value(0)
        pyb.udelay(1)
        self.spi.write(cmd_buffer)
        pyb.udelay(10)
        self.cs_pin.value(1)

    def write_register(self, reg_addr, value):
        """Write single register (WREG command)."""
        self.cs_pin.value(0)
        pyb.udelay(1)

        # Multi-byte command with inter-byte delays
        self.spi.write(bytearray([0x40 | reg_addr]))
        pyb.udelay(8)
        self.spi.write(bytearray([0x00]))
        pyb.udelay(8)
        self.spi.write(bytearray([value]))

        pyb.udelay(10)
        self.cs_pin.value(1)

        if self.verbose:
            print(f'FTS: WREG 0x{reg_addr:02X} = 0x{value:02X}')

    def read_register(self, reg_addr):
        """Read single register (RREG command)."""
        self.cs_pin.value(0)
        pyb.udelay(1)

        self.spi.write(bytearray([0x20 | reg_addr]))
        pyb.udelay(8)
        self.spi.write(bytearray([0x00]))
        pyb.udelay(8)
        result = bytearray(1)
        self.spi.readinto(result)
        self.cs_pin.value(1)

        if self.verbose:
            print(f'FTS: RREG 0x{reg_addr:02X} = 0x{result[0]:02X}')

        return result[0]

    # ========================================================================
    # START/STOP CONVERSIONS
    # ========================================================================

    def start_conversions(self):
        """
        Start continuous conversions.

        Puts ADS1292 in RDATAC mode and begins sampling.
        """
        self.send_command(_CMD_RDATAC)
        pyb.udelay(10)

        self.drdy_pin.enable()

        if self.use_start_pin:
            self.start_pin.value(1)
        else:
            self.send_command(_CMD_START)

        if self.verbose:
            print('FTS: Conversions started, device in RDATAC mode')

    def stop_conversions(self):
        """
        Stop continuous conversions.

        Halts sampling and exits RDATAC mode.
        """
        self.drdy_pin.disable()

        if self.use_start_pin:
            self.start_pin.value(0)
        else:
            self.send_command(_CMD_STOP)

        pyb.delay(1)
        self.send_command(_CMD_SDATAC)

        if self.verbose:
            print('FTS: Conversions stopped, device in SDATAC mode')

    # ========================================================================
    # INTERRUPT SERVICE ROUTINE
    # ========================================================================

    @micropython.native
    def isr(self, line):
        """
        DRDY interrupt service routine.

        Called when DRDY falls (new data ready). Reads 9 bytes:
        STATUS (3) + CH1 (3) + CH2 (3)

        Includes two-layer transient rejection:
        1. Status word validation (always active): ADS1292 status byte upper
           nibble must be 0xC.
        2. Rate-of-change guard (when transient_reject_threshold > 0): rejects
           samples where either channel changes by more than the threshold from
           the previous valid reading.

        Must be ISR-safe: no heap allocation, pre-allocated buffers only.
        """
        self.cs_pin.value(0)
        self.spi.readinto(self.sample_buffer)
        self.cs_pin.value(1)

        # Layer 1: Validate ADS1292 status word upper nibble (must be 0xC)
        if (self.sample_buffer[0] & 0xF0) != 0xC0:
            self.rejected_count += 1
            return

        # Parse 24-bit two's complement channel data into locals
        ch1 = self._bytes_to_int24(
            self.sample_buffer[3],
            self.sample_buffer[4],
            self.sample_buffer[5]
        )
        ch2 = self._bytes_to_int24(
            self.sample_buffer[6],
            self.sample_buffer[7],
            self.sample_buffer[8]
        )

        # Layer 2: Rate-of-change guard (after first valid sample)
        threshold = self.transient_reject_threshold
        if threshold > 0 and self._filter_armed:
            delta1 = ch1 - self.ch1_raw[0]
            delta2 = ch2 - self.ch2_raw[0]
            if delta1 < 0:
                delta1 = -delta1
            if delta2 < 0:
                delta2 = -delta2
            if delta1 > threshold or delta2 > threshold:
                self.rejected_count += 1
                return

        # Accept sample
        self.ch1_raw[0] = ch1
        self.ch2_raw[0] = ch2
        self._filter_armed = True

        # Copy status bytes
        self.status_word[0] = self.sample_buffer[0]
        self.status_word[1] = self.sample_buffer[1]
        self.status_word[2] = self.sample_buffer[2]

        self.data_ready = True

    @micropython.native
    def _bytes_to_int24(self, msb, mid, lsb):
        """Convert 24-bit two's complement bytes to signed 32-bit integer."""
        value = (msb << 16) | (mid << 8) | lsb

        # Sign extension: check bit 23
        if value & 0x800000:
            value = value - 0x1000000

        return value

    # ========================================================================
    # DEVICE VERIFICATION
    # ========================================================================

    def check_device_id(self):
        """
        Read and verify ADS1292 device ID register.

        Expected ID:
        - Bits[7:5] = 010 (ADS1x9x device)
        - Bits[1:0] = 11 (ADS1292/ADS1292R)

        Returns:
            True if device ID matches, False otherwise
        """
        device_id = self.read_register(_REG_ID)

        rev_id_high = (device_id >> 5) & 0x07
        rev_id_low = device_id & 0x03

        if rev_id_high == 0b010 and rev_id_low == 0b11:
            if self.verbose:
                print(f'FTS: Device ID verified: 0x{device_id:02X}')
            return True
        else:
            print(f'FTS: WARNING - Unexpected device ID: 0x{device_id:02X}')
            return False

    # ========================================================================
    # UTILITY METHODS
    # ========================================================================

    def standby(self):
        """Enter low-power standby mode (~15 uW)."""
        self.stop_conversions()
        self.send_command(_CMD_STANDBY)

        if self.verbose:
            print('FTS: Entered standby mode')

    def wakeup(self):
        """Wake from standby mode."""
        self.send_command(_CMD_WAKEUP)
        pyb.delay(1)

        if self.verbose:
            print('FTS: Woke from standby mode')
