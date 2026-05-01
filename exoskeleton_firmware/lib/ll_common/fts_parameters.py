# FTS Parameters

'''
Stores per-unit parameters for Force Torque Sensors. Each entry contains
calibration coefficients, zero-reference voltages, and the PDCP NID set
used for CAN output. FTS units are identified by sequential integer IDs
and can move between devices. tunable_settings.FTS_UNIT on each device
specifies which unit is installed.

Calibration coefficient order: [v1, v2, v1^2, v2^2, v1*v2, intercept]
v0: [ch1_zero_voltage, ch2_zero_voltage]

NID sets follow the PDCP CID convention:
  nid_control  — Control NID (receives start/stop commands from CAPS)
  nid_data1    — Data NID 1 (channels 1-3, 16-bit)
  nid_data2    — Data NID 2 (channels 4-6, 16-bit)

Available NID sets:
  0x40 / 0x41 / 0x42 — Drop-in replacement for load cell
  0x44 / 0x45 / 0x46 — Separate device for side-by-side testing
'''

def get_parameters(unit_id):
    """Return parameter dict for the given FTS unit ID, or None.

    Only the matched unit's data is allocated; unmatched branches are never
    evaluated, keeping memory usage minimal.

    Returns dict with keys:
        cal_fz: list of 6 polynomial coefficients for Fz
        cal_my: list of 6 polynomial coefficients for My
        v0: list of 2 zero-reference voltages [ch1, ch2]
        nid_control: int, PDCP control NID
        nid_data1: int, PDCP data NID 1 (channels 1-3)
        nid_data2: int, PDCP data NID 2 (channels 4-6)

    Parameters:
        unit_id: integer ID of the FTS unit, as specified in
            tunable_settings.FTS_UNIT
    """

    if unit_id == 1:
        return {
            'cal_fz': [26414.530190, 28074.532314, 83392.240076,
                       54010.542885, 177390.160741, -8.761811],
            'cal_my': [558.550149, -220.673255, -5325.275840,
                       -9601.913565, -18511.895307, 0.520985],
            'v0': [0.663443, 0.643838],
            'nid_control': 0x044,
            'nid_data1': 0x045,
            'nid_data2': 0x046
        }
    elif unit_id == 2:
        return {
            'cal_fz': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'cal_my': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'v0': [0.0, 0.0],
            'nid_control': 0x044,
            'nid_data1': 0x045,
            'nid_data2': 0x046,
        }
    return None
