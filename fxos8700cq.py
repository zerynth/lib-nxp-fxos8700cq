"""
.. module:: fxos8700cq

*****************
FXOS8700CQ Module
*****************

This module contains the driver for NXP FXOS8700CQ accelerometer and magnetometer.
The FXOS8700CQ provides direct I2C communication and the accelerometer can be set on 3 different full-scale range and 8 different over sample rate values  (`datasheet <http://www.nxp.com/assets/documents/data/en/data-sheets/FXOS8700CQ.pdf>`_).
    """

import i2c

FXOS_I2C_ADDRESS    = 0x1E

FXOS_OSR_SHIFT      = 2
FXOS_ODR_SHIFT      = 3
FXOS_MODE_SHIFT     = 0
FXOS_ACCRANGE_SHIFT = 0


# STATUS Register
STATUS_00_REG         = 0x00

# F_STATUS FIFO Status Register
# MMA8451 only - when F_MODE != 0
F_STATUS_REG          = 0x00

# XYZ Data Registers
OUT_X_MSB_REG         = 0x01
OUT_X_LSB_REG         = 0x02
OUT_Y_MSB_REG         = 0x03
OUT_Y_LSB_REG         = 0x04
OUT_Z_MSB_REG         = 0x05
OUT_Z_LSB_REG         = 0x06

# F_SETUP FIFO Setup Register
# MMA8451 only
F_SETUP_REG           = 0x09

F_MODE_DISABLED       = 0x00

# TRIG_CFG FIFO Trigger Configuration Register
# MMA8451 only
TRIG_CFG_REG          = 0x0A

# SYSMOD System Mode Register
SYSMOD_REG            = 0x0B

SYSMOD_STANDBY        = 0x00

# INT_SOURCE System Interrupt Status Register
INT_SOURCE_REG        = 0x0C

# WHO_AM_I Device ID Register
WHO_AM_I_REG          = 0x0D

# Content
kFXOS_WHO_AM_I_Device_ID = 0xC7

# XYZ_DATA_CFG Sensor Data Configuration Register
XYZ_DATA_CFG_REG      = 0x0E

__define(HPF_OUT_MASK,0x10)    # MMA8451 and MMA8452 only
__define(FS1_MASK,0x02)
__define(FS0_MASK,0x01)
__define(FS_MASK,0x03)

FULL_SCALE_2G         = 0x00
FULL_SCALE_4G         = FS0_MASK
FULL_SCALE_8G         = FS1_MASK

# HP_FILTER_CUTOFF High Pass Filter Register
HP_FILTER_CUTOFF_REG  = 0x0F

# PL_STATUS Portrait/Landscape Status Register
PL_STATUS_REG         = 0x10

# PL_CFG Portrait/Landscape Configuration Register
PL_CFG_REG            = 0x11

PL_EN_MASK            = 0x40


# PL_COUNT Portrait/Landscape Debounce Register
PL_COUNT_REG          = 0x12


# PL_BF_ZCOMP Back/Front and Z Compensation Register
PL_BF_ZCOMP_REG       = 0x13

# PL_P_L_THS Portrait to Landscape Threshold Register
PL_P_L_THS_REG        = 0x14

# FF_MT_CFG Freefall and Motion Configuration Register
FF_MT_CFG_REG         = 0x15

# FF_MT_SRC Freefall and Motion Source Registers
FF_MT_SRC_REG         = 0x16

# FF_MT_THS Freefall and Motion Threshold Registers
# TRANSIENT_THS Transient Threshold Register
FT_MT_THS_REG         = 0x17
TRANSIENT_THS_REG     = 0x1F

# FF_MT_COUNT Freefall Motion Count Registers
FF_MT_COUNT_REG       = 0x18

# TRANSIENT_CFG Transient Configuration Register
TRANSIENT_CFG_REG     = 0x1D

# TRANSIENT_SRC Transient Source Register
TRANSIENT_SRC_REG     = 0x1E

# TRANSIENT_COUNT Transient Debounce Register
TRANSIENT_COUNT_REG   = 0x20

# PULSE_CFG Pulse Configuration Register
PULSE_CFG_REG         = 0x21

# PULSE_SRC Pulse Source Register
PULSE_SRC_REG         = 0x22

# PULSE_THS XYZ Pulse Threshold Registers
PULSE_THSX_REG        = 0x23
PULSE_THSY_REG        = 0x24
PULSE_THSZ_REG        = 0x25

PTHS_MASK             = 0x7F

# PULSE_TMLT Pulse Time Window Register
PULSE_TMLT_REG        = 0x26

# PULSE_LTCY Pulse Latency Timer Register
PULSE_LTCY_REG        = 0x27

# PULSE_WIND Second Pulse Time Window Register
PULSE_WIND_REG        = 0x28

# ASLP_COUNT Auto Sleep Inactivity Timer Register
ASLP_COUNT_REG        = 0x29

# FXOS_CTRL_REG1 System Control 1 Register
FXOS_CTRL_REG1        = 0x2A

__define(ASLP_RATE1_MASK,0x80)
__define(ASLP_RATE0_MASK,0x40)
__define(DR2_MASK       ,0x20)
__define(DR1_MASK       ,0x10)
__define(DR0_MASK       ,0x08)
__define(LNOISE_MASK    ,0x04)
__define(FREAD_MASK     ,0x02)
__define(ACTIVE_MASK    ,0x01)
__define(ASLP_RATE_MASK ,0xC0)
__define(DR_MASK        ,0x38)

# ACTIVE                (ACTIVE_MASK)
STANDBY               = 0x00

# FXOS_CTRL_REG2 System Control 2 Register
FXOS_CTRL_REG2        = 0x2B

__define(ST_MASK        ,0x80)
__define(RST_MASK       ,0x40)
__define(SMODS1_MASK    ,0x10)
__define(SMODS0_MASK    ,0x08)
__define(SLPE_MASK      ,0x04)
__define(MODS1_MASK     ,0x02)
__define(MODS0_MASK     ,0x01)
__define(SMODS_MASK     ,0x18)
__define(MODS_MASK      ,0x03)

# FXOS_CTRL_REG3 Interrupt Control Register
FXOS_CTRL_REG3        = 0x2C

__define(FIFO_GATE_MASK  ,0x80)     # MMA8451 only
__define(WAKE_TRANS_MASK ,0x40)
__define(WAKE_LNDPRT_MASK,0x20)
__define(WAKE_PULSE_MASK ,0x10)
__define(WAKE_FF_MT_MASK ,0x08)
__define(IPOL_MASK       ,0x02)
__define(PP_OD_MASK      ,0x01)

# FXOS_CTRL_REG4 Interrupt Enable Register
FXOS_CTRL_REG4        = 0x2D

__define(INT_EN_ASLP_MASK  ,0x80)
__define(INT_EN_FIFO_MASK  ,0x40)     # MMA8451 only
__define(INT_EN_TRANS_MASK ,0x20)
__define(INT_EN_LNDPRT_MASK,0x10)
__define(INT_EN_PULSE_MASK ,0x08)
__define(INT_EN_FF_MT_MASK ,0x04)
__define(INT_EN_DRDY_MASK  ,0x01)

# FXOS_CTRL_REG5 Interrupt Configuration Register
FXOS_CTRL_REG5        = 0x2E

__define(INT_CFG_ASLP_MASK  ,0x80)
__define(INT_CFG_FIFO_MASK  ,0x40)
__define(INT_CFG_TRANS_MASK ,0x20)
__define(INT_CFG_LNDPRT_MASK,0x10)
__define(INT_CFG_PULSE_MASK ,0x08)
__define(INT_CFG_FF_MT_MASK ,0x04)
__define(INT_CFG_DRDY_MASK  ,0x01)

# XYZ Offset Correction Registers
FXOS_OFF_X_REG             = 0x2F
FXOS_OFF_Y_REG             = 0x30
FXOS_OFF_Z_REG             = 0x31

# M_DR_STATUS Register
M_DR_STATUS_REG       = 0x32

# MAG XYZ Data Registers
M_OUT_X_MSB_REG       = 0x33
M_OUT_X_LSB_REG       = 0x34
M_OUT_Y_MSB_REG       = 0x35
M_OUT_Y_LSB_REG       = 0x36
M_OUT_Z_MSB_REG       = 0x37
M_OUT_Z_LSB_REG       = 0x38

# MAG CMP Data Registers
CMP_X_MSB_REG         = 0x39
CMP_X_LSB_REG         = 0x3A
CMP_Y_MSB_REG         = 0x3B
CMP_Y_LSB_REG         = 0x3C
CMP_Z_MSB_REG         = 0x3D
CMP_Z_LSB_REG         = 0x3E

# MAG XYZ Offset Correction Registers
M_OFF_X_MSB_REG       = 0x3F
M_OFF_X_LSB_REG       = 0x40
M_OFF_Y_MSB_REG       = 0x41
M_OFF_Y_LSB_REG       = 0x42
M_OFF_Z_MSB_REG       = 0x43
M_OFF_Z_LSB_REG       = 0x44

# MAG MAX XYZ Registers
MAX_X_MSB_REG         = 0x45
MAX_X_LSB_REG         = 0x46
MAX_Y_MSB_REG         = 0x47
MAX_Y_LSB_REG         = 0x48
MAX_Z_MSB_REG         = 0x49
MAX_Z_LSB_REG         = 0x4A

# MAG MIN XYZ Registers
MIN_X_MSB_REG         = 0x4B
MIN_X_LSB_REG         = 0x4C
MIN_Y_MSB_REG         = 0x4D
MIN_Y_LSB_REG         = 0x4E
MIN_Z_MSB_REG         = 0x4F
MIN_Z_LSB_REG         = 0x50

# TEMP Registers
TEMP_REG              = 0x51

# M_THS CONFIG Registers
M_THS_CFG_REG         = 0x52

# M_THS SRC Registers
M_THS_SRC_REG         = 0x53

# MAG THRESHOLD XYZ Registers
M_THS_X_MSB_REG       = 0x54
M_THS_X_LSB_REG       = 0x55
M_THS_Y_MSB_REG       = 0x56
M_THS_Y_LSB_REG       = 0x57
M_THS_Z_MSB_REG       = 0x58
M_THS_Z_LSB_REG       = 0x59

# M_THS COUNT Registers
M_THS_COUNT           = 0x5A

# MAG FXOS_CTRL_REG1 System Control 1 Register
FXOS_M_CTRL_REG1      = 0x5B

__define(M_ACAL_MASK,0x80)
__define(M_RST_MASK ,0x40)
__define(M_OST_MASK ,0x20)
__define(M_OSR2_MASK,0x10)
__define(M_OSR1_MASK,0x08)
__define(M_OSR0_MASK,0x04)
__define(M_HMS1_MASK,0x02)
__define(M_HMS0_MASK,0x01)
__define(M_OSR_MASK,0x1C)
__define(M_HMS_MASK,0x03)

# MAG FXOS_CTRL_REG2 System Control 2 Register
FXOS_M_CTRL_REG2      = 0x5C

__define(M_HYB_AUTOINC_MASK   ,0x20)
__define(M_MAXMIN_DIS_MASK    ,0x10)
__define(M_MAXMIN_DIS_THS_MASK,0x08)
__define(M_MAXMIN_RST_MASK    ,0x04)
__define(M_RST_CNT1_MASK      ,0x02)
__define(M_RST_CNT0_MASK      ,0x01)

# MAG FXOS_CTRL_REG3 System Control 3 Register
FXOS_M_CTRL_REG3      = 0x5D

__define(M_RAW_MASK       ,0x80)
__define(M_ASLP_OS_2_MASK ,0x40)
__define(M_ASLP_OS_1_MASK ,0x20)
__define(M_ASLP_OS_0_MASK ,0x10)
__define(M_THS_XYZ_MASK   ,0x08)
__define(M_ST_Z_MASK      ,0x04)
__define(M_ST_XY1_MASK    ,0x02)
__define(M_ST_XY0_MASK    ,0x01)
__define(M_ASLP_OSR_MASK  ,0x70)
__define(M_ST_XY_MASK     ,0x03)

# MAG INT SOURCE Register
M_INT_SOURCE          = 0x5E

# ACCEL VECTOR CONFIG Register
A_VECM_CFG            = 0x5F

# ACCEL VECTOR THS MSB AND LSB Register
A_VECM_THS_MSB        = 0x60

A_VECM_THS_LSB        = 0x61

# ACCEL VECTOR CNT Register
A_VECM_CNT            = 0x62

# ACCEL INITIAL XYZ VECTORS Register
A_VECM_INITX_MSB      = 0x63
A_VECM_INITX_LSB      = 0x64
A_VECM_INITY_MSB      = 0x65
A_VECM_INITY_LSB      = 0x66
A_VECM_INITZ_MSB      = 0x67
A_VECM_INITZ_LSB      = 0x68

# MAG VECTOR CONFIG Register
M_VECM_CFG            = 0x69

# MAG VECTOR THS MSB AND LSB Register
M_VECM_THS_MSB        = 0x6A

M_VECM_THS_LSB        = 0x6B

# MAG VECTOR CNT Register
M_VECM_CNT            = 0x6C

# MAG INITIAL XYZ VECTORS Register
M_VECM_INITX_MSB      = 0x6D
M_VECM_INITX_LSB      = 0x6E
M_VECM_INITY_MSB      = 0x6F
M_VECM_INITY_LSB      = 0x70
M_VECM_INITZ_MSB      = 0x71
M_VECM_INITZ_LSB      = 0x72

# ACCEL FFMT THS X MSB AND LSB Register
A_FFMT_THS_X_MSB       = 0x73

A_FFMT_THS_X_LSB       = 0x74

# ACCEL FFMT THS Y MSB AND LSB Register
A_FFMT_THS_Y_MSB       = 0x75

A_FFMT_THS_Y_LSB       = 0x76

# ACCEL FFMT THS Z MSB AND LSB Register
A_FFMT_THS_Z_MSB       = 0x77

A_FFMT_THS_Z_LSB       = 0x78

# ACCEL TRANSIENT INIT Register
A_TRAN_INIT_XYZ_MSB    = 0x79
A_TRAN_INIT_X_LSB      = 0x7A
A_TRAN_INIT_Y_LSB      = 0x7B
A_TRAN_INIT_Z_LSB      = 0x7C

# FXOS8700 output logic data rate selection
ODR0 = (0 << FXOS_ODR_SHIFT)
ODR1 = (1 << FXOS_ODR_SHIFT)
ODR2 = (2 << FXOS_ODR_SHIFT)
ODR3 = (3 << FXOS_ODR_SHIFT)
ODR4 = (4 << FXOS_ODR_SHIFT)
ODR5 = (5 << FXOS_ODR_SHIFT)
ODR6 = (6 << FXOS_ODR_SHIFT)
ODR7 = (7 << FXOS_ODR_SHIFT)

# FXOS8700 oversample ratio selection
OSR0 = (0 << FXOS_OSR_SHIFT)
OSR1 = (1 << FXOS_OSR_SHIFT)
OSR2 = (2 << FXOS_OSR_SHIFT)
OSR3 = (3 << FXOS_OSR_SHIFT)
OSR4 = (4 << FXOS_OSR_SHIFT)
OSR5 = (5 << FXOS_OSR_SHIFT)
OSR6 = (6 << FXOS_OSR_SHIFT)
OSR7 = (7 << FXOS_OSR_SHIFT)

# FXOS8700 accelerometer/magnetometer selection
ACCONLY = (0 << FXOS_MODE_SHIFT)  # accelerometer only
MAGONLY = (1 << FXOS_MODE_SHIFT)  # magnetometer only
ACCMAG = (3 << FXOS_MODE_SHIFT)  # select both accelerometer and magnetometer

# FXOS8700 accelerometer full-scale range
RANGE2G = (0 << FXOS_ACCRANGE_SHIFT)  # 2g mode
RANGE4G = (1 << FXOS_ACCRANGE_SHIFT)  # 4g mode
RANGE8G = (2 << FXOS_ACCRANGE_SHIFT)  # 8g mode

ACC_MAG_SENSE_COEFF = [0.244, 0.488, 0.976, 0.1]
#ACC_REVERSE_SENSITIVITY = [4096, 2048, 1024]


class FXOS8700CQ(i2c.I2C):
    """

.. class:: FXOS8700CQ(i2cdrv, addr=0x1E, clk=400000)

    Creates an intance of a new FXOS8700CQ.

    :param i2cdrv: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x1E
    :param clk: Clock speed, default 400kHz

    Example: ::

        from nxp.fxos8700cq import fxos8700cq

        ...

        fxos = fxos8700cq.FXOS8700CQ(I2C0)
        fxos.start()
        fxos.init()
        acc = fxos.get_acc()
        mag = fxos.get_mag()

    """

    def __init__(self, i2cdrv, addr=FXOS_I2C_ADDRESS, clk=400000):
        i2c.I2C.__init__(self, i2cdrv, addr, clk)
        self._addr = addr

    def init(self, mode=ACCMAG, odr=ODR0, osr=OSR0, range=RANGE4G):
        """

.. method:: init(mode=ACCMAG, odr=0, osr=0, range=RANGE4G)

        Initialize the FXOS8700CQ setting the operating mode, the output data rate , the full-scale range and the oversample ratio.

        :param mode: select the operating mode (allowed values: ACCONLY for accelerometer only, MAGONLY for magnetometer only, ACCMAG for both active), default ACCMAG
        :param odr: set the output data rate (from 0 to 7 - refer to page 44 of the FXOS8700CQ datasheet), default 0
        :param osr: set the over sample ratio for magnetometer (from 0 to 7 - refer to page 97 of the FXOS8700CQ datasheet), default 0
        :param range: accelerometer full-scale range (allowed values RANGE2G, RANGE4G, RANGE8G), default RANGE4G

=== ============ ================== =========== =================
ODR ACC/MAG Mode Data Ready ACC/MAG Hybrid Mode Data Ready Hybrid
=== ============ ================== =========== =================
0     800 Hz           1.25 ms      400 Hz              2.5 ms          
1     400 Hz           2.5 ms       200 Hz              5 ms          
2     200 Hz           5 ms         100 Hz              10 ms
3     100 Hz           10 ms        50 Hz               20 ms          
4     50 Hz            20 ms        25 Hz               80 ms
5     12.5 Hz          80 ms        6.25 Hz             160 ms
6     6.25 Hz          160 ms       3.125 Hz            320 ms
7     1.56 Hz          640 ms       0.7813              1280 ms
=== ============ ================== =========== =================

======= ===== ===== ===== ===== ===== ===== ===== =====
ODR     OSR=0 OSR=1 OSR=2 OSR=3 OSR=4 OSR=5 OSR=6 OSR=7
======= ===== ===== ===== ===== ===== ===== ===== =====
1.56 Hz  16    16    32    64    128   256   512   1024
6.25 Hz  4     4     8     16    32    64    128   256
12.5 Hz  2     2     4     8     16    32    64    128
50 Hz    2     2     2     2     4     8     16    32
100 Hz   2     2     2     2     2     4     8     16
200 Hz   2     2     2     2     2     2     4     8
400 Hz   2     2     2     2     2     2     2     4
800 Hz   2     2     2     2     2     2     2     2
======= ===== ===== ===== ===== ===== ===== ===== =====
        """
        self.mode = mode
        self.odr = odr
        self.osr = osr
        self.range = range
        self._standby()
        self._set_osr()
        self._set_mode()
        self._set_range()
        self._set_odr()
        self._active()

    def _modify_reg(self, reg, mask, val):
        r_val = self.write_read(reg, 1)[0]
        r_val = r_val & ~mask if (mask != 0) else r_val
        r_val = r_val | val if (val != 0) else r_val
        self.write_bytes(reg, r_val)

    def _standby(self):
        self._modify_reg(FXOS_CTRL_REG1, ACTIVE_MASK, 0)

    def _set_osr(self):
        if self.mode > ACCONLY:
            self.write_bytes(FXOS_M_CTRL_REG1, M_RST_MASK)
            self._modify_reg(FXOS_M_CTRL_REG1, M_OSR_MASK, self.osr)

    def _set_mode(self):
        self._modify_reg(FXOS_M_CTRL_REG1, M_HMS_MASK, self.mode)
        # enable hybrid mode auto increment
        if self.mode == ACCMAG:
            self._modify_reg(FXOS_M_CTRL_REG2, 0, M_HYB_AUTOINC_MASK)

    def _set_range(self):
        self._modify_reg(XYZ_DATA_CFG_REG, FS_MASK, self.range)

    def _set_odr(self):
        self._modify_reg(FXOS_CTRL_REG1, DR_MASK, self.odr)
        # apply low noise filter
        if (self.range <= RANGE4G):
            self._modify_reg(FXOS_CTRL_REG1, 0, LNOISE_MASK)

    def _active(self):
        self._modify_reg(FXOS_CTRL_REG1, 0, ACTIVE_MASK)

    def get_raw_acc(self):
        """

.. method:: get_raw_acc()

        Retrieves the current accelerometer data as a tuple of X, Y, Z, raw values 

        Returns [ax, ay, az]

        """
        data = self.write_read(OUT_X_MSB_REG, 6)
        ax = (data[0] << 8 | data[1])
        ay = (data[2] << 8 | data[3])
        az = (data[4] << 8 | data[5])
        return [ax, ay, az]

    def get_raw_mag(self):
        """

.. method:: get_raw_mag()

        Retrieves the current magnetometer data as a tuple of X, Y, Z, raw values 

        Returns [mx, my, mz]

        """
        data = self.write_read(M_OUT_X_MSB_REG, 6)
        mx = (data[0] << 8 | data[1])
        my = (data[2] << 8 | data[3])
        mz = (data[4] << 8 | data[5])
        return [mx, my, mz]

    def get_raw_int_temp(self):
        """

.. method:: get_raw_int_temp()

        Retrieves the current internal temperature data as raw values 

        Returns raw_t

        """
        raw_t = self.write_read(TEMP_REG, 1)[0]
        return raw_t

    def get_acc(self, axis=None):
        """

.. method:: get_acc(axis=None)

        Retrieves the current accelerometer data in m/s^2 as a tuple of X, Y, Z values or single axis value if axis argument is provided.

        :param axis: select the axis (allowed values: "x" for x-axis, "y" for y-axis, "z" for z-axis); default None for all values

        Returns [acc_x, acc_y, acc_z] or acc_x or acc_y or acc_z

        """
        raw_acc = self.get_raw_acc()
        if raw_acc[0] >= 32768:
            raw_acc[0] -= 65536
        if raw_acc[1] >= 32768:
            raw_acc[1] -= 65536
        if raw_acc[2] >= 32768:
            raw_acc[2] -= 65536
        acc = [(((a * ACC_MAG_SENSE_COEFF[self.range])/4000)*9.81) for a in raw_acc]
        if axis in ("x", "X"):
            return acc[0]
        elif axis in ("y", "Y"):
            return acc[1]
        elif axis in ("z", "Z"):
            return acc[2]
        else:
            return acc

    def get_mag(self, axis=None):
        """

.. method:: get_mag(axis=None)

        Retrieves the current magnetometer data in uT as a tuple of X, Y, Z values or single axis value if axis argument is provided.

        :param axis: select the axis (allowed values: "x" for x-axis, "y" for y-axis, "z" for z-axis); default None for all values

        Returns [mag_x, mag_y, mag_z] or mag_x or mag_y or mag_z

        """
        raw_mag = self.get_raw_mag()
        if raw_mag[0] >= 32768:
            raw_mag[0] -= 65536
        if raw_mag[1] >= 32768:
            raw_mag[1] -= 65536
        if raw_mag[2] >= 32768:
            raw_mag[2] -= 65536
        # convert in uT unit
        mag = [m * ACC_MAG_SENSE_COEFF[3] for m in raw_mag]
        if axis in ("x", "X"):
            return mag[0]
        elif axis in ("y", "Y"):
            return mag[1]
        elif axis in ("z", "Z"):
            return mag[2]
        else:
            return mag

    def get_int_temp(self, unit=None):
        """

.. method:: get_int_temp(unit="C")

        Retrieves the current device internal temperature value in Celtius, Kelvin or Fahrenheit degrees.

        :param unit: select the unit of measure for internal temperature (allowed values: "C" for Celtius degrees, "K" for Kelvin degrees, "F" Fahrenheit degrees); default "C"

        Returns int_temp

        """
        raw_t = self.get_raw_int_temp()
        if raw_t >= 128:
            temp_cels = (raw_t - 256) * 0.96
        else:
            temp_cels = raw_t * 0.96

        if unit in ("c", "C"):
            return temp_cels
        elif unit in ("k", "K"):
            temp_kelv = temp_cels + 273.15
            return temp_kelv
        elif unit in ("f", "F"):
            temp_fahr = (temp_cels * 1.8) + 32
            return temp_fahr
        else:
            return None

