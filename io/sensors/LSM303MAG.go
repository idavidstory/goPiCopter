package sensors

import (
	"goPiCopter/io/sensors/i2c"
)

/**
* The LSM303MAG is a triple-axis Magnetometer (compass).
* The LSM303 has two devices on it, this code accesses the Magnetometer.
**/
const (
	LSM303MAG_ADDR                        = 0x1E
	LSM303MAG_SENSORS_GAUSS_TO_MICROTESLA = 100

	LSM303MAG_CRA_REG    = 0x00
	LSM303MAG_CRB_REG    = 0x01
	LSM303MAG_MR_REG     = 0x02
	LSM303MAG_OUT_X_H    = 0x03
	LSM303MAG_OUT_X_L    = 0x04
	LSM303MAG_OUT_Z_H    = 0x05
	LSM303MAG_OUT_Z_L    = 0x06
	LSM303MAG_OUT_Y_H    = 0x07
	LSM303MAG_OUT_Y_L    = 0x08
	LSM303MAG_SR_REGg    = 0x09
	LSM303MAG_IRA_REG    = 0x0A
	LSM303MAG_IRB_REG    = 0x0B
	LSM303MAG_IRC_REG    = 0x0C
	LSM303MAG_TEMP_OUT_H = 0x31
	LSM303MAG_TEMP_OUT_L = 0x32

	LSM303MAG_GAIN_1_3 = 0x20 // +/- 1.3
	LSM303MAG_GAIN_1_9 = 0x40 // +/- 1.9
	LSM303MAG_GAIN_2_5 = 0x60 // +/- 2.5
	LSM303MAG_GAIN_4_0 = 0x80 // +/- 4.0
	LSM303MAG_GAIN_4_7 = 0xA0 // +/- 4.7
	LSM303MAG_GAIN_5_6 = 0xC0 // +/- 5.6
	LSM303MAG_GAIN_8_1 = 0xE0 // +/- 8.1
)

type LSM303MAG struct {
	bus          *i2c.I2CBus
	gain         byte
	gauss_lsb_xy float32
	gauss_lsb_z  float32
}

// Return a new Device
func NewLSM303MAG() (bp *LSM303MAG, err error) {
	bp = new(LSM303MAG)
	bp.bus, err = i2c.Bus(1)
	if err == nil {
		// Turn it on
		err = bp.bus.WriteByte(LSM303MAG_ADDR, LSM303MAG_MR_REG, 0x00)
		if err == nil {
			// Set the gain to a known level
			err = bp.SetGain(LSM303MAG_GAIN_1_3)
		}
	}
	return
}

// Read a byte from the specified register
func (bp *LSM303MAG) ReadRegister(reg byte) (value int8, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(LSM303MAG_ADDR, reg, 1)
	if err == nil {
		value = int8(bytes[0])
	}
	return
}

// Write a byte to the specified register
func (bp *LSM303MAG) WriteRegister(reg byte, data byte) (err error) {
	err = bp.bus.WriteByte(LSM303MAG_ADDR, reg, data)
	return
}

// Set the magnetometer's gain
func (bp *LSM303MAG) SetGain(gain byte) (err error) {
	err = bp.WriteRegister(LSM303MAG_CRB_REG, gain)
	if err == nil {
		bp.gain = gain
		switch gain {
		case LSM303MAG_GAIN_1_3:
			bp.gauss_lsb_xy = 1100.0
			bp.gauss_lsb_z = 980.0
		case LSM303MAG_GAIN_1_9:
			bp.gauss_lsb_xy = 855.0
			bp.gauss_lsb_z = 760.0
		case LSM303MAG_GAIN_2_5:
			bp.gauss_lsb_xy = 670.0
			bp.gauss_lsb_z = 600.0
		case LSM303MAG_GAIN_4_0:
			bp.gauss_lsb_xy = 450.0
			bp.gauss_lsb_z = 400.0
		case LSM303MAG_GAIN_4_7:
			bp.gauss_lsb_xy = 400.0
			bp.gauss_lsb_z = 255.0
		case LSM303MAG_GAIN_5_6:
			bp.gauss_lsb_xy = 330.0
			bp.gauss_lsb_z = 295.0
		case LSM303MAG_GAIN_8_1:
			bp.gauss_lsb_xy = 230.0
			bp.gauss_lsb_z = 205.0
		}
	}
	return
}

// Read the raw x, y, z values from their registers
func (bp *LSM303MAG) ReadRaw() (x, y, z int16, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(LSM303MAG_ADDR, LSM303MAG_OUT_X_H|0x80, 6)
	if err == nil {
		// Extract the values (higb byte first) and x, z, y order
		x = int16(uint16(bytes[1]) | (uint16(bytes[0]) << 8))
		z = int16(uint16(bytes[3]) | (uint16(bytes[2]) << 8))
		y = int16(uint16(bytes[5]) | (uint16(bytes[4]) << 8))
	}
	return
}

// Read the X, Y, and Z values from their registers,
// and adjust them according to the magnetic gain setting
func (bp *LSM303MAG) ReadXYZ() (x, y, z float32, err error) {
	var (
		xi, yi, zi int16
	)
	xi, yi, zi, err = bp.ReadRaw()
	if err == nil {
		// Apply the gain
		x = float32(xi) / bp.gauss_lsb_xy * LSM303MAG_SENSORS_GAUSS_TO_MICROTESLA
		y = float32(yi) / bp.gauss_lsb_xy * LSM303MAG_SENSORS_GAUSS_TO_MICROTESLA
		z = float32(zi) / bp.gauss_lsb_z  * LSM303MAG_SENSORS_GAUSS_TO_MICROTESLA
	}
	return
}
