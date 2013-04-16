package sensors

import (
	"goPiCopter/i2c"
)

/**
* The L3GD20 is a triple-axis Gyroscope.
**/
const (
	L3GD20_ADDR = 0x6B
	L3GD20_ID   = 0xD4

	L3GD20_WHO_AM_I      = 0x0F
	L3GD20_CTRL_REG1     = 0x20
	L3GD20_CTRL_REG2     = 0x21
	L3GD20_CTRL_REG3     = 0x22
	L3GD20_CTRL_REG4     = 0x23
	L3GD20_CTRL_REG5     = 0x24
	L3GD20_REFERENCE     = 0x25
	L3GD20_OUT_TEMP      = 0x26
	L3GD20_STATUS_REG    = 0x27
	L3GD20_OUT_X_L       = 0x28
	L3GD20_OUT_X_H       = 0x29
	L3GD20_OUT_Y_L       = 0x2A
	L3GD20_OUT_Y_H       = 0x2B
	L3GD20_OUT_Z_L       = 0x2C
	L3GD20_OUT_Z_H       = 0x2D
	L3GD20_FIFO_CTRL_REG = 0x2E
	L3GD20_FIFO_SRC_REG  = 0x2F
	L3GD20_INT1_CFG      = 0x30
	L3GD20_INT1_SRC      = 0x31
	L3GD20_INT1_TSH_XH   = 0x32
	L3GD20_INT1_TSH_XL   = 0x33
	L3GD20_INT1_TSH_YH   = 0x34
	L3GD20_INT1_TSH_YL   = 0x35
	L3GD20_INT1_TSH_ZH   = 0x36
	L3GD20_INT1_TSH_ZL   = 0x37
	L3GD20_INT1_DURATION = 0x38

	L3GD20_RANGE_250DPS  = 0x00
	L3GD20_RANGE_500DPS  = 0x01
	L3GD20_RANGE_2000DPS = 0x02

	L3GD20_SENSITIVITY_250DPS  = 0.00875     // Roughly 22/256 for fixed point match
	L3GD20_SENSITIVITY_500DPS  = 0.0175      // Roughly 45/256
	L3GD20_SENSITIVITY_2000DPS = 0.070       // Roughly 18/256
	L3GD20_DPS_TO_RADS         = 0.017453293 // degress/s to rad/s multiplier
)

type L3GD20 struct {
	bus      *i2c.I2CBus
	dpsRange byte
}

// Return a new Device
func NewL3GD20() (bp *L3GD20, err error) {
	bp = new(L3GD20)
	bp.bus, err = i2c.Bus(1)
	bp.dpsRange = L3GD20_RANGE_250DPS
	// Turn it on, enable all 3 axis
	err = bp.bus.WriteByte(L3GD20_ADDR, L3GD20_CTRL_REG1, 0x0F)
	return
}

// Read a byte from the specified register
func (bp *L3GD20) ReadRegister(reg byte) (value int8, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(L3GD20_ADDR, reg, 1)
	if err == nil {
		value = int8(bytes[0])
	}
	return
}

// Write a byte to the specified register
func (bp *L3GD20) WriteRegister(reg byte, data byte) (err error) {
	err = bp.bus.WriteByte(L3GD20_ADDR, reg, data)
	if err == nil && reg == L3GD20_CTRL_REG4 {
		bp.dpsRange = (data >> 4) & 0x03
	}
	return
}

// Read the temperature
func (bp *L3GD20) ReadTemperature() (deg int8, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(L3GD20_ADDR, L3GD20_OUT_TEMP, 1)
	if err == nil {
		deg = int8(bytes[0])
	}
	return
}

// Read the X, Y, and Z values from their registers,
// and adjust them according to the current sensitivity
func (bp *L3GD20) ReadXYZ() (x, y, z float32, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(L3GD20_ADDR, L3GD20_OUT_X_L|0x80, 6)
	if err == nil {
		// Extract the values
		xi := int16(uint16(bytes[0]) | (uint16(bytes[1]) << 8))
		yi := int16(uint16(bytes[2]) | (uint16(bytes[3]) << 8))
		zi := int16(uint16(bytes[4]) | (uint16(bytes[5]) << 8))
		// Compensate values depending on the sensitivity
		switch bp.dpsRange {
		case L3GD20_RANGE_250DPS:
			x = float32(xi) * L3GD20_SENSITIVITY_250DPS
			y = float32(yi) * L3GD20_SENSITIVITY_250DPS
			z = float32(zi) * L3GD20_SENSITIVITY_250DPS
		case L3GD20_RANGE_500DPS:
			x = float32(xi) * L3GD20_SENSITIVITY_500DPS
			y = float32(yi) * L3GD20_SENSITIVITY_500DPS
			z = float32(zi) * L3GD20_SENSITIVITY_500DPS
		case L3GD20_RANGE_2000DPS:
			x = float32(xi) * L3GD20_SENSITIVITY_2000DPS
			y = float32(yi) * L3GD20_SENSITIVITY_2000DPS
			z = float32(zi) * L3GD20_SENSITIVITY_2000DPS
		}
	}
	return
}
