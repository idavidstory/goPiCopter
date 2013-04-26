package sensors

import (
	"errors"
	"goPiCopter/i2c"
)

/**
* The LSM303ACCEL is a triple-axis Accelerometer.
* The LSM303 has two devices on it, this code accesses the Accelerometer.
**/
const (
	LSM303ACCEL_ADDR          = 0x19
	LSM303ACCEL_GRAVITY_EARTH = 9.80665 // Earth's gravity in m/s^2
	LSM303ACCEL_Accel_MG_LSB  = 0.001   // 1, 2, 4 or 12 mg per lsb

	LSM303ACCEL_CTRL_REG1     = 0x20 // 00000111   rw
	LSM303ACCEL_CTRL_REG2     = 0x21 // 00000000   rw
	LSM303ACCEL_CTRL_REG3     = 0x22 // 00000000   rw
	LSM303ACCEL_CTRL_REG4     = 0x23 // 00000000   rw
	LSM303ACCEL_CTRL_REG5     = 0x24 // 00000000   rw
	LSM303ACCEL_CTRL_REG6     = 0x25 // 00000000   rw
	LSM303ACCEL_REFERENCE     = 0x26 // 00000000   r
	LSM303ACCEL_STATUS_REG    = 0x27 // 00000000   r
	LSM303ACCEL_OUT_X_L       = 0x28
	LSM303ACCEL_OUT_X_H       = 0x29
	LSM303ACCEL_OUT_Y_L       = 0x2A
	LSM303ACCEL_OUT_Y_H       = 0x2B
	LSM303ACCEL_OUT_Z_L       = 0x2C
	LSM303ACCEL_OUT_Z_H       = 0x2D
	LSM303ACCEL_FIFO_CTRL_REG = 0x2E
	LSM303ACCEL_FIFO_SRC_REG  = 0x2F
	LSM303ACCEL_INT1_CFG      = 0x30
	LSM303ACCEL_INT1_SOURCE   = 0x31
	LSM303ACCEL_INT1_THS      = 0x32
	LSM303ACCEL_INT1_DURATION = 0x33
	LSM303ACCEL_INT2_CFG      = 0x34
	LSM303ACCEL_INT2_SOURCE   = 0x35
	LSM303ACCEL_INT2_THS      = 0x36
	LSM303ACCEL_INT2_DURATION = 0x37
	LSM303ACCEL_CLICK_CFG     = 0x38
	LSM303ACCEL_CLICK_SRC     = 0x39
	LSM303ACCEL_CLICK_THS     = 0x3A
	LSM303ACCEL_TIME_LIMIT    = 0x3B
	LSM303ACCEL_TIME_LATENCY  = 0x3C
	LSM303ACCEL_TIME_WINDOW   = 0x3D
)

type LSM303ACCEL struct {
	bus       *i2c.I2CBus
	biasX     float32
	biasY     float32
	biasZ     float32
	sampleX   int
	sampleY   int
	sampleZ   int
	sampleCnt int
}

// Return a new Device
func NewLSM303ACCEL() (bp *LSM303ACCEL, err error) {
	bp = new(LSM303ACCEL)
	bp.bus, err = i2c.Bus(1)
	// Turn it on, enable all 3 axis
	err = bp.bus.WriteByte(LSM303ACCEL_ADDR, LSM303ACCEL_CTRL_REG1, 0x27)
	return
}

// Read a byte from the specified register
func (bp *LSM303ACCEL) ReadRegister(reg byte) (value int8, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(LSM303ACCEL_ADDR, reg, 1)
	if err == nil {
		value = int8(bytes[0])
	}
	return
}

// Write a byte to the specified register
func (bp *LSM303ACCEL) WriteRegister(reg byte, data byte) (err error) {
	err = bp.bus.WriteByte(LSM303ACCEL_ADDR, reg, data)
	return
}

// Read the raw x, y, z values from their registers
func (bp *LSM303ACCEL) ReadRaw() (x, y, z int16, err error) {
	var bytes []byte
	bytes, err = bp.bus.ReadByteBlock(LSM303ACCEL_ADDR, LSM303ACCEL_OUT_X_L|0x80, 6)
	if err == nil {
		// Extract the values
		x = int16(uint16(bytes[0]) | (uint16(bytes[1]) << 8))
		y = int16(uint16(bytes[2]) | (uint16(bytes[3]) << 8))
		z = int16(uint16(bytes[4]) | (uint16(bytes[5]) << 8))
	}
	return
}

// Return adjusted x, y, z values
func (bp *LSM303ACCEL) ReadXYZ() (x, y, z float32, err error) {
	var (
		xi, yi, zi int16
	)
	xi, yi, zi, err = bp.ReadRaw()
	if err == nil {
		x = float32(xi) - bp.biasX
		y = float32(yi) - bp.biasY
		z = float32(zi) - bp.biasZ
	}
	return
}

// Take a sample
func (bp *LSM303ACCEL) Measure() {
	var (
		err     error
		x, y, z int16
	)
	x, y, z, err = bp.ReadRaw()
	if err == nil {
		bp.sampleX += int(x)
		bp.sampleY += int(y)
		bp.sampleZ += int(z)
		bp.sampleCnt++
	}
}

// Evaluate the samples
func (bp *LSM303ACCEL) Evaluate() (x, y, z float32, err error) {
	if bp.sampleCnt > 0 {
		x = float32(bp.sampleX/bp.sampleCnt) - bp.biasX
		y = float32(bp.sampleY/bp.sampleCnt) - bp.biasY
		z = float32(bp.sampleZ/bp.sampleCnt) - bp.biasZ
		bp.sampleCnt, bp.sampleX, bp.sampleY, bp.sampleZ = 0, 0, 0, 0
	} else {
		err = errors.New("No accelerometer samples to Evaluate")
	}
	return
}

// Compute bias from samples
func (bp *LSM303ACCEL) ComputeBias() (err error) {
	if bp.sampleCnt > 0 {
		bp.biasX = float32(bp.sampleX / bp.sampleCnt)
		bp.biasY = float32(bp.sampleY / bp.sampleCnt)
		bp.biasZ = float32(bp.sampleZ / bp.sampleCnt)
		bp.sampleCnt, bp.sampleX, bp.sampleY, bp.sampleZ = 0, 0, 0, 0
	} else {
		err = errors.New("No accelerometer samples to ComputeBias")
	}
	return
}
