package i2c

import (
	"fmt"
	"os"
	"sync"
	"syscall"
	"unsafe"
)

const (
	// as defined in /usr/include/linux/i2c-dev.h
	I2C_SLAVE = 0x0703
	I2C_SMBUS = 0x0720
	// as defined in /usr/include/linux/i2c.h
	I2C_SMBUS_WRITE          = 0
	I2C_SMBUS_READ           = 1
	I2C_SMBUS_I2C_BLOCK_DATA = 8
	I2C_SMBUS_BLOCK_MAX      = 32
)

// as defined in /usr/include/linux/i2c-dev.h
type i2c_smbus_ioctl_data struct {
	readWrite byte
	command   byte
	size      uint32
	data      uintptr
}

var busMap map[byte]*I2CBus
var busMapLock sync.Mutex
var readBuffer []byte

type I2CBus struct {
	// i2c-dev file pointer
	file *os.File
	// last transmitted address, track here
	// so we don't have to redo the ioctl
	// call if the address hasn't changed since the
	// last access
	addr byte
	// simple bus access lock to ensure address
	// set and data writes occur atomically
	lock sync.Mutex
}

func init() {
	busMap = make(map[byte]*I2CBus)
	readBuffer = make([]byte, 256)
}

// Returns an instance to an I2CBus.  If we already have an I2CBus
// created for the requested bus number, just return that, otherwise
// set up a new one and open up its associated i2c-dev file
func Bus(bus byte) (i2cbus *I2CBus, err error) {
	busMapLock.Lock()
	defer busMapLock.Unlock()

	if i2cbus = busMap[bus]; i2cbus == nil {
		i2cbus = new(I2CBus)
		if i2cbus.file, err = os.OpenFile(fmt.Sprintf("/dev/i2c-%v", bus), os.O_RDWR, os.ModeExclusive); err != nil {
			busMap[bus] = i2cbus
		}
	}

	return
}

func (i2cbus *I2CBus) setAddress(addr byte) (err error) {
	if addr != i2cbus.addr {
		if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, i2cbus.file.Fd(), I2C_SLAVE, uintptr(addr)); errno != 0 {
			err = syscall.Errno(errno)
			return
		}

		i2cbus.addr = addr
	}

	return
}

func (i2cbus *I2CBus) WriteByte(addr, reg, value byte) (err error) {
	i2cbus.lock.Lock()
	defer i2cbus.lock.Unlock()

	if err = i2cbus.setAddress(addr); err != nil {
		return
	}

	// If we're only writing a single byte, we can just write it directly
	// to the i2c-dev file as a 2-tuple with the register
	var n int

	n, err = i2cbus.file.Write([]byte{reg, value})

	if n != 2 {
		err = fmt.Errorf("i2c: Unexpected number (%v) of bytes written in I2CBus.WriteByte", n)
	}

	return
}

func (i2cbus *I2CBus) ReadByteBlock(addr, reg byte, readLength byte) (data []byte, err error) {
	i2cbus.lock.Lock()
	defer i2cbus.lock.Unlock()

	if err = i2cbus.setAddress(addr); err != nil {
		return
	}

	readBuffer[0] = readLength
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL,
		i2cbus.file.Fd(), I2C_SMBUS, uintptr(unsafe.Pointer(&i2c_smbus_ioctl_data{
			readWrite: I2C_SMBUS_READ,
			command:   reg,
			size:      I2C_SMBUS_I2C_BLOCK_DATA,
			data:      uintptr(unsafe.Pointer(&readBuffer[0]))}))); errno != 0 {
		err = syscall.Errno(errno)
	}

	data = make([]byte, readLength)
	copy(data, readBuffer[1:])

	return
}

func (i2cbus *I2CBus) WriteByteBlock(addr, reg byte, list []byte) (err error) {
	i2cbus.lock.Lock()
	defer i2cbus.lock.Unlock()

	if err = i2cbus.setAddress(addr); err != nil {
		return
	}

	blockData := make([]byte, len(list)+1)
	blockData[0] = byte(len(list))
	copy(blockData[1:], list)

	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL,
		i2cbus.file.Fd(), I2C_SMBUS, uintptr(unsafe.Pointer(&i2c_smbus_ioctl_data{
			readWrite: I2C_SMBUS_WRITE,
			command:   reg,
			size:      I2C_SMBUS_I2C_BLOCK_DATA,
			data:      uintptr(unsafe.Pointer(&blockData[0]))}))); errno != 0 {
		err = syscall.Errno(errno)
	}

	return
}
