package main

import (
	"fmt"
	"goPiCopter/imus"
	"goPiCopter/io"
	"math"
	"time"
)

/**
* Remotely control a quadcopter using a RaspberryPi as
* the on board computer, reading the sensors and computing
* how much power to apply to the motors. The current implementation
* uses three sensors (gyroscope, accelerometer, and magnetometer).
**/
func main() {
	const (
		d2r = math.Pi / 180.0 // Used to convert degrees to radians
		r2d = 180.0 / math.Pi // Used to convert radians to degrees
	)
	var (
		imu *imus.ImuMayhony
		sData    io.SensorData
		cData    io.CmdData
		i        int // number of iterations in the for loop
		cnt      int // number of printf calls  (or ~seconds)
		now      int64
		lastTime int64 // last time a printf was called
		second   int64 // One second in nanoseconds
		yaw      float32
		pitch    float32
		roll     float32
	)

	imu = imus.NewImuMayhony()
	sensorChannel := make(chan io.SensorData)
	cmdChannel := make(chan io.CmdData)

	go io.ReadSensors(sensorChannel)

	go io.ReadCommands(cmdChannel)

	second = int64(time.Second)
	lastTime = time.Now().UnixNano()
	for {
		select {
		case sData = <-sensorChannel:
			yaw, pitch, roll = imu.Update(sData.When, sData.Gx*d2r, sData.Gy*d2r, sData.Gz*d2r, sData.Ax, sData.Ay, sData.Az, sData.Mx, sData.My, sData.Mz)
			i++
			now = time.Now().UnixNano()
			if (now - lastTime) >= second {
				cnt++
				fmt.Printf("%d YPR(%10.5f, %10.5f, %10.5f)\n", cnt, yaw*r2d, pitch*r2d, roll*r2d)
				lastTime = now
			}
		case cData = <-cmdChannel:
			fmt.Printf("%d CMD(%d, %d, %d, %d, %d, %d\n", cData.Yaw, cData.Pitch, cData.Roll, cData.Aux1, cData.Aux2)
		}
	}
}
