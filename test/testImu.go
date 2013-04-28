package main

import (
	"fmt"
	"goPiCopter/imus"
	"goPiCopter/io/sensors"
	"math"
	"time"
)

/**
* This test program reads data from the three sensors
* gyroscope, accelerometer, and magnetometer, runs it
* through the filter, and prints the computed
* Yaw, Pitch, and Roll every 100 iterations.
* Each iteration takes approximately 3 milliseconds
* on a RaspberryPi model B.
**/
func main() {
	const (
		d2r = math.Pi / 180.0 // Used to convert degrees to radians
		r2d = 180.0 / math.Pi // Used to convert radians to degrees
	)
	var (
		gyroscope     *sensors.L3GD20      = nil
		accelerometer *sensors.LSM303ACCEL = nil
		magnetometer  *sensors.LSM303MAG   = nil
		imu           *imus.ImuMayhony     = nil

		err              error
		dbgPrint         bool
		gx, gy, gz       float32 // Gyroscope rate in degrees
		ax, ay, az       float32 // Accelerometer data
		mx, my, mz       float32 // Magnetometer data
		yaw, pitch, roll float32 // IMU results in radians
	)

	gyroscope, err = sensors.NewL3GD20()
	if err != nil {
		fmt.Printf("Error: getting device L3GD20, err=%v\n", err)
		return
	}

	accelerometer, err = sensors.NewLSM303ACCEL()
	if err != nil {
		fmt.Printf("Error: getting device LSM303ACCEL, err=%v\n", err)
		return
	}

	magnetometer, err = sensors.NewLSM303MAG()
	if err != nil {
		fmt.Printf("Error: getting device LSM303MAG, err=%v\n", err)
		return
	}

	imu = imus.NewImuMayhony()

	maxIterations := 10000
	for {
		startTime := time.Now().UnixNano()
		for i := 0; i < maxIterations; i++ {
			dbgPrint = (i % 100) == 0
			now := time.Now().UnixNano()
			gx, gy, gz, err = gyroscope.ReadXYZ()
			if err == nil {
				ax, ay, az, err = accelerometer.ReadXYZ()
				if err == nil {
					mx, my, mz, err = magnetometer.ReadXYZ()
					if err == nil {
						yaw, pitch, roll = imu.Update(now, gx*d2r, gy*d2r, gz*d2r, ax, ay, az, mx, my, mz)
						if dbgPrint {
							fmt.Printf("  YPR(%10.5f, %10.5f, %10.5f)\n", yaw*r2d, pitch*r2d, roll*r2d)
						}
					} else {
						fmt.Printf("Error: reading magnetometer, err=%v\n", err)
					}
				} else {
					fmt.Printf("Error: reading accelerometer, err=%v\n", err)
				}
			} else {
				fmt.Printf("Error: reading gyroscope, err=%v\n", err)
			}
		}
		finishTime := time.Now().UnixNano()
		delta := finishTime - startTime
		fmt.Printf("%4.2f milliseconds per iteration\n", (float64(delta)/float64(maxIterations))/1000000.0)
	}
}
