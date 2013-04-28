package io

import (
	"fmt"
	"goPiCopter/io/sensors"
	"time"
)

type Sensors struct {
}

type SensorData struct {
	When       int64
	Count      int     // number of samples taken
	Gx, Gy, Gz float32 // Gyroscope data
	Ax, Ay, Az float32 // Accelerometer data
	Mx, My, Mz float32 // Magnetometer data
}

/**
* Setup the sensors
**/
func setup() (gyroscope *sensors.L3GD20, accelerometer *sensors.LSM303ACCEL, magnetometer *sensors.LSM303MAG, err error) {
	fmt.Printf("Setup...\n")
	gyroscope, err = sensors.NewL3GD20()
	if err != nil {
		fmt.Printf("Error: getting device L3GD20, err=%v\n", err)
	} else {
		accelerometer, err = sensors.NewLSM303ACCEL()
		if err != nil {
			fmt.Printf("Error: getting device LSM303ACCEL, err=%v\n", err)
		} else {
			magnetometer, err = sensors.NewLSM303MAG()
			if err != nil {
				fmt.Printf("Error: getting device LSM303MAG, err=%v\n", err)
				//} else {
				//	err = calibrate(gyroscope, accelerometer)
				//	if err != nil {
				//		fmt.Printf("Error: %v\n", err)
				//	}
			}
		}
	}
	return
}

/**
* Calibrate the sensors
**/
func calibrate(gyroscope *sensors.L3GD20, accelerometer *sensors.LSM303ACCEL) (err error) {
	const (
		ITERATIONS int = 100
	)
	var (
		t time.Duration
		n int
	)
	fmt.Printf("Calibrating...\n")
	t = time.Millisecond * 100
	for n = 0; n < ITERATIONS; n++ {
		gyroscope.Measure()
		accelerometer.Measure()
		time.Sleep(t)
	}
	err = gyroscope.ComputeBias()
	if err == nil {
		err = accelerometer.ComputeBias()
	}
	return
}

/**
* Summarize the sensor data and send it off to be processed
**/
func sendSensorData(sensorChannel chan SensorData, when int64, gyroscope *sensors.L3GD20, accelerometer *sensors.LSM303ACCEL, mx, my, mz float32, count int) {
	var (
		err  error
		data SensorData
	)
	data.When = when
	data.Count = count
	data.Mx = mx
	data.My = my
	data.Mz = mz
	data.Gx, data.Gy, data.Gz, err = gyroscope.Evaluate()
	if err == nil {
		data.Ax, data.Ay, data.Az, err = accelerometer.Evaluate()
		if err == nil {
			sensorChannel <- data // send the data over the sensorChannel
		}
	}
}

/**
* Loop reading sensors, attempt to summarize at 50Hz
**/
func ReadSensors(sensorChannel chan SensorData) {
	var (
		count         int
		now           int64   // Current time in nanoseconds
		lastTime      int64   // Last 100Hz time in nanoseconds
		hz            int64   // Hz in nanoseconds
		magCount      int     // Read the Magnetometer every 5th summarize
		mx, my, mz    float32 // Magnetometer data
		err           error
		gyroscope     *sensors.L3GD20
		accelerometer *sensors.LSM303ACCEL
		magnetometer  *sensors.LSM303MAG
	)

	fmt.Printf("Allocating sensors...\n")
	gyroscope, accelerometer, magnetometer, err = setup()
	if err != nil {
		close(sensorChannel)
		return
	}

	fmt.Printf("Reading sensors...\n")
	hz = int64(time.Second/50) - 200000 // minus overhead to send sensor data

	lastTime = time.Now().UnixNano()
	mx, my, mz, err = magnetometer.ReadXYZ()
	if err != nil {
		fmt.Printf("readSensors: failed to read magnetometer, err=%v\n", err)
	}
	for {
		count++
		gyroscope.Measure()
		now = time.Now().UnixNano()
		if (now - lastTime) >= hz {
			sendSensorData(sensorChannel, now, gyroscope, accelerometer, mx, my, mz, count)
			lastTime = now
			count = 0
			magCount++
		}

		accelerometer.Measure()
		now = time.Now().UnixNano()
		if (now - lastTime) >= hz {
			sendSensorData(sensorChannel, now, gyroscope, accelerometer, mx, my, mz, count)
			lastTime = now
			count = 0
			magCount++
		}

		if magCount == 5 {
			magCount = 0
			mx, my, mz, err = magnetometer.ReadXYZ()
			if err != nil {
				fmt.Printf("readSensors: failed to read magnetometer, err=%v\n", err)
			}
		}
	}
}
