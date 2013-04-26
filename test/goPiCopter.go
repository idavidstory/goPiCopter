package main

import (
	"fmt"
	"goPiCopter/imus"
	"goPiCopter/sensors"
	"math"
	"time"
)

type SensorData struct {
	when       int64
	count      int     // number of samples taken
	gx, gy, gz float32 // Gyroscope data
	ax, ay, az float32 // Accelerometer data
	mx, my, mz float32 // Magnetometer data
}

/**
* Remotely control a quadcopter using a RaspberryPi as
* the on board computer, reading the sensors and computing
* how much power to apply to the motors. The current implementation
* uses three sensors (gyroscope, accelerometer, and magnetometer).
**/
func main() {
	var (
		err           error
		gyroscope     *sensors.L3GD20
		accelerometer *sensors.LSM303ACCEL
		magnetometer  *sensors.LSM303MAG
		imu           *imus.ImuMayhony
	)

	gyroscope, accelerometer, magnetometer, err = setup()
	if err == nil {
		imu = imus.NewImuMayhony()
		sensorChannel := make(chan SensorData)
		go process(sensorChannel, imu)
		err = loop(sensorChannel, gyroscope, accelerometer, magnetometer)
		if err == nil {
			fmt.Printf("Done\n")
		}
	}
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
func summarize(sensorChannel chan SensorData, when int64, gyroscope *sensors.L3GD20, accelerometer *sensors.LSM303ACCEL, mx, my, mz float32, count int) {
	var (
		err  error
		data SensorData
	)
	data.when = when
	data.count = count
	data.mx = mx
	data.my = my
	data.mz = mz
	data.gx, data.gy, data.gz, err = gyroscope.Evaluate()
	if err == nil {
		data.ax, data.ay, data.az, err = accelerometer.Evaluate()
		if err == nil {
			sensorChannel <- data // send the data over the sensorChannel
		}
	}
}

/**
* Loop reading sensors, attempt to summarize at 100Hz
**/
func loop(sensorChannel chan SensorData, gyroscope *sensors.L3GD20, accelerometer *sensors.LSM303ACCEL, magnetometer *sensors.LSM303MAG) (err error) {
	var (
		count      int
		now        int64   // Current time in nanoseconds
		lastTime   int64   // Last 100Hz time in nanoseconds
		hz         int64   // Hz in nanoseconds
		magCount   int     // Read the Magnetometer every 5th summarize
		mx, my, mz float32 // Magnetometer data
	)
	fmt.Printf("Looping...\n")
	hz = int64(time.Second/50) - 200000 // minus overhead to send sensor data

	lastTime = time.Now().UnixNano()
	mx, my, mz, err = magnetometer.ReadXYZ()
	if err != nil {
		fmt.Printf("loop: failed to read magnetometer, err=%v\n", err)
		return
	}
	for {
		count++
		gyroscope.Measure()
		now = time.Now().UnixNano()
		if (now - lastTime) >= hz {
			summarize(sensorChannel, now, gyroscope, accelerometer, mx, my, mz, count)
			lastTime = now
			count = 0
			magCount++
		}

		accelerometer.Measure()
		now = time.Now().UnixNano()
		if (now - lastTime) >= hz {
			summarize(sensorChannel, now, gyroscope, accelerometer, mx, my, mz, count)
			lastTime = now
			count = 0
			magCount++
		}

		if magCount == 5 {
			magCount = 0
			mx, my, mz, err = magnetometer.ReadXYZ()
			if err != nil {
				fmt.Printf("loop: failed to read magnetometer, err=%v\n", err)
				return
			}
		}
	}
	return
}

/**
* Process summarized sensor data
**/
func process(sensorChannel chan SensorData, imu *imus.ImuMayhony) {
	const (
		d2r = math.Pi / 180.0 // Used to convert degrees to radians
		r2d = 180.0 / math.Pi // Used to convert radians to degrees
	)
	var (
		data     SensorData
		i        int // number of iterations in the for loop
		cnt      int // number of printf calls  (or ~seconds)
		now      int64
		lastTime int64 // last time a printf was called
		second   int64 // One second in nanoseconds
		yaw      float32
		pitch    float32
		roll     float32
	)
	second = int64(time.Second)
	lastTime = time.Now().UnixNano()
	for {
		data = <-sensorChannel // block waiting for data on the sensorChannel
		yaw, pitch, roll = imu.Update(data.when, data.gx*d2r, data.gy*d2r, data.gz*d2r, data.ax, data.ay, data.az, data.mx, data.my, data.mz)
		i++
		now = time.Now().UnixNano()
		if (now - lastTime) >= second {
			cnt++
			fmt.Printf("%d YPR(%10.5f, %10.5f, %10.5f)\n", cnt, yaw*r2d, pitch*r2d, roll*r2d)
			lastTime = now
		}
	}
}
