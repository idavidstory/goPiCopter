package main

import (
	"fmt"
	"goPiCopter/io/sensors"
	"time"
)

/**
* Read the raw gyro values and display every 100th one
**/
func main() {
	var (
		err        error
		errCnt     int
		pause      time.Duration
		i          int
		x, y, z    int16
		xx, yy, zz int
		px, py, pz int
		gyroscope  *sensors.L3GD20 = nil
	)
	gyroscope, err = sensors.NewL3GD20()
	if err != nil {
		fmt.Printf("Error: getting device L3GD20, err=%v\n", err)
	} else {
		pause = time.Millisecond * 100
		for {
			x, y, z, err = gyroscope.ReadRaw()
			if err != nil {
				errCnt++
			} else {
				i++
				xx += int(x)
				yy += int(y)
				zz += int(z)
				if i == 10 {
					xx = xx / i
					yy = yy / i
					zz = zz / i
					fmt.Printf("%4d, %4d, %4d\n", xx-px, yy-py, zz-pz)
					i = 0
					px = xx
					py = yy
					pz = zz
					xx = 0
					yy = 0
					zz = 0
				}
			}
			time.Sleep(pause)
		}
	}
}
