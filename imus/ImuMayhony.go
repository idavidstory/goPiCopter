package imus

import (
	"math"
	"time"
)

type ImuMayhony struct {
	lastUpdate          int64
	sampleFreq          float32 // half the sample period expressed in seconds
	exInt, eyInt, ezInt float32
	q0, q1, q2, q3      float32
	integralFBx         float32
	integralFBy         float32
	integralFBz         float32
	twoKp               float32 // 2 * poportional gain
	twoKi               float32 // 2 * integral gain
}

/**
* Create a new ImuMayhony struct, and initialize it
**/
func NewImuMayhony() (imu *ImuMayhony) {
	imu = new(ImuMayhony)
	imu.lastUpdate = time.Now().UnixNano()
	imu.q0 = 1.0
	imu.twoKp = 2.0 * 0.5
	imu.twoKi = 2.0 * 0.1
	return
}

/**
* Inverse square root
**/
func invSqrt(x float32) float32 {
	return float32(1.0 / math.Sqrt(float64(x)))
}

/**
* Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
* compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
* direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
* axis only.
* 
* @see: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
**/
func ahrsUpdate(imu *ImuMayhony, gx, gy, gz, ax, ay, az, mx, my, mz float32) {
	var (
		recipNorm              float32
		hx, hy, bx, bz         float32
		halfx, halfy, halfz    float32
		halfex, halfey, halfez float32
		qa, qb, qc             float32
		q0q0, q0q1, q0q2, q0q3 float32
		q1q1, q1q2, q1q3       float32
		q2q2, q2q3, q3q3       float32
	)

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = imu.q0 * imu.q0
	q0q1 = imu.q0 * imu.q1
	q0q2 = imu.q0 * imu.q2
	q0q3 = imu.q0 * imu.q3
	q1q1 = imu.q1 * imu.q1
	q1q2 = imu.q1 * imu.q2
	q1q3 = imu.q1 * imu.q3
	q2q2 = imu.q2 * imu.q2
	q2q3 = imu.q2 * imu.q3
	q3q3 = imu.q3 * imu.q3

	// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
	if mx != 0.0 && my != 0.0 && mz != 0.0 {
		// Normalize magnetometer measurement
		recipNorm = invSqrt(mx*mx + my*my + mz*mz)
		mx *= recipNorm
		my *= recipNorm
		mz *= recipNorm

		// Reference direction of Earth's magnetic field
		hx = 2.0 * (mx*(0.5-q2q2-q3q3) + my*(q1q2-q0q3) + mz*(q1q3+q0q2))
		hy = 2.0 * (mx*(q1q2+q0q3) + my*(0.5-q1q1-q3q3) + mz*(q2q3-q0q1))
		bx = float32(math.Sqrt(float64(hx*hx + hy*hy)))
		bz = 2.0 * (mx*(q1q3-q0q2) + my*(q2q3+q0q1) + mz*(0.5-q1q1-q2q2))

		// Estimated direction of magnetic field
		halfx = bx*(0.5-q2q2-q3q3) + bz*(q1q3-q0q2)
		halfy = bx*(q1q2-q0q3) + bz*(q0q1+q2q3)
		halfz = bx*(q0q2+q1q3) + bz*(0.5-q1q1-q2q2)

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (my*halfz - mz*halfy)
		halfey = (mz*halfx - mx*halfz)
		halfez = (mx*halfy - my*halfx)
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if ax != 0.0 && ay != 0.0 && az != 0.0 {
		// Normalize accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Estimated direction of gravity
		halfx = q1q3 - q0q2
		halfy = q0q1 + q2q3
		halfz = q0q0 - 0.5 + q3q3

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay*halfz - az*halfy)
		halfey += (az*halfx - ax*halfz)
		halfez += (ax*halfy - ay*halfx)
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if halfex != 0.0 && halfey != 0.0 && halfez != 0.0 {
		// Compute and apply integral feedback if enabled
		if imu.twoKi > 0.0 {
			imu.integralFBx += imu.twoKi * halfex * (1.0 / imu.sampleFreq) // integral error scaled by Ki
			imu.integralFBy += imu.twoKi * halfey * (1.0 / imu.sampleFreq)
			imu.integralFBz += imu.twoKi * halfez * (1.0 / imu.sampleFreq)
			gx += imu.integralFBx // apply integral feedback
			gy += imu.integralFBy
			gz += imu.integralFBz
		} else {
			imu.integralFBx = 0.0 // prevent integral windup
			imu.integralFBy = 0.0
			imu.integralFBz = 0.0
		}
		// Apply proportional feedback
		gx += imu.twoKp * halfex
		gy += imu.twoKp * halfey
		gz += imu.twoKp * halfez
	}
	// Integrate rate of change of quaternion
	gx *= (0.5 * (1.0 / imu.sampleFreq)) // pre-multiply common factors
	gy *= (0.5 * (1.0 / imu.sampleFreq))
	gz *= (0.5 * (1.0 / imu.sampleFreq))
	qa = imu.q0
	qb = imu.q1
	qc = imu.q2
	imu.q0 += (-qb*gx - qc*gy - imu.q3*gz)
	imu.q1 += (qa*gx + qc*gz - imu.q3*gy)
	imu.q2 += (qa*gy - qb*gz + imu.q3*gx)
	imu.q3 += (qa*gz + qb*gy - qc*gx)

	// Normalize quaternion
	recipNorm = invSqrt(imu.q0*imu.q0 + imu.q1*imu.q1 + imu.q2*imu.q2 + imu.q3*imu.q3)
	imu.q0 *= recipNorm
	imu.q1 *= recipNorm
	imu.q2 *= recipNorm
	imu.q3 *= recipNorm
}

/**
* Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
* the Earth North and the IMU Z axis (yaw), the Earth ground plane and the IMU Y axis (pitch)
* and the Earth ground plane and the IMU X axis (roll).
*
* Returns Yaw, Pitch and Roll angles in radians
**/
func (imu *ImuMayhony) Update(when int64, gx, gy, gz, ax, ay, az, mx, my, mz float32) (yaw, pitch, roll float32) {
	var (
		gravx, gravy, gravz float64 // estimated gravity direction
	)
	imu.sampleFreq = 1.0 / (float32(when-imu.lastUpdate) / 1000000000.0) // nanoseconds to fractions of a second
	imu.lastUpdate = when
	ahrsUpdate(imu, gx, gy, gz, ax, ay, az, mx, my, mz)
	gravx = float64(2.0 * (imu.q1*imu.q3 - imu.q0*imu.q2))
	gravy = float64(2.0 * (imu.q0*imu.q1 + imu.q2*imu.q3))
	gravz = float64(imu.q0*imu.q0 - imu.q1*imu.q1 - imu.q2*imu.q2 + imu.q3*imu.q3)
	yaw   = float32(math.Atan2(float64(2.0*imu.q1*imu.q2-2.0*imu.q0*imu.q3), float64(2.0*imu.q0*imu.q0+2.0*imu.q1*imu.q1-1.0)))
	pitch = float32(math.Atan(gravx / math.Sqrt(gravy*gravy+gravz*gravz)))
	roll  = float32(math.Atan(gravy / math.Sqrt(gravx*gravx+gravz*gravz)))
	return
}
