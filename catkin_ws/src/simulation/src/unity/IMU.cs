using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IMU : ICommandable {	
    Rigidbody rb;
	Vector3 last_velocity = new Vector3(0.0f, 0.0f, 0.0f);	
	System.Random rand = new System.Random();
	const uint type = 2;

    public delegate void IMUCallback(long timestamp, Vector3 accel, Vector3 gyro, Vector3 mag);
    public IMUCallback OnIMUMeasurement;

	public double gyroNoiseMean = 0.0;
	public double gyroNoiseStdDev = 0.001;

	public double accelNoiseMean = 0.0;
	public double accelNoiseStdDev = 0.001;

    public double magNoiseMean = 0.0;
    public double magNoiseStdDev = 0.1;
	
	double GaussianNoise(double mean, double stdDev) {
		double u1 = 1.0 - rand.NextDouble();
		double u2 = 1.0 - rand.NextDouble();
		double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
             Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
		double randNormal =
             mean + stdDev * randStdNormal;
        return randNormal;
	}

	double UniformNoise(double scale) {
		return scale * (rand.NextDouble() - 0.5);
	}

	Vector3 RandomVec(double mean, double stdDev) {
		return new Vector3((float)GaussianNoise(mean, stdDev),
			               (float)GaussianNoise(mean, stdDev),
			               (float)GaussianNoise(mean, stdDev));
	}


	void Awake() {
	  rb = gameObject.GetComponentInParent(typeof(Rigidbody)) as Rigidbody;
	}

	void FixedUpdate () {
		Quaternion world_to_body = Quaternion.Inverse(rb.rotation);
        Vector3 accel_world = (rb.velocity - last_velocity) / Time.deltaTime;
        Vector3 accel_body = world_to_body * (accel_world - Physics.gravity);
        Vector3 angvel_body = world_to_body * rb.angularVelocity;
        last_velocity = rb.velocity;
        Vector3 world_north = new Vector3(1, 0, 0);
        long timestamp = time_server.GetPhysicsTicks();
        Vector3 noisy_accel = accel_body + RandomVec(accelNoiseMean, accelNoiseStdDev);
        Vector3 noisy_gyro = angvel_body + RandomVec(gyroNoiseMean, gyroNoiseStdDev);
        Vector3 noisy_mag = world_to_body * world_north + RandomVec(magNoiseMean, magNoiseStdDev);

        server.SendHeader(type, full_name, timestamp);
        server.SendData(noisy_accel);
        server.SendData(noisy_gyro);
        if (OnIMUMeasurement != null) {
            OnIMUMeasurement(timestamp, noisy_accel, noisy_gyro, noisy_mag);
        }
	}
}
