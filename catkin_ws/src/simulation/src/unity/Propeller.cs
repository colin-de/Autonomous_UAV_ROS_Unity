using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Propeller : MonoBehaviour {
	public bool counter_clockwise = true;
	public float max_thrust = 1.0f;  // N
	public float max_torque = 1.0f;  // Nm
	public float throttle = 0.0f;  // [0.0-1.0]
	public Rigidbody chassis_body;

	private float ThrustModel(float throttle) {
		return throttle * max_thrust;
	}

	private float TorqueModel(float throttle) {
		return throttle * max_torque;
	}
	
	void Update() {
		Debug.DrawLine(transform.position, transform.position + transform.up * throttle, Color.white);
	}

	void FixedUpdate() {
		float sign = counter_clockwise ? 1.0f : -1.0f;
		Vector3 up = new Vector3(0.0f, 1.0f, 0.0f);
		chassis_body.AddForceAtPosition(transform.up * ThrustModel(throttle), transform.position);
		chassis_body.AddRelativeTorque(up * sign * TorqueModel(throttle));		
	}
}

