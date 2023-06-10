using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Quadrotor : ICommandable {
	public float bodyMass = 3.0f;  // [kg]
	public float propellerMaxThrust = 8.5f;  // [N]
    public float propellerMaxTorque = 1.0f;  // [Nm]

    Vector3 last_velocity = new Vector3(0.0f, 0.0f, 0.0f);
    Rigidbody chassis_body;
    List<GameObject> propellers = new List<GameObject> ();
    List<Propeller> propeller_scripts = new List<Propeller> ();

    float[] x_signs = {1.0f, -1.0f, 1.0f, -1.0f};
    float[] z_signs = {-1.0f, 1.0f, 1.0f, -1.0f};

    public void SetPosition(Vector3 position) {
        chassis_body.position = position;
        chassis_body.velocity = new Vector3(0.0f, 0.0f, 0.0f);
        chassis_body.angularVelocity = new Vector3(0.0f, 0.0f, 0.0f);
    }

    public void SetMotorCommands(float[] motor_commands) {
        for(int i = 0; i < 4; i++) {
            propeller_scripts[i].throttle = motor_commands[i];
        }
    }

    public void Awake() {
        Debug.Log("Awake called");
        ApplyParameters();
    }

    public void OnValidate() {
        Debug.Log("Applying Parameters");
        ApplyParameters();
    }

    private void FindPropellers() {
        for(int i = 1; i <= 4; i++) {
            Debug.Log("Found Prop " + i);
            GameObject prop = transform.Find("Propeller_" + i).gameObject;
            propellers.Add(prop);
            Propeller propeller_script = prop.GetComponent<Propeller>();
            propeller_scripts.Add(propeller_script);
        }
    }

    public void ApplyParameters() {
        if(propellers.Count == 0) {
            FindPropellers();
        }
        chassis_body = GetComponent<Rigidbody>();
        chassis_body.mass = bodyMass;
    }

	public override void OnStart () {

	}

    public override bool OnCommand(string[] command) {
        string command_name = command[1];

        if(command_name == "motors" && command.Length == 6) {
            float[] motor_commands = new float[4];
            for(int i = 0; i < 4; i++) {
                if(!float.TryParse(command[i + 2], out motor_commands[i])) {
                    Debug.Log("Quadrotor: Couldn't parse float from " + command[i +1]);
                    return false;
                }
            }
            SetMotorCommands(motor_commands);
            return true;
        }
        return false;
    }
}

