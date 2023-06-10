using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : ICommandable {
	//spawnable objects
	public GameObject quadrotor;

	//mapping from name to object
	Dictionary<string, GameObject> prefab_map = new Dictionary<string, GameObject>();

	void InitializePrefabMap() {
		prefab_map.Add("Quadrotor", quadrotor);
	}

	public override void OnStart () {
		InitializePrefabMap();
	} 

	public override bool OnCommand(string[] command) {
		string command_name = command[1];
		string prefab_name = command[2];
		string object_name = command[3];
		bool success = false;
		
		if(command_name == "spawn" && prefab_map.ContainsKey(prefab_name)) {
			GameObject new_obj = Instantiate(prefab_map[prefab_name]);
			new_obj.name = object_name;
			success = true;
		} 
		return success;
	}

	void Update () {
		
	}
}
