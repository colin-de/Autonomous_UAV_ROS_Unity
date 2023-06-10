using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;


using UnityEngine;

public class ObjectRecognitionServer : ICommandable {

	private List<RecognizableObject> recognizable_objects;
	private List<RecognizableObject> detected_objects = new List<RecognizableObject> ();

	float time_last_detection_sent = 0.0f;
	public float targetFrameRate = 30.0f;
	
	const uint type = 5;

	Texture2D MakeTex(int width, int height, Color col)
	{
		Color[] pix = new Color[width * height];
		for (int i = 0; i < pix.Length; ++i)
		{
			pix[i] = col;
		}
		Texture2D result = new Texture2D(width, height);
		result.SetPixels(pix);
		result.Apply();
		return result;
	}


	public void RegisterObject(RecognizableObject obj) {
		if (recognizable_objects == null) {
			recognizable_objects = new List<RecognizableObject> ();
		}
		recognizable_objects.Add (obj);
		Debug.Log (obj.object_name + " object registered with server");
	}
		
	void ComputeDetectedObjects() {
		detected_objects.Clear ();
		foreach (RecognizableObject recognizable_object in recognizable_objects) {
			if (recognizable_object.InFoV () && !recognizable_object.IsOccluded () && recognizable_object.InRange()) {
				detected_objects.Add (recognizable_object); 
			}
		}	
	}

	void SendDetections () {
		Debug.Log("SendDetections: " + detected_objects.Count);
		
		long timestamp = time_server.GetFrameTicks();
		server.SendHeader(type, full_name, timestamp);
		server.SendData(detected_objects.Count);
		server.SendData(recognizable_objects[0].cam.name); // TODO: many camera support
		foreach (RecognizableObject detected_object in detected_objects) {
			Vector3 position = detected_object.GetPosition ();
			Rect screen_rect = detected_object.GetScreenRect ();
			string object_name = detected_object.object_name;
			int object_id = detected_object.GetID ();

			byte[] objectIDBytes = BitConverter.GetBytes (object_id);

			byte[] xMinBytes = BitConverter.GetBytes (screen_rect.xMin);
			byte[] yMinBytes = BitConverter.GetBytes (screen_rect.yMin);
			byte[] xMaxBytes = BitConverter.GetBytes (screen_rect.xMax);
			byte[] yMaxBytes = BitConverter.GetBytes (screen_rect.yMax);

			//convert to right hand coordinate system
			byte[] xPositionBytes = BitConverter.GetBytes (position.x);
			byte[] yPositionBytes = BitConverter.GetBytes (position.z);
			byte[] zPositionBytes = BitConverter.GetBytes (position.y);

			server.SendData (objectIDBytes);
			server.SendData (xMinBytes);
			server.SendData (yMinBytes);
			server.SendData (xMaxBytes);
			server.SendData (yMaxBytes);
			server.SendData (object_name);
			server.SendData (xPositionBytes);
			server.SendData (yPositionBytes);
			server.SendData (zPositionBytes);
		}
	}

	void Update () {
		float time_since_last_detection_sent = Time.time - time_last_detection_sent;
		if (time_since_last_detection_sent > (1/targetFrameRate - Time.fixedDeltaTime * 0.5f)) {
			if(recognizable_objects.Count > 0) {
				ComputeDetectedObjects ();
				SendDetections();
				time_last_detection_sent = Time.time;
			}
		}
	}
}
