using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RecognizableObject : MonoBehaviour {
	static int next_object_id;

	public Camera cam;
	public string object_name;
    public double detection_range = 30.0;
	public double min_range = 5.0;
	Renderer rend;
	ObjectRecognitionServer object_server;
	Vector3[] world_corners;

	int object_id;
                 
	void Start () {
		object_server = (ObjectRecognitionServer) GameObject.FindObjectOfType (typeof(ObjectRecognitionServer));

		if (object_server == null) {
			Debug.LogError ("Couldn't find ObjectRecognitionServer script! Make sure it is attached to the vehicle object");
		} else {
			object_server.RegisterObject (this);
			object_id = ++next_object_id;
		}

		if (cam == null)
		{
			Debug.LogWarning("No Object Recognition Camera Specified for " + name + "defaulting to MainCamera");
			cam = Camera.main;
		}
		rend = GetComponent<Renderer>(); 

		if (rend != null) {
			world_corners = GetCorners (rend.bounds);
		} else {
			Debug.LogError ("RecognizableObject must be attached to an GameObject with a renderer component");
		}
	}		

	Vector3[] GetCorners(Bounds bounds)
	{
		Vector3[] corners = new Vector3[8];
		//bottom corners
		corners[0] = bounds.min;
		corners[1] = new Vector3(bounds.max.x, bounds.min.y, bounds.min.z);
		corners[2] = new Vector3(bounds.max.x, bounds.min.y, bounds.max.z);
		corners[3] = new Vector3(bounds.min.x, bounds.min.y, bounds.max.z);
		//top corners
		corners[4] = new Vector3(bounds.min.x, bounds.max.y, bounds.min.z);
		corners[5] = new Vector3(bounds.max.x, bounds.max.y, bounds.min.z);
		corners[6] = bounds.max;
		corners[7] = new Vector3(bounds.min.x, bounds.max.y, bounds.max.z);
		return corners;
	}

	public int GetID() {
		return object_id;
	}

	public bool InFoV() {
		Debug.Log(cam);
		Vector3 viewport_point = cam.WorldToViewportPoint(rend.bounds.center);
		return viewport_point.x > 0 && viewport_point.x < 1 && viewport_point.x > 0 && viewport_point.y <= 1 && viewport_point.z >= 0;
	}

	public bool IsOccluded() {
		Ray ray = new Ray();
		ray.origin = cam.transform.position;
		ray.direction = rend.bounds.center - cam.transform.position;
		RaycastHit rayInfo = new RaycastHit();
		if(Physics.Raycast(ray, out rayInfo))
		{
			if(rayInfo.collider.name == name)
			{
				return false;
			}
		}
		return true;
	}
        
    public bool InRange() {
		double range_to_object = (GetPosition() - cam.transform.position).magnitude;
		return range_to_object < detection_range && range_to_object > min_range;
    }

	public Rect GetScreenRect() {
		return GetScreenRect (rend.bounds);
	}

	public Vector3 GetPosition() {
		return transform.position;
	}

	Rect GetScreenRect(Bounds world_bounds)
	{
		float xMin, xMax, yMin, yMax;
		xMin = yMin = float.MaxValue;
		xMax = yMax = float.MinValue;
		Rect screenRect = new Rect();
		foreach (Vector3 corner in world_corners)
		{
			Vector3 screenCorner = cam.WorldToScreenPoint(corner);
			if (screenCorner.x < xMin) xMin = screenCorner.x;
			if (screenCorner.y < yMin) yMin = screenCorner.y;
			if (screenCorner.x > xMax) xMax = screenCorner.x;
			if (screenCorner.y > yMax) yMax = screenCorner.y;
		}

		int width = cam.pixelWidth;
		int height = cam.pixelHeight;

		screenRect.xMin = Mathf.Clamp (xMin, 0, width);
		screenRect.xMax = Mathf.Clamp (xMax, 0, width);
		screenRect.yMin = Mathf.Clamp (height - yMin, 0, height);
		screenRect.yMax = Mathf.Clamp (height - yMax, 0, height);

		return screenRect;
	}

	void Update()  {
	}
}
