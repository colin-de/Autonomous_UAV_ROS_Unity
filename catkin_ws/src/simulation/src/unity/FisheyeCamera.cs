using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FisheyeCamera : RGBCamera {
	const int type = 4;
	RenderFisheye fisheye_script;

	void Update () {
		if(fisheye_script == null) {
			fisheye_script = GetComponent<RenderFisheye>();
		}
        server.SendHeader(type, full_name, time_server.GetFrameTicks());
		server.SendData(fisheye_script.alpha);
		server.SendData(fisheye_script.chi);
		server.SendData(fisheye_script.focalLength);
		SendImage();
	}
}
