using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DepthCamera : RGBCamera {
	const int type = 3;	
	public Shader shader;
	
	private Material _material;
	private Material material {
		get {
			if (_material == null) {
				_material = new Material(shader);
				_material.hideFlags = HideFlags.HideAndDontSave;
			}
			return _material;
		}
	}
	
	private void OnDisable() {
		if (_material != null)
			DestroyImmediate(_material);
	}

	void OnRenderImage(RenderTexture src, RenderTexture dest) {
		Graphics.Blit(src, dest, material);
		if (send_image) {
            server.SendHeader(type, full_name, time_server.GetFrameTicks());
			SendImage();
			send_image = false;
		}
	}
}
