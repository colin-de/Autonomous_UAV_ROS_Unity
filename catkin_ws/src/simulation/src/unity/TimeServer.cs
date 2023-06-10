using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TimeServer : MonoBehaviour
{
    long physics_time = 0;
    long frame_time = 0;

    public long GetPhysicsTicks()
    {
        return physics_time;
    }

    public long GetFrameTicks()
    {
        return frame_time;
    }

    public long GetTimeNow()
    {
        return DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).Ticks;
    }

    void FixedUpdate()
    {
        physics_time = GetTimeNow();
    }

    void Update()
    {
        frame_time = GetTimeNow();
    }
}
