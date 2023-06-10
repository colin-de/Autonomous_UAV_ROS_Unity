using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using MavLink;

public class Pixhawk : MonoBehaviour {
    public IMU imu;
    public Quadrotor quadrotor;

    int port = 14560;
    UdpClient udp_client = new UdpClient();
    IPEndPoint pixhawk_endpoint;
    Mavlink mavlink;
    Thread motorCommandThread;

    void StartMotorCommandThread() {
        motorCommandThread = new Thread(new ThreadStart(ReceiveMotorCommands));
        motorCommandThread.IsBackground = true;
        motorCommandThread.Start();
    }

    void OnApplicationQuit() {
        motorCommandThread.Abort();
    }

    private void ReceiveMotorCommands()
    {
        IPEndPoint ip_endpoint = new IPEndPoint(IPAddress.Any, port);
        mavlink.PacketReceived += new PacketReceivedEventHandler(OnMavlinkPacket);
        byte[] receiveByteArray;
        while (true)
        {
            receiveByteArray = udp_client.Receive(ref ip_endpoint);
            mavlink.ParseBytes(receiveByteArray);
        }
    }

    void OnMavlinkPacket(object sender, MavlinkPacket packet)
    {
        if (packet.Message.GetType() == typeof(MavLink.Msg_hil_controls))
        {
            Msg_hil_controls msg = packet.Message as Msg_hil_controls;
            float[] motor_commands = new float[] { msg.roll_ailerons,
                                                   msg.pitch_elevator,
                                                   msg.yaw_rudder,
                                                   msg.throttle };
            quadrotor.SetMotorCommands(motor_commands);
        }
    }

    void Start () {
        pixhawk_endpoint = new IPEndPoint(IPAddress.Loopback, port);
        mavlink = new Mavlink();
        imu.OnIMUMeasurement += HandleIMU;
        StartMotorCommandThread();
	}

    void HandleIMU(long timestamp, Vector3 accel, Vector3 gyro, Vector3 mag) {
        Msg_hil_sensor imu_msg = new Msg_hil_sensor();     

        imu_msg.time_usec = (ulong) timestamp / 10;
        imu_msg.xacc = accel.x;
        imu_msg.yacc = -accel.z;
        imu_msg.zacc = -accel.y;

        imu_msg.xgyro = -gyro.x;
        imu_msg.ygyro = gyro.z;
        imu_msg.zgyro = gyro.y;

        imu_msg.xmag = mag.x;
        imu_msg.ymag = -mag.z;
        imu_msg.zmag = -mag.y;

        MavlinkPacket imu_packet = new MavlinkPacket();
        imu_packet.Message = imu_msg;
        imu_packet.ComponentId = 1;
        imu_packet.SystemId = 1;
        byte[] imu_bytes = mavlink.Send(imu_packet);
        udp_client.Send(imu_bytes, imu_bytes.Length, pixhawk_endpoint);
    }
}