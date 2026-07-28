#!/usr/bin/env python3
"""Summarise a ROS 2 bag for conservative safety-supervisor tuning."""
import json
import math
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def summary(values):
    values = sorted(values)
    if not values:
        return {}
    def pct(p):
        return values[min(len(values) - 1, round((len(values) - 1) * p))]
    return {"count": len(values), "min": values[0], "mean": statistics.fmean(values),
            "stddev": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "p95": pct(.95), "p99": pct(.99), "max": values[-1]}


def main(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    acc_h, acc_norm, gyro, roll, pitch, wheel, command = [], [], [], [], [], [], []
    gps = []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        if topic == "/imu/data":
            a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
            acc_h.append(math.hypot(a.x, a.y)); acc_norm.append(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))
            gyro.append(math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z))
            n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if .5 < n < 1.5 and msg.orientation_covariance[0] >= 0:
                roll.append(math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))))
                pitch.append(math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))))
        elif topic == "/wheel_odom": wheel.append(msg.twist.twist.linear.x)
        elif topic == "/cmd_vel": command.append(msg.twist.linear.x)
        elif topic == "/gps/pose_cov":
            p = msg.pose.pose.position; c = msg.pose.covariance
            gps.append((stamp / 1e9, p.x, p.y, math.sqrt(max(c[0], c[7]))))
    gps_step = []
    for a, b in zip(gps, gps[1:]):
        dt = b[0] - a[0]
        if dt > 0: gps_step.append(math.hypot(b[1]-a[1], b[2]-a[2]) / dt)
    result = {"imu_horizontal_accel_mps2": summary(acc_h), "imu_accel_norm_mps2": summary(acc_norm),
              "gyro_norm_rad_s": summary(gyro), "roll_deg": summary(roll), "pitch_deg": summary(pitch),
              "wheel_speed_mps": summary(wheel), "command_speed_mps": summary(command),
              "gps_sigma_m": summary([x[3] for x in gps]), "gps_step_speed_mps": summary(gps_step)}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main(sys.argv[1])
