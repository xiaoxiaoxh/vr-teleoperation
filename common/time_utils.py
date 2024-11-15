from rclpy.time import Time
from rclpy.clock import ClockType


def convert_float_to_ros_time(timestamp: float):
    """
    Convert a float timestamp (in seconds) to a Time object
    """
    seconds = int(timestamp)
    nanoseconds = int((timestamp - seconds) * 1e9)
    return Time(seconds=seconds, nanoseconds=nanoseconds, clock_type=ClockType.ROS_TIME)

def convert_ros_time_to_float(time: Time) -> float:
    """
    Convert a Time object to a float timestamp (in seconds)
    """
    return time.nanoseconds * 1e-9