import time
from tqdm import tqdm
from rosbags.highlevel import AnyReader
import transformations as tf

S_TO_NS = 1e9

def stamp_to_ns(stamp) -> int:
    return int(stamp.sec * S_TO_NS + stamp.nanosec)

def ns_to_s(ns) -> float:
    return ns / S_TO_NS

def ns_to_relative_s(ns, start_ns) -> float:
    return ns_to_s(ns - start_ns)

def s_to_ns(s) -> int:
    return int(s * S_TO_NS)


def flatten_dict(d, parent_key="", sep="."):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def read_messages_by_topics(bag_path, topics, start_time_ns=None, end_time_ns=None, progress_bar=True):
    ret = {t: [] for t in topics}
    with AnyReader([bag_path]) as reader:
        connections = [x for x in reader.connections if x.topic in topics]

        if start_time_ns is None:
            start_time_ns = reader.start_time
        if end_time_ns is None:
            end_time_ns = reader.end_time

        if progress_bar:
            pbar = tqdm(total=round(ns_to_s(end_time_ns - start_time_ns), 2), desc="Processing rosbag", unit="s")
        
        for connection, timestamp, rawdata in reader.messages(connections):
            if progress_bar:
                current_progress = round(ns_to_s(timestamp - start_time_ns), 2)
                pbar.update(current_progress - pbar.n)

            if start_time_ns is not None and timestamp < start_time_ns:
                continue
            if end_time_ns is not None and timestamp > end_time_ns:
                continue

            ret[connection.topic].append(
                {
                    "timestamp": timestamp,
                    "msg": reader.deserialize(rawdata, connection.msgtype),
                }
            )
        
        if progress_bar:
            pbar.close()

    return ret

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion to euler angles (roll, pitch, yaw)
    """
    return tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w], axes='xyzs')
