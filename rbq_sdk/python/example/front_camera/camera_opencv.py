import sys
import threading

import cv2
import numpy as np

from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from rbq_sdk_py.idl.sensor_msgs.msg.dds_ import CompressedImage_

# The robot publishes each camera as a ROS-shaped CompressedImage_ (JPEG) while someone subscribes.
# sensor_4 is the front camera — see the ROS topics page for the other slots.
DEFAULT_TOPIC = "rt/rbq/vision/sensor_4/rgb/compressed"


if __name__ == "__main__":
    # Defaults to loopback, matching the simulator and the on-robot stack.
    # Pass a NIC name to reach a robot from another host, and a topic to pick another camera.
    interface = sys.argv[1] if len(sys.argv) > 1 else "lo"
    topic = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TOPIC
    ChannelFactoryInitialize(0, interface)

    lock = threading.Lock()
    latest = {"data": None}

    def on_image(msg: CompressedImage_):
        with lock:
            latest["data"] = bytes(msg.data)

    sub = ChannelSubscriber(topic, CompressedImage_)
    sub.Init(handler=on_image, queueLen=1)
    print(f"Subscribed to {topic}. Press ESC to save front_image.jpg and quit.")

    image = None
    while True:
        with lock:
            data, latest["data"] = latest["data"], None
        if data is not None:
            decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is not None:
                image = decoded
                cv2.imshow("front_camera", image)
        # Press ESC to stop
        if cv2.waitKey(20) == 27:
            break

    sub.Close()
    if image is None:
        print(f"No image received on {topic}.")
    else:
        cv2.imwrite("front_image.jpg", image)
    cv2.destroyAllWindows()
