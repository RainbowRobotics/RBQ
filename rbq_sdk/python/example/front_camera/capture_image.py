import sys
import threading

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

    got = threading.Event()
    frame = {}

    def on_image(msg: CompressedImage_):
        if not got.is_set():
            frame["msg"] = msg
            got.set()

    sub = ChannelSubscriber(topic, CompressedImage_)
    sub.Init(handler=on_image, queueLen=1)

    # The camera starts streaming when the first subscriber appears, so allow a few seconds.
    if not got.wait(5.0):
        print(f"No image on {topic} within 5 s (vision not running, or another camera slot).")
        sys.exit(1)
    sub.Close()

    msg = frame["msg"]
    image_name = "./img.jpg"
    with open(image_name, "wb") as f:
        f.write(bytes(msg.data))
    print(f"Saved {image_name} ({len(msg.data)} bytes, format '{msg.format}')")
