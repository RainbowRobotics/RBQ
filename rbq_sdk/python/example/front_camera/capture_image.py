import sys
import time
import os
from rbq_sdk_py.core.channel import ChannelFactoryInitialize
from rbq_sdk_py.video.video_client import VideoClient


if __name__ == "__main__":
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    client = VideoClient()  # Create a video client
    client.SetTimeout(3.0)
    client.Init()
    
    code, data = client.GetImageSample()

    if code != 0:
        print("get image sample error. code:", code)
    else:
        imageName = "./img.jpg"
        print("ImageName:", imageName)

        with open(imageName, "+wb") as f:
            f.write(bytes(data))

    time.sleep(1)
