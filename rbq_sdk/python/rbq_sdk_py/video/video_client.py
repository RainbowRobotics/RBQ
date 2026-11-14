import json
import warnings

from ..rpc.client import Client
from .video_api import *


"""
" class VideoClient
"""
class VideoClient(Client):
    def __init__(self):
        # Deprecated — no robot service answers it. Removed in the next release.
        warnings.warn("VideoClient is deprecated and will be removed in the next release: no robot service answers it. "
                      "Subscribe to the camera topic instead (rt/rbq/vision/sensor_4/rgb/compressed).",
                      FutureWarning, stacklevel=2)
        super().__init__(VIDEO_SERVICE_NAME, False)


    def Init(self):
        # set api version
        self._SetApiVerson(VIDEO_API_VERSION)
        # regist api
        self._RegistApi(VIDEO_API_ID_GETIMAGESAMPLE, 0)

    # 1001
    def GetImageSample(self):
        return self._CallBinary(VIDEO_API_ID_GETIMAGESAMPLE, [])
