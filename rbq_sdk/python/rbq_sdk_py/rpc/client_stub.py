import time

from collections import deque
from enum import Enum
from threading import Thread, Condition

from ..idl.rbq_api.msg.dds_ import Request_ as Request
from ..idl.rbq_api.msg.dds_ import Response_ as Response

from ..core.channel import ChannelFactory
from ..core.channel_name import ChannelType, GetClientChannelName
from .request_future import RequestFuture, RequestFutureQueue


"""
" class ClientStub
"""
class ClientStub:
    def __init__(self, serviceName: str):
        self.__serviceName = serviceName
        self.__futureQueue = None

        self.__sendChannel = None
        self.__recvChannel = None
        # The robot answers no-reply requests too. Keep the answer to our own latest one per API, so a
        # streaming call (Move) can report a refusal without waiting. Other clients' answers share the topic.
        self.__noReplyIds = deque(maxlen=64)
        self.__noReplyCode = {}   # apiId -> (request id, code)

    def Init(self):
        factory = ChannelFactory()
        self.__futureQueue = RequestFutureQueue()

        # create channel
        self.__sendChannel = factory.CreateSendChannel(GetClientChannelName(self.__serviceName, ChannelType.SEND), Request)
        self.__recvChannel = factory.CreateRecvChannel(GetClientChannelName(self.__serviceName, ChannelType.RECV), Response,
                                    self.__ResponseHandler,10)
        time.sleep(0.5)


    def Send(self, request: Request, timeout: float):
        self.__noReplyIds.append(request.header.identity.id)
        if self.__sendChannel.Write(request, timeout):
            return True
        else:
            print("[ClientStub] send error. id:", request.header.identity.id)
            return False

    def SendRequest(self, request: Request, timeout: float):
        id = request.header.identity.id

        future = RequestFuture()
        future.SetRequestId(id)
        self.__futureQueue.Set(id, future)

        if self.__sendChannel.Write(request, timeout):
            return future
        else:
            print("[ClientStub] send request error. id:", request.header.identity.id)
            self.__futureQueue.Remove(id)
            return None

    def NoReplyCode(self, apiId: int, maxAge: float = 1.0):
        # Only an answer to a request sent within maxAge s: an old refusal must not outlive a mode change.
        # The request id is its send time (time.monotonic_ns(), ClientBase.__SetHeader).
        id, code = self.__noReplyCode.get(apiId, (0, 0))
        return code if time.monotonic_ns() - id < maxAge * 1e9 else 0

    def RemoveFuture(self, requestId: int):
        self.__futureQueue.Remove(requestId)

    def __ResponseHandler(self, response: Response):
        id = response.header.identity.id
        # apiId = response.header.identity.api_id
        # print("[ClientStub] responseHandler recv response id:", id, ", apiId:", apiId)
        future = self.__futureQueue.Get(id)
        if future is None:
            if id in self.__noReplyIds:
                self.__noReplyCode[response.header.identity.api_id] = (id, response.header.status.code)
        elif not future.Ready(response):
            print("[ClientStub] set future ready error.")
