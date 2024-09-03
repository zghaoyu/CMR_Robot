import threading
import socket
import struct
import LogHelper as LogObj

#Singleton class
class ComSocket(object):
    _instance_lock = threading.Lock()
    _socket_obj = None

    def __init__(self, *args, **kwargs):
        with ComSocket._instance_lock:
            pass

    def __del__(self):
        self.disconnect()

    def __new__(cls, *args, **kwargs):
        with ComSocket._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    ComSocket._instance = super().__new__(cls)
            return ComSocket._instance

    def connect(self, ip, port=6666):
        bOK = True
        try:
            self._socket_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket_obj.connect((ip, port))
        except socket.error as msg:
            LogObj.logSystemError()
            self._socket_obj = None
            bOK = False
        return bOK

    def disconnect(self):
        if not self._socket_obj is None:
            self._socket_obj.close()

    def send(self, data, bSendHex=False):
        bOK = False
        if not self._socket_obj is None:
            if self._send_ex(data, 3, bSendHex):
                recvData = self._recv_ex(3, bSendHex)
                if recvData is None:
                    recvData = "recv data failed"
                else:
                    bOK = True
            else:
                recvData = "send data failed"
        else:
            recvData = "connect to server first"
        return bOK, recvData

    def _send_ex(self, data, retryCnt=3, bSendHex=False):
        bOK = False
        for i in range(retryCnt):
            try:
                if bSendHex:
                    senddata = struct.pack("%dB" % (len(data)), *data)
                    self._socket_obj.send(senddata)
                else:
                    senddata = data.encode()
                    self._socket_obj.send(senddata)
                bOK = True
                break
            except Exception as e:
                LogObj.logSystemError()
        return bOK

    def _recv_ex(self, retryCnt=3, bRecvHex=False):
        for i in range(retryCnt):
            try:
                if bRecvHex:
                    recvData = (self._socket_obj.recv(1024)).hex()
                else:
                    recvData = (self._socket_obj.recv(1024)).decode()
                break
            except Exception as e:
                LogObj.logSystemError()
                recvData = None
        return recvData