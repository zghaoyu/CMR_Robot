import serial
import serial.tools.list_ports
import threading
import LogHelper as LogObj

#Singleton class
class RS232COM(object):
    _instance_lock = threading.Lock()
    com_objs = {}

    def __init__(self, *args, **kwargs):
        with RS232COM._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with RS232COM._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    RS232COM._instance = super().__new__(cls)
            return RS232COM._instance

    @classmethod
    def getDictVal(cls, dictData, key, defaultVal):
        try:
            val = dictData[key]
        except Exception as e:
            val = defaultVal
        return val

    def openPort(self, portX, bps, timeout=5):
        with RS232COM._instance_lock:
            ser = self.getDictVal(self.com_objs, portX, None)
            if ser is None:
                try:
                    ser = serial.Serial(portX, bps, timeout=timeout)
                    if (ser.is_open):
                        self.com_objs[portX] = ser
                    else:
                        print("Open " + portX + " failed")
                except Exception as e:
                    ser = None
                    LogObj.logSystemError()
            return ser

    def readPort(self, portX, dec="utf-8"):
        with RS232COM._instance_lock:
            rtn = None
            ser = self.getDictVal(self.com_objs, portX, None)
            if not ser is None:
                try:
                    if dec == "hex":
                        rtn = ser.readline().hex()
                    else:
                        rtn = ser.readline().decode(dec) #Content(To be read) must be end with \n, or else blocking will be triggered
                except Exception as e:
                    LogObj.logSystemError()
            else:
                print("Port "+portX+" is not opened yet")
            return rtn

    def writePort(self, portX, writeContent, enc="utf-8"):
        with RS232COM._instance_lock:
            rtn = -1
            ser = self.getDictVal(self.com_objs, portX, None)
            if not ser is None:
                try:
                    if enc == "hex":
                        rtn = ser.write(writeContent)
                    else:
                        rtn = ser.write(writeContent.encode(enc))
                except Exception as e:
                    LogObj.logSystemError()
            else:
                print("Port "+portX+" is not opened yet")
            return rtn

    def closePort(self, portX):
        with RS232COM._instance_lock:
            ser = self.getDictVal(self.com_objs, portX, None)
            if not ser is None:
                ser.close()
                self.com_objs[portX] = None

    def getAvailablePorts(self):
        port_list = list(serial.tools.list_ports.comports())
        return port_list