import cv2 as cv
import threading
import serial
import pyzbar.pyzbar as pyzbar
import numpy as np
import LogHelper as LogObj
import UtilsFunc as FuncObj
from ClassConfig import CobotCfg
from ClassSocket import ComSocket

#Singleton class
class CameraLogitech(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerCamera'
    _camera_hwd = None
    _config_dat = None
    _from_file = None
    _camera_idx = 0
    _cobot_cfg = CobotCfg()

    def __init__(self, *args, **kwargs):
        with CameraLogitech._instance_lock:
            if self._config_dat is None:
                self._config_dat = self._cobot_cfg.get_marker_cfg()
                self._camera_idx = int(FuncObj.getDictVal(self._config_dat, "camera_index", "0"))

    def __del__(self):
        self.closeCamera()

    def __new__(cls, *args, **kwargs):
        with CameraLogitech._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    CameraLogitech._instance = super().__new__(cls)
            return CameraLogitech._instance

    def clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)

    def cameraHasOpened(self):
        if self._camera_hwd is None:
            return False
        else:
            return True

    def openCamera(self):
        with CameraLogitech._instance_lock:
            if self._camera_hwd is None:
                self._camera_hwd = cv.VideoCapture(self._camera_idx, cv.CAP_DSHOW )  # = cv.VideoCapture(self._camera_idx) # change by Paris 2021/9/23
                #self._camera_hwd = cv.VideoCapture(self._camera_idx)
                #640-480  960-720 1280-720 1920-1080
                self._camera_hwd.set(3, 1920)  # add by Hui Zhi 2022/7/27
                self._camera_hwd.set(4, 1080)  # add by Hui Zhi 2022/7/27
                w = self._camera_hwd.get(cv.CAP_PROP_FRAME_WIDTH)
                h = self._camera_hwd.get(cv.CAP_PROP_FRAME_HEIGHT)
                print(f"(w,h)={(w,h)}")
            if not self._camera_hwd.isOpened():
                self._camera_hwd = None
                LogObj.logAppError("Camera", "Cannot open camera")
            return self._camera_hwd

    # add by Hui Zhi 2021.11.8
    def cameraSetting(self, marker_name, show_settings=False):
        if marker_name == "CircleBasedMkr_01":  # CircleBasedMkr_01
            self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -5)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 47)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 26)  # 151
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 20)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 4000)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_HUE,0)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN,0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM,110)
            self._camera_hwd.set(cv.CAP_PROP_TILT,0)

        elif marker_name == "CircleBasedMkr_03": # CircleBasedMkr_03
            self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -5)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 50)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 88)
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 20)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 2000)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 0)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION,0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_HUE, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN, 0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM, 100)
            self._camera_hwd.set(cv.CAP_PROP_TILT, 0)

        elif marker_name == "CircleBasedMkr_04":  # CircleBasedMkr_04
            self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -5)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 45)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 100)
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 12)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 2000)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 0)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION, 0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_HUE, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN, 0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM, 100)
            self._camera_hwd.set(cv.CAP_PROP_TILT, 0)

        elif marker_name == "CircleBasedMkr_05":
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 10)
            self._camera_hwd.set(cv.CAP_PROP_AUTO_EXPOSURE,1)
            # self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -6)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 70)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 110)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 128)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION, 128)
            self._camera_hwd.set(cv.CAP_PROP_HUE, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN, 0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM, 100)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 6400)

        elif marker_name == "CircleBasedMkr_06":
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 10)
            self._camera_hwd.set(cv.CAP_PROP_AUTO_EXPOSURE,1)
            # self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -6)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 45)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 110)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 128)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION, 128)
            self._camera_hwd.set(cv.CAP_PROP_HUE, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN, 0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM, 100)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 6400)

        elif marker_name == "Barcode_Scan":
            self._camera_hwd.set(cv.CAP_PROP_FOCUS, 15)
            self._camera_hwd.set(cv.CAP_PROP_AUTO_EXPOSURE,1)
            # self._camera_hwd.set(cv.CAP_PROP_EXPOSURE, -6)
            self._camera_hwd.set(cv.CAP_PROP_BRIGHTNESS, 45)
            self._camera_hwd.set(cv.CAP_PROP_CONTRAST, 110)
            self._camera_hwd.set(cv.CAP_PROP_SHARPNESS, 128)
            self._camera_hwd.set(cv.CAP_PROP_SATURATION, 128)
            self._camera_hwd.set(cv.CAP_PROP_HUE, 0)
            self._camera_hwd.set(cv.CAP_PROP_GAIN, 0)
            self._camera_hwd.set(cv.CAP_PROP_BACKLIGHT, 0)
            self._camera_hwd.set(cv.CAP_PROP_ZOOM, 250)
            self._camera_hwd.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, 6400)
        if show_settings:
            self._camera_hwd.set(cv.CAP_PROP_SETTINGS, 1)

    def closeCamera(self):
        with CameraLogitech._instance_lock:
            if not self._camera_hwd is None:
                self._camera_hwd.release()
                self._camera_hwd = None

    def takePicture(self):
        frame = None
        if self._from_file is None:
            if self._camera_hwd is None: self.openCamera()
            with CameraLogitech._instance_lock:
                if not self._camera_hwd is None:
                    ret, frame = self._camera_hwd.read()
                    if not ret: frame = None
                return frame
        else:
            frame = cv.imread(self._from_file)
            return frame

    def setFromFile(self, from_file_path=None):
        self._from_file = from_file_path

    def readBarcode(self, barcodeGrayImg, bByOpenCV=False):
        barcodeTxt = []
        if bByOpenCV:
            qrcoder = cv.QRCodeDetector()
            codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(barcodeGrayImg)
            ctext = "{}".format(codeinfo)
            if len(ctext) > 0:
                ctype = "QRCode"
                pts = np.int32(points)
                barcodeTxt.append([ctype, ctext, (pts[0][0][0]-1,pts[0][0][1]-1,(pts[2][0][0]-pts[0][0][0]+2),(pts[2][0][1]-pts[0][0][1]+2))])

            # barcode = cv.barcode_BarcodeDetector()
            # ok, codeinfo, decoded_type, points = barcode.detectAndDecode(barcodeGrayImg)
            # if (ok):
            #     print('info', codeinfo)
            #     print('type', decoded_type)
        else:
            barcodes = pyzbar.decode(barcodeGrayImg)
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                ctext = "{}".format(barcodeData)
                ctype = "{}".format(barcodeType)
                pts = np.array([barcode.polygon],np.int32)# add by hui zhi 20220507 for testing
                pts = pts.reshape((-1,1,2))
                cv.polylines(barcodeGrayImg,[pts],True,(255,0,255),3)
                # cv.rectangle(barcodeGrayImg, (x, y), (x + w, y + h), (0, 0, 255), 1)  # add by Hui Zhi 2022/3/7 for testing
                barcodeTxt.append([ctype, ctext, (x, y, w, h)])
        return barcodeTxt

#Singleton class
class CameraLight(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerCamera'
    _portx = "COM6"
    _bps = 19200
    _timex = 30  #None: wait forever, 0: return immediately, other: timeout(s)

    def __init__(self, *args, **kwargs):
        with CameraLight._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with CameraLight._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    CameraLight._instance = super().__new__(cls)
            return CameraLight._instance

    def light_switch(self, bOn=True, iChannel=0):
        if iChannel < 0: iChannel = 0
        if iChannel > 3: iChannel = 3
        bOK = False
        try:
            ser = serial.Serial(self._portx, self._bps, timeout=self._timex)
            cmd = [0x40, 0x05, 0x01, 0x00, 0x2A, iChannel, 0x01 if bOn else 0x00, 0x71]
            chkSum = 0
            for i in range(7):
                chkSum = chkSum + cmd[i]
            chkSum = chkSum & 255
            cmd[7] = chkSum

            if len(cmd) == ser.write(cmd):
                rd = []
                for i in range(6):
                    rd.append(ser.read().hex())
                if rd[5] == "44": bOK = True
            ser.close()
        except Exception as e:
            LogObj.logSystemError()
        return bOK

    def light_on(self):
        return self.light_switch(True)

    def light_off(self):
        return self.light_switch(False)

    def light_adjust(self, brightness=25):
        if brightness < 0: brightness = 0
        if brightness > 255: brightness = 255

        bOK = False
        try:
            ser = serial.Serial(self._portx, self._bps, timeout=self._timex)
            cmd = [0x40, 0x05, 0x01, 0x00, 0x1A, 0x00]
            chkSum = brightness
            for i in cmd:
                chkSum = chkSum + i
            chkSum = chkSum & 255

            cmd.append(brightness)
            cmd.append(chkSum)
            if len(cmd) == ser.write(cmd):
                rd = []
                for i in range(6):
                    rd.append(ser.read().hex())
                if rd[5] == "44": bOK = True
            ser.close()
        except Exception as e:
            LogObj.logSystemError()
        return bOK

#Singleton class
class LightCtrlLCPW(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerCamera'
    _ip = "192.168.1.253"
    _port = 1030
    _sck = ComSocket()

    def __init__(self, *args, **kwargs):
        with LightCtrlLCPW._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with LightCtrlLCPW._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    LightCtrlLCPW._instance = super().__new__(cls)
            return LightCtrlLCPW._instance

    def light_switch(self, bOn=True, iChannel=4, brightness=-1):
        if iChannel < 1: iChannel = 1
        if iChannel > 4: iChannel = 4
        bOK = False
        if brightness < 0: brightness = self._get_brightness(iChannel)
        if brightness > 0:
            try:
                if self._sck.connect(self._ip, self._port):
                    if bOn:
                        bOK, rtn = self._sck.send("@setup#"+str(iChannel)+":0:"+str(brightness)+":0"+chr(13))
                    else:
                        bOK, rtn = self._sck.send("@setup#" + str(iChannel) + ":0:"+str(brightness)+":1" + chr(13))
                self._sck.disconnect()
            except Exception as e:
                LogObj.logSystemError()
        return bOK

    def light_on(self, iChannel=4, brightness=-1):
        return self.light_switch(True, iChannel, brightness)

    def light_off(self, iChannel=4, brightness=-1):
        return self.light_switch(False, iChannel, brightness)

    def light_adjust(self, brightness=25, iChannel=4):
        if brightness < 0: brightness = 0
        if brightness > 255: brightness = 255

        bOK = False
        try:
            if self._sck.connect(self._ip, self._port):
                print("开灯command："+"@setup#"+str(iChannel)+":0:"+str(brightness)+":0"+chr(13))
                bOK, rtn = self._sck.send("@setup#"+str(iChannel)+":0:"+str(brightness)+":0"+chr(13))
            self._sck.disconnect()
        except Exception as e:
            LogObj.logSystemError()
        return bOK
    def new_light_adjust(self, brightness=25, iChannel=4):
        if brightness < 0: brightness = 0
        if brightness > 255: brightness = 255

        rtn = " @setup#"+str(iChannel)+":0:"+str(brightness)+":0"+chr(13)+"\n"
        return rtn

    def _get_brightness(self, iChannel=4):
        brightness = -1
        try:
            if self._sck.connect(self._ip, self._port):
                bOK, rtn = self._sck.send("@query#"+str(iChannel)+chr(13))
                if bOK: brightness = int(rtn.split(":")[1])
            self._sck.disconnect()
        except Exception as e:
            LogObj.logSystemError()
            brightness = -1
        return brightness