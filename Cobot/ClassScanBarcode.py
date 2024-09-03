from ClassCamera import CameraLogitech, LightCtrlLCPW
import cv2
import pyzbar.pyzbar as pyzbar
import time
import LogHelper as LogObj
import numpy as np
CamCtrl = CameraLogitech()
_camera_ctrl = CameraLogitech()
_light_ctrl = LightCtrlLCPW()
_home_dir = 'E:/CobotHome/Barcode_Scan/'
class ScanBarcode:
    def decodeDisplay(self,image):
        # cv2.imwrite(_home_dir + "fixture_barcode_" + LogObj.getCurrTime() + ".jpg", image)
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
            # 提取条形码的边界框的位置
            # 画出图像中条形码的边界框
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.imwrite(_home_dir+"fixture_barcode_success"+LogObj.getCurrTime()+".jpg", image)
            # 条形码数据为字节对象，所以如果我们想在输出图像上
            # 画出来，就需要先将它转换成字符串
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type

            # 绘出图像上条形码的数据和条形码类型
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)

            # 向终端打印条形码数据和条形码类型
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image,barcodeData


    def detect(self,light,scanTime = 2):
        # camera = cv2.VideoCapture(0)
        barcodeData = "None"
        suceess = False

        camera = CamCtrl.openCamera()
        CamCtrl.cameraSetting("Barcode_Scan", False)
        _light_ctrl.light_adjust(light)
        startTime = time.time()
        endTime = time.time()
        while (endTime-startTime)<=scanTime:
            # 读取当前帧
            ret, frame = camera.read()
            cv2.imshow("1",frame)
            # frame = cv2.imread("E:/CobotHome/Barcode_Scan/fixture.jpg")
            # 转为灰度图像

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # gray = cv2.convertScaleAbs(gray, 1.5, 0)

            try:
                im,barcodeData = self.decodeDisplay(gray)
            except Exception as e:
                suceess = False
                barcodeData = "None"
            cv2.waitKey(5)
            # cv2.imshow("camera", im)
            if(not barcodeData is None and barcodeData !="None"):
                suceess = True
                break

            # key = cv2.waitKey(1) & 0xFF
            # # 当用户按下q键时退出整个循环q
            # if key == ord("q"):
            #     break
            endTime = time.time()
        _light_ctrl.light_off()
        _camera_ctrl.closeCamera()
        # camera.release()
        # cv2.destroyAllWindows()
        print("suceess:"+str(suceess)+"   barcodeData:"+barcodeData)
        return suceess,barcodeData
if __name__ == '__main__':
    sc = ScanBarcode()
    sc.detect(60,20)