import time
import threading
import cv2 as cv
import numpy as np
import math
import LogHelper as LogObj
import UtilsMath as mathObj
import UtilsFunc as FuncObj
import matplotlib.pyplot as plot
from ClassCamera import CameraLogitech, LightCtrlLCPW
from ClassArm import ArmUR
from ClassConfig import CobotCfg


#Singleton class
class MarkerFixture(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerCamera'
    _read_img_from_file = False
    _stop_showing_img = False
    _camera_ctrl = CameraLogitech()
    _light_ctrl = LightCtrlLCPW()
    _cobot_cfg = CobotCfg()
    _config_dat = {}
    _arm_ctrl = ArmUR()

    def __init__(self, *args, **kwargs):
        with MarkerFixture._instance_lock:
            pass

    def __del__(self):
        cv.destroyAllWindows()

    def __new__(cls, *args, **kwargs):
        with MarkerFixture._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    MarkerFixture._instance = super().__new__(cls)
            return MarkerFixture._instance

    def _getImgWH(self, image):
        if self._read_img_from_file:
            w = len(image[0])
            h = len(image)
        else:
            sp = image.shape
            w = sp[1]
            h = sp[0]
        return w, h

    def _canny(self, img):
        img_blur = cv.blur(img, (3, 3))
        dst_canny = cv.Canny(img_blur, 0, 200, 3)
        return dst_canny

    def _sobel(self, img):
        ddepth = cv.CV_16S
        scale = 1
        delta = 0
        grad_x = cv.Sobel(img, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
        grad_y = cv.Sobel(img, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
        abs_grad_x = cv.convertScaleAbs(grad_x)
        abs_grad_y = cv.convertScaleAbs(grad_y)
        grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        return grad

    def _laplacian(self, img):
        dst = cv.Laplacian(img, cv.CV_16S, ksize=3)
        abs_lap = cv.convertScaleAbs(dst)
        return abs_lap

    def _show_imgs(self, win_name="win_name", img = None, scale=1):
        if not img is None:
            #add by Hui Zhi 2022/7/28
            # h = len(img)
            # w = len(img[0])
            w, h = self._getImgWH(img)
            cv.namedWindow(win_name, 1)
            # cv.namedWindow(win_name, cv.WINDOW_AUTOSIZE)
            cv.resizeWindow(win_name, int(w*scale), int(h*scale))
            #end
            cv.imshow(win_name, img)

    # def _show_imgs(self, win_name="win_name", img = None):
    #     if not img is None: cv.imshow(win_name, img)

    def stop_showing_img(self, bEnabled = False):
        self._stop_showing_img = bEnabled
    
    def _take_picture(self):
        frame = None
        for i in range(3):
            frame = self._camera_ctrl.takePicture()  # Turn on camera and take first picture
            if frame is None:
                self._camera_ctrl.closeCamera()
                time.sleep(1)
                self._camera_ctrl.openCamera()
                time.sleep(1)
            else:
                break
        return frame

    def adj_light_brightness(self, ctrs = [], grayTarget = -1):
        bOK = False
        if len(ctrs) <= 0 or grayTarget < 0:
            print("Invalid input paras")
            return bOK

        frame = self._take_picture() #Turn on camera and take first picture
        if frame is None:
            print("Can't receive frame (stream end?). Exiting ...")
            return bOK

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        grayVal = self._getCtrsAvgGrays(ctrs, gray)
        print("Mean Gray", grayVal)

        #calBrightness = [i for i in range(2,50)]
        calBrightness = [i for i in range(40, 50)]
        corrPts = []
        if not self._light_ctrl.light_on():
            print("light_on failed")
            return bOK

        for brightness in calBrightness:
            if self._light_ctrl.light_adjust(brightness):
                time.sleep(2) #Wait camera to do focus
                frame = self._take_picture()
                if frame is None:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                grayVal = self._getCtrsAvgGrays(ctrs, gray)
                corrPts.append([grayVal, brightness])
                if grayVal >= grayTarget:
                    bOK = True
                    break
        
        dtPts = len(corrPts)
        if dtPts > 0:
            iBrightness = corrPts[dtPts - 1][1]
            print("grays", corrPts)
            print("G&B: " + str(grayVal) + "," + str(iBrightness))
        else:
            self._light_ctrl.light_off()
        return bOK, grayVal
    
    def cal_marker_position(self, bShowImg = False, timeout_s = 5, markerName = "CircleBasedMkr_01", bSaveImg=False, bChkEnvLight=False):
        markerPos = []; ngInfo = []; draw_img = None; mkrsInfo = []
        config = FuncObj.getDictVal(self._config_dat, markerName, None)
        if config is None:
            config = self._cobot_cfg.get_marker_cfg(markerName)
            self._config_dat[markerName] = config
        roi_x_start = int(FuncObj.getDictVal(config, "roi_x_start(%)", "0"))
        roi_x_stop = int(FuncObj.getDictVal(config, "roi_x_stop(%)", "85"))
        roi_y_start = int(FuncObj.getDictVal(config, "roi_y_start(%)", "35"))
        roi_y_stop = int(FuncObj.getDictVal(config, "roi_y_stop(%)", "75"))
        gray_target = int(FuncObj.getDictVal(config, "gray_target", "80"))
        mkr_max_chg = int(FuncObj.getDictVal(config, "mkr_max_chg(pixel)", "2"))
        proc_method = FuncObj.getDictVal(config, "proc_method", "circles_01")
        procMethods = ["circles_01", "circles_02", "circles_03", "polygon_01"]

        frame = self._take_picture()

        if frame is None:
            print("Can't receive frame (stream end?). Exiting ...")
            LogObj.logAppError("MkrPos", "Fail to take picture")
            return None, [], None, [], []

        # add by Hui Zhi 2021.11.8
        self._camera_ctrl.cameraSetting(markerName,bShowImg)

        #location = templateMacthing(frame, template)
        w, h = self._getImgWH(frame)
        ctrRegion = [[int(w * roi_x_start / 100), int(h * roi_y_start / 100)], [int(w * roi_x_stop / 100), int(h * roi_y_stop / 100)]]

        startT = time.time()
        iLightAdjTimes = 0
        thrVSmkrcnt = {}
        while True:
            roi = [[ctrRegion[0][0], ctrRegion[0][1]], [ctrRegion[1][0], ctrRegion[1][1]]]
            frame = self._take_picture()
            if frame is None:
                print("Can't receive frame (stream end?). Exiting ...")
                LogObj.logAppError("MkrPos", "Fail to take picture")
                break

            if proc_method in procMethods:
                if proc_method.startswith("circles_"):
                    # binaryThr = 100
                    binaryThr = 140
                    if iLightAdjTimes>0 and len(markerPos)<=0:#Target marker not found after light adjustment
                        if iLightAdjTimes<5 and len(mkrsInfo)>0 and grayVal>=gray_target and grayVal<128: binaryThr = grayVal
                        if iLightAdjTimes>=5:
                            maxValidMkrs = thrVSmkrcnt[100]; minValidMkrs = thrVSmkrcnt[100]; maxMkrsThr = 100; minMkrsThr = 100
                            for thr in thrVSmkrcnt.keys():
                                if thrVSmkrcnt[thr] > maxValidMkrs:
                                    maxMkrsThr = thr
                                    maxValidMkrs = thrVSmkrcnt[thr]
                                if thrVSmkrcnt[thr] < minValidMkrs:
                                    minMkrsThr = thr
                                    minValidMkrs = thrVSmkrcnt[thr]
                            if maxMkrsThr >= minMkrsThr: #Larger binary threshold is better
                                binaryThr = maxMkrsThr
                            else: #Smaller binary threshold is better
                                binaryThr = maxMkrsThr - 1
                    draw_img, markerPos, ngInfo, mkrsInfo = self.getMarkerPosByCircles(frame, roi, config, proc_method, bShowImg, bSaveImg, binaryThr)
                    thrVSmkrcnt[binaryThr] = (0 if(len(mkrsInfo)<=0) else len(mkrsInfo[0]))
                    if bChkEnvLight:
                        if len(mkrsInfo) > 0:
                            maxR = mkrsInfo[2][0]; minR = mkrsInfo[2][1]; mkrSimilarity1 = mkrsInfo[2][2]; mkrSimilarity2 = mkrsInfo[2][3]; mkrSimilarity3 = mkrsInfo[2][4]
                            if (len(markerPos) <= 0 or maxR-minR > mkr_max_chg or (not (mkrSimilarity1 > 0.5 and mkrSimilarity2 > 0.99 or mkrSimilarity3 > 0.85))) and iLightAdjTimes < 10:
                                cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_00.jpg", frame)
                                cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_01.jpg", draw_img)
                                if len(markerPos) <= 0:
                                    bLightAdjOK, grayVal = self.adj_light_brightness([[[ctrRegion[0][0], ctrRegion[0][1]], [ctrRegion[1][0], ctrRegion[1][1]]]], gray_target)
                                else:
                                    bLightAdjOK, grayVal = self.adj_light_brightness(mkrsInfo[3], gray_target)
                                if bLightAdjOK:
                                    iLightAdjTimes = iLightAdjTimes + 1
                                    continue
                        else:
                            if iLightAdjTimes < 10:
                                cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_00.jpg", frame)
                                cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_01.jpg", draw_img)
                                bLightAdjOK, grayVal = self.adj_light_brightness([[[ctrRegion[0][0], ctrRegion[0][1]], [ctrRegion[1][0], ctrRegion[1][1]]]], gray_target)
                                if bLightAdjOK:
                                    iLightAdjTimes = iLightAdjTimes + 1
                                    continue
                    if iLightAdjTimes > 0:
                        cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_02.jpg", frame)
                        cv.imwrite(self._home_dir + "/" + "LightAdj_" + str(iLightAdjTimes) + "_03.jpg", draw_img)
                if "polygon_01" == proc_method:
                    draw_img, markerPos = self.getMarkerPositionByPolygon(frame, roi, config, bShowImg)
            else:
                print("Invalid process method: " + proc_method)
                return None, [], None, [], []

            procT = time.time() - startT
            if len(markerPos) > 0:
                if bShowImg or bSaveImg:
                    cv.putText(draw_img,"ProcT: " + str(round(procT, 3)) + "s", (20, 10), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv.LINE_AA)
                    cv.putText(draw_img, "ProcR: " + str(markerPos[0]) + "," + str(markerPos[1]), (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv.LINE_AA)
                if timeout_s < 0:
                    startT = time.time()
                else:
                    break
            else:
                cv.putText(draw_img, "ProcT: " + str(round(procT, 3)) + "s", (20, 10), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)

            if bShowImg:
                self._show_imgs("Marker Position", draw_img)
                key = cv.waitKey(1)
                if key == ord('q') or self._stop_showing_img:
                    break
                elif key == ord('s'):# add by Hui Zhi 2022/8/5
                    t = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
                    cv.imwrite('E:/CobotHome/Picture/temp/' + t + '-0.jpg', frame)
                    cv.imwrite('E:/CobotHome/Picture/temp/' + t + '.jpg', draw_img)
                    break
            if timeout_s < 0:
                if procT > 3600: startT = time.time()
            else:
                if procT > timeout_s: break
        return draw_img, markerPos, frame, ngInfo, mkrsInfo

    def getMaxContour(self, contours):
        maxContour = None
        if not contours is None:
            maxArea = 0
            for c in contours:
                x, y, w, h = cv.boundingRect(c)
                if maxArea < (w * h):
                    maxArea = w * h
                    maxContour = c
        return maxContour

    def getMarkerPosByCircles(self, img, checkRegion, config, procMethod, bShowImg=False, bSaveImg=True, binaryThr=-1):
        markerPos = []; ngInfo = []
        validCirclesInfo = []; validCircles = []; validCtrs = []
        circleMinR = int(FuncObj.getDictVal(config, "circle_min_r(pixel)", "8"))
        circleMaxR = int(FuncObj.getDictVal(config, "circle_max_r(pixel)", "24"))
        circlesQty = int(FuncObj.getDictVal(config, "circle_qty", "8"))
        markerCenterXR = float(FuncObj.getDictVal(config, "marker_centerx_rate", "0.6"))
        isVertical = int(FuncObj.getDictVal(config, "is_vertical", "0"))

        draw_img = cv.rectangle(img.copy(), (checkRegion[0][0],checkRegion[0][1]), (checkRegion[1][0],checkRegion[1][1]), (0, 0, 0), 1)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # gray = cv.GaussianBlur(gray, (7, 7), 0)
        gray = cv.GaussianBlur(gray, (5, 5), 0) # 2021/11/9
        if binaryThr < 0: binaryThr = 100

        # skip canny by Hui Zhi
        # binary = cv.threshold(gray, binaryThr, 255, cv.THRESH_BINARY)[1]
        # cny = self._canny(binary)
        # if bShowImg: self._show_imgs("Cany", cny)
        # chkRegion = cny[checkRegion[0][1]:checkRegion[1][1], checkRegion[0][0]:checkRegion[1][0]]

        # add by Hui Zhi  2022/8/3
        binary = cv.threshold(gray, binaryThr, 255, cv.THRESH_BINARY_INV)[1]
        if bShowImg: self._show_imgs("Binary", binary)
        cny = binary #skip canny
        chkRegion = cny[checkRegion[0][1]:checkRegion[1][1], checkRegion[0][0]:checkRegion[1][0]]
        # plot.imshow(img)
        # plot.show()
        #end

        #circles, contours = self._getFittingCircles(chkRegion, 0.9, circleMinR, circleMaxR)
        circles, contours = self._getFittingCircles(chkRegion, 0.9, circleMinR, circleMaxR)  # change to 0.8 by Hui Zhi
        if len(circles) <= 0 and (not contours is None):
            maxContour = self.getMaxContour(contours)
            if not maxContour is None:
                x, y, w, h = cv.boundingRect(maxContour)
                checkRegion[0][0] = checkRegion[0][0] + x + 20
                checkRegion[0][1] = checkRegion[0][1] + y + 20
                x = checkRegion[0][0]
                y = checkRegion[0][1]
                chkRegion = cny[y:(y+h), x:(x+w)]
                #circles, contours = self._getFittingCircles(chkRegion, 0.9, circleMinR, circleMaxR)
                circles, contours = self._getFittingCircles(chkRegion, 0.9, circleMinR, circleMaxR)  # change to 0.8 by Hui Zhi
        #circles, invalids = self._getValidCircles(circles, 0.85, procMethod)
        circles, invalids = self._getValidCircles(circles, 0.85, procMethod)    # change to 0.8 by Hui Zhi

        validCnt = len(circles)
        if validCnt > 0: #Store circles info for light adjustment in the future if necessary
            minR = circles[0][2]; maxR = minR; midR1 = minR; midR2 = minR; sameCircleCnt1 = 0.0; sameCircleCnt2 = 0.0
            sameCircleCnt3 = 0.0; sameCircleCnt4 = 0.0; sameCircleCnt5 = 0.0; sameCircleCnt6 = 0.0
            for i in range(validCnt):
                x = circles[i][0] + checkRegion[0][0]
                y = circles[i][1] + checkRegion[0][1]
                if minR > circles[i][2]:
                    midR1 = minR
                    minR = circles[i][2]
                if maxR < circles[i][2]:
                    midR2 = maxR
                    maxR = circles[i][2]
                validCircles.append([x, y, circles[i][2]])
                validCtrs.append([(x-circles[i][2],y-circles[i][2]), (x+circles[i][2],y+circles[i][2])])
            for i in range(validCnt):
                if maxR - circles[i][2] <= 1: sameCircleCnt1 = sameCircleCnt1 + 1.0
                if circles[i][2] - minR <= 1: sameCircleCnt2 = sameCircleCnt2 + 1.0
                if maxR - circles[i][2] <= 2: sameCircleCnt3 = sameCircleCnt3 + 1.0
                if circles[i][2] - minR <= 2: sameCircleCnt4 = sameCircleCnt4 + 1.0
                if abs(midR1 - circles[i][2]) <= 1: sameCircleCnt5 = sameCircleCnt5 + 1.0
                if abs(midR2 - circles[i][2]) <= 1: sameCircleCnt6 = sameCircleCnt6 + 1.0
            r1 = sameCircleCnt1 / validCnt
            r2 = sameCircleCnt2 / validCnt
            r3 = sameCircleCnt3 / validCnt
            r4 = sameCircleCnt4 / validCnt
            r5 = sameCircleCnt5 / validCnt
            r6 = sameCircleCnt6 / validCnt
            sameCircleR1 = (r1 if r1 > r2 else r2)
            sameCircleR2 = (r3 if r3 > r4 else r4)
            sameCircleR3 = (r5 if r5 > r6 else r6)
            avgGray = self._getCtrsAvgGrays(validCtrs, gray)
            validCirclesInfo = [validCircles, avgGray, [maxR, minR, sameCircleR1, sameCircleR2, sameCircleR3], validCtrs, gray]
        
        if validCnt == circlesQty:
            # markerPos, posLinePts = self._calcCircleMarkerPos(circles, checkRegion, markerCenterXR, procMethod)
            markerPos, posLinePts = self._calcCircleMarkerPos(circles, checkRegion, markerCenterXR, procMethod, isVertical) #change by Hui Zhi 2022/8/5
        else:
            # mkrPos, posLinePts = self._calcCircleMarkerPos(circles, checkRegion, markerCenterXR, procMethod)
            mkrPos, posLinePts = self._calcCircleMarkerPos(circles, checkRegion, markerCenterXR, procMethod, isVertical)
            ngInfo = [mkrPos, posLinePts]
            cv.putText(draw_img, "Circles QTY NG: " + str(validCnt), (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)

        if validCnt > 0 and (len(markerPos)<=0 or len(markerPos)>0 and (bShowImg or bSaveImg)):
            for i in range(validCnt):
                x = circles[i][0] + checkRegion[0][0]
                y = circles[i][1] + checkRegion[0][1]
                if len(markerPos) > 0:
                    cv.circle(draw_img, (x, y), circles[i][2], (0, 255, 0), 2)
                    # add by Hui Zhi 2022/8/15
                    cv.putText(draw_img,str((x,y)),(x+30, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.4, (180, 0, 0), 1, cv.LINE_AA)
                else:
                    cv.circle(draw_img, (x, y), circles[i][2], (0, 0, 255), 2)
                cv.circle(draw_img, (x, y), 2, (0, 0, 255), 2)
                cv.putText(draw_img, str(circles[i][2]), (x - 5, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_AA)
            cv.putText(draw_img, "thr: " + str(binaryThr), (20, 40), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_AA)
            if len(markerPos) > 1: cv.circle(draw_img, (markerPos[1][0], markerPos[1][1]), 2, (0, 255, 0), 2)
            if len(posLinePts) > 2:
                cv.line(draw_img, (posLinePts[0][0], posLinePts[0][1]), (posLinePts[1][0], posLinePts[1][1]), (0, 255, 0), 1) #Circle centers fitting line
                cv.line(draw_img, (posLinePts[0][0], posLinePts[0][1]), (posLinePts[2][0], posLinePts[2][1]), (255, 0, 0), 1) #Horizontal-reference line
        if len(markerPos)<=0:
            if len(invalids) > 0:
                for cc in invalids:
                    cv.circle(draw_img, (cc[0] + checkRegion[0][0], cc[1] + checkRegion[0][1]), cc[2], (255, 0, 0), 2)
            if not contours is None:
                for ctr in contours:
                    x, y, w, h = cv.boundingRect(ctr)
                    x = x + checkRegion[0][0]; y = y + checkRegion[0][1]
                    cv.rectangle(draw_img, (x, y), (x + w, y + h), (0, 255, 255), 1)
        return draw_img, markerPos, ngInfo, validCirclesInfo

    def _calcCircleMarkerPos(self, circles, checkRegion, markerCenterXR, procMethod = "circles_01", isVertical=False):
        markerPos = []; posLinePts = []
        validCnt = len(circles)
        if validCnt > 1: # Calculate the marker _center and rotating angle
            positionLine = []
            if isVertical:  # add by Hui Zhi 2022/8/3
                firstEdgeY = circles[0][1] + checkRegion[0][1]
                lastEdgeY = circles[0][1] + checkRegion[0][1]
                for i in range(validCnt):
                    x = circles[i][0] + checkRegion[0][0]
                    y = circles[i][1] + checkRegion[0][1]

                    if firstEdgeY > y: firstEdgeY = y
                    if lastEdgeY < y: lastEdgeY = y
                    positionLine.append([y, x])

                slope, offset, rsq, sParas = mathObj.lineFitting(positionLine)
                # print(f"slope={slope},offset={offset}")
                vectorOrigin = [int(slope * firstEdgeY + offset), firstEdgeY]
                vectorPosLine = [int(slope * lastEdgeY + offset), lastEdgeY]
                vectorVerical = [vectorOrigin[0], lastEdgeY]
                v1 = [vectorOrigin[0], vectorOrigin[1], vectorVerical[0], vectorVerical[1]]
                v2 = [vectorOrigin[0], vectorOrigin[1], vectorPosLine[0], vectorPosLine[1]]

                angleDir = 1
                if vectorPosLine[0] <vectorVerical[0]: angleDir = -1
                positionAngle = round(mathObj.vectorCrossAngle(v1, v2), 3) * angleDir
                if "circles_02" == procMethod:  # Circles are standing in a round shape
                    (y, x), r = cv.minEnclosingCircle(np.array(positionLine))
                else:  # Circles are standing in a line or in corners of a square shape
                    y = int(firstEdgeY + abs(lastEdgeY - firstEdgeY) * markerCenterXR)
                    x = int(slope * y + offset)
                center = [int(x), int(y)]
                markerPos = [positionAngle, center]
                posLinePts = [[vectorOrigin[0], vectorOrigin[1]], [vectorPosLine[0], vectorPosLine[1]],
                              [vectorVerical[0], vectorVerical[1]]]
                # print(f"pos(y,x)={positionLine}")
                print(f"marker angle ={markerPos[0]},center={markerPos[1]}")
            else:
                leftEdgeX = circles[0][0] + checkRegion[0][0]
                rightEdgeX = circles[0][0] + checkRegion[0][0]
                for i in range(validCnt):
                    x = circles[i][0] + checkRegion[0][0]
                    y = circles[i][1] + checkRegion[0][1]

                    if leftEdgeX > x: leftEdgeX = x
                    if rightEdgeX < x: rightEdgeX = x
                    positionLine.append([x, y])

                slope, offset, rsq, sParas = mathObj.lineFitting(positionLine)
                vectorOrigin = [leftEdgeX, int(slope * leftEdgeX + offset)]
                vectorPosLine = [rightEdgeX, int(slope * rightEdgeX + offset)]
                vectorHorizontal = [rightEdgeX, vectorOrigin[1]]
                v1 = [vectorOrigin[0], vectorOrigin[1], vectorHorizontal[0], vectorHorizontal[1]]
                v2 = [vectorOrigin[0], vectorOrigin[1], vectorPosLine[0], vectorPosLine[1]]

                angleDir = 1
                if vectorPosLine[1] < vectorHorizontal[1]: angleDir = -1
                positionAngle = round(mathObj.vectorCrossAngle(v1, v2), 3) * angleDir
                if "circles_02" == procMethod: #Circles are standing in a round shape
                    (x, y), r = cv.minEnclosingCircle(np.array(positionLine))
                else: #Circles are standing in a line or in corners of a square shape
                    x = int(leftEdgeX + abs(rightEdgeX - leftEdgeX) * markerCenterXR)
                    y = int(slope * x + offset)
                center = [int(x), int(y)]
                markerPos = [positionAngle, center]
                posLinePts = [[vectorOrigin[0], vectorOrigin[1]], [vectorPosLine[0], vectorPosLine[1]], [vectorHorizontal[0], vectorHorizontal[1]]]

        return markerPos, posLinePts

    def _getFittingCircles(self, canyImg, minSimilarity=0.9, circleMinR=-1, circleMaxR=-1):
        circles = []
        # contours, hierarchy = cv.findContours(canyImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv.findContours(canyImg, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if not contours is None:
            cnts = []
            for cnt in contours:
                bRadiusOK = True
                (x, y), radius = cv.minEnclosingCircle(cnt)
                if circleMinR > 0 and circleMaxR > 0:
                    if radius < circleMinR or radius > circleMaxR:
                            bRadiusOK = False
                    else :
                        x1,y1,w1,h1 = cv.boundingRect(cnt)

                        if not (circleMaxR >= h1 / 2 >= circleMinR and circleMaxR >= w1 / 2 >= circleMinR) :
                            bRadiusOK = False
                        else :
                            #print(w1, h1, cv.contourArea(cnt), radius,(radius * radius * 3.1415926 ))
                            if cv.contourArea(cnt)/(radius * radius * 3.1415926 ) < 0.8 :
                                bRadiusOK = False

                if bRadiusOK:
                    perimeter = cv.arcLength(cnt, True)
                    r = perimeter / (2 * 3.1415926)
                    ratio = r / radius
                    if ratio > 1: ratio = 1 / ratio
                    if ratio >= minSimilarity:
                        circles.append([int(x), int(y), int(radius)])
                        cnts.append(cnt)
        return circles, cnts #contours

    def _getValidCircles(self, circles, sameCircleMinRatio=0.95, procMethod="circles_01"):
        validCircles = []; invalidCircles = []
        cnt = len(circles)
        sameCircleQty = {}
        if cnt > 0:
            for i in range(cnt):#Get the same circles(brothers) for each circle via radius comparison
                sameIdx = []
                for j in range(cnt):
                    if i != j:
                        ratio = circles[i][2] / circles[j][2] + 0.0
                        if ratio > 1: ratio = 1 / ratio
                        if ratio > sameCircleMinRatio: sameIdx.append(j)
                sameCircleQty[i] = sameIdx

            maxIdx = -1; maxCnt = 0
            for i in range(cnt): #Get the circle which has the most brothers
                sameQty = len(sameCircleQty[i])
                if sameQty > 0 and maxCnt < sameQty:
                    maxCnt = sameQty
                    maxIdx = i

            if maxIdx >= 0: #The most popular circles are the valid ones
                validCircles.append(circles[maxIdx])
                for idx in sameCircleQty[maxIdx]:
                    validCircles.append([int(circles[idx][0]), int(circles[idx][1]), int(circles[idx][2])])

                cnt = len(validCircles)
                if cnt > 2: #Sort the circles by their relative position
                    dminC2C = []; dC2Cs = []
                    for i in range(cnt):
                        minD = 40960
                        ds = {}
                        for j in range(cnt):
                            if i != j:
                                d = mathObj.calTwoPointsDistance([int(validCircles[i][0]), int(validCircles[i][1])], [int(validCircles[j][0]), int(validCircles[j][1])])
                                if minD > d: minD = d
                                ds[j] = d
                        dminC2C.append(minD) #Distance to the most closed brother
                        dC2Cs.append(ds) #Distances to all brothers

                    maxCnt = 0; sameCnts = []
                    for i in range(cnt):
                        sameCnt = 0
                        for j in range(cnt):
                            if i != j:
                                ratio = dminC2C[i] / dminC2C[j]
                                if ratio > 1: ratio = 1 / ratio
                                if ratio > sameCircleMinRatio: sameCnt = sameCnt + 1
                        if sameCnt > maxCnt: maxCnt = sameCnt
                        sameCnts.append(sameCnt)

                    distanceC2C = 0.0; cntr = 0
                    for i in range(cnt):
                        if maxCnt == sameCnts[i]:
                            cntr = cntr + 1
                            distanceC2C += dminC2C[i]
                    distanceC2C = distanceC2C / cntr  #Get the valid _center to _center distance btw two most closed circles

                    #Double check the valid circles
                    validCircles, invalidCircles = self._classify_circles(validCircles, distanceC2C, dC2Cs, sameCircleMinRatio, procMethod)
            else:
                invalidCircles = circles
        return validCircles, invalidCircles

    def _classify_circles(self, validCircles, distanceC2C, dC2Cs, sameCircleMinRatio, procMethod):
        cnt = len(validCircles)
        invalidCircles = []
        validFlag = {}; xyPts = []; sameCnts = []; closedIdx = {}
        for i in range(cnt):
            validFlag[i] = False

        for i in range(cnt):
            sameCnt = 0; nearXY = []; nearIdx = []
            nearXY.append([validCircles[i][0], validCircles[i][1]])
            for j in dC2Cs[i].keys():
                ratio = distanceC2C / dC2Cs[i][j]
                if ratio > 1: ratio = 1 / ratio
                if ratio > sameCircleMinRatio:
                    sameCnt = sameCnt + 1
                    nearXY.append([validCircles[j][0], validCircles[j][1]])  # The most closed neighbour
                    nearIdx.append(j)
            closedIdx[i] = nearIdx
            sameCnts.append(sameCnt)
            if sameCnt == 2:  # Get the valid circle which has only two neighbours
                slope, offset, rsq, ls = mathObj.lineFitting(nearXY)
                ratio = mathObj.calTwoPointsDistance(nearXY[1], nearXY[2]) / distanceC2C
                if ("circles_01" == procMethod and ratio > 1.9 and ratio < 2.1): #Adjust rsq
                    minX = nearXY[0][0]; maxX = minX; minY = nearXY[0][1]; maxY = minY
                    for xy in nearXY:
                        if minX > xy[0]: minX = xy[0]
                        if maxX < xy[0]: maxX = xy[0]
                        if minY > xy[1]: minY = xy[1]
                        if maxY < xy[1]: maxY = xy[1]
                    if maxX - minX < 2 or maxY - minY < 2: rsq = 1.0 #Too small range will get worse R square
                if ("circles_01" == procMethod and rsq > 0.9 and ratio > 1.9 and ratio < 2.1) or "circles_02" == procMethod and rsq > 0.6 and ratio > 1.79 and ratio < 2.0:
                    # The three-closed circles are standing in a straight line
                    validFlag[i] = True
                    xyPts.append([validCircles[i][0], validCircles[i][1]])

        ccs = []
        if "circles_01" == procMethod: #markers of "circles_01" are standing in a straight line
            if len(xyPts) > 2:
                slope, offset, rsq, ls = mathObj.lineFitting(xyPts)
                for i in range(cnt):
                    if not validFlag[i]:
                        d = mathObj.calcPointToLineDistance(xyPts, [validCircles[i][0], validCircles[i][1]], [slope, offset, rsq])
                        if d * 10 < distanceC2C and sameCnts[i] > 0: validFlag[i] = True  # Adjust the valid circles(_center should be very closed)
            else:
                for i in range(cnt):
                    if not validFlag[i] and sameCnts[i] > 0: validFlag[i] = True

        if "circles_02" == procMethod or "circles_03" == procMethod: #markers of "circles_02" are in a circle shape; markers of "circles_03" are in the coners of a square shape
            for i in range(cnt):
                if 2 == len(closedIdx[i]):
                    if (not validFlag[closedIdx[i][0]] and sameCnts[closedIdx[i][0]] > 0) or (not validFlag[closedIdx[i][1]] and sameCnts[closedIdx[i][1]] > 0):
                        if (not validFlag[closedIdx[i][0]] and sameCnts[closedIdx[i][0]] > 0): xyPts.append([validCircles[closedIdx[i][0]][0], validCircles[closedIdx[i][0]][1]])
                        if (not validFlag[closedIdx[i][1]] and sameCnts[closedIdx[i][1]] > 0): xyPts.append([validCircles[closedIdx[i][1]][0], validCircles[closedIdx[i][1]][1]])
            if len(xyPts) > 2:
                (x, y), radius = cv.minEnclosingCircle(np.array(xyPts))
                d2cs = {}
                validCnt = 0
                for i in range(cnt):  # Double confirmed all valide circles
                    ratio = mathObj.calTwoPointsDistance((x, y), (validCircles[i][0], validCircles[i][1]))
                    ratio = ratio / radius
                    if ratio > 1: ratio = 1 / ratio
                    if ratio > 0.9:
                        validFlag[i] = True  # Center on edge of the big circle
                        d2cs[i] = abs(y - validCircles[i][1])
                        validCnt = validCnt + 1
                    else:
                        validFlag[i] = False
                        d2cs[i] = y
                # Sort validCircles by d2cs
                for i in range(cnt - 1):
                    for j in range(i + 1, cnt):
                        if d2cs[i] > d2cs[j]:
                            d2cs[i], d2cs[j] = FuncObj.swapVal(d2cs[i], d2cs[j])
                            validCircles[i], validCircles[j] = FuncObj.swapVal(validCircles[i], validCircles[j])
                            validFlag[i], validFlag[j] = FuncObj.swapVal(validFlag[i], validFlag[j])
                # Get the four-most-closed-to _center(x,y) of big circle
                fourPts = []
                for i in range(cnt):
                    if len(fourPts) >= 4: break
                    if validFlag[i]: fourPts.append([validCircles[i][0], validCircles[i][1]])
                if len(fourPts) == 4:
                    for i in range(3):
                        for j in range(i + 1, 4):
                            if fourPts[i][0] > fourPts[j][0]:
                                fourPts[i], fourPts[j] = FuncObj.swapVal(fourPts[i], fourPts[j])
                    if fourPts[0][1] > fourPts[1][1]: fourPts[0], fourPts[1] = FuncObj.swapVal(fourPts[0], fourPts[1])
                    if fourPts[2][1] > fourPts[3][1]: fourPts[2], fourPts[3] = FuncObj.swapVal(fourPts[2], fourPts[3])
                    r1 = mathObj.calTwoPointsDistance(fourPts[0], fourPts[3]) / radius
                    r2 = mathObj.calTwoPointsDistance(fourPts[1], fourPts[2]) / radius
                    r3 = mathObj.calTwoPointsDistance(fourPts[0], fourPts[1]) / abs(fourPts[0][1] - y)
                    r4 = mathObj.calTwoPointsDistance(fourPts[0], fourPts[1]) / abs(fourPts[1][1] - y)
                    v1 = [fourPts[1][0], fourPts[1][1], fourPts[2][0], fourPts[2][1]]
                    v2 = [fourPts[0][0], fourPts[0][1], fourPts[3][0], fourPts[3][1]]
                    crossAngle = round(mathObj.vectorCrossAngle(v1, v2), 3)
                    if r1 > 1.85 and r2 > 1.85 and r3 > 1.0 and r4 > 1.0 and ("circles_02" == procMethod and crossAngle > 40 and crossAngle < 50 or "circles_03" == procMethod and crossAngle > 85 and crossAngle < 95):  # Finalize the valid circles
                        counter = 0
                        for i in range(cnt):
                            if counter >= 4: validFlag[i] = False
                            if validFlag[i]: counter = counter + 1
                    else:
                        fourPts = {}
                if len(fourPts) < 4 and 4 == validCnt:
                    counter = 0
                    for i in range(cnt):
                        if counter >= 3: validFlag[i] = False
                        if validFlag[i]: counter = counter + 1
        for i in range(cnt):
            if not validFlag[i]: invalidCircles.append(validCircles[i])

        okCnt = 0
        for i in range(cnt):
            if validFlag[i]: okCnt = okCnt + 1
        if okCnt > 0:
            for i in range(cnt):
                if validFlag[i]:
                    ccs.append(validCircles[i])
            validCircles = ccs
        else:
            validCircles = []
        return validCircles, invalidCircles

    def _getGrayHist(self, image):
        hist = {}
        for key in range(256):
            hist[key] = 0

        w, h = self._getImgWH(image)
        for y in range(h):
            for x in range(w):
                grayVal = int(image[y][x])
                hist[grayVal] = hist[grayVal] + 1
        return hist

    def _getAvgGray(self, image):
        avgG = 0.0
        w, h = self._getImgWH(image)
        avgCnt = w * h + 0.0
        for y in range(h):
            for x in range(w):
                avgG = avgG + int(image[y][x]) / avgCnt
        return int(avgG)

    def _getCtrsAvgGrays(self, ctrs, grayImg):
        avgG = 0.0
        ctrCnt = len(ctrs)
        for i in range(ctrCnt):
            x_start = ctrs[i][0][0]
            x_stop = ctrs[i][1][0]
            y_start = ctrs[i][0][1]
            y_stop = ctrs[i][1][1]
            avgG = avgG + self._getAvgGray(grayImg[y_start:y_stop, x_start:x_stop]) / ctrCnt
        return int(avgG)

    def _getBinaryThrFromGrayHist(self, grayHist):
        binThr = -1
        maxCnt = 0.0  # Peak value
        avgCnt = 0.0  # Base Level
        peakIdx = -1
        cnt = len(grayHist) + 0.0
        for idx in grayHist.keys():
            if maxCnt < grayHist[idx]:
                maxCnt = grayHist[idx]
                peakIdx = idx
            avgCnt = avgCnt + grayHist[idx] / cnt

        if peakIdx > 0:
            underAvgCnt = 0
            for idx in range(peakIdx, int(cnt - 1)):
                if grayHist[idx] < avgCnt:
                    underAvgCnt = underAvgCnt + 1
                if underAvgCnt >= 2:
                    binThr = idx
                    break
            if binThr < 0: binThr = 255
        else:
            binThr = 128

        return binThr

    def _exchangeVal(obj_1, obj_2):
        return obj_2, obj_1

    def getMarkerPositionByPolygon(self, img, checkRegion, config, bShowImg=False):
        markerPos = []
        margin_pixels = int(FuncObj.getDictVal(config, "contour_margin(pixel)", "1"))
        margin_angle = int(FuncObj.getDictVal(config, "angle_detection_tol(degree)", "10"))
        margin_crossP = int(FuncObj.getDictVal(config, "corner_detection_tol(pixel)", "10"))
        polygon_primeter = int(FuncObj.getDictVal(config, "polygon_primeter(pixel)", "100"))
        polygon_minSideLength = int(FuncObj.getDictVal(config, "polygon_min_side_len(pixel)", "15"))
        binaryThrFactor = float(FuncObj.getDictVal(config, "binary_thr_factor", "1.0"))
        trianglePrimeterMinR = float(FuncObj.getDictVal(config, "triangle_primeter_min_ratio", "0.85"))
        triangleSpaceMinR = float(FuncObj.getDictVal(config, "triangle_space_min_ratio", "0.9"))
        rectangleSpaceMinR = float(FuncObj.getDictVal(config, "rectangle_space_min_ratio", "0.9"))

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        draw_img = img.copy()

        gray = cv.GaussianBlur(gray, (5, 5), 0)
        grayHist = self._getGrayHist(gray[checkRegion[0][1]:checkRegion[1][1], checkRegion[0][0]:checkRegion[1][0]])
        binaryThr = int(self._getBinaryThrFromGrayHist(grayHist) * binaryThrFactor)

        # gray = self._sobel(gray)
        # gray = self._laplacian(gray)
        binary = cv.threshold(gray, binaryThr, 255, cv.THRESH_BINARY)[1]
        cny = self._canny(binary)
        contours, hierarchy = cv.findContours(cny[checkRegion[0][1] - margin_pixels:checkRegion[1][1]
                                + margin_pixels, checkRegion[0][0]
                                - margin_pixels:checkRegion[1][0]
                                + margin_pixels], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if bShowImg:
            self._show_imgs("Binary", binary)
            self._show_imgs("Cany", cny)
        if not contours is None:
            triangles = []
            rectangle = []
            for ctr in contours:
                x, y, w, h = cv.boundingRect(ctr)
                x = checkRegion[0][0] + x
                y = checkRegion[0][1] + y
                primeter = (w + h) * 2
                minLen = w if w < h else h
                if minLen < polygon_minSideLength or (primeter < polygon_primeter / 2) or (primeter > polygon_primeter * 2):
                    continue

                edgeImg = cny[(y - margin_pixels):(y + h + margin_pixels), (x - margin_pixels):(x + w + margin_pixels)]
                lines = self.searchLines(edgeImg, 1, [x - margin_pixels, y - margin_pixels])
                linesQty = len(lines)

                if linesQty == 3 or linesQty == 4:
                    # Get lines cross points
                    crossPts = []
                    for i in range(0, linesQty - 1):
                        for j in range(i + 1, linesQty):
                            crossP = mathObj.calTwoLinesCrossPoint(lines[i], lines[j])
                            if len(crossP) > 0 and crossP[0] > (x - margin_crossP) and crossP[0] < (x + w + margin_crossP) and crossP[1] > (y - margin_crossP) and crossP[1] < (y + h + margin_crossP):  # Cross point found
                                crossPts.append([int(crossP[0]), int(crossP[1]), int(crossP[2])])
                    ptsCnt = len(crossPts)
                    if ptsCnt <= 0: continue

                    # Check valid cross points via cross angle
                    validPtCnt = 0
                    rightAngles = []
                    validPts = []
                    angles = ""
                    for pt in crossPts:
                        if abs(pt[2] - 180) > margin_angle and pt[2] > margin_angle:  # Not parallel or overlap
                            validPts.append([pt[0], pt[1]])
                            validPtCnt = validPtCnt + 1
                            if abs(pt[2] - 90) <= margin_angle:  # Right Angle
                                rightAngles.append(pt)
                                draw_img = cv.circle(draw_img, (pt[0], pt[1]), 1, (0, 255, 0), 2)
                            else:
                                draw_img = cv.circle(draw_img, (pt[0], pt[1]), 1, (0, 0, 255), 2)
                            angles = angles + "," + str(pt[2])

                    if validPtCnt > 1:  # Sort the data by x coordinate
                        for i in range(validPtCnt - 1):
                            for j in range(i + 1, validPtCnt):
                                if validPts[i][0] > validPts[j][0]:
                                    tmp = validPts[i]
                                    validPts[i] = validPts[j]
                                    validPts[j] = tmp

                    if validPtCnt == 3:  # Triangle detected
                        if bShowImg:
                            cv.line(draw_img, (validPts[0][0], validPts[0][1]), (validPts[1][0], validPts[1][1]), (0, 255, 0), 1)
                            cv.line(draw_img, (validPts[0][0], validPts[0][1]), (validPts[2][0], validPts[2][1]), (0, 255, 0), 1)
                            cv.line(draw_img, (validPts[1][0], validPts[1][1]), (validPts[2][0], validPts[2][1]), (0, 255, 0), 1)
                        primeter = math.sqrt(math.pow(validPts[1][0] - validPts[0][0], 2) + math.pow(validPts[1][1] - validPts[0][1], 2))
                        primeter = primeter + math.sqrt(math.pow(validPts[2][0] - validPts[0][0], 2) + math.pow(validPts[2][1] - validPts[0][1], 2))
                        primeter = primeter + math.sqrt(math.pow(validPts[2][0] - validPts[1][0], 2) + math.pow(validPts[2][1] - validPts[1][1], 2))
                        triangles.append([validPts, primeter, rightAngles])
                    if validPtCnt == 4:  # Rectangle detected
                        if len(rightAngles) > 1:
                            if validPts[0][1] > validPts[1][1]: validPts[0], validPts[1] = self._exchangeVal(validPts[0], validPts[1])
                            if validPts[2][1] > validPts[3][1]: validPts[2], validPts[3] = self._exchangeVal(validPts[2], validPts[3])

                            d1 = math.sqrt(math.pow(validPts[0][0] - validPts[1][0], 2) + math.pow(validPts[0][1] - validPts[1][1], 2))
                            d2 = math.sqrt(math.pow(validPts[0][0] - validPts[2][0], 2) + math.pow(validPts[0][1] - validPts[2][1], 2))
                            primeter = (d1 + d2) * 2
                            diagonalIdx = [0, 3]

                            cp0 = [(validPts[0][0] + validPts[1][0]) / 2, (validPts[0][1] + validPts[1][1]) / 2]
                            cp1 = [(validPts[2][0] + validPts[3][0]) / 2, (validPts[2][1] + validPts[3][1]) / 2]
                            cp2 = [(validPts[0][0] + validPts[2][0]) / 2, (validPts[0][1] + validPts[2][1]) / 2]
                            cp3 = [(validPts[1][0] + validPts[3][0]) / 2, (validPts[1][1] + validPts[3][1]) / 2]
                            center = mathObj.calTwoLinesCrossPoint([cp0, cp1], [cp2, cp3])

                            for i in diagonalIdx:
                                for j in range(len(validPts)):
                                    if j in diagonalIdx: continue
                                    if bShowImg: cv.line(draw_img, (validPts[i][0], validPts[i][1]), (validPts[j][0], validPts[j][1]), (0, 255, 0), 1)
                            if len(center) > 0:
                                center[0] = int(center[0]); center[1] = int(center[1])
                                if bShowImg: draw_img = cv.circle(draw_img, (center[0], center[1]), 1, (0, 255, 0), 2)
                                rectangle = [validPts, primeter, center]
                        else:
                            if bShowImg: draw_img = cv.rectangle(draw_img, (x - margin_pixels, y - margin_pixels), (x + w + margin_pixels, y + h + margin_pixels), (255, 0, 0), 1)
                    if validPtCnt > 0 and validPtCnt < 3:
                        if bShowImg: draw_img = cv.rectangle(draw_img, (x - margin_pixels, y - margin_pixels), (x + w + margin_pixels, y + h + margin_pixels), (0, 0, 255), 1)
                        primeter = 0.0
                    if validPtCnt > 0:
                        if bShowImg: cv.putText(draw_img, str(round(primeter)), (validPts[0][0], 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv.LINE_AA)

            strI = ""
            if len(triangles) == 4 or (len(triangles) > 2 and len(rectangle) > 0):
                # Check primeter
                cnt = len(triangles);
                primeterOK = True
                primeterR = 0.0
                for i in range(1, cnt):
                    primeterRatio = triangles[0][1] / triangles[i][1]
                    if primeterRatio > 1: primeterRatio = 1.0 / primeterRatio
                    primeterR = primeterR + primeterRatio
                primeterR = round(primeterR / (cnt - 1), 3)
                if primeterR < trianglePrimeterMinR:
                    primeterOK = False
                    strI = "primeterR: " + str(primeterR)

                if primeterOK:
                    # Sort the triangles(x coordinate ascending)
                    for i in range(cnt - 1):
                        for j in range(i + 1, cnt):
                            if triangles[i][0][0][0] > triangles[j][0][0][0]:
                                tmp = triangles[i]
                                triangles[i] = triangles[j]
                                triangles[j] = tmp

                    # Calculate the marker _center
                    if len(rectangle) > 0:
                        center = rectangle[2]
                    else:  # 4 triangles are detected
                        x = int(((triangles[1][0][0][0] + triangles[1][0][1][0]) / 2 + (triangles[2][0][1][0] + triangles[2][0][2][0]) / 2) / 2)
                        y = int(((triangles[1][0][0][1] + triangles[1][0][1][1]) / 2 + (triangles[2][0][1][1] + triangles[2][0][2][1]) / 2) / 2)
                        center = [x, y]

                    # Calculate the marker rotating angle
                    positionLine = []
                    for i in range(cnt):
                        if triangles[i][0][2][0] < center[0]:  # Center of the shortest side
                            x = (triangles[i][0][0][0] + triangles[i][0][1][0]) / 2
                            y = (triangles[i][0][0][1] + triangles[i][0][1][1]) / 2
                        else:  # The last two points of the triangle
                            x = (triangles[i][0][1][0] + triangles[i][0][2][0]) / 2
                            y = (triangles[i][0][1][1] + triangles[i][0][2][1]) / 2
                        positionLine.append([x, y])

                    slope, offset, rsq, sParas = mathObj.lineFitting(positionLine)
                    vectorOrigin = [triangles[0][0][0][0], int(slope * triangles[0][0][0][0] + offset)]
                    vectorPosLine = [triangles[cnt - 1][0][2][0], int(slope * triangles[cnt - 1][0][2][0] + offset)]
                    vectorHorizontal = [triangles[cnt - 1][0][2][0], vectorOrigin[1]]
                    v1 = [vectorOrigin[0], vectorOrigin[1], vectorHorizontal[0], vectorHorizontal[1]]
                    v2 = [vectorOrigin[0], vectorOrigin[1], vectorPosLine[0], vectorPosLine[1]]

                    angleDir = 1
                    if vectorPosLine[1] < vectorHorizontal[1]: angleDir = -1
                    positionAngle = round(mathObj.vectorCrossAngle(v1, v2), 3) * angleDir
                    if cnt == 4:
                        dr_1 = round(mathObj.calTwoPointsDistance(positionLine[0], positionLine[1]) / mathObj.calTwoPointsDistance(positionLine[2], positionLine[3]), 3)
                        dr_2 = round(mathObj.calTwoPointsDistance(positionLine[1], center) / mathObj.calTwoPointsDistance(positionLine[2], center), 3)
                    else:
                        dr_2 = round(mathObj.calTwoPointsDistance(positionLine[0], center) / mathObj.calTwoPointsDistance(positionLine[1], center), 3)
                        if dr_2 > 1: dr_2 = round(1 / dr_2, 3)
                        if dr_2 < rectangleSpaceMinR:
                            dr_1 = round(mathObj.calTwoPointsDistance(positionLine[0], center) / mathObj.calTwoPointsDistance(positionLine[2], center), 3)
                            dr_2 = dr_1
                        else:
                            dr_1 = dr_2
                    if dr_1 > 1: dr_1 = round(1 / dr_1, 3)
                    if dr_2 > 1: dr_2 = round(1 / dr_2, 3)

                    if dr_1 >= triangleSpaceMinR and dr_2 >= rectangleSpaceMinR:
                        markerPos = [positionAngle, center]
                        if bShowImg:
                            cv.line(draw_img, (vectorOrigin[0], vectorOrigin[1]), (vectorPosLine[0], vectorPosLine[1]), (0, 255, 0), 1)
                            cv.line(draw_img, (vectorOrigin[0], vectorOrigin[1]), (vectorHorizontal[0], vectorHorizontal[1]), (255, 0, 0), 1)
                            cv.putText(draw_img, str(positionAngle), (vectorOrigin[0] - 40, vectorOrigin[1]), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv.LINE_AA)
                            cv.putText(draw_img, "OK: " + str(round(primeterR, 3)) + "," + str(dr_1) + "," + str(dr_2), (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv.LINE_AA)
                    else:
                        cv.line(draw_img, (vectorOrigin[0], vectorOrigin[1]), (vectorPosLine[0], vectorPosLine[1]), (0, 0, 255), 1)
                        cv.putText(draw_img, str(positionAngle), (vectorOrigin[0] - 40, vectorOrigin[1]), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)
                        cv.putText(draw_img, "NG: " + str(round(primeterR, 3)) + "," + str(dr_1) + "," + str(dr_2), (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)
                else:
                    cv.putText(draw_img, "NG" + strI, (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)
            else:
                strI = "(rect:" + str(len(rectangle)) + ",tri:" + str(len(triangles)) + ")"
                cv.putText(draw_img, "NG" + strI, (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv.LINE_AA)
        return draw_img, markerPos

    def _getLineRangeAndMostClosedPt(self, line, closedPt):
        minX = line[0][0]; maxX = minX; minY = line[0][1]; maxY = minY
        minD = 10240; mostClosedIdx = 0
        lenPts = len(line)
        for i in range(lenPts):
            d = math.sqrt(math.pow(closedPt[0] - line[i][0], 2) + math.pow(closedPt[1] - line[i][1], 2))
            if minD > d:
                minD = d
                mostClosedIdx = i
            if minX > line[i][0]: minX = line[i][0]
            if maxX < line[i][0]: maxX = line[i][0]
            if minY > line[i][1]: minY = line[i][1]
            if maxY < line[i][1]: maxY = line[i][1]
        return line[mostClosedIdx], [minX, maxX, minY, maxY]

    def _getLineSamePointCnt(self, line_1, line_2):
        samePtCnt = 0
        len_1 = len(line_1)
        len_2 = len(line_2)

        if len_1 > 0 and len_2 > 0:
            for i in range(len_1):
                for j in range(len_2):
                    if line_1[i] == line_2[j]:
                        samePtCnt = samePtCnt + 1
                        break
        return samePtCnt

    def searchLines(self, edgeImg, maxDistanceForSameLine_pixels, imgOrigin=[0, 0], maxPossibleLines=8):
        w, h = self._getImgWH(edgeImg)
        lineMinDataPts = int((h if h < w else w) * 0.6)
        lines = []
        for x in range(w):
            for y in range(h):
                if edgeImg[y][x] == 255:
                    linesQty = len(lines)
                    if linesQty <= 0:
                        newLine = []
                        newLine.append([x + imgOrigin[0], y + imgOrigin[1]])
                        lines.append(newLine)
                    else:
                        inLineCnt = 0
                        for line in lines:
                            bInLine = False
                            lenPts = len(line)
                            if lenPts > 2:
                                slope, offset, rsq, lineStatisticParas = mathObj.lineFitting(line)
                                if rsq < 0.5:
                                    slope, offset, rsq, lineStatisticParas = mathObj.lineFitting(line, -1, -1, True)
                                deltaX = mathObj.calcPointToLineDistance(line, [x + imgOrigin[0], y + imgOrigin[1]], [slope, offset, rsq])
                                deltaY = deltaX
                                if deltaX <= maxDistanceForSameLine_pixels and deltaY <= maxDistanceForSameLine_pixels:
                                    if rsq > 0.5:
                                        bInLine = True
                                    else:
                                        closedP, lineRng = self._getLineRangeAndMostClosedPt(line, [x + imgOrigin[0], y + imgOrigin[1]])
                                        deltaX = abs(closedP[0] - x - imgOrigin[0])
                                        deltaY = abs(closedP[1] - y - imgOrigin[1])
                                        if deltaX <= maxDistanceForSameLine_pixels and deltaY <= maxDistanceForSameLine_pixels:
                                            if abs(lineRng[1] - lineRng[0]) > abs(lineRng[3] - lineRng[2]):
                                                deltaMin = lineRng[3] - lineRng[2]
                                            else:
                                                deltaMin = lineRng[1] - lineRng[0]
                                            if deltaMin <= 2 * maxDistanceForSameLine_pixels:
                                                bInLine = True
                                    if bInLine: line.append([x + imgOrigin[0], y + imgOrigin[1]])

                            if not bInLine:
                                closedP, lineRng = self._getLineRangeAndMostClosedPt(line, [x + imgOrigin[0], y + imgOrigin[1]])
                                deltaX = abs(closedP[0] - x - imgOrigin[0])
                                deltaY = abs(closedP[1] - y - imgOrigin[1])
                                if deltaX <= maxDistanceForSameLine_pixels and deltaY <= maxDistanceForSameLine_pixels:
                                    if abs(lineRng[1] - lineRng[0]) > abs(lineRng[3] - lineRng[2]):
                                        deltaMin = lineRng[3] - lineRng[2]
                                    else:
                                        deltaMin = lineRng[1] - lineRng[0]
                                    if deltaMin <= maxDistanceForSameLine_pixels:
                                        line.append([x + imgOrigin[0], y + imgOrigin[1]])
                                        bInLine = True

                            if bInLine: inLineCnt = inLineCnt + 1
                        if inLineCnt <= 0:
                            newLine = []
                            newLine.append([x + imgOrigin[0], y + imgOrigin[1]])
                            lines.append(newLine)

        sortLines = []
        coef = []
        linesQty = len(lines)
        if linesQty > 0:
            for line in lines:
                if len(line) >= lineMinDataPts:
                    sortLines.append(line)
                    slope, offset, rsq, lineStatisticParas = mathObj.lineFitting(line)
                    if rsq < 0.5:
                        slope, offset, rsq, lineStatisticParas = mathObj.lineFitting(line, -1, -1, True)
                    coef.append([slope, offset, rsq])

        finalLines = []
        if linesQty < maxPossibleLines:
            # Delete duplicated line
            linesQty = len(sortLines)
            duplicatedIdx = []
            if linesQty > 1:
                for i in range(linesQty - 1):
                    for j in range(i + 1, linesQty):
                        slope_ratio = 0.0
                        if coef[j][0] == 0 and coef[i][0] == 0:
                            slope_ratio = 1.0
                        if coef[j][0] != 0 and coef[i][0] != 0:
                            slope_ratio = abs(coef[i][0]) / abs(coef[j][0])
                            if slope_ratio > 1: slope_ratio = 1 / slope_ratio

                        offset_ratio = 0.0
                        if coef[j][1] == 0 and coef[i][1] == 0:
                            offset_ratio = 1.0
                        if coef[j][1] != 0 and coef[i][1] != 0:
                            offset_ratio = abs(coef[i][1]) / abs(coef[j][1])
                            if offset_ratio > 1: offset_ratio = 1 / offset_ratio

                        rsq_ratio = 0.0
                        if coef[j][2] == 0 and coef[i][2] == 0:
                            rsq_ratio = 1.0
                        if coef[j][2] != 0 and coef[i][2] != 0:
                            rsq_ratio = abs(coef[i][2]) / abs(coef[j][2])
                            if rsq_ratio > 1: rsq_ratio = 1 / rsq_ratio

                        lineSamePtCnt = self._getLineSamePointCnt(sortLines[i], sortLines[j])
                        samePtRate = lineSamePtCnt / (len(sortLines[i]) if len(sortLines[i]) < len(sortLines[j]) else len(sortLines[j])) + 0.0

                        if slope_ratio > 0.95 and offset_ratio > 0.95 and rsq_ratio > 0.95 or samePtRate > 0.6:
                            duplicatedIdx.append(i)
            if len(duplicatedIdx) > 0:
                for i in range(linesQty):
                    if not i in duplicatedIdx:
                        finalLines.append(sortLines[i])
            else:
                finalLines = sortLines
        return finalLines

    def pose_cal_factors_ready(self, configCal):
        if "" == FuncObj.getDictVal(configCal, "rz_slope", ""): return False
        if "" == FuncObj.getDictVal(configCal, "rz_offset", ""): return False
        if "" == FuncObj.getDictVal(configCal, "x_slope", ""): return False
        if "" == FuncObj.getDictVal(configCal, "x_offset", ""): return False
        if "" == FuncObj.getDictVal(configCal, "x_type", ""): return False
        if "" == FuncObj.getDictVal(configCal, "y_slope", ""): return False
        if "" == FuncObj.getDictVal(configCal, "y_offset", ""): return False
        if "" == FuncObj.getDictVal(configCal, "y_type", ""): return False
        if "" == FuncObj.getDictVal(configCal, "x_ref", ""): return False
        if "" == FuncObj.getDictVal(configCal, "y_ref", ""): return False
        if "" == FuncObj.getDictVal(configCal, "rz_ref", ""): return False
        if "" == FuncObj.getDictVal(configCal, "xy_tol(mm)", ""): return False
        if "" == FuncObj.getDictVal(configCal, "rz_tol(deg)", ""): return False
        if "" == FuncObj.getDictVal(configCal, "x_pixel_res(mm)", ""): return False
        if "" == FuncObj.getDictVal(configCal, "y_pixel_res(mm)", ""): return False
        return True

    def cal_pose_correction(self, configCal, markerPos, bCorrByRef=[]):
        rz_slope = float(FuncObj.getDictVal(configCal, "rz_slope", ""))
        rz_offset = float(FuncObj.getDictVal(configCal, "rz_offset", ""))
        x_slope = float(FuncObj.getDictVal(configCal, "x_slope", ""))
        x_offset = float(FuncObj.getDictVal(configCal, "x_offset", ""))
        x_marker = int(FuncObj.getDictVal(configCal, "x_type", ""))
        y_slope = float(FuncObj.getDictVal(configCal, "y_slope", ""))
        y_offset = float(FuncObj.getDictVal(configCal, "y_offset", ""))
        y_marker = int(FuncObj.getDictVal(configCal, "y_type", ""))
        x_ref = float(FuncObj.getDictVal(configCal, "x_ref", ""))
        y_ref = float(FuncObj.getDictVal(configCal, "y_ref", ""))
        rz_ref = float(FuncObj.getDictVal(configCal, "rz_ref", ""))
        x_pixel_res = float(FuncObj.getDictVal(configCal, "x_pixel_res(mm)", ""))
        y_pixel_res = float(FuncObj.getDictVal(configCal, "y_pixel_res(mm)", ""))

        corX = 0.0
        corY = 0.0
        corRz = 0.0
        if len(markerPos) > 0:
            if 3 == len(bCorrByRef):
                if 1 == x_marker:  # Robot X-Axis is mapped to camera pixel-y axis
                    corY = (markerPos[1][0] - x_ref) * x_pixel_res
                    corX = (markerPos[1][1] - y_ref) * y_pixel_res
                else:
                    corX = (markerPos[1][0] - x_ref) * x_pixel_res
                    corY = (markerPos[1][1] - y_ref) * y_pixel_res

                corX = round(corX / 1000, 7) * ((-1) if (x_slope > 0) else 1)
                corY = round(corY / 1000, 7) * ((-1) if (y_slope > 0) else 1)
                # corRz = round(markerPos[0] - rz_ref, 5) * ((-1) if (rz_slope > 0) else 1)
                corRz = round(0.0 - (markerPos[0] - rz_offset) / rz_slope, 5)
                if not bCorrByRef[0]: corX = 0
                if not bCorrByRef[1]: corY = 0
                if not bCorrByRef[2]: corRz = 0
            else:
                if x_marker in range(0, 2):
                    corX = markerPos[1][x_marker]
                else:
                    corX = markerPos[1][1]
                if y_marker in range(0, 2):
                    corY = markerPos[1][y_marker]
                else:
                    corY = markerPos[1][0]

                corX = round(0.0 - (corX - x_offset) / x_slope, 5)
                corY = round(0.0 - (corY - y_offset) / y_slope, 5)
                corRz = round(0.0 - (markerPos[0] - rz_offset) / rz_slope, 5)
        return corX,corY,corRz

    def apply_pose_correction(self, configCal, markerPos, delayAfter_s=0.2, bCorrByRef=[]):
        startT = time.time()
        ok = True
        max_angle = 25.0 #TODO para or config
        timeout_s = 5
        corX, corY, corRz = self.cal_pose_correction(configCal, markerPos, bCorrByRef)
        # Change by Hui Zhi 2022/4/7
        if corRz != 0 and abs(corRz) < max_angle:
            # if 0 != self._arm_ctrl.relative_shift([0, 0, 0, 0, 0, round(corRz / 180 * 3.1415926, 7)], 5, True):
            if 0 != self._arm_ctrl.move_tool([0, 0, 0, 0, 0, round(corRz / 180 * 3.1415926, 7)], timeout_s, True):
                ok = False
        if ok and (corX != 0 or corY != 0):
            # if 0 != self._arm_ctrl.relative_shift([corX, corY, 0, 0, 0, 0], 5, False):
            if 0 != self._arm_ctrl.move_tool([corX, corY, 0, 0, 0, 0], timeout_s, False,False,0.1,0.25):
                ok = False
        if ok and delayAfter_s > 0:
            time.sleep(delayAfter_s)
        procT = round(time.time() - startT, 3)
        corXYZ = [corX, corY, corRz, procT]
        return ok, corXYZ

    def adjust_gripper_pose(self, timeout_s=60, delayAfter_s=0.2, poseName="", factorName="PickFactor", bSaveImg=True):
        ok = False
        configCal = self._cobot_cfg.get_arm_pose_factor(poseName, factorName)
        markerName = FuncObj.getDictVal(configCal,"marker_name",None)
        if markerName is None:
            print("adjust_gripper_pose failed: marker is not configured yet for (" + poseName + "," + factorName + ")")
            return ok, [0, 0, 0, 0, 0], None
        if not self.pose_cal_factors_ready(configCal):
            print("adjust_gripper_pose failed: calibration is not done yet for (" + poseName + "," + factorName + ")")
            return ok, [0, 0, 0, 0, 0], None

        mkrCfg = FuncObj.getDictVal(self._config_dat, markerName, None)
        if mkrCfg is None:
            mkrCfg = self._cobot_cfg.get_marker_cfg(markerName)
            self._config_dat[markerName] = mkrCfg
        mkr_max_chg = int(FuncObj.getDictVal(mkrCfg, "mkr_max_chg(pixel)", "2"))

        startT = time.time()
        corX = 0.0; corY = 0.0; corRz = 0.0; motionT = 0.0
        rzOK = False; retryCnt = 0; adj_idx = -1
        sumX = 0.0; sumY = 0.0; sumRz = 0.0 #To record whether there is retrial or not
        bChkEnvLight = True
        while (True):
            draw_img, markerPos, ori_img, ngInfo, mkrsInfo = self.cal_marker_position(False, 5, markerName, bSaveImg, bChkEnvLight)
            if ori_img is None: break
            if len(markerPos) > 0: #First rough correction
                rzOK, corXYZ_1 = self.apply_pose_correction(configCal, markerPos, delayAfter_s)
                if rzOK:
                    corX = corX + corXYZ_1[0]; corY = corY + corXYZ_1[1]; corRz = corRz + corXYZ_1[2]
                if bSaveImg:
                    cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_1_00" + ".jpg", ori_img)
                    cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_1_01" + ".jpg", draw_img)
                break
            else:
                if not ori_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_1_" + str(retryCnt) + "_"  + LogObj.getCurrTime() + "_00.jpg", ori_img)
                if not draw_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_1_" + str(retryCnt) + "_"  + LogObj.getCurrTime() + "_01.jpg", draw_img)
                procT = time.time() - startT
                if timeout_s>0 and procT > timeout_s or procT > 300: break
                if len(ngInfo) > 0 and retryCnt < 10:
                    if len(ngInfo[0]) > 0:
                        retryOK, corXYZ_0 = self.apply_pose_correction(configCal, ngInfo[0], delayAfter_s)
                    else:
                        retryOK, corXYZ_0 = self._arm_ctrl.random_shift([True, True, True], delayAfter_s)
                    if retryOK:
                        sumX = sumX + corXYZ_0[0]; sumY = sumY + corXYZ_0[1]; sumRz = sumRz + corXYZ_0[2]
                        corX = corX + corXYZ_0[0]; corY = corY + corXYZ_0[1]; corRz = corRz + corXYZ_0[2]
                        retryCnt = retryCnt + 1
                    else:
                        break
                else:
                    break

        if rzOK:
            retryCnt = 0
            while (True):
                draw_img, mkrPos, ori_img, ngInfo, mkrsInfo = self.cal_marker_position(False, 5, markerName, bSaveImg)
                if ori_img is None: break
                if len(mkrPos) > 0: #Second precise correction
                    ok, corXYZ_2 = self.apply_pose_correction(configCal, mkrPos, delayAfter_s)
                    motionT = corXYZ_1[3] + corXYZ_2[3]
                    if ok:
                        corX = corX + corXYZ_2[0]; corY = corY + corXYZ_2[1]; corRz = corRz + corXYZ_2[2]
                    if bSaveImg:
                        cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_2_00" + ".jpg", ori_img)
                        cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_2_01" + ".jpg", draw_img)
                    break
                else:
                    if not ori_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_2_" + str(retryCnt) + "_"  + LogObj.getCurrTime() + "_00.jpg", ori_img)
                    if not draw_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_2_" + str(retryCnt) + "_"  + LogObj.getCurrTime() + "_01.jpg", draw_img)
                    procT = time.time() - startT
                    if timeout_s>0 and procT > timeout_s or procT > 300: break
                    if len(ngInfo) > 0 and retryCnt < 10:
                        if len(ngInfo[0]) > 0:
                            retryOK, corXYZ_0 = self.apply_pose_correction(configCal, ngInfo[0], delayAfter_s)
                        else:
                            retryOK, corXYZ_0 = self._arm_ctrl.random_shift([True, True, True], delayAfter_s)
                        if retryOK:
                            sumX = sumX + corXYZ_0[0]; sumY = sumY + corXYZ_0[1]; sumRz = sumRz + corXYZ_0[2]
                            corX = corX + corXYZ_0[0]; corY = corY + corXYZ_0[1]; corRz = corRz + corXYZ_0[2]
                            retryCnt = retryCnt + 1
                        else:
                            break
                    else:
                        break
            if ok:
                corX = round(corX, 5); corY = round(corY, 5); corRz = round(corRz, 5); bChkEnvLight = False
                #Final Verification
                ok = False
                x_ref = float(FuncObj.getDictVal(configCal, "x_ref", "0.0"))
                y_ref = float(FuncObj.getDictVal(configCal, "y_ref", "0.0"))
                rz_ref = float(FuncObj.getDictVal(configCal, "rz_ref", "0.0"))
                if float(FuncObj.getDictVal(configCal, "x_pixel_res(mm)", "0.28")) < float(FuncObj.getDictVal(configCal, "y_pixel_res(mm)", "0.28")):
                    xy_tol = float(FuncObj.getDictVal(configCal, "xy_tol(mm)", "0.3")) / float(FuncObj.getDictVal(configCal, "x_pixel_res(mm)", "0.28"))
                else:
                    xy_tol = float(FuncObj.getDictVal(configCal, "xy_tol(mm)", "0.3")) / float(FuncObj.getDictVal(configCal, "y_pixel_res(mm)", "0.28"))
                rz_tol = float(FuncObj.getDictVal(configCal, "rz_tol(deg)", "0.5"))
                iChkCycles = int(FuncObj.getDictVal(configCal, "check_cycles", "10"))
                for i in range(iChkCycles):
                    sData = poseName + "," + factorName + "," + str(corX) + "," + str(corY) + "," + str(corRz) + "," + str(sumX) + "," + str(sumY) + "," + str(sumRz)
                    draw_img, mkrPos, ori_img, ngInfo, mkrsInfo = self.cal_marker_position(False, 5, markerName, bSaveImg, bChkEnvLight)
                    if ori_img is None: break
                    if len(mkrPos) > 0:
                        sData = sData + "," + str(mkrPos[1][0]) + "," + str(mkrPos[1][1]) + "," + str(mkrPos[0])
                        sData = sData + "," + FuncObj.getDictVal(configCal, "x_ref", "0")
                        sData = sData + "," + FuncObj.getDictVal(configCal, "y_ref", "0")
                        sData = sData + "," + FuncObj.getDictVal(configCal, "rz_ref", "0")
                    else:
                        sData = sData + ",,,,,,"
                    if len(mkrsInfo) > 0:
                        if mkrsInfo[2][0] - mkrsInfo[2][1] > mkr_max_chg or (not (mkrsInfo[2][2] > 0.5 and mkrsInfo[2][3] > 0.99 or mkrsInfo[2][4] > 0.85)):
                            bChkEnvLight = True
                            if (i > 0 and mkrsInfo[2][4] > 0.85): bChkEnvLight = False
                        else:
                            bChkEnvLight = False
                        sData = sData + "," + str(mkrsInfo[1]) + "," + str(mkrsInfo[2][0]) + "," + str(mkrsInfo[2][1]) + "," + str(round(mkrsInfo[2][2],3))
                    else:
                        sData = sData + ",,,,"

                    if len(mkrPos) > 0: #Check the position accuracy only after the 1st precise correction
                        dx = abs(mkrPos[1][0] - x_ref)
                        dy = abs(mkrPos[1][1] - y_ref)
                        drz = abs(mkrPos[0] - rz_ref)
                        bDxyOK = True
                        if (dx + dy) > xy_tol:
                            bDxyOK = False
                        else:
                            if xy_tol > 2 and (dx > xy_tol/2 or dy > xy_tol/2): bDxyOK = False
                        if bDxyOK and drz <= rz_tol:
                            if bChkEnvLight:
                                LogObj.logInfo(self._home_dir, "adjust_gripper_pose", sData + ",ADJ_" + str(i))
                                continue
                            else:
                                ok = True
                                LogObj.logInfo(self._home_dir, "adjust_gripper_pose", sData + ",CFM_" + str(i))
                                if bSaveImg:
                                    cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_3_00" + ".jpg", ori_img)
                                    cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_OK_3_01" + ".jpg", draw_img)
                                break
                        else:
                            if bSaveImg and (iChkCycles-1) == i:
                                cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_NG_00" + ".jpg", ori_img)
                                cv.imwrite(self._home_dir + "/" + poseName + "_" + factorName + "_NG_01" + ".jpg", draw_img)
                            LogObj.logInfo(self._home_dir, "adjust_gripper_pose", sData + ",CHK_" + str(i))
                            retryOK, corXYZ_3 = self.apply_pose_correction(configCal, mkrPos, delayAfter_s, [True, True, True])
                            if retryOK:
                                corX = round(corX + corXYZ_3[0],5); corY = round(corY + corXYZ_3[1],5); corRz = round(corRz + corXYZ_3[2],5)
                    else:
                        if not ori_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_3_" + str(i) + "_" + LogObj.getCurrTime() + "_00.jpg", ori_img)
                        if not draw_img is None: cv.imwrite(self._home_dir + "/CalMarker_NG_3_" + str(i) + "_" + LogObj.getCurrTime() + "_01.jpg", draw_img)
                        LogObj.logInfo(self._home_dir, "adjust_gripper_pose", sData + ",CHK_" + str(i))
        if not ok:
            if corX != 0 or corY != 0: self._arm_ctrl.relative_shift([corX * (-1), corY * (-1), 0, 0, 0, 0])
            if  -0.1 < corRz < 0.1 :
                if corRz != 0: self._arm_ctrl.relative_shift([0, 0, 0, 0, 0, round(corRz * (-1) / 180 * 3.1415926, 7)], 5, True)
        corrT = round(time.time() - startT - motionT, 3)
        return ok, [corX, corY, corRz, corrT, motionT], draw_img

    def adjust_marker_pose(self, marker_pose_name, factor_name, timeout_s=60, bSaveImg=True):
        """
        2022/9/5 Hui Zhi Fang
        调整图像中marker的角度和中心坐标,直到与初始配置中的相同
        :param marker_pose_name:已配置的marker位姿名称，如"input_rack_a_marker_1"
        :param factor_name:PickFactor / PlaceFactor
        :param timeout_s: default 60 s
        """
        adjust_ok = False
        config_cal = self._cobot_cfg.get_arm_pose_factor(marker_pose_name, factor_name)
        marker_name = FuncObj.getDictVal(config_cal, "marker_name", None)
        rz_ref = float(FuncObj.getDictVal(config_cal, "rz_ref", 0.0))
        rz_slope = float(FuncObj.getDictVal(config_cal, "rz_slope", 0.0))
        failed_count = 0
        move_delay_s = 0.2

        t1 = time.time()
        while True:  # roughly
            pass_time = time.time() - t1
            if pass_time > timeout_s:
                LogObj.logAppError("Marker", f"Adjust_marker_pose timeout,{pass_time}s is more than {timeout_s}s")
                break
            draw_img, marker_pos, frame, ngInfo, mkrsInfo = self.cal_marker_position(False, 5, marker_name, bSaveImg)
            cv.imwrite(self._home_dir + "/CalMarker_Origin_draw" + LogObj.getCurrTime() + ".jpg", draw_img)
            cv.imwrite(self._home_dir + "/CalMarker_Origin_frame" + LogObj.getCurrTime() + ".jpg", frame)
            if len(marker_pos) > 0:
                move_xyrz_ok, corXYZ = self.apply_pose_correction(config_cal, marker_pos, move_delay_s,)
                print(corXYZ)
                break

        while True:
            pass_time = time.time() - t1
            if pass_time > timeout_s:
                LogObj.logAppError("Marker",f"Adjust_marker_pose timeout,{pass_time}s is more than {timeout_s}s")
                break
            draw_img, marker_pos, frame, ngInfo, mkrsInfo = self.cal_marker_position(False, 5, marker_name,bSaveImg)
            # 当从图像中找到marker时
            if len(marker_pos) > 0:
                angle = marker_pos[0]
                if angle != rz_ref:
                    # 旋转
                    # rotate_rz_ok, corXYZ = self.apply_pose_correction(config_cal,marker_pos, move_delay_s, [False, False, True])  # rz
                    corRz = 0.1 * ((-1) if (rz_slope > 0) else 1)
                    if angle < rz_ref:
                        corRz = -corRz
                    rotate_rz_ok = True
                    if 0 != self._arm_ctrl.move_tool([0, 0, 0, 0, 0, round(corRz / 180 * 3.1415926, 7)], timeout_s,True):
                        rotate_rz_ok = False
                    print(corRz)

                    if not rotate_rz_ok:
                        LogObj.logAppError("Marker","Apply marker pose failed, check if the robotic arm is in remote mode.")
                        break
                else:  # 当角度旋转至与初始配置的角度相同时
                    move_ok, corXYZ = self.apply_pose_correction(config_cal, marker_pos, move_delay_s,[True,True,False])  # x,y
                    print(corXYZ)
                    if not move_ok:
                        LogObj.logAppError("Marker","Apply marker pose failed, check if the robotic arm is in remote mode.")
                        break
                    correct_x_meter = corXYZ[0]
                    correct_y_meter = corXYZ[1]
                    if correct_x_meter == 0 and correct_y_meter == 0:  # success
                        if draw_img is not None and bSaveImg:
                            cv.imwrite(self._home_dir + "/CalMarker_OK_draw" + LogObj.getCurrTime() + ".jpg",draw_img)
                            cv.imwrite(self._home_dir + "/CalMarker_OK_frame" + LogObj.getCurrTime() + ".jpg", frame)
                        adjust_ok = True
                        break
            else:  # TODO 当识别不到marker时
                failed_count += 1
                if failed_count > 10:
                    if not draw_img is None:
                        cv.imwrite(self._home_dir + "/CalMarker_NG_" + LogObj.getCurrTime() + ".jpg", draw_img)
                        LogObj.logAppError("Marker","Adjust marker pose failed.")
                    break
        adjust_time = round(time.time() - t1,2)
        print(f"adjust marker total time:{adjust_time}s")
        return adjust_ok, adjust_time, draw_img

