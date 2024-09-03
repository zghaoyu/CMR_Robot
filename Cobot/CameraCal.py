import LogHelper as LogObj
import UtilsFunc as FuncObj
import UtilsMath as MathObj
import time
import cv2 as cv
from ClassArm import ArmUR
from ClassPositionMarker import MarkerFixture
from ClassCamera import CameraLogitech, LightCtrlLCPW
from ClassAGV import AgvMIR
from ClassConfig import CobotCfg

homeDir = 'E:/CobotHome/ControllerCamera'
draw_img = None
URCtrl = ArmUR()
MkrFixture = MarkerFixture()
CamCtrl = CameraLogitech()
LightCtrl = LightCtrlLCPW()
AgvCtrl = AgvMIR()
CbtConfig = CobotCfg()
dSettleTime_s = 3
factorNames = ["PickFactor", "PlaceFactor","PickAndPlaceFactor"]
cal_pos_name = "cnc_a_marker_74"
factorName = "PickAndPlaceFactor"
markerName = "CircleBasedMkr_05"
refPosition = {}
pixel_res = {}

def get_pose_relative_shift(poseName, factorName, shiftType):
    rtnMsg = ""
    xyz_shift = []
    poseFactor = CbtConfig.get_arm_pose_factor(poseName, factorName)
    if len(poseFactor) > 0:
        xyz_shift_take_picture = FuncObj.getDictVal(poseFactor, shiftType, "")
        if "" != xyz_shift_take_picture:
            xyz_shift_settings = xyz_shift_take_picture.split(";")
            for shift in xyz_shift_settings:
                xyz = shift.split(",")
                if 3 != len(xyz):
                    rtnMsg = "Wrong pose_relative_shift setting(" + poseName + "," + factorName + "," + shiftType + ")"
                    break
                else:
                    xyz_shift.append([float(val) for val in xyz])
        else:
            rtnMsg = "Get pose_relative_shift(" + poseName + "," + factorName + "," + shiftType + ") failed_01"
    else:
        rtnMsg = "Get pose_relative_shift(" + poseName + "," + factorName + "," + shiftType + ") failed_02"
    return rtnMsg, xyz_shift

def do_pose_relative_shift(poseName, factorName, shiftType):
    rtnMsg, xyz_shift = get_pose_relative_shift(poseName, factorName, shiftType)
    if "" == rtnMsg:
        for xyz in xyz_shift:
            if 0 != round(xyz[0], 6) or 0 != round(xyz[1], 6) or 0 != round(xyz[2], 6):
                if 0 != URCtrl.relative_shift([xyz[0], xyz[1], xyz[2], 0, 0, 0]):
                    rtnMsg = "relative_shift(xyz) failed"
                    break
    return rtnMsg, xyz_shift

def cal_exe(pos_start = -0.02, pos_stop = 0.02, pos_step = 0.004, cal_type = 0):
    if not factorName in factorNames:
        print(factorName + " is not valid")
        return
    poseFactor = CbtConfig.get_arm_pose_factor(cal_pos_name, factorName)
    corrIdx = 0
    posStep = [0,0,0,0,0,0]

    if 0 == cal_type: #calibrate x
        posStep[0] = pos_step
        posStart = [pos_start, 0, 0, 0, 0, 0]
        corrIdx = int(FuncObj.getDictVal( poseFactor, "x_type", "1"))
    if 1 == cal_type: #calibrate y
        posStep[1] = pos_step
        posStart = [0, pos_start, 0, 0, 0, 0]
        corrIdx = int(FuncObj.getDictVal(poseFactor, "y_type", "0"))
    if 2 == cal_type: #calibrate UR Wrist 3(consider it as rz)
        posStep[5] = round(pos_step / 180 * 3.1415926, 7)
        posStart = [0, 0, 0, 0, 0, round(pos_start / 180 * 3.1415926, 7)]
        corrIdx = int(FuncObj.getDictVal(poseFactor, "rz_type", "2"))
    if corrIdx < 0 or corrIdx > 2:
        print("Setting is not correct")
        return

    bSetJoint = False
    if 2 == cal_type: bSetJoint = True
    if "arm_home" != URCtrl.get_curr_pose_name():
        if 0 != URCtrl.move_to_by_pose_name("new_arm_home"):
            print("Arm Home failed")
            return
    if 0 != URCtrl.move_to_by_pose_name(cal_pos_name):
        print("Move to " + cal_pos_name + " failed")
        return
    #disable by Hui Zhi 2022/8/8 testing
    # rtnMsg, xyz_shifts = do_pose_relative_shift(cal_pos_name, factorName, "xyz_shift_take_picture(m)")
    # if "" != rtnMsg:
    #     print(rtnMsg)
    #     return

    # rtn = URCtrl.relative_shift(posStart, 5, bSetJoint)
    rtn = URCtrl.move_tool(posStart, 5, bSetJoint,False,0.1,0.1) #change by Hui Zhi 2022/8/9
    time.sleep(dSettleTime_s)
    xyPts = []
    currPos = pos_start
    if (not CamCtrl.openCamera() is None) and LightCtrl.light_on():
        if 0 == cal_type:
            LogObj.logInfo(homeDir,"calData_","robot_x,marker_x,marker_y,marker_angle")
        if 1 == cal_type:
            LogObj.logInfo(homeDir,"calData_","robot_y,marker_x,marker_y,marker_angle")
        if 2 == cal_type:
            LogObj.logInfo(homeDir,"calData_","robot_rz,marker_x,marker_y,marker_angle")

        while(True):
            if 0 == rtn:
                draw_img, markerPos, ori_img, ngInfo, mkrsInfo = MkrFixture.cal_marker_position(True, 5, markerName)
                if len(markerPos) > 0:
                    if 2 == corrIdx:
                        xyPts.append([currPos, markerPos[0]]) #Ratating angle
                    else:
                        xyPts.append([currPos, markerPos[1][corrIdx]]) #Marker _center(x,y)
                    LogObj.logInfo(homeDir, "calData_", str(currPos)+","+str(markerPos[1][0])+","+str(markerPos[1][1])+","+str(markerPos[0]))
                    cv.imwrite(homeDir+"/"+str(cal_type)+"_"+str(currPos).replace(".","")+".jpg",draw_img)
                    if 0 == round(currPos, 2):
                        if 2 == cal_type: refPosition["rz_ref"] = markerPos[0]
                        if 1 == cal_type: refPosition["y_ref"] = markerPos[1][1]
                        if 0 == cal_type: refPosition["x_ref"] = markerPos[1][0]
                currPos = currPos + pos_step
                if currPos > pos_stop:
                    break
                # rtn = URCtrl.relative_shift(posStep, 5, bSetJoint)
                rtn = URCtrl.move_tool(posStep, 5, bSetJoint,False,0.1,0.1) #change by Hui Zhi 2022/8/9
                time.sleep(dSettleTime_s)
            else:
                break

        calDataPts = len(xyPts)
        if calDataPts > 5:
            slope, offset, rsq, lineStatisticParas = MathObj.lineFitting(xyPts) #X-Axis is robot position data, Y-Axis is pixel position data
            pixel_res[cal_type] = round(abs(xyPts[0][0] - xyPts[calDataPts - 1][0]) * 1000 / (abs(xyPts[0][1] - xyPts[calDataPts - 1][1])+1), 3) #unit: mm
            calFactor = {}
            calFactor["marker_name"] = markerName
            if 3 == len(refPosition):
                calFactor["x_ref"] = refPosition["x_ref"]
                calFactor["y_ref"] = refPosition["y_ref"]
                calFactor["rz_ref"] = refPosition["rz_ref"]
                calFactor["xy_tol(mm)"] = round((pixel_res[0] if pixel_res[0] > pixel_res[1] else pixel_res[1])*1.1, 2)
                calFactor["rz_tol(deg)"] = 0.5
                print("ref_position: ", str(refPosition))
            if 0 == cal_type:
                calFactor["x_slope"] = round(slope, 5)
                calFactor["x_offset"] = round(offset, 5)
                calFactor["x_type"] = corrIdx
                calFactor["x_pixel_res(mm)"] = pixel_res[cal_type]
                CbtConfig.save_arm_pose_factor(cal_pos_name, factorName, calFactor)
                print("x_cal_factor: " + str(calFactor))
            if 1 == cal_type:
                calFactor["y_slope"] = round(slope, 5)
                calFactor["y_offset"] = round(offset, 5)
                calFactor["y_type"] = corrIdx
                calFactor["y_pixel_res(mm)"] = pixel_res[cal_type]
                CbtConfig.save_arm_pose_factor(cal_pos_name, factorName, calFactor)
                print("y_cal_factor: " + str(calFactor))
            if 2 == cal_type:
                calFactor["rz_slope"] = round(slope, 5)
                calFactor["rz_offset"] = round(offset, 5)
                calFactor["rz_type"] = corrIdx
                CbtConfig.save_arm_pose_factor(cal_pos_name, factorName, calFactor)
                print("rz_cal_factor: " + str(calFactor))

    URCtrl.move_to_by_pose_name("new_arm_home")
    LightCtrl.light_off()

cal_exe(-0.01,0.01,0.002,0)
cal_exe(-0.01,0.01,0.002,1)
cal_exe(-5,5,1,2)
cv.destroyAllWindows()
