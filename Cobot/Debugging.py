import time
import cv2 as cv
import LogHelper as LogObj
import UtilsFunc as FuncObj
import threading
import numpy as np

from UtilsCOM import RS232COM
from ClassArm import ArmUR
from ClassGripper import GripperSMC
from ClassPositionMarker import MarkerFixture
from ClassCamera import CameraLogitech, LightCtrlLCPW
from ClassAGV import AgvMIR
from ClassConfig import CobotCfg
from ClassRemoteIO import RemoteIO
from ClassDaemon import Daemon


URCtrl = ArmUR()
GripperCtrl = GripperSMC()
MkrFixture = MarkerFixture()
CamCtrl = CameraLogitech()
LightCtrl = LightCtrlLCPW()
AGVCtrl = AgvMIR()
CbtConfig = CobotCfg()
RmtIO = RemoteIO()
DaemonCtrl = Daemon()
pnp_offset_z = (-1)*abs(URCtrl.get_cal_z_height()) #Negative to move down

def modbus_crc16(hexData):
    crcVal = 0xFFFF
    for h in hexData:
        crcVal = h ^ crcVal
        for i in range(8):
            if 1 == (crcVal & 0x01):
                crcVal = crcVal >> 1
                crcVal = crcVal ^ 0xA001
            else:
                crcVal = crcVal >> 1
    return crcVal

def COMTest(portX):
    com = RS232COM()
    ser = com.openPort(portX,115200)
    if(not ser is None):
        writeContent = [0x01, 0x03, 0x00, 0x0F, 0x00, 0x01]
        crc16 = modbus_crc16(writeContent)
        writeContent.append(crc16 & 0xFF)
        writeContent.append(((crc16 & 0xFF00) >> 8))

        count=com.writePort(portX,writeContent,"hex")
        print("Write: ",count)
        print("Read: ",com.readPort(portX, "hex"))
        com.closePort(portX)

def printComPorts():
    com = RS232COM()
    port_list = com.getAvailablePorts()
    if len(port_list) > 0:
        for i in range(0, len(port_list)):
            print(port_list[i])
    else:
        print("There is no available COM port")

def singletonTest():
    def task(arg):
        obj = RS232COM()
        print("T"+str(arg),obj)

    for i in range(10):
        t = threading.Thread(target=task, args=[i])
        t.start()

def gripper_pos_check():
    URCtrl.relative_shift([0,0,pnp_offset_z,0,0,0])
    GripperCtrl.gripper_close()
    GripperCtrl.gripper_open()
    URCtrl.relative_shift([0,0,(-1)*pnp_offset_z,0,0,0])

def agv_goto_charge(bGetChargingCurve=False, bSkipReadyCheck=False):
    if not bSkipReadyCheck:
        if not AGVCtrl.agv_is_ready(): return False
        if not AGVCtrl.mission_continue(): return False
    if AGVCtrl.go_to_charge():
        if bGetChargingCurve:
            while(True):
                status = AGVCtrl.get_agv_status()
                battery = FuncObj.getDictVal(status, "battery_percentage", None)
                if not battery is None:
                    LogObj.logInfo("E:/CobotHome/ControllerAGV", "cobotCharging", str(round(battery,2)))
                    if battery >= 100: break
                    time.sleep(60)
        return True
    else:
        return False

def gripper_init():
    rtn = GripperCtrl.gripper_init()
    if 0 != rtn:
        rtn = GripperCtrl.gripper_init()
    if 0 == rtn:
        rtn = GripperCtrl.gripper_open()
    return rtn

def marker_proc_debug():
    if 0 == URCtrl.move_to_by_pose_name("arm_home"):
        if 0 == URCtrl.move_to_by_pose_name("input_rack_slot_01"):
            MkrFixture.adjust_gripper_pose(60, 0.2, "input_rack_slot_01", "PickFactor")
            MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_01")
        else:
            print("move_to_by_pose_name(input_rack_slopt) failed")
    else:
        print("move_to_by_pose_name(arm_home) failed")

def relative_shift_ex(corrX, corrY, corrRz):
    if (corrX != 0 or corrY != 0) and 0 != URCtrl.relative_shift([corrX, corrY, 0, 0, 0, 0], 5, False):
        print("relative_shift_ex NG")
        return
    if corrRz == 0 or corrRz != 0 and 0 == URCtrl.relative_shift([0, 0, 0, 0, 0, round(corrRz / 180 * 3.1415926, 7)], 5, True):
        print("relative_shift_ex OK")

def take_picture():
    pic = CamCtrl.takePicture()
    if not pic is None:
        cv.imwrite("E:/CobotHome/marker.jpg", pic)

def swap_test():
    a = 1; b = 2
    print("Before: "+str(a)+","+str(b))
    a, b = FuncObj.swapVal(a, b)
    print("After: "+str(a) + "," + str(b))

def read_barcode(bByOpenCV=False):
    CamCtrl.setFromFile("E:/CobotHome/ControllerCamera/2D_Barcode_04.png")
    pic = CamCtrl.takePicture()
    gray = cv.cvtColor(pic, cv.COLOR_BGR2GRAY)
    barcodes = CamCtrl.readBarcode(gray, bByOpenCV)
    print("barcodes", barcodes)

def check_corr_direction(iFlag=0):
    shiftD = 0; shiftStep = 0.0003
    for i in range(10):
        if 0 == iFlag:
            URCtrl.relative_shift([shiftStep, 0, 0, 0, 0, 0])
        if 1 == iFlag:
            URCtrl.relative_shift([0, shiftStep, 0, 0, 0, 0])
        if 2 == iFlag:
            URCtrl.relative_shift([0, 0, 0, 0, 0, round(shiftStep / 180 * 3.1415926, 7)], 5, True)
        shiftD = shiftD + shiftStep
        draw_img, mkrPos, ori_img, ngInfo, mkrsInfo = MkrFixture.cal_marker_position(True, 5, "CircleBasedMkr_03")
        if len(mkrPos) > 0: print(str(shiftD)+","+str(mkrPos[1][0])+","+str(mkrPos[1][1])+","+str(mkrPos[0]))
    if 0 == iFlag:
        URCtrl.relative_shift([(-1)*shiftD, 0, 0, 0, 0, 0])
    if 1 == iFlag:
        URCtrl.relative_shift([0, (-1)*shiftD, 0, 0, 0, 0])
    if 2 == iFlag:
        URCtrl.relative_shift([0, 0, 0, 0, 0, round((-1)*shiftD / 180 * 3.1415926, 7)], 5, True)

def hex_str():
    import struct
    a = [0xaa, 0x06, 0x01]
    data = struct.pack("%dB" % (len(a)), *a)
    print(data)

    for i in range(1, 9):
        RmtIO.set_output(True, i)
        time.sleep(2)
        RmtIO.set_output(False, i)
        time.sleep(2)

def adj_light_brightness(grayTarget = 80):
    mkrName = "CircleBasedMkr_06"
    draw_img, mkrPos, ori_img, ngInfo, mkrsInfo = MkrFixture.cal_marker_position(False, 5, mkrName, False)
    if len(mkrPos) > 0 and len(mkrsInfo) > 0:
        for ctr in mkrsInfo[3]:
            cv.rectangle(draw_img, (ctr[0][0],ctr[0][1]), (ctr[1][0],ctr[1][1]), (255, 0, 0), 1)
        for cc in mkrsInfo[0]:
            cv.putText(draw_img, str(cc[2]), (cc[0] - 5, cc[1] - 5), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_AA)
        cv.imshow("Marker Position", draw_img)
        cv.waitKey(0)
        cv.destroyAllWindows()

        print("Original Avg G: " + str(mkrsInfo[1]))
        MkrFixture.adj_light_brightness(mkrsInfo[3], grayTarget)

def put_fixture_onto_robot(pose_name="robot_slot_11"):
    if 0 != URCtrl.move_to_by_pose_name(pose_name): return
    if 0 != URCtrl.relative_shift([0, 0, pnp_offset_z+0.005, 0, 0, 0]): return
    if 0 != URCtrl.relative_shift([0, 0, -0.005, 0, 0, 0]): return
    if 0 != GripperCtrl.gripper_open(): return
    if 0 != URCtrl.relative_shift([0,0,(-1)*pnp_offset_z,0,0,0]): return
    URCtrl.move_to_by_pose_name("arm_home")

def calc_mkr_position():
    CamCtrl.setFromFile("E:/CobotHome/ControllerCamera/20210506220007_LightAdj_0_02.jpg")
    draw_img, mkrPos, ori_img, ngInfo, mkrsInfo = MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_06")
    print("mrkPos", len(mkrPos))
    print("mkrsInfo",len(mkrsInfo))

# add by Hui Zhi 2022/3/4
def read_barcode_test():
    # img =cv.imread("E:/CobotHome/Picture/20220221/barcode.jpg")
    while True:
        img = CamCtrl.takePicture()
        CamCtrl.cameraSetting("Barcode_Scan",True)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        barcode = CamCtrl.readBarcode(gray, False)
        if(len(barcode)>0):
            print("barcode", barcode)
        cv.imshow('result', gray)
        cv.waitKey(1)

# add by Hui Zhi 2022/3/7
def getAGVPostion():
    status = AGVCtrl.get_agv_status()
    position = FuncObj.getDictVal(status, "position", None)
    print(position)
    LogObj.logInfo("E:/CobotHome/TestData", "AGV_position_", position)
    # Charger marker offset position
    # result = FuncObj.get(AGVCtrl._url + '/positions/237d44f3-ff31-11eb-87b1-000129998290', {}, AGVCtrl._headers)
    # print(result)

def agvMoveBackwardAndCharging(move_path=False):
    start_time = time.time()
    CbtConfig.reset_arm_pose_corr_factor()
    if URCtrl.move_to_by_pose_name("robot_slot_2") !=0:
        return False
    if not AGVCtrl.move_backward():
        return False
    if move_path:
        if not AGVCtrl.move_to("path_start"):
            return False
        if not AGVCtrl.move_to("path_end"):
            return False
        if not AGVCtrl.move_to("t601"):
            return False
    if AGVCtrl.dock_to("ts6charger"):
        end_time = time.time()
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing","agv move@ok@" + str(round(end_time - start_time, 2)))
        return True
    else:
        end_time = time.time()
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       "agv move@failed@" + str(round(end_time - start_time, 2)))
        return False

def pick_and_place_demo():
    t1 = time.time()
    message = ""
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_1","robot_slot_1")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_2", "robot_slot_2")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_3", "robot_slot_3")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_4","robot_slot_4")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_5", "robot_slot_5")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("input_rack_a_slot_6", "robot_slot_6")
    if "" != message: return message
    t2 = time.time()
    LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                    "loading@total_time@" + str(round(t2-t1,2)))
    if not agvMoveBackwardAndCharging(True):
        return "move to charger failed"
    t3 = time.time()
    # CbtConfig.reset_arm_pose_corr_factor()
    message = DaemonCtrl.pick_and_place_("robot_slot_1","input_rack_a_slot_1")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("robot_slot_2", "input_rack_a_slot_2")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("robot_slot_3", "input_rack_a_slot_3")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("robot_slot_4","input_rack_a_slot_4")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("robot_slot_5", "input_rack_a_slot_5")
    if "" != message: return message
    message = DaemonCtrl.pick_and_place_("robot_slot_6", "input_rack_a_slot_6")
    if "" != message: return message
    t4= time.time()
    LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                   "unloading@totaltime@" + str(round(t4 - t3, 2)))
    if 0 != URCtrl.move_to_by_pose_name("arm_home"):
        return "move arm failed"
    t5 = time.time()
    print(f"total time:{round(t5 - t1, 2)}s")
    return message

def pick_and_place_loop_testing():
    for i in range(15):
        battery = AGVCtrl.get_agv_battery()
        print(f"====No.{str(i)}:battery={battery}====")
        if battery > 30:
            if not agvMoveBackwardAndCharging(True):
                break
            t1 = time.time()
            message = pick_and_place_demo()
            t2 = time.time() - t1
            LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                           "_loadAndUnload@total_time@" + str(t2))
            if "" != message:
                LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                               "error@message@" + message)
                break
        else:
            if agvMoveBackwardAndCharging(False):
                if AGVCtrl.enable_charging():
                    print(f"battery={battery},charging")
            break

def load_unload_one_fixture(id):
    if id > 0:
        i = str(id)
        DaemonCtrl.pick_and_place_("input_rack_a_slot_"+i, "robot_slot_"+i)
        DaemonCtrl.pick_and_place_("robot_slot_"+i, "input_rack_a_slot_"+i)

# new gripper
# URCtrl.move_to_by_pose_name("new_gripper_robot_home")
# URCtrl.move_to_by_pose_name("new_gripper_robot_home_60_degree")
# URCtrl.move_to_by_pose_name("arm_home")
# URCtrl.move_to_by_pose_name("virtual_cnc_prepare_pose")
# URCtrl.move_to_by_pose_name("virtual_cnc_pallet_a_slot_3")
# URCtrl.tool_relative_move_from_pose_name("virtual_cnc_pallet_a_slot_1",[-0.08,0,-0.15,0,0,0])
# new_gripper_pick_and_place_demo_60() # 60 deg gripper
# new_gripper_pick_and_place_demo_90() # 90 deg gripper
# URCtrl.move_to_by_pose_name("new_table_slot_5")
# URCtrl.move_to_by_pose_name("new_80table_slot_4")
# URCtrl.move_to_by_pose_name("new_80table_slot_b_5")
# URCtrl.move_tool([0,0,0.05,0,0,0])

# URCtrl.move_to_by_pose_name("robot_pallet_a_slot_1")
# URCtrl.move_tool([0,0,-0.1,0,0,0])
# URCtrl.move_tool([0,0,0.1,0,0,0])

# URCtrl.move_to_by_pose_name("new_gripper_back_home")

# marker testing
# LightCtrl.light_adjust(100)
# LightCtrl.light_off()
# pick_and_place_demo()
# URCtrl.move_to_by_pose_name("input_rack_b_marker_1")
# URCtrl.move_to_by_pose_name("input_cnc_a_marker_79")
# MkrFixture.adjust_marker_pose("input_cnc_a_marker_79","PickAndPlaceFactor")
# MkrFixture.adjust_marker_pose("input_rack_1_marker_1","PickAndPlaceFactor")

# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_05")  # Hor 5 circles
# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_06")  # vertical 5 circles

#pick and place demo
# pick_and_place_loop_testing()
# agvMoveBackwardAndCharging(True)
# load_unload_one_fixture(6)

# UR realtime data
# print(URCtrl.get_realtime_data())
print(URCtrl.get_actual_joint_position(True))#angle 86.09, -79.17, 125.3, 44.53, 80.56, -230.65
# print(URCtrl.get_actual_joint_position(False)) 90.33, -66.12, 130.15, 25.91, 80.39, -226.46
# print(URCtrl.get_actual_tcp_pose())

# take_picture()
# gripper_init()
# check_corr_direction()
# put_fixture_onto_robot("robot_slot_11")

#Remote I/O Test
#hex_str()
#RmtIO.set_output(False, 2)
#RmtIO.set_output(True, 2)

# Camera debug
# URCtrl.move_to_by_pose_name("arm_home")
# URCtrl.move_to_by_pose_name("arm_back_home")
# URCtrl.move_to_by_pose_name("arm_home")
# URCtrl.move_to_by_pose_name("arm_back_home")

# URCtrl.move_to_by_pose_name("cnc_71_slot_1")
# URCtrl.move_to_by_pose_name("cnc02_pallet_a_slot_1")
# URCtrl.move_to_by_pose_name("temp_pose")
# URCtrl.move_to_by_pose_name("input_rack_slot_1")
# URCtrl.move_to_by_pose_name("robot_slot_2")
# URCtrl.move_to_by_pose_name("input_rack_a_marker_1")

# relative_shift_ex(0.02035,-0.00922,0.23781)
# URCtrl.relative_shift([-0.13,-0.01,-0.05,0,0,0])
# URCtrl.relative_shift([0,0,0.1,0,0,0])
# URCtrl.relative_shift([0,0,-0.04,0,0,0])
# URCtrl.relative_shift([0,0,0,0,0,-0.01],5,True)
# URCtrl.move_ref_pose_tcp([0.10568, -1.47913, -2.41363, -0.82165, 1.56625, -1.50018],[0,0,0,0,0,0])
# URCtrl.move_tool([-0.001,0,0,0,0,0])
# URCtrl.move_tool([0,0,0,0,0,-0.034],10,True)#wrist3
# MkrFixture.adjust_gripper_pose(-1, 0.2, "input_rack_slot_1", "PickFactor")
# CamCtrl.setFromFile("E:/2021-11-10_17-26-26.jpg")
# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_01")
# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_04")
# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_03")
# MkrFixture.cal_marker_position(True, -1, "CircleBasedMkr_05")  # veqtical

# calc_mkr_position()
# gripper_pos_check()
# marker_proc_debug()
# read_barcode(True)
# read_barcode()
# read_barcode_test()

#Light debug
# LightCtrl.light_adjust(80)
# LightCtrl.light_on()
# LightCtrl.light_off()
# adj_light_brightness(80)
# CamCtrl.openCamera()
#Arm Debug
# URCtrl.move_to_by_pose_name("robot_slot_2")
# URCtrl.move_to_by_pose_name("arm_home")
#URCtrl.relative_shift([0,0,(-1)*pnp_offset_z,0,0,0])
# URCtrl.relative_shift([0,-0.01,0,0,0,0])
#URCtrl.relative_shift([-0.095,0,-0.03,0,0,0])
#URCtrl.move_to_by_pose_name("input_rack_slot_22")
# URCtrl.startup(-1)
# URCtrl.shutdown()
# URCtrl.get_mode()

#Gripper Debug
# gripper_init()
# GripperCtrl.gripper_close()
# GripperCtrl.gripper_open()
#URCtrl.relative_shift([0,0,pnp_offset_z,0,0,0])
#URCtrl.relative_shift([0,0,(-1)*pnp_offset_z,0,0,0])

#AGV Debug
# agv_goto_charge()
#AGVCtrl.clear_error()
# AGVCtrl.move_to("pos01")
# AGVCtrl.dock_to_cnc01()
# AGVCtrl.move_to("virtual_product_rack")
# AGVCtrl.dock_to_pallet_test()
# print(AGVCtrl.get_agv_status())
# print(AGVCtrl.get_agv_info())
# print(AGVCtrl.get_state_text())
# print(AGVCtrl.get_mission_text())
# print(AGVCtrl.get_guid_by_mission_name("ChargeAtStation"))
# print(AGVCtrl.get_guid_by_mission_name("gotocharge"))
# print(AGVCtrl.get_guid_by_mission_name("move_backward"))
# print(AGVCtrl.add_mission("168"))
# AGVCtrl.move_to("t601")
# AGVCtrl.dock_to("ts6charger")
# AGVCtrl.dock_to("cnc79marker")
# AGVCtrl.move_to("path_start")
# AGVCtrl.move_to("path_end")
# AGVCtrl.move_to("start")
# AGVCtrl.move_to("pos04")

# print(AGVCtrl.get_curr_position())

#COM Debug
#printComPorts()
#COMTest("COM7")
#singletonTest()

#UtilsFunc Debug
#rtn = FuncObj.ping("10.10.207.56")
#print("Ping returns " + "OK" if rtn else "NG")
#print(FuncObj.get_host_ip())
#print(FuncObj.calcValidationCode("FW,001,"))
#print(modbus_crc16([0x01,0x03,0x00,0x07,0x00,0x01]))
# read_barcode_test()