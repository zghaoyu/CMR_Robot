from ClassNewGripper import NewGripper
from ClassArm import ArmUR
from ClassCamera import CameraLogitech, LightCtrlLCPW
from ClassAGV import AgvMIR
from ClassConfig import CobotCfg
import LogHelper as LogObj
import time
import socket
from ClassDaemon import Daemon
from ClassPositionMarker import MarkerFixture

URCtrl = ArmUR()
CamCtrl = CameraLogitech()
LightCtrl = LightCtrlLCPW()
Gripper = NewGripper()
AGVCtrl = AgvMIR()
CbtConfig = CobotCfg()
MkrFixture = MarkerFixture()
TestDeamon = Daemon()

_target_ip = ("192.168.12.40", 30001)
_status_ip = ("192.168.12.40", 29999)
_realtime_ip = ("192.168.12.40", 30003)
_cobot_cfg = CobotCfg()
Daem = Daemon()
def agvMoveBackwardAndCharging(move_path=False):
    start_time = time.time()
    CbtConfig.reset_arm_pose_corr_factor()
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_5"): return False
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

# pick and place Testing
def pick_and_place(pick_from, place_to):
    payload = 3.0

    if 0 != Gripper.open():
        return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_1_marker_1"): return False
    MkrFixture.adjust_marker_pose("input_rack_1_marker_1", "PickAndPlaceFactor")

    # pick
    if 0 != URCtrl.move_to_by_pose_name("input_rack_1_slot_" + str(pick_from)):
        return False
    if 0 != URCtrl.move_tool([0, 0, -0.10, 0, 0, 0]):
        return False
    if 0 != Gripper.close(payload):
        return False
    if 0 != URCtrl.move_tool([0, 0, 0.02, 0, 0, 0]):
        return False
    if 0 != URCtrl.move_tool([0, 0, 0.08, 0, 0, 0]):
        return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("home_table_test_1"): return False
    # if 0 != URCtrl.move_tool([0.05, 0, 0, 0, 0, 0]):return False
    # time.sleep(1)
    # if 0 != URCtrl.move_tool([-0.05, 0, 0, 0, 0, 0]):return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False

    if 0 != URCtrl.move_to_by_pose_name("input_rack_1_slot_" + str(place_to)):
        return False
    # place
    if 0 != URCtrl.move_tool([0, 0, -0.08, 0, 0, 0]):
        return False
    if 0 != URCtrl.move_tool([0, 0, -0.02, 0, 0, 0]):
        return False
    if 0 != Gripper.open(payload):
        return False
    if 0 != URCtrl.move_tool([0, 0, 0.10, 0, 0, 0]):
        return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    return True

def getMovejPose(poseAngles):
    pose = ""
    for p in poseAngles:
        sP = ""
        iCnt = len(p)

        if iCnt != 6:
            return -1  # Invalid pose found

        for i in range(len(p)):
            if i == 0:
                sP = str(round(p[i] / 180 * 3.1415926, 5))
            else:
                sP = sP + "," + str(round(p[i] / 180 * 3.1415926, 5))

        if pose == "":
            pose = "[" + sP + "]"
        else:
            pose = pose + ";" + "[" + sP + "]"
    return pose


    def sendPickAndPlace(self,markerName,pick_from,pick_to):
        if 0 != Gripper.open():
            return False
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
            if 0 != URCtrl.move_to_by_pose_name(markerName): return False
            LightCtrl.light_adjust(70)
            adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(markerName, "PickAndPlaceFactor")
            LightCtrl.light_off()
            if not adjust_ok: return False
            marker = URCtrl.get_actual_joint_position(True)
            send_data1 = 'def pickAndPlace():\n'
            # back_home
            send_data1+= 'movej([2.31658,-1.61984,2.1862,1.00304,1.58319,-2.39581],a=3,v=10)\n'
            # new_back_home
            send_data1 += 'movej([-0.34208,-1.68442,1.89892,1.35403,1.58389,-2.66128],a=3,v=10)\n'
            send_data1 += 'movej(['+pick_from+'],a=3,v=10)\n'

        finally:
            return 1


# -0.34208,-1.68442,1.89892,1.35403,1.58389,-2.66128  new_back_home

# slot 1 3 7 pick and place demo
def demo1():
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # time.sleep(6)
    if not pick_and_place(1, 3): return False
    if not pick_and_place(9, 7): return False
    if not pick_and_place(3, 1): return False
    if not pick_and_place(7, 9): return False

    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_5"): return False

    return True

def pick_and_place_loop_testing():
    for i in range(15):
        battery = AGVCtrl.get_agv_battery()
        print(f"====No.{str(i)}:battery={battery}====")
        if battery > 30:
            if not agvMoveBackwardAndCharging(True):
                break
            t1 = time.time()
            message = demo1()
            t2 = time.time() - t1
            LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                           "_loadAndUnload@total_time@" + str(t2))
            if not message:
                LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                               "error@message@" + message)
                break
        else:
            if agvMoveBackwardAndCharging(False):
                if AGVCtrl.enable_charging():
                    print(f"battery={battery},charging")
            break

def pick_and_place_0510():
    payload=3.0
    moveCharger()

    marker_pose_name = "input_rack_b_marker_1"
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name(marker_pose_name): return False
    home1 = URCtrl.get_actual_joint_position(True)
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    home2 = URCtrl.get_actual_joint_position(True)

    # pick from table
    # Rack position 1
    if 0 != URCtrl.move_tool([-0.05635,-0.06916,-0.00448,-0.00016,0.17444,-0.00009]): return False #1
    if not PickRackToRobot() : return False
    # place to robot
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_1"): return False
    if not PlaceToRobot() : return False

    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_b_marker_1"): return False
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    if 0 != URCtrl.move_tool([-0.05505,-0.25844,-0.00435,-0.00052,0.17446,0.00001]): return False #2

    if not PickRackToRobot() : return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_2"): return  False
    if not PlaceToRobot() : return False

    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_b_marker_1"): return False
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    if 0 != URCtrl.move_tool([-0.05352,-0.44847,-0.00438,0.00009,0.17472,0.00014]): return False #3

    if not PickRackToRobot() : return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3"): return False
    if not PlaceToRobot() : return False

    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False

    # move AGV to CNC 79
    moveCNC79()
    #1
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_1"): return False
    if not PickRobot(): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_a_marker_79"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1b"): return False
    time.sleep(3)
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_1"): return False
    if not PlaceToRobot(): return False

    #2
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_2"): return False
    if not PickRobot() : return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_a_marker_79"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1b"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_2b"): return False
    time.sleep(3)
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1b"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_2"): return False
    if not PlaceToRobot() : return False

    #3
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3"): return False
    if not PickRobot() : return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_a_marker_79"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1b"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_3b"): return False
    time.sleep(3)
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1b"): return False
    if 0 != URCtrl.move_to_by_pose_name("cnc_pos_1a"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3"): return False
    if not PlaceToRobot() : return False

    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    moveCharger()

    #Place
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_1"): return False
    if not PickRobot(): return  False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_b_marker_1"): return False
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    if 0 != URCtrl.move_tool([-0.05635,-0.06916,-0.00448,-0.00016,0.17444,-0.00009]): return False  # 1
    if not PlaceRack() : return False

    #2
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_2"): return False
    if not PickRobot(): return  False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_b_marker_1"): return False
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    if 0 != URCtrl.move_tool([-0.05505,-0.25844,-0.00435,-0.00052,0.17446,0.00001]): return False  # 2
    if not PlaceRack() : return  False

    # 3
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3"): return False
    if not PickRobot(): return  False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    # if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("input_rack_b_marker_1"): return False
    LightCtrl.light_adjust(70)
    adjust_ok, adjust_time, draw_img = MkrFixture.adjust_marker_pose(marker_pose_name, "PickAndPlaceFactor")
    LightCtrl.light_off()
    if not adjust_ok: return False
    if 0 != URCtrl.move_tool([-0.05352,-0.44847,-0.00438,0.00009,0.17472,0.00014]): return False  # 3
    if not PlaceRack(): return  False

    return True

def PickRobot():

    if 0 != URCtrl.move_tool([0,0,-0.10,0,0,0]): return False
    if Gripper.close() !=0 :return False
    if 0 != URCtrl.move_tool([0,0,0.01,0,0,0]): return False
    if 0 != URCtrl.move_tool([0,0,0.09,0,0,0]): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    return True

def PickRackToRobot():

    if 0 != URCtrl.move_tool([0,0,-0.10,0,0,0]): return False
    if Gripper.close() !=0 :return False
    if 0 != URCtrl.move_tool([0,0,0.01,0,0,0]): return False
    if 0 != URCtrl.move_tool([0,0,0.09,0,0,0]): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False
    return True

def PlaceToRobot():
    if 0 != URCtrl.move_tool([0, 0, -0.08, 0, 0, 0]): return False
    if 0 != URCtrl.move_tool([0, 0, -0.02, 0, 0, 0]): return False
    if Gripper.open() !=0 : return False
    if 0 != URCtrl.move_tool([0, 0, 0.10, 0, 0, 0]): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_back_home"): return False

    return True

def PlaceRack():
    if 0 != URCtrl.move_tool([0, 0, -0.08, 0, 0, 0]): return False
    if 0 != URCtrl.move_tool([0, 0, -0.02, 0, 0, 0]): return False
    if Gripper.open() !=0: return False
    if 0 != URCtrl.move_tool([0, 0, 0.10, 0, 0, 0]): return False
    if 0 != URCtrl.move_to_by_pose_name("new_arm_home"): return False

    return True

def moveCNC79():
    AGVCtrl.move_backward(10)
    AGVCtrl.dock_to("cnc79marker")

def moveCharger():
    AGVCtrl.move_backward(10)
    AGVCtrl.dock_to("ts6charger")
def gripper_open_data():
    send_data1 = ' set_standard_digital_out(0,False)\n'
    send_data1 += ' set_standard_digital_out(1,False)\n'
    send_data1 += ' set_standard_digital_out(2,False)\n'
    send_data1 += ' set_payload(5.0)\n'
    # when alarming
    send_data1 += ' while (get_standard_digital_in(0)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # when gripper is not ready
    send_data1 += ' while (not get_standard_digital_in(1)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # if ready, execute
    send_data1 += ' set_standard_digital_out(2,True)\n'
    send_data1 += ' sleep(0.2)\n'
    send_data1 += ' set_standard_digital_out(2,False)\n'
    # if status is false, gripper is running
    send_data1 += ' while (not get_standard_digital_in(1)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # when alarming
    send_data1 += ' while (get_standard_digital_in(0)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    return send_data1
def gripper_close_data():
    send_data1 = ' set_standard_digital_out(0,False)\n'
    send_data1 += ' set_standard_digital_out(1,False)\n'
    send_data1 += ' set_standard_digital_out(2,False)\n'
    send_data1 += ' set_payload(5.0)\n'
    # when alarming
    send_data1 += ' while (get_standard_digital_in(0)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # when gripper is not ready
    send_data1 += ' while (not get_standard_digital_in(1)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # if ready, execute
    send_data1 += ' set_standard_digital_out(0,True)\n'
    send_data1 += ' sleep(0.2)\n'
    send_data1 += ' set_standard_digital_out(0,False)\n'
    # if status is false, gripper is running
    send_data1 += ' while (not get_standard_digital_in(1)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    # when alarming
    send_data1 += ' while (get_standard_digital_in(0)):\n'
    send_data1 += '  sleep(0.1)\n'
    send_data1 += ' end\n'
    return send_data1
def move_tool_data(pos_delta):
    send_data = ' tcpPose = get_actual_tcp_pose()\n'
    send_data += ' poseTo = pose_trans(tcpPose,p' + str(pos_delta)+ ')\n'
    send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
    send_data += ' movel(jointPose,1,1)\n'
    return send_data
def pick_robot_1_to_rack_1():
    rtnVal = -1
    timeout_s=60
    try:
    # Connect
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(_target_ip)
        statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        statusSck.connect(_status_ip)
        send_data = 'def mov():\n'
        send_data+= ' movej([-0.34208,-1.68442,1.89892,1.35403,1.58389,-2.66128],a=3, v=10)\n'
        send_data+= ' movej([-0.5976,-0.49428,0.30264,1.61617,1.48301,-2.95798],a=3, v=10)\n'
        send_data+= gripper_open_data()
        send_data+= move_tool_data([0.0, 0.0, -0.1, 0.0, 0.0, 0.0])
        send_data+= gripper_close_data()
        send_data+= move_tool_data([0.0, 0.0, 0.005, 0.0, 0.0, 0.0])
        send_data+= move_tool_data([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
        send_data+= ' movej([-0.34208,-1.68442,1.89892,1.35403,1.58389,-2.66128],a=3, v=10)\n'
        send_data+= ' movej([2.31658,-1.61984,2.1862,1.00304,1.58319,-2.39581],a=3, v=10)\n'
        send_data+= ' movej([1.6064,-0.89954,1.24966,1.21737,1.58197,-3.89331],a=3, v=10)\n'
        send_data+=move_tool_data([-0.2,-0.5,0,0,0,0])
        send_data+= move_tool_data([0.20229, -0.44751, -0.04115, -0.00034, 0.17206, 0.0238])

        send_data+= move_tool_data([0.0, 0.0, -0.095, 0.0, 0.0, 0.0])
        send_data+= move_tool_data([0.0, 0.0, -0.005, 0.0, 0.0, 0.0])
        send_data+= gripper_open_data()
        send_data+= move_tool_data([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
        send_data+= ' movej([2.31658,-1.61984,2.1862,1.00304,1.58319,-2.39581],a=3, v=10)\n'
    finally:

        send_data = send_data + 'end\n'
        print(send_data)
        s.send(send_data.encode('utf8'))
        startT = time.time()
        # Wait for running
        while (True):
            statusSck.send("running\n".encode())
            recvData = (statusSck.recv(1024)).decode()
            # print(recvData)
            if recvData.__contains__("running: true"):
                break
        if rtnVal < 0:
            startT = time.time()
            # Wait for finishing
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                # print(recvData)
                if recvData.__contains__("running: false"):
                    rtnVal = 0  # Success
                    # print(time.time())
                    passT = time.time() - startT
                    print("_______________________________________________________")
                    # print(startT)
                    # print(passT)
                    print("\npassT:"+str(passT))
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > timeout_s:
                        print("waitForFinishing: timeout")

                        rtnVal = 2
                        break

        s.close()
        statusSck.close()
        return rtnVal
def testFuc(pick_from_pose_name,place_to_pose_name):
    pick_config = CbtConfig.get_arm_pose_factor(pick_from_pose_name, "PickFactor")
    # place_config = CbtConfig.get_arm_pose_factor(place_to_pose_name, "PlaceFactor")
    poseCorr = CbtConfig.get_arm_pose_factor(pick_from_pose_name, "CorrectXYZ")
    print("pick:"+str(pick_config)+'\n')
    # print("pick:"+str(place_config)+'\n')
    # print("poseCorr:"+str(poseCorr)+'\n')
    rtnMsg, xyz_shift = TestDeamon._get_pose_relative_shift(pick_from_pose_name,"PickFactor","xyz_shift_safe_leaving(m)")
    print(rtnMsg)
    print(xyz_shift)
    res = 1692931880.4338515 - 1692931861.730031
    print(str(res)+"s")




# ScanCode.detect()
# print(test)
# URCtrl.send_command(test)
# Daem.new_pick_and_place_("robot_pallet_a_slot_7","input_rack_1_slot_7")

# Daem.new_do_tool_relative_shift("input_rack_1_slot_1","PickFactor","xyz_shift_do_operation(m)",timeout_s=15)
# demo testing
# print(ArmUR.new_move_tool([0.0, 0.0, -0.1, 0.0, 0.0, 0.0] ,30,False, False, 0.25, 1))
# demo1()
# print(URCtrl.new_move_to_by_pose_name("new_arm_back_home"))

# testFuc("robot_pallet_a_slot_1","input_rack_1_slot_1")
# print(move_tool_data([0.0, 0.0, -0.1, 0.0, 0.0, 0.0]))
# pick_and_place(3,7)
# pick_and_place(5, 5)
# pick_robot_1_to_rack_1()
# Arm position
# 58.31, -35.71, 94.83, -59.26, 238.34, -135.1
# 169.64, -70.14, 83.34, -13.16, 78.78, -136.32    cnc79pick pose
# print(URCtrl.get_actual_joint_position(True))#angle
# [-22.35, -87.5, 138.42, 29.31, 86.48, -158.29] slot 15
# 153.73, -59.82, 86.81, 63.02, 90.72, -153.37 home
# 151.48, -68.46, 101.16, 57.27, 90.7, -163.79
# URCtrl.move_to_by_pose_name("test_16_pos")
# URCtrl.move_to_by_pose_name("cnc_pos_1b")
# URCtrl.move_to_by_pose_name("cnc_pos_3b")
# URCtrl.move_to_by_pose_name("b")

# place
# URCtrl.move_to_by_pose_name("new_arm_back_home")
# URCtrl.move_to_by_pose_name("input_rack_1_marker_1")
# URCtrl.move_to_by_pose_name("cnc_a_marker_79")
# URCtrl.move_to_by_pose_name("output_rack_1_marker_1")
# URCtrl.move_to_by_pose_name("cnc_a_marker_72")
# URCtrl.move_to_by_pose_name("robot_pallet_a_slot_2")
# MkrFixture.adjust_marker_pose("cn
# c_a_marker_79","PickAndPlaceFactor")

# MkrFixture.adjust_marker_pose("cnc_a_marker_79","PickAndPlaceFactor")
# MkrFixture.adjust_marker_pose("cnc_a_marker_74","PickAndPlaceFactor")
# MkrFixture.adjust_marker_pose("output_rack_1_marker_1","PickAndPlaceFactor")
# URCtrl.move_tool([0,0,0.1,0,0,0])
# URCtrl.move_tool([-0.2,-0.5,0,0,0,0])
# URCtrl.move_tool([ 0.20246,-0.22473,-0.03985,-0.00001,0.1744,0.00023])


# URCtrl.move_tool([0.01526,-0.22585,-0.03857,-0.00026,0.17214,0.02364])
# URCtrl.move_tool([-0.005,0,0.00,0,0,0]) *2
# URCtrl.move_tool([0.00,0.00,-0.002,0,0,0])
# URCtrl.move_tool([0,0,-0.01,0,0,0])---------------------------------------------------

# URCtrl.move_tool([0,0,0,0,1.57076,0])

# URCtrl.move_tool([0,0,-0.2,0,0,0])


# URCtrl.move_tool([0,0,0,0,1.57076,0])
# URCtrl.move_tool([0,0,0.150,0,0,0])
# URCtrl.move_tool([-0.28557,-0.42383,-0.03432,-0.00102,0.18051,0.00139])
# URCtrl.move_tool([-0.09895,-0.42335,-0.03726,-0.00152,0.18039,0.00092])
# URCtrl.move_tool([0.20418,-0.22449,-0.03996,-0.00027,0.1747,0.00006])
# URCtrl.move_tool([0.01874,-0.22712,-0.03808,-0.00003,0.17463,0.00022])
# URCtrl.move_tool([0.02122,-0.45084,-0.03888,0.00014,0.17461,0.00021])
# URCtrl.move_tool([0.10214,-0.33787,-0.03894,-0.00209,0.17939,-0.00007])
# URCtrl.move_tool([-0.08599,-0.11283,-0.03584,-0.00199,0.17948,-0.00015])
# URCtrl.move_tool([0.20604,-0.44949,-0.04062,-0.00002,0.17453,0.00006])
# URCtrl.move_tool([0.01595,-0.22477,-0.03797,-0.00037,0.17219,0.02362])

# URCtrl.move_tool([0.20399,-0.44828,-0.04047,-0.00046,0.17233,0.0252])

# URCtrl.move_tool([0,0,0.3,0,0,0]) # cnc up 200mm
# URCtrl.move_tool([0,0,0,0,1.57076,0]) # cnc up 200mm
# URCtrl.move_tool([0,0,0,0,1.57076,0]) # cnc Y Rotation 90
# URCtrl.move_tool([0,0,-0.1,0,0,0]) # cnc Z -0.38301 in
# URCtrl.move_tool([0,-0.0544,0,0,0,0]) # cnc Y -0.0544 left
# URCtrl.move_tool([0,0,-0.28301,0,0,0]) # cnc Z -0.38301 in
# URCtrl.move_tool([0.103,0,0,0,0,0]) # cnc X 0.103 down
# open gripper
# URCtrl.move_tool([0,0,0.3,0,0,0]) # cnc z 0.4 out (400mm)
# URCtrl.move_tool([0,0,0,0,-1.57076,0]) # cnc Y Rotation -90

#Pick
# URCtrl.move_to_by_pose_name("new_arm_home")
# URCtrl.move_to_by_pose_name("a")
# URCtrl.move_to_by_pose_name("cnc_a_marker_79")
# URCtrl.move_to_by_pose_name("cnc_a_marker_72")
# MkrFixture.adjust_marker_pose("cnc_a_marker_72","PickAndPlaceFactor")
# MkrFixture.adjust_marker_pose("cnc_a_marker_79","PickAndPlaceFactor")
# URCtrl.move_tool([0,0,0.097,0,0,0]) # cnc up 97mm
# URCtrl.move_tool([0,0,0,0,1.57076,0]) # cnc Y Rotation 90
# URCtrl.move_tool([0,-0.0544,0,0,0,0]) # cnc Y -0.0544 left
# URCtrl.move_tool([0,0,-0.38301,0,0,0]) # cnc Z -0.38301 in

# Close gripper
# URCtrl.move_tool([0.04,0,0,0,0,0]) # cnc X 0.005 up
# URCtrl.move_tool([-0.1,0,0,0,0,0]) # cnc X 0.100 up
# URCtrl.move_tool([0,0,0.4,0,0,0]) # cnc z 0.4 out (400mm)
# URCtrl.move_tool([0,0,0,0,-1.57076,0]) # cnc Y Rotation -90

#     y=0.004  x=-0.009     z=-0.001
# URCtrl.move_tool([-0.009, 0.004, -0.001, 0.000, 0, 0])
# URCtrl.move_tool([0,0,-0.150,0,0,0])
# URCtrl.move_to_by_pose_name("cnc72_arm_turn")

# URCtrl.move_tool([0,0,0.00002,0.00006,-1.57076,0.00004]) # cnc Y Rotation -90
# URCtrl.move_tool([0.00108,-0.3099,-0.03772,-0.00004,0.17454,-0.00006]) # cnc Y Rotation -90
# URCtrl.move_tool([0,0,0.2,0,1.57076,0]) # cnc up 200mm and rotation 90


# LightCtrl.light_adjust(100)
# LightCtrl.light_off()
# URCtrl.move_to_by_pose_name("input_rack_b_marker_1")
# URCtrl.move_to_by_pose_name("input_rack_b_marker_1")
# URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3")
# 5 point cal
# MkrFixture.adjust_marker_pose("input_rack_1_marker_1","PickAndPlaceFactor")
# URCtrl.move_tool([0.20229,-0.44751,-0.04115,-0.00034,0.17206,0.0238])#1
# URCtrl.move_tool([-0.05505,-0.25844,-0.00435,-0.00052,0.17446,0.00001])#2
# URCtrl.move_to_by_pose_name("input_rack_b_marker_1")
# MkrFixture.adjust_marker_pose("input_rack_b_marker_1","PickAndPlaceFactor")
# URCtrl.move_tool([-0.05004,-0.44817,-0.00467,0.00041,0.17451,-0.00027])#3 old
# URCtrl.move_tool([-0.05352,-0.44847,-0.00438,0.00009,0.17472,0.00014])#3

# cnc pick and place start poisition cnc79
# URCtrl.move_tool([0.1304,0.02673,0.22021,0.03075,1.61852,-0.03226])
# URCtrl.move_tool([-0.02885,-0.00343,0.19476,-0.14398,1.6522,0.06857]) #old
#CNC position one
# URCtrl.move_tool([0.05442,-0.10253,-0.50716,-0.04273,-0.01664,0.00519])

# Z
# URCtrl.move_tool([0,0,-0.15,0,0,0])
# URCtrl.move_tool([0,0,0.1,0,0,0])
#Y
# URCtrl.move_tool([0,-0.225,0,0,0,0])
#X
# URCtrl.move_tool([0.04,0,0,0,0,0])

# Gripper.close(6)
#
# URCtrl.move_to_by_pose_name("output_rack_1_slot_1")
URCtrl.move_to_by_pose_name("new_arm_back_home")
# URCtrl.move_to_by_pose_name("robot_pallet_a_slot_3")
# URCtrl.move_to_by_pose_name("home_table_test_1")
# URCtrl.move_to_by_pose_name("new_arm_home")

# URCtrl.move_tool([0.025, 0.025, -0.02, 0, 0, 0])  # down
# URCtrl.move_tool([0, 0,-0.01, 0, 0, 0])  # down
# URCtrl.move_tool([0,0,0.10,0,0,0])
# Gripper.close(6)
# URCtrl.move_to_by_pose_name("new_gripper_back_home")

# Gripper Testing
# Gripper.reset()
# Gripper.open()
# Gripper.close()

# pick_and_place_loop_testing()
# pick_and_place_0510()

# robot_pallet_a_slot_13=10.43, -78.26, 106.42, 51.0, 92.72, -125.92

# URCtrl.testParams()
# CamCtrl.cameraSetting("BarCode_Scan",True)