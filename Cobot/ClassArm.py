import socket
import time
import threading
import LogHelper as LogObj
import UtilsFunc as FuncObj
import random
import struct
from ClassConfig import CobotCfg
from ClassNewGripper import NewGripper


# Singleton class
class ArmUR(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerArm'
    _target_ip = ("192.168.12.40", 30001)
    _status_ip = ("192.168.12.40", 29999)
    _realtime_ip = ("192.168.12.40", 30003)
    _config_arm_pose = None
    _curr_arm_pose = "Unkown"
    _cal_z_height = 0.126  # z-height for calibration
    _cobot_cfg = CobotCfg()
    _arm_v = "v=5"  # 5
    _arm_a = "a=3"  # 3
    _rotate_v = "v=1" #0.4
    _rotate_a = "a=1" #0.25
    _tool_v = "v=5"  # Max: 1m/s #1   0.8
    _tool_a = "a=1"#0.25    0.15
    Gripper = NewGripper()

    def __init__(self, *args, **kwargs):
        with ArmUR._instance_lock:
            if self._config_arm_pose is None:
                self._config_arm_pose = self._cobot_cfg.get_arm_pose_cfg()

    def __new__(cls, *args, **kwargs):
        with ArmUR._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    ArmUR._instance = super().__new__(cls)
            return ArmUR._instance

    def clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)

    def get_cal_z_height(self):
        return self._cal_z_height

    def get_curr_pose_name(self):
        return self._curr_arm_pose
    def moveJ(self,pose):
        send_data1 =  ' movej(' + pose + ',' + self._arm_a + ', ' + self._arm_v + ')\n'
        return send_data1
    def shutdown(self, timeout_s=5):
        rtnVal = -1
        try:
            startT = time.time()
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            # CB3 Safety Mode: NORMAL|REDUCED|PROTECTIVE_STOP|RECOVERY|SAFEGUARD_STOP|SYSTEM_EMERGENCY_STOP|ROBOT_EMERGENCY_STOP|VIOLATION|FAULT
            statusSck.send("safetymode\n".encode())
            time.sleep(0.25)
            safetyMode = (statusSck.recv(1024)).decode()
            print("safety mode: " + safetyMode)

            statusSck.send("shutdown\n".encode())
            time.sleep(0.1)
            while (True):
                statusSck.send("robotmode\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                if recvData.__contains__("Shutting down"):
                    rtnVal = 0  # Success
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if timeout_s > 0 and passT > timeout_s:
                        print("shutdown: timeout")
                        rtnVal = 1
                        break
            statusSck.close()
        except socket.error as msg:
            LogObj.logSystemError()
        return rtnVal

    def startup(self, timeout_s=5):
        rtnVal = -1
        try:
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            startT = time.time()
            while (True):
                # CB3 Robot Mode: NO_CONTROLLER|DISCONNECTED|CONFIRM_SAFETY|BOOTING|POWER_OFF|POWER_ON|IDLE|BACKDRIVE|RUNNING
                statusSck.send("robotmode\n".encode())
                time.sleep(0.25)
                recvData = (statusSck.recv(1024)).decode()
                print("robot mode: " + recvData)
                if recvData.__contains__("Powering on") or recvData.__contains__("POWER_ON") or recvData.__contains__(
                        "RUNNING"):
                    rtnVal = 0  # Success
                    break

                if recvData.__contains__("NO_CONTROLLER") or recvData.__contains__(
                        "DISCONNECTED") or recvData.__contains__("BACKDRIVE") or recvData.__contains__("Powering off"):
                    LogObj.logAppError("Arm", "power_on failed: " + recvData)
                    rtnVal = 1
                    break
                if recvData.__contains__("BOOTING"):
                    time.sleep(1)
                    startT = time.time()
                    continue
                if recvData.__contains__("CONFIRM_SAFETY"):
                    statusSck.send("close safety popup\n".encode())
                    time.sleep(1)
                    startT = time.time()
                    continue
                if recvData.__contains__("POWER_OFF"):
                    statusSck.send("power on\n".encode())
                    time.sleep(1)
                    startT = time.time()
                    continue
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if timeout_s > 0 and passT > timeout_s:
                        print("power_on: timeout")
                        LogObj.logAppError("Arm",
                                           "power_on timeout(" + str(round(passT, 3)) + ">" + str(timeout_s) + "s)")
                        rtnVal = 2
                        break
            statusSck.close()
        except socket.error as msg:
            LogObj.logSystemError()
        return rtnVal

    def random_shift(self, xyzEnabled=[True, True, True], delayAfter_s=0.2):
        ok = True
        randomX = 0.0;
        randomY = 0.0;
        randomRz = 0.0
        if xyzEnabled[0]: randomX = (random.randint(-9, 12) - 1.0) / 1000  # -0.01 to 0.01
        if xyzEnabled[1]: randomY = (random.randint(-9, 12) - 1.0) / 1000  # -0.01 to 0.01
        if xyzEnabled[2]: randomRz = (random.randint(-4, 7) - 1.0)  # -5 to 5

        if (xyzEnabled[0] or xyzEnabled[1]) and 0 != self.relative_shift([randomX, randomY, 0, 0, 0, 0], 5, False):
            ok = False
        if ok and xyzEnabled[2] and 0 != self.relative_shift([0, 0, 0, 0, 0, round(randomRz / 180 * 3.1415926, 7)], 5,
                                                             True):
            ok = False
        if ok and delayAfter_s > 0: time.sleep(delayAfter_s)
        return ok, [randomX, randomY, randomRz]

    # 2022/8/11 angles -> rad
    def get_joint_position_by_pose_name(self, pose_name):
        ori_pose = FuncObj.getDictVal(self._config_arm_pose, pose_name, None)

        poseAngles = []
        angles = ori_pose.split(",")
        try:
            for agl in angles:
                poseAngles.append(float(agl))
        except Exception as e:
            LogObj.logSystemError()
            return -1

        pose = []
        for i in range(len(poseAngles)):
            pose.append(round(poseAngles[i] / 180 * 3.1415926, 5))

        print(pose)
        return pose

    #add by Hui Zhi
    def get_rotate_wrist3_position(self, joint_pose_name, rotate_val, is_angle):
        pose = self.get_joint_position_by_pose_name(joint_pose_name)

        if len(pose) == 6:
            if is_angle:  # 角度
                if -20 <= rotate_val <= 20:
                    pose[-1] += round(rotate_val / 180 * 3.1415926, 5)
                else:
                    print("大于20度")
            else:  # 弧度
                if -20 <= rotate_val * 180 / 3.1415926 <= 20:
                    pose[-1] += rotate_val
                else:
                    print("大于20度")
        return pose

    #add by Hui Zhi 2022/12/8
    def tool_relative_move_from_pose_name(self, poseFromName, posDelta, timeout_s=30, bRetry=False):
        rtnVal = -1
        poseFrom = self.get_joint_position_by_pose_name(poseFromName)
        rtnVal = self.tool_relative_move_from_rad_pose(poseFrom,posDelta,timeout_s,bRetry)
        return rtnVal

    # add by Hui Zhi
    # 计算从poseFrom(joint_pos(rad))位姿，以工具坐标系原点，移动posDelta后的位置。 (add by Hui Zhi 2022/3/30)
    # 谨慎使用，poseFrom必须是rad的关节参数
    def tool_relative_move_from_rad_pose(self, poseFrom, posDelta, timeout_s=30, bRetry=False):
        if not bRetry: ArmUR._instance_lock.acquire()
        rtnVal = -1

        # Check variable
        if len(posDelta) != 6 or len(poseFrom) != 6:
            return rtnVal

        pos_delta = 'p' + str(posDelta)

        try:
            # Connect
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            # Write UR Script and send data
            send_data = 'def mov():\n'
            send_data += ' tcpPose = get_forward_kin(' + str(poseFrom) + ')\n'  # joint to tcp pose
            send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
            send_data += ' movej(poseTo,' + self._arm_a + ', ' + self._arm_v + ')\n'
            send_data += 'end\n'

            print(send_data)
            s.send(send_data.encode())

            # Wait for running
            startT = time.time()
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                if recvData.__contains__("running: true"):
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        if not bRetry:
                            rtnVal = self.move_tcp(poseFrom, posDelta, timeout_s, True)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Arm", "move_ref_pose_tcp start,timeout(" + str(round(passT, 3)) + ">5s)")
                            rtnVal = 1
                        break

            # Wait for finishing
            if rtnVal < 0:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        rtnVal = 0  # Success
                        passT = time.time() - startT
                        LogObj.logInfo(self._home_dir, "armMotionTime",
                                       "move_ref_pose_tcp," + str(posDelta) + "," + str(round(passT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Arm", "move_ref_pose_tcp stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            rtnVal = 2
                            break

        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: ArmUR._instance_lock.release()
            return rtnVal

        # Close
        s.close()
        statusSck.close()
        # if 0 == rtnVal:
        #     self._curr_arm_pose = ""  # TODO
        # else:
        #     self._curr_arm_pose = "Unkown"
        if not bRetry: ArmUR._instance_lock.release()
        return rtnVal

    def move_tool(self, posDelta, timeout_s=30, bSetJoint=False, bRetry=False,a=1,v=5):
        """
        以当前位置为原点，根据工具坐标系进行移动 Hui Zhi Fang
        :param posDelta: [x,y,z,rx,ry,rz]
        :param timeout_s: 默认30秒
        :param bSetJoint: 是否旋转关节
        :param bRetry: 是否失败重试
        :return:
        """
        if not bRetry: ArmUR._instance_lock.acquire()
        rtnVal = -1

        try:
            # Connect
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            # Write UR Script and send data
            if bSetJoint:
                a = self._rotate_a
                v = self._rotate_v
                # send_data = 'def mov():\n'
                # send_data += ' pos = get_actual_joint_positions()\n'
                # send_data += ' newPose = [pos[0]+' + str(posDelta[0]) + ',pos[1]+' + str(
                #     posDelta[1]) + ',pos[2]+' + str(posDelta[2]) + ',pos[3]+' + str(posDelta[3]) + ',pos[4]+' + str(
                #     posDelta[4]) + ',pos[5]+' + str(posDelta[5]) + ']\n'
                # send_data += ' movej(newPose,' + str(a) + ', ' + str(v) + ')\n'
                # send_data += 'end\n'
                pos_delta = 'p' + str(posDelta)
                send_data = 'def mov():\n'
                send_data += ' tcpPose = get_actual_tcp_pose()\n'
                send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
                send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
                send_data += ' movej(jointPose,' + str(a) + ', ' + str(v) + ')\n'
                send_data += 'end\n'
                print("TRUEjoint send_data:\n"+send_data)
                s.send(send_data.encode())
            else:
                pos_delta = 'p' + str(posDelta)
                send_data = 'def mov():\n'
                send_data += ' tcpPose = get_actual_tcp_pose()\n'
                send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
                send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
                send_data += ' movel(jointPose,' + str(a) + ', ' + str(v) + ')\n'
                send_data += 'end\n'
                print("Falsejoint send_data:\n" + send_data)
                s.send(send_data.encode())

            # Wait for running
            startT = time.time()
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                if recvData.__contains__("running: true"):
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        if not bRetry:
                            rtnVal = self.move_tool(posDelta, timeout_s, bSetJoint,True,a,v)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Arm", "move_tool start,timeout(" + str(round(passT, 3)) + ">5s)")
                            rtnVal = 1
                        break

            # Wait for finishing
            if rtnVal < 0:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        rtnVal = 0  # Success
                        passT = time.time() - startT
                        LogObj.logInfo(self._home_dir, "armMotionTime",
                                       "move_tool," + str(posDelta) + "," + str(round(passT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Arm", "move_tool stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            rtnVal = 2
                            break

        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: ArmUR._instance_lock.release()
            return rtnVal

        # Close
        s.close()
        statusSck.close()
        if not bRetry: ArmUR._instance_lock.release()
        return rtnVal

    def relative_shift(self, posDelta, timeout_s=10, bSetJoint=False, bRetry=False):
        if not bRetry: ArmUR._instance_lock.acquire()
        startT1 = time.time()
        rtnVal = -1
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            send_data1 = 'def mov():\n'
            if bSetJoint:
                send_data1 = send_data1 + ' pos = get_actual_joint_positions()\n'
                send_data1 = send_data1 + ' newPose = [pos[0]+' + str(posDelta[0]) + ',pos[1]+' + str(
                    posDelta[1]) + ',pos[2]+' + str(posDelta[2]) + ',pos[3]+' + str(posDelta[3]) + ',pos[4]+' + str(
                    posDelta[4]) + ',pos[5]+' + str(posDelta[5]) + ']\n'
                # send_data1 = send_data1 + ' movej(newPose,a=0.5, v=1.047)\n' #a=1.396
                send_data1 = send_data1 + ' movej(newPose,' + self._arm_a + ', ' + self._arm_v + ')\n'  # add by Hui Zhi 2022/3/7
            else:
                send_data1 = send_data1 + ' tcp = get_actual_tcp_pose()\n'
                send_data1 = send_data1 + ' newPose = p[tcp[0]+' + str(posDelta[0]) + ',tcp[1]+' + str(
                    posDelta[1]) + ',tcp[2]+' + str(posDelta[2]) + ',tcp[3]+' + str(posDelta[3]) + ',tcp[4]+' + str(
                    posDelta[4]) + ',tcp[5]+' + str(posDelta[5]) + ']\n'
                send_data1 = send_data1 + ' poseGo = pose_add(p[0,0,0,0,0,0],newPose)\n'
                # send_data1 = send_data1 + ' movej(poseGo,a=0.5, v=1.047)\n'
                send_data1 = send_data1 + ' movel(poseGo,' + self._tool_a + ', ' + self._tool_v + ')\n'  # add by Hui Zhi 2022/3/7
            if bRetry: send_data1 = send_data1 + ' sleep(1)\n'
            send_data1 = send_data1 + 'end\n'
            s.send(send_data1.encode('utf8'))

            # Wait for running
            startT = time.time()
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv( )).decode()
                if recvData.__contains__("running: true"):
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        if not bRetry:
                            rtnVal = self.relative_shift(posDelta, timeout_s, bSetJoint, True)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Arm", "relative_shift start,timeout(" + str(round(passT, 3)) + ">5s)")
                            rtnVal = 1
                        break

            # Wait for finishing
            if rtnVal < 0:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        rtnVal = 0  # Success
                        passT = time.time() - startT
                        LogObj.logInfo(self._home_dir, "armMotionTime",
                                       "relative_shift," + str(posDelta) + "," + str(round(passT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Arm", "relative_shift stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            rtnVal = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: ArmUR._instance_lock.release()
            return rtnVal

        s.close()
        statusSck.close()
        if not bRetry: ArmUR._instance_lock.release()
        return rtnVal

    def new_move_to_by_pose_name(self, poseName, timeout_s=30):
        startT = time.time()
        pose = FuncObj.getDictVal(self._config_arm_pose, poseName, None)
        if pose is None: return -1
        ps = pose.split(";")

        poseAngles = []
        try:
            for p in ps:
                angles = p.split(",")
                poseAngles.append([float(agl) for agl in angles])
        except Exception as e:
            LogObj.logSystemError()
            return -1

        rtn = self.new_move_to_ex(poseAngles, timeout_s, poseName)
        rtn = self.moveJ(rtn)
        passT = time.time() - startT
        if rtn == "":
            # LogObj.logInfo(self._home_dir, "armMotionTime", "fail at new_move_to_by_pose_name")
            print("fail at new_move_to_by_pose_name")
        return rtn

    def move_to_by_pose_name(self, poseName, timeout_s=30):
        startT = time.time()
        pose = FuncObj.getDictVal(self._config_arm_pose, poseName, None)
        if pose is None: return -1
        ps = pose.split(";")

        poseAngles = []
        try:
            for p in ps:
                angles = p.split(",")
                poseAngles.append([float(agl) for agl in angles])
        except Exception as e:
            LogObj.logSystemError()
            return -1
        rtn = self.move_to_ex(poseAngles, timeout_s, poseName)
        passT = time.time() - startT
        if rtn == 0:
            LogObj.logInfo(self._home_dir, "armMotionTime", "move_to," + poseName + "," + str(round(passT, 3)))
        return rtn



    def move_to_ex(self, poseAngles, timeout_s=30, poseName=""):
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
        return self.move_to(pose, timeout_s, False, poseName)
    def new_move_to_ex(self, poseAngles, timeout_s=30, poseName=""):
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

    def move_to(self, poseMeters, timeout_s=30, bRetry=False, poseName=""):
        if not bRetry: ArmUR._instance_lock.acquire()
        rtnVal = -1
        ps = poseMeters.split(";")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            # if not self._is_in_remote_control(statusSck, 1):
            #     LogObj.logAppError("Arm", "arm is not in remote control")
            #     return rtnVal

            send_data1 = 'def mov():\n'
            for p in ps:
                # send_data1 = send_data1 + ' movej(' + p + ',a=0.5, v=1.047)\n'
                send_data1 = send_data1 + ' movej(' + p + ',' + self._arm_a + ', ' + self._arm_v + ')\n'  # add by Hui Zhi 2022/3/7
            if bRetry: send_data1 = send_data1 + ' sleep(1)\n'
            send_data1 = send_data1 + 'end\n'
            print(send_data1)
            s.send(send_data1.encode('utf8'))

            # Wait for running
            startT = time.time()
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                if recvData.__contains__("running: true"):
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        if not bRetry:
                            rtnVal = self.move_to(poseMeters, timeout_s, True, poseName)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Arm", "move_to start,timeout(" + str(round(passT, 3)) + ">5s)")
                            rtnVal = 1
                        break

            # Wait for finishing
            if rtnVal < 0:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        rtnVal = 0  # Success
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Arm", "move_to stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            rtnVal = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: ArmUR._instance_lock.release()
            return rtnVal
        s.close()
        statusSck.close()
        if 0 == rtnVal:
            self._curr_arm_pose = poseName
        else:
            self._curr_arm_pose = "Unkown"
        if not bRetry: ArmUR._instance_lock.release()
        return rtnVal
    def send_command(self,command):  #发送命令字符串给ur机器人
        rtnVal = -1
        timeout_s = 60
        try:
            # Connect
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
        finally:
            # print(command)
            s.send(command.encode('utf8'))
            startT = time.time()
            # Wait for running
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                # print(recvData)
                if recvData.__contains__("running: true"):
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        print("waitForRunning: timeout")
                        LogObj.logAppError("Arm", "move_to start,timeout(" + str(round(passT, 3)) + ">5s)")
                        rtnVal = 1
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

                        # print(startT)
                        # print(passT)
                        print("\npassT:" + str(passT))
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
            if 0 != rtnVal:
                self._curr_arm_pose = "Unkown"
            return rtnVal
    # test robotParams
    def testParams(self):

            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(self._target_ip)
                statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                statusSck.connect(self._status_ip)

                # if not self._is_in_remote_control(statusSck, 1):
                #     LogObj.logAppError("Arm", "arm is not in remote control")
                #     return rtnVal

                send_data1 = 'def test():\n'
                # send_data1 = send_data1 + ' movej(' + p + ',a=0.5, v=1.047)\n'
                # send_data1 = send_data1 + ' movej(' + p + ',' + self._arm_a + ', ' + self._arm_v + ')\n'  # add by Hui Zhi 2022/3/7
                # send_data1+= '$ 2 "pos1 = [-0.18326,-1.30551,1.77535,0.91996,1.54811,-2.55551]"\n'
                # send_data1+= ' global pos1 = [0.18326,-1.30551,1.77535,0.91996,1.54811,-2.55551]\n'
                # send_data1+= '$ 3 "pos2 = [-0.7957,-1.29695,1.78285,0.95452,1.45037,-3.16201]"\n'
                # send_data1+= ' global pos2 = [-0.7957,-1.29695,1.78285,0.95452,1.45037,-3.16201]\n'
                send_data1+='movej([-0.5976,-0.49428,0.30264,1.61617,1.48301,-2.95798],a=1,v=5)\n'
                # send_data1 =  send_data1 + ' local var_1  = 10\n'  # add by Hui Zhi 2022/3/7

            finally:


                send_data1 = send_data1 + 'end\n'
                print(send_data1)
                s.send(send_data1.encode('utf8'))

            # Wait for running
            #     startT = time.time()
                # print("excute running")
                # statusSck.send('running\n'.encode())

                # time.sleep(2)
                # print("excute play")
                # statusSck.send('play\n'.encode())


                s.close()
                statusSck.close()
            return 1




    # add by Hui Zhi 2022/6/24
    def get_mode(self, time_out=5):
        try:
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            statusSck.send("robotmode\n".encode())
            statusSck.send("safetystatus\n".encode())
            statusSck.send("is in remote control\n".encode())
            statusSck.send("running\n".encode())
            time.sleep(0.25)
            data = (statusSck.recv(1024)).decode()
            print("receive data: " + data)
            statusSck.close()

        except socket.error as msg:
            LogObj.logSystemError()

    # add by Hui Zhi 2022/8/17
    def get_realtime_data(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._realtime_ip)
            data = s.recv(1116)
            s.close()
        except socket.error as msg:
            LogObj.logSystemError()

        dic = {'MessageSize': 'i',
               'Time': 'd',
               'q target': '6d',  # 关节目标位置
               'qd target': '6d',  # 速度
               'qdd target': '6d',  # 加速度
               'I target': '6d',  # 电流
               'M target': '6d',  # 扭矩
               'q actual': '6d',  # 实际位置
               'qd actual': '6d',  # 速度
               'I actual': '6d',  # 电流
               'I control': '6d',
               'Tool vector actual': '6d',  # tcp位置
               'TCP speed actual': '6d',
               'TCP force': '6d',
               'Tool vector target': '6d',
               'TCP speed target': '6d',
               'Digital input bits': 'd',
               'Motor temperatures': '6d',
               'Controller Timer': 'd',
               'Test value': 'd',
               'Robot Mode': 'd',
               'Joint Modes': '6d',
               'Safety Mode': 'd',
               'empty1': '6d',
               'Tool Accelerometer values': '3d',  # tcp加速度
               'empty2': '6d',
               'Speed scaling': 'd',
               'Linear momentum norm': 'd',
               'empty3': 'd',
               'empty4': 'd',
               'V main': 'd',
               'V robot': 'd',
               'I robot': 'd',
               'V actual': '6d',
               'Digital outputs': 'd',
               'Program state': 'd',
               'Elbow position': '3d',
               'Elbow velocity': '3d',
               'Safety Status': 'd',
               }

        j = range(len(dic))
        for key, i in zip(dic, j):
            fmt_size = struct.calcsize(dic[key])
            target_data, data = data[0:fmt_size], data[fmt_size:]
            fmt = "!" + dic[key]
            result = struct.unpack(fmt, target_data)
            dic[key] = result

        # print(dic)
        return dic

    def get_actual_joint_position(self,get_degree=False):
        result = []
        data = self.get_realtime_data()
        pos = data["q actual"]
        if len(pos) == 6:
            if(get_degree):
                for i in range(len(pos)):
                    result.append(round(pos[i] * 180 / 3.1415926, 2))
                # print(result)
            else:#rad
                for i in range(len(pos)):
                    result.append(round(pos[i], 5))
                # print(result)
        return result

    def get_actual_tcp_pose(self):
        result = []
        data = self.get_realtime_data()
        pos = data["Tool vector actual"]
        if len(pos) == 6:
                for i in range(len(pos)):
                    result.append(round(pos[i], 5))
                # print(result)
        return result

    #add by Hui Zhi 2022/9/27
    def _is_in_remote_control(self, statusSck, timeout_s=1):
        remote_mode = False
        if statusSck is None:
            print("parameter error")
            return remote_mode
        start_time = time.time()
        try:
            while True:
                pass_time = time.time() - start_time
                if pass_time > timeout_s:
                    break
                statusSck.send("is in remote control\n".encode())
                is_remote = (statusSck.recv(1024)).decode()
                if is_remote.__contains__("false"):
                    remote_mode = False
                    break
                elif is_remote.__contains__("true"):
                    remote_mode = True
                    break
                time.sleep(0.02)
        except socket.error as msg:
            LogObj.logSystemError()
        return remote_mode

    def gripper_open_data(self):
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

    def gripper_close_data(self):
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
        send_data += ' poseTo = pose_trans(tcpPose,p' + str(pos_delta) + ')\n'
        send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
        send_data += ' movel(jointPose,1,1)\n'
        return send_data

    def new_move_tool(self, posDelta, timeout_s=30, bSetJoint=False, bRetry=False,a=_tool_a,v=_tool_v):
        """
        以当前位置为原点，根据工具坐标系进行移动 Hui Zhi Fang
        :param posDelta: [x,y,z,rx,ry,rz]
        :param timeout_s: 默认30秒
        :param bSetJoint: 是否旋转关节
        :param bRetry: 是否失败重试
        :return:
        """

        # print("进入new_move_tool")
        # Write UR Script and send data
        if bSetJoint:
                a = self._rotate_a
                v = self._rotate_v
                # send_data = 'def mov():\n'
                # send_data += ' pos = get_actual_joint_positions()\n'
                # send_data += ' newPose = [pos[0]+' + str(posDelta[0]) + ',pos[1]+' + str(
                #     posDelta[1]) + ',pos[2]+' + str(posDelta[2]) + ',pos[3]+' + str(posDelta[3]) + ',pos[4]+' + str(
                #     posDelta[4]) + ',pos[5]+' + str(posDelta[5]) + ']\n'
                # send_data += ' movej(newPose,' + str(a) + ', ' + str(v) + ')\n'
                # send_data += 'end\n'
                pos_delta = 'p' + str(posDelta)

                send_data = ' tcpPose = get_actual_tcp_pose()\n'
                send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
                send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
                send_data += ' movej(jointPose,' + str(a) + ', ' + str(v) + ')\n'
                return send_data
        else:
                pos_delta = 'p' + str(posDelta)

                send_data = ' tcpPose = get_actual_tcp_pose()\n'
                send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
                send_data += ' jointPose = get_inverse_kin(poseTo)\n'  # joint
                send_data += ' movel(jointPose,' + str(a) + ', ' + str(v) + ')\n'

                return send_data

    def  new_tool_relative_move_from_rad_pose(self, poseFrom, posDelta, timeout_s=30, bRetry=False):
        rtnVal = -1

        # Check variable
        if len(posDelta) != 6 or len(poseFrom) != 6:
            return rtnVal

        pos_delta = 'p' + str(posDelta)

            #
            # Write UR Script and send data

        send_data = ' tcpPose = get_forward_kin(' + str(poseFrom) + ')\n'  # joint to tcp pose
        send_data += ' poseTo = pose_trans(tcpPose,' + pos_delta + ')\n'
        send_data += ' movej(poseTo,' + self._arm_a + ', ' + self._arm_v + ')\n'
        print(send_data)
            # Wait for running

        return send_data


