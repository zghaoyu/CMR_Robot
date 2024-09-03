import threading
import socket
import time
import inspect
import ctypes
import UtilsFunc as FuncObj
import LogHelper as LogObj
import ClassScanBarcode
from ClassScanBarcode import ScanBarcode
# from ClassGripper import GripperSMC
from ClassNewGripper import NewGripper
from ClassCamera import CameraLogitech, LightCtrlLCPW
from ClassAGV import AgvMIR
from ClassArm import ArmUR
from ClassPositionMarker import MarkerFixture
from ClassConfig import CobotCfg
from ClassRemoteIO import RemoteIO


# Singleton class
class Daemon(object):
    _instance_lock = threading.Lock()
    # _gripper_ctrl = GripperSMC()
    _new_gripper = NewGripper()
    _camera_ctrl = CameraLogitech()
    _agv_ctrl = AgvMIR()
    _ur_ctrl = ArmUR()
    _marker_ctrl = MarkerFixture()
    _light_ctrl = LightCtrlLCPW()
    _cobot_cfg = CobotCfg()
    _remote_io = RemoteIO()
    _home_dir = "E:/CobotHome/Daemon"
    _pnp_zdown_offset = (-1) * abs(_ur_ctrl.get_cal_z_height())  # Negative to move down
    _pnp_zdown_margin = 0.005
    _daemon_started = False
    _stop_daemon = False
    _daemon_ip = "Unknown"
    _daemon_thread = None
    _socket_server = None
    _curr_status_info_ptr = 10
    _status_info = {}
    _last_cmd_state = "None"
    _status = "Default"  # add by Hui Zhi 2022/6/23
    _barcode_Ctrl = ScanBarcode()
    def __init__(self, *args, **kwargs):
        with Daemon._instance_lock:
            pass

    def __del__(self):
        if not self._socket_server is None:
            self._socket_server.close()

    def __new__(cls, *args, **kwargs):
        with Daemon._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    Daemon._instance = super().__new__(cls)
            return Daemon._instance

    def start_daemon(self):
        if not self._daemon_started:
            self._daemon_thread = threading.Thread(target=self._socket_service,
                                                   args=[])  # Response to every connection by thread
            self._daemon_thread.start()
            startT = time.time()
            while (True):
                if self._daemon_started: break
                waitT = time.time() - startT
                if waitT > 5: break
        return self._daemon_started

    def stop_daemon(self):
        self._stop_daemon = True
        if not self._socket_server is None:
            self._async_raise(self._daemon_thread.ident, SystemExit)
            self._socket_server.close()

    def get_daemon_ip(self):
        return self._daemon_ip

    def get_status_info(self):
        status = ""
        if len(self._status_info) > 0:
            for k in self._status_info.keys():
                status = status + self._status_info[k] + "\n"
        return status

    def _dump_status_info(self, status_info):
        if self._curr_status_info_ptr <= 0:
            self._curr_status_info_ptr = 10
            self._status_info.clear()
        self._status_info[self._curr_status_info_ptr] = FuncObj.getCurrTimeStr("[%H:%M:%S] ") + status_info
        self._curr_status_info_ptr = self._curr_status_info_ptr - 1

    def _async_raise(self, tid, exctype):
        """Raises an exception in the threads with id tid"""
        if not inspect.isclass(exctype):
            raise TypeError("Only types can be raised (not instances)")
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def _socket_service(self):
        with Daemon._instance_lock:
            try:
                ip = FuncObj.get_host_ip()
                self._socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._socket_server.bind((ip, 6666))
                self._socket_server.listen(10)
                self._daemon_ip = ip + ":6666"
                self._dump_status_info(self._daemon_ip)
                self._clear_sys_log()
            except socket.error as msg:
                LogObj.logSystemError()
                self._dump_status_info('CMR Daemon fails to start')
                return

            self._daemon_started = True
            while (True):
                if self._stop_daemon: break
                conn, addr = self._socket_server.accept()
                t = threading.Thread(target=self._execute_cmd,
                                     args=(conn, addr))  # Response to every connection by thread
                t.daemon = 1
                t.start()
            self._daemon_started = False

    def _socket_send(self, conn, data):
        try:
            conn.send(data.encode())
            bOK = True
            self._dump_status_info('Send -> ' + data)
        except Exception as e:
            LogObj.logSystemError()
            self._dump_status_info('_socket_send error')
            bOK = False
        return bOK

    def _socket_recv(self, conn):
        try:
            data = (conn.recv(1024)).decode()
            bOK = True
            self._dump_status_info('Recv <- ' + data)
        except Exception as e:
            LogObj.logSystemError()
            self._dump_status_info('_socket_recv error')
            data = ""
            bOK = False
        return bOK, data

    def _socket_close(self, conn):
        try:
            conn.close()
        except Exception as e:
            pass
        self._dump_status_info('_socket_close')

    def _open_camera(self):
        if not self._camera_ctrl.cameraHasOpened():
            t = threading.Thread(target=self._open_cam, args=[])  # Response to every connection by thread
            t.start()

    def _open_cam(self):
        self._camera_ctrl.openCamera()

    def _close_camera(self):
        if self._camera_ctrl.cameraHasOpened():
            t = threading.Thread(target=self._close_cam, args=[])  # Response to every connection by thread
            t.start()

    def _close_cam(self):
        self._camera_ctrl.closeCamera()

    def _clear_sys_log(self):
        t = threading.Thread(target=self._clear_log, args=[])  # Response to every connection by thread
        t.start()

    def _clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)
        self._agv_ctrl.clear_log()
        self._ur_ctrl.clear_log()
        self._camera_ctrl.clear_log()
        # self._gripper_ctrl.clear_log()
        self._new_gripper.clear_log()
        self._dump_status_info('_clear_sys_log')

    def _need_to_adjust_pose(self, poseName, factorName):
        # Position Marker is set and gripper-adjust-data is empty
        bNeed = False
        poseFactorCfg = self._cobot_cfg.get_arm_pose_factor(poseName, factorName)
        mkrName = FuncObj.getDictVal(poseFactorCfg, "marker_name", "")
        if "" != mkrName:
            poseCorr = self._cobot_cfg.get_arm_pose_factor(poseName, "CorrectXYZ")
            if len(poseCorr) <= 0: bNeed = True
        return bNeed

    def _get_individual_io(self, poseName, factorName):
        io_chn = -1
        poseFactor = self._cobot_cfg.get_arm_pose_factor(poseName, factorName)
        if len(poseFactor) > 0:
            chn = FuncObj.getDictVal(poseFactor, "individual_lock", "")
            if "" != chn: io_chn = int(chn)
        return io_chn

    def _get_pose_relative_shift(self, poseName, factorName, shiftType):
        rtnMsg = ""
        xyz_shift = []
        # shiftTypes = ["xyz_shift_take_picture(m)", "xyz_shift_do_operation(m)", "xyz_shift_before_op(m)", "xyz_shift_after_op(m)", "xyz_shift_safe_leaving(m)"]
        poseFactor = self._cobot_cfg.get_arm_pose_factor(poseName, factorName)
        if len(poseFactor) > 0:
            xyz_shift_take_picture = FuncObj.getDictVal(poseFactor, shiftType, "")
            if "" != xyz_shift_take_picture:
                xyz_shift_settings = xyz_shift_take_picture.split(";")
                for shift in xyz_shift_settings:
                    xyz = shift.split(",")
                    # if 3 != len(xyz):
                    #     rtnMsg = "Wrong pose_relative_shift setting(" + poseName + "," + factorName + "," + shiftType + ")"
                    #     break
                    # else:
                    xyz_shift.append([float(val) for val in xyz])
            else:
                rtnMsg = "Get pose_relative_shift(" + poseName + "," + factorName + "," + shiftType + ") failed_01"
        else:
            rtnMsg = "Get pose_relative_shift(" + poseName + "," + factorName + "," + shiftType + ") failed_02"
        return rtnMsg, xyz_shift

    def _do_pose_relative_shift(self, poseName, factorName, shiftType):
        rtnMsg, xyz_shift = self._get_pose_relative_shift(poseName, factorName, shiftType)
        if "" == rtnMsg:
            for xyz in xyz_shift:
                if 0 != round(xyz[0], 6) or 0 != round(xyz[1], 6) or 0 != round(xyz[2], 6):
                    if 0 != self._ur_ctrl.relative_shift([xyz[0], xyz[1], xyz[2], 0, 0, 0]):
                        rtnMsg = "relative_shift(xyz) failed"
                        break
        return rtnMsg, xyz_shift

    # add by Hui Zhi 2022/9/6 (call move_tool())
    def _do_tool_relative_shift(self, poseName, factorName, shiftType, timeout_s=15):
        rtnMsg, shifts = self._get_pose_relative_shift(poseName, factorName, shiftType)
        if "" == rtnMsg:
            for pose_delta in shifts:
                if len(pose_delta) == 6:
                    if 0 != self._ur_ctrl.move_tool(pose_delta, timeout_s, False, False, 0.25, 1):
                        rtnMsg = "move tool:" + pose_delta + "failed"
                        break
                else:
                    rtnMsg = f"The {shiftType} setting for pose:{poseName} is wrong "
        return rtnMsg
    def new_do_tool_relative_shift(self, poseName, factorName, shiftType, timeout_s=15):
        rtnMsg, shifts = self._get_pose_relative_shift(poseName, factorName, shiftType)
        send_data = ""
        if "" == rtnMsg:

            for pose_delta in shifts:
                if len(pose_delta) == 6:
                    send_data += self._ur_ctrl.new_move_tool(pose_delta, timeout_s, False, False, 1, 2)

                else:
                    rtnMsg = f"The {shiftType} setting for pose:{poseName} is wrong "

        return rtnMsg,send_data

    def _calc_arm_corr_factor(self, poseName, factorName, bMoveArmHome=True):
        rtnMsg = ""
        if self._need_to_adjust_pose(poseName, factorName):
            if (not self._light_ctrl.light_on()): rtnMsg = "light_on failed"
            if "" == rtnMsg:
                currPoseName = self._ur_ctrl.get_curr_pose_name()
                if currPoseName != poseName:
                    poseCfg = self._cobot_cfg.get_arm_pose_cfg()
                    routing = FuncObj.getDictVal(poseCfg, currPoseName + "_routing", "")
                    if "" != routing:
                        if 0 != self._ur_ctrl.move_to_by_pose_name(
                            currPoseName + "_routing"): return "move_to(" + currPoseName + "_routing" + ") failed"
                    routing = FuncObj.getDictVal(poseCfg, poseName + "_routing", "")
                    if "" != routing:
                        if 0 != self._ur_ctrl.move_to_by_pose_name(
                            poseName + "_routing"): return "move_to(" + poseName + "_routing" + ") failed"
                    if 0 != self._ur_ctrl.move_to_by_pose_name(poseName): return "move_to(" + poseName + ") failed"

                rtnMsg, xyz_shift_take_picture = self._do_pose_relative_shift(poseName, factorName,
                                                                              "xyz_shift_take_picture(m)")
                time.sleep(2)  # Wait for arm stablized

                if "" == rtnMsg:
                    rtn, corPos, img = self._marker_ctrl.adjust_gripper_pose(60, 1.0, poseName, factorName)
                    self._light_ctrl.light_off()
                    if not rtn:
                        rtnMsg = "Adjust gripper pose at " + poseName + " failed"
                    else:
                        self._cobot_cfg.refresh_arm_pose_factor(poseName, "CorrectXYZ",
                                                                {"corr_x": corPos[0], "corr_y": corPos[1],
                                                                 "corr_rz": corPos[2]})
                        # disable by Hui Zhi for testing 2022/4/7
                        # shifts_qty = len(xyz_shift_take_picture)
                        # for i in range(shifts_qty):
                        #     xyz = xyz_shift_take_picture[shifts_qty-i-1]
                        #     if 0 != round(xyz[0], 6) or 0 != round(xyz[1], 6) or 0 != round(xyz[2], 6):
                        #         if 0 != self._ur_ctrl.relative_shift([(-1)*xyz[0], (-1)*xyz[1], (-1)*xyz[2], 0, 0, 0]):
                        #             rtnMsg = "relative_shift(xyz) failed"
                        #             break
                        # if "" == rtnMsg:
                        #     if bMoveArmHome: #Move arm home after getting the xyz compensation factors
                        #         if 0 != self._ur_ctrl.move_to_by_pose_name("arm_home"): rtnMsg = "move_to(arm_home) failed"
                        #     else: #Apply xyz compensation factors directly
                        #         if 0 != self._ur_ctrl.relative_shift([0, 0, 0, 0, 0, round(corPos[2] / 180 * 3.1415926, 7)], 5, True):
                        #             rtnMsg = "relative_shift(rz) failed"
                        #         else:
                        #             if 0 != self._ur_ctrl.relative_shift([corPos[0], corPos[1], 0, 0, 0, 0], 5, False):
                        #                 rtnMsg = "relative_shift(xy) failed"
        return rtnMsg

    def _pick_retry(self, pickFromPose=""):
        for i in range(3):
            rtnMsg = ""
            # if 0 != self._gripper_ctrl.gripper_init(): rtnMsg = "gripper_init failed"
            if 0 != self._new_gripper.reset(): rtnMsg = "gripper_reset failed"
            if "" != rtnMsg: return rtnMsg
            if 2 == i and "" != pickFromPose: break
            # if 0 != self._gripper_ctrl.gripper_close():
            if 0 != self._new_gripper.close():
                rtnMsg = "gripper_close failed"
            else:
                return rtnMsg

        if "" != pickFromPose:
            rtnMsg = ""
            self._cobot_cfg.refresh_arm_pose_factor(pickFromPose, "PickFactor", "CorrectXYZ", {})

            if 0 != self._ur_ctrl.relative_shift(
                [0, 0, (-1) * self._pnp_zdown_offset, 0, 0, 0]): rtnMsg = "Z axis move up failed"
            if "" != rtnMsg: return rtnMsg

            if not self._light_ctrl.light_on(): rtnMsg = "light_on failed"
            if "" != rtnMsg: return rtnMsg

            rtnMsg = self._calc_arm_corr_factor(pickFromPose, "PickFactor", False)
            if "" != rtnMsg: return rtnMsg

            if not self._light_ctrl.light_off(): rtnMsg = "light_off failed"
            if "" != rtnMsg: return rtnMsg

            if 0 != self._ur_ctrl.relative_shift(
                [0, 0, self._pnp_zdown_offset, 0, 0, 0]): rtnMsg = "Z axis move down failed"
            if "" != rtnMsg: return rtnMsg

            # if 0 != self._gripper_ctrl.gripper_close(): rtnMsg = "gripper_close failed"
            if 0 != self._new_gripper.close(): rtnMsg = "gripper_close failed"
            if "" != rtnMsg: return rtnMsg
        return rtnMsg

    def _pick_and_place(self, pickFromPose, placeToPose):
        rtnMsg = ""
        bLightOnOff = self._need_to_adjust_pose(pickFromPose, "PickFactor") or self._need_to_adjust_pose(placeToPose,
                                                                                                         "PlaceFactor")
        if bLightOnOff: self._open_camera()

        # Step 1 check home
        # currPoseName = self._ur_ctrl.get_curr_pose_name()

        # disable by Hui Zhi 2022/3/9
        # with cal and position at robot_slot
        # if currPoseName.startswith("robot") :
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_back_home"): rtnMsg = "arm_back_home failed"

        # if "" != rtnMsg:
        #     if bLightOnOff: self._close_camera()
        #     return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # if "arm_home" != currPoseName : #self._ur_ctrl.get_curr_pose_name():
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_home"): rtnMsg = "arm_home failed"

        # if "" != rtnMsg:
        #     if bLightOnOff: self._close_camera()
        #     return rtnMsg

        rtnMsg = self._calc_arm_corr_factor(pickFromPose, "PickFactor")
        if "" != rtnMsg:
            if bLightOnOff: self._close_camera()
            return rtnMsg

        rtnMsg = self._calc_arm_corr_factor(placeToPose, "PlaceFactor")
        if "" != rtnMsg:
            if bLightOnOff: self._close_camera()
            return rtnMsg
        if bLightOnOff: self._close_camera()

        currPoseName = self._ur_ctrl.get_curr_pose_name()
        poseCfg = self._cobot_cfg.get_arm_pose_cfg()
        routing = FuncObj.getDictVal(poseCfg, currPoseName + "_routing", "")
        if "" != routing:
            if 0 != self._ur_ctrl.move_to_by_pose_name(
                currPoseName + "_routing"): rtnMsg = "move_to(" + currPoseName + "_routing" + ") failed"
            if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # pick check pick poistion
        # if pickFromPose.startswith("robot"):
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_back_home"): rtnMsg = "arm_back_home failed"
        #     if "" != rtnMsg: return rtnMsg

        routing = FuncObj.getDictVal(poseCfg, pickFromPose + "_routing", "")
        if "" != routing:
            if 0 != self._ur_ctrl.move_to_by_pose_name(
                pickFromPose + "_routing"): rtnMsg = "move_to(" + pickFromPose + "_routing" + ") failed"
            if "" != rtnMsg: return rtnMsg

        if 0 != self._ur_ctrl.move_to_by_pose_name(pickFromPose): rtnMsg = "move_to(" + pickFromPose + ") failed"
        if "" != rtnMsg: return rtnMsg

        corrPos = self._cobot_cfg.get_arm_pose_factor(pickFromPose, "CorrectXYZ")
        if len(corrPos) > 0:
            if 0 != self._ur_ctrl.relative_shift([0, 0, 0, 0, 0, round(corrPos["corr_rz"] / 180 * 3.1415926, 7)], 5,
                                                 True):
                rtnMsg = "relative_shift(rz) failed"
            else:
                if 0 != self._ur_ctrl.relative_shift([corrPos["corr_x"], corrPos["corr_y"], 0, 0, 0, 0], 5, False):
                    rtnMsg = "relative_shift(xy) failed"
        if "" != rtnMsg: return rtnMsg

        # if 0 != self._gripper_ctrl.gripper_open(): rtnMsg = "gripper_open failed"
        if 0 != self._new_gripper.open(): rtnMsg = "gripper_open failed"
        if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(pickFromPose, "PickFactor", "xyz_shift_do_operation(m)")
        if "" != rtnMsg: return rtnMsg

        # if 0 != self._gripper_ctrl.gripper_close(): rtnMsg = "gripper_close failed"
        if 0 != self._new_gripper.close(): rtnMsg = "gripper_close failed"
        # if "" != rtnMsg: rtnMsg = self._pick_retry(pickFromPose)
        if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(pickFromPose, "PickFactor", "xyz_shift_after_op(m)")
        if "" != rtnMsg: return rtnMsg

        io_chn = self._get_individual_io(pickFromPose, "PickFactor")
        if io_chn >= 0:
            if not self._remote_io.set_output(False, io_chn): rtnMsg = "unlock fixture(" + str(io_chn) + ") failed"
            if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(pickFromPose, "PickFactor", "xyz_shift_safe_leaving(m)")
        if "" != rtnMsg: return rtnMsg

        routing = FuncObj.getDictVal(poseCfg, pickFromPose + "_routing", "")
        if "" != routing:
            if 0 != self._ur_ctrl.move_to_by_pose_name(
                pickFromPose + "_routing"): rtnMsg = "move_to(" + pickFromPose + "_routing" + ") failed"
            if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # pick goto home
        # if pickFromPose.startswith("robot"):
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_back_home"): rtnMsg = "arm_back_home failed"
        #     if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # if 0 != self._ur_ctrl.move_to_by_pose_name("arm_home"): rtnMsg = "arm_home failed"
        # if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # place check location
        # if placeToPose.startswith("robot") :
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_back_home"): rtnMsg = "arm_back_home failed"
        #     if "" != rtnMsg: return rtnMsg

        routing = FuncObj.getDictVal(poseCfg, placeToPose + "_routing", "")
        if "" != routing:
            if 0 != self._ur_ctrl.move_to_by_pose_name(
                placeToPose + "_routing"): rtnMsg = "move_to(" + placeToPose + "_routing" + ") failed"
            if "" != rtnMsg: return rtnMsg

        if 0 != self._ur_ctrl.move_to_by_pose_name(placeToPose): rtnMsg = "move_to(" + placeToPose + ") failed"
        if "" != rtnMsg: return rtnMsg

        corrPos = self._cobot_cfg.get_arm_pose_factor(placeToPose, "CorrectXYZ")
        if len(corrPos) > 0:
            if 0 != self._ur_ctrl.relative_shift([0, 0, 0, 0, 0, round(corrPos["corr_rz"] / 180 * 3.1415926, 7)], 5,
                                                 True):
                rtnMsg = "relative_shift(rz) failed"
            else:
                if 0 != self._ur_ctrl.relative_shift([corrPos["corr_x"], corrPos["corr_y"], 0, 0, 0, 0], 5, False):
                    rtnMsg = "relative_shift(xy) failed"
        if "" != rtnMsg: return rtnMsg

        io_chn = self._get_individual_io(placeToPose, "PlaceFactor")
        if io_chn >= 0:
            if not self._remote_io.set_output(False, io_chn): rtnMsg = "unlock base(" + str(io_chn) + ") failed"
            if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(placeToPose, "PlaceFactor", "xyz_shift_do_operation(m)")
        if "" != rtnMsg: return rtnMsg

        io_chn = self._get_individual_io(placeToPose, "PlaceFactor")
        if io_chn >= 0:
            if not self._remote_io.set_output(True, io_chn): rtnMsg = "lock fixture(" + str(io_chn) + ") failed"
            if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(placeToPose, "PlaceFactor", "xyz_shift_before_op(m)")
        if "" != rtnMsg: return rtnMsg

        # if 0 != self._gripper_ctrl.gripper_open(): rtnMsg = "gripper_open failed"
        if 0 != self._new_gripper.open(): rtnMsg = "gripper_open failed"
        if "" != rtnMsg: return rtnMsg

        rtnMsg, xyz_shift = self._do_pose_relative_shift(placeToPose, "PlaceFactor", "xyz_shift_safe_leaving(m)")
        if "" != rtnMsg: return rtnMsg

        routing = FuncObj.getDictVal(poseCfg, placeToPose + "_routing", "")
        if "" != routing:
            if 0 != self._ur_ctrl.move_to_by_pose_name(
                placeToPose + "_routing"): rtnMsg = "move_to(" + placeToPose + "_routing" + ") failed"
            if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # Check finished
        # if placeToPose.startswith("robot") :
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("arm_back_home"): rtnMsg = "arm_back_home failed"
        #     if "" != rtnMsg: return rtnMsg

        # disable by Hui Zhi 2022/3/9
        # if 0 != self._ur_ctrl.move_to_by_pose_name("arm_home"): rtnMsg = "arm_home failed"

        return rtnMsg

    def new_adjust_correct_marker_pose(self, marker_pose_name, factor_name):
        adjust_ok = False

        if not self._need_to_adjust_pose(marker_pose_name, factor_name):
            adjust_ok = True
            return adjust_ok,''

        config_cal = self._cobot_cfg.get_arm_pose_factor(marker_pose_name, factor_name)
        light_brightness = int(FuncObj.getDictVal(config_cal, "light_brightness", 80))


        # Move Arm to home
        move_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")
        if move_back_home is None:
            return adjust_ok,''

        move_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_home")
        if move_home is None:
            return adjust_ok,''

        move_marker = self._ur_ctrl.new_move_to_by_pose_name(marker_pose_name)
        if move_marker is None:
            return adjust_ok,''
        light = self._light_ctrl.new_light_adjust(light_brightness)
        if light is None:
            return adjust_ok
        send_data = "def fun():\n"+move_back_home+move_home+move_marker+light+"end\n"
        # print(send_data)
        self._ur_ctrl.send_command(send_data)
        self._open_camera()
        real_marker = ''

        adjust_ok, adjust_time, draw_img = self._marker_ctrl.adjust_marker_pose(marker_pose_name, factor_name, 30)
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       marker_pose_name + "@adjust_marker_time@" + str(adjust_time))
        if adjust_ok:
            # record current pose
            time.sleep(0.5)
            joint_pose = self._ur_ctrl.get_actual_joint_position()  # rad
            if len(joint_pose) == 6:
                self._cobot_cfg.refresh_arm_pose_factor(marker_pose_name, "CorrectXYZ",
                                                        {"correct_marker_pose(rad)": joint_pose})
                LogObj.logInfo("E:/CobotHome/ControllerArm", "arm_pose",
                               marker_pose_name + "@actual_joint_pose@" + str(joint_pose))
            else:
                adjust_ok = False
                LogObj.logInfo("E:/CobotHome/ControllerArm", "arm_pose",
                               marker_pose_name + "@actual_joint_pose@get actual joint pose failed")
        self._close_camera()
        self._light_ctrl.light_off()


        #调整相机位记录下来,此时的手臂在marker调整好的位置
        return adjust_ok

    #最初版adjust_correct_marker_pose
    def adjust_correct_marker_pose(self, marker_pose_name, factor_name):
        adjust_ok = False

        if not self._need_to_adjust_pose(marker_pose_name, factor_name):
            adjust_ok = True
            return adjust_ok

        config_cal = self._cobot_cfg.get_arm_pose_factor(marker_pose_name, factor_name)
        light_brightness = int(FuncObj.getDictVal(config_cal, "light_brightness", 100))

        self._open_camera()

        # Move Arm to home
        if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_back_home"):
            return adjust_ok

        if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
            return adjust_ok

        if 0 != self._ur_ctrl.move_to_by_pose_name(marker_pose_name):
            return adjust_ok

        if not self._light_ctrl.light_adjust(light_brightness):
            return adjust_ok

        adjust_ok, adjust_time, draw_img = self._marker_ctrl.adjust_marker_pose(marker_pose_name, factor_name, 30)
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       marker_pose_name + "@adjust_marker_time@" + str(adjust_time))
        if adjust_ok:
            # record current pose
            time.sleep(0.5)
            joint_pose = self._ur_ctrl.get_actual_joint_position()  # rad
            if len(joint_pose) == 6:
                self._cobot_cfg.refresh_arm_pose_factor(marker_pose_name, "CorrectXYZ",
                                                        {"correct_marker_pose(rad)": joint_pose})
                LogObj.logInfo("E:/CobotHome/ControllerArm", "arm_pose",
                               marker_pose_name + "@actual_joint_pose@" + str(joint_pose))
            else:
                adjust_ok = False
                LogObj.logInfo("E:/CobotHome/ControllerArm", "arm_pose",
                               marker_pose_name + "@actual_joint_pose@get actual joint pose failed")
        self._close_camera()
        self._light_ctrl.light_off()

        if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
            return adjust_ok

        return adjust_ok

        def pick_from_name_(self, pick_from_pose_name):
            send_data = "def pick():\n"
            pick_config = self._cobot_cfg.get_arm_pose_factor(pick_from_pose_name, "PickFactor")

            if len(pick_config) < 0:
                message = f"{pick_from_pose_name}configuration is empty."
                return message

            # marker name
            pick_marker_pose_name = FuncObj.getDictVal(pick_config, "marker_name", "")

            # if not self.new_adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
            #     message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
            #     return message
            # if not self.new_adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
            #     message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
            #     return message

            if not self.adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
                message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
                return message

            move_new_arm_home = self._ur_ctrl.new_move_to_by_pose_name(
                "new_arm_home")  # new_move_to_by_pose_name返回的是ur script字符串
            move_new_arm_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")
            move_pick_pose = self._ur_ctrl.new_move_to_by_pose_name(pick_from_pose_name)

            if move_new_arm_home is None:
                message = "arm move to home failed"
                return message
            if move_new_arm_back_home is None:
                message = "arm move to back home failed"
                return message
            if move_pick_pose is None:
                message = "arm move to " + pick_from_pose_name + " failed"
                return message

            # if not pick_from_pose_name.startswith('robot'):
            #     send_data += move_new_arm_home              #调整完marker后回home位
            start_time = time.time()
            # 执行动作
            # pick
            if pick_from_pose_name.startswith('robot'):  # TODO 配置一个flag    pickFromRobot

                send_data += move_new_arm_back_home
                # 移动到机器人的托盘抓取夹具

                send_data += move_pick_pose

                send_data += self._ur_ctrl.gripper_open_data()

                message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "xyz_shift_do_operation(m)")

                if "" != message:
                    return message
                send_data += do_relative_shift

                send_data += self._ur_ctrl.gripper_close_data()

                message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "xyz_shift_safe_leaving(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift
                send_data += move_new_arm_back_home

                # input Rack station and cnc                pick from cnc or rack
            else:

                # # TODO 优化调用方法
                send_data += move_new_arm_home

                message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "relative_marker")
                if "" != message:
                    return message

                # move to maker 2023/6/2
                # if 0 != self._ur_ctrl.move_to_by_pose_name(pick_marker_pose_name):
                #     message = "arm move to " + pick_from_pose_name + " failed"
                #     return message

                if len(relative_marker_pos) == 1:
                    correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name,
                                                                                     "CorrectXYZ")
                    # print("CorrectXYZ:" + correct_marker_pose_factor)
                    correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                    # print("correct_marker_pose(rad):" + correct_marker_pose)
                    if correct_marker_pose == '':
                        message = pick_from_pose_name + ":correct_marker_pose is empty"
                        return message
                    send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                    relative_marker_pos[0])

                send_data += self._ur_ctrl.gripper_open_data()

                message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "xyz_shift_do_operation(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift

                send_data += self._ur_ctrl.gripper_close_data()  # 加紧

                message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "xyz_shift_safe_leaving(m)")  # 离开动作
                if "" != message:
                    return message
                send_data += do_relative_shift
                # move to home
                send_data += move_new_arm_home
            send_data += "end \n"
            print("-----------------------------------------------------------\n" + send_data)
            rtnVal = self._ur_ctrl.send_command(send_data)
            if (rtnVal != 0):
                message = "URrobot action failed"
            total_time = round(time.time() - start_time, 2)
            LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                           pick_from_pose_name + "@" + str(total_time))
            return message

        # --------------------------------------------------------------------------------------------------------------------------------

        def place_to_name_(self, place_to_pose_name):
            send_data = "def Place():\n"
            place_config = self._cobot_cfg.get_arm_pose_factor(place_to_pose_name, "PlaceFactor")
            if len(place_config) < 0:
                message = f"{place_to_pose_name} configuration is empty."
                return message

            # marker name

            place_marker_pose_name = FuncObj.getDictVal(place_config, "marker_name", "")
            # if not self.new_adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
            #     message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
            #     return message
            # if not self.new_adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
            #     message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
            #     return message

            if not self.adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
                message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
                return message

            move_new_arm_home = self._ur_ctrl.new_move_to_by_pose_name(
                "new_arm_home")  # new_move_to_by_pose_name返回的是ur script字符串
            move_new_arm_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")

            move_place_pose = self._ur_ctrl.new_move_to_by_pose_name(place_to_pose_name)
            if move_new_arm_home is None:
                message = "arm move to home failed"
                return message
            if move_new_arm_back_home is None:
                message = "arm move to back home failed"
                return message

            if place_to_pose_name is None:
                message = "arm move to " + place_to_pose_name + " failed"
                return message
            # if not pick_from_pose_name.startswith('robot'):
            #     send_data += move_new_arm_home              #调整完marker后回home位
            start_time = time.time()
            # 执行动作
            if place_to_pose_name.startswith('robot'):

                # move to back home 2023/6/2
                send_data += move_new_arm_back_home
                send_data += move_place_pose

                # robot这里继续使用 relative shift 做相对移动，因为调试时使用了base坐标系
                # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                #                                                   "xyz_shift_do_operation(m)")
                message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                             "xyz_shift_do_operation(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift

                send_data += self._ur_ctrl.gripper_open_data()
                # if 0 != self._gripper_ctrl.gripper_open():

                # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                #                                                   "xyz_shift_safe_leaving(m)")
                message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                             "xyz_shift_safe_leaving(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift
                # move to back home 2023/6/2
                send_data += move_new_arm_back_home

                if "" != message:
                    return message

            # input Rack station and cnc                  place to cnc or rack
            else:

                # remove 2023/6/2
                # if pick_from_pose_name.startswith('robot'):
                #     if 0 != self._ur_ctrl.move_to_by_pose_name("temp_pose"):
                #         message = "arm move to temp pose failed"
                #         return message

                # move home 2023/6/2
                send_data += move_new_arm_home

                # move to maker 2023/6/2

                message, relative_marker_pos = self._get_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                             "relative_marker")
                if "" != message:
                    return message
                if len(relative_marker_pos) == 1:
                    correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(place_marker_pose_name,
                                                                                     "CorrectXYZ")
                    correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                    if correct_marker_pose == '':
                        message = place_to_pose_name + ":correct_marker_pose is empty"
                        return message
                    send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                    relative_marker_pos[0])

                message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                             "xyz_shift_do_operation(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift
                # if 0 != self._gripper_ctrl.gripper_open():
                send_data += self._ur_ctrl.gripper_open_data()
                # error twic time
                message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                             "xyz_shift_safe_leaving(m)")
                if "" != message:
                    return message
                send_data += do_relative_shift
                # move home 2023/6/2
                send_data += move_new_arm_home
            send_data += "end \n"
            # print("-----------------------------------------------------------\n" + send_data)
            rtnVal = self._ur_ctrl.send_command(send_data)
            if (rtnVal != 0):
                message = "URrobot action failed"
            total_time = round(time.time() - start_time, 2)
            LogObj.logInfo("E:/CobotHome/TestData", "place_testing",
                           place_to_pose_name + "@" + str(total_time))
            return message
    def pick_from_name_(self,pick_from_pose_name):
        send_data = "def pick():\n"
        pick_config = self._cobot_cfg.get_arm_pose_factor(pick_from_pose_name, "PickFactor")

        if len(pick_config) < 0 :
            message = f"{pick_from_pose_name}configuration is empty."
            return message

        # marker name
        pick_marker_pose_name = FuncObj.getDictVal(pick_config, "marker_name", "")

        # if not self.new_adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
        #     message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
        #     return message
        # if not self.new_adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
        #     message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
        #     return message

        if not self.adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
            return message

        move_new_arm_home = self._ur_ctrl.new_move_to_by_pose_name(
            "new_arm_home")  # new_move_to_by_pose_name返回的是ur script字符串
        move_new_arm_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")
        move_pick_pose = self._ur_ctrl.new_move_to_by_pose_name(pick_from_pose_name)

        if move_new_arm_home is None:
            message = "arm move to home failed"
            return message
        if move_new_arm_back_home is None:
            message = "arm move to back home failed"
            return message
        if move_pick_pose is None:
            message = "arm move to " + pick_from_pose_name + " failed"
            return message

        # if not pick_from_pose_name.startswith('robot'):
        #     send_data += move_new_arm_home              #调整完marker后回home位
        start_time = time.time()
        # 执行动作
        # pick
        if pick_from_pose_name.startswith('robot'):  # TODO 配置一个flag    pickFromRobot

            send_data += move_new_arm_back_home
            # 移动到机器人的托盘抓取夹具

            send_data += move_pick_pose

            send_data += self._ur_ctrl.gripper_open_data()

            message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "xyz_shift_do_operation(m)")

            if "" != message:
                return message
            send_data += do_relative_shift

            send_data += self._ur_ctrl.gripper_close_data()

            message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            send_data += move_new_arm_back_home

            # input Rack station and cnc                pick from cnc or rack
        else:

            # # TODO 优化调用方法
            send_data += move_new_arm_home

            message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "relative_marker")
            if "" != message:
                return message

            # move to maker 2023/6/2
            # if 0 != self._ur_ctrl.move_to_by_pose_name(pick_marker_pose_name):
            #     message = "arm move to " + pick_from_pose_name + " failed"
            #     return message

            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name, "CorrectXYZ")

                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]

                if correct_marker_pose == '':
                    message = pick_from_pose_name + ":correct_marker_pose is empty"
                    return message
                send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                relative_marker_pos[0])

            send_data += self._ur_ctrl.gripper_open_data()

            message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift

            send_data += self._ur_ctrl.gripper_close_data()  # 加紧

            message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "xyz_shift_safe_leaving(m)")  # 离开动作
            if "" != message:
                return message
            send_data += do_relative_shift
            # move to home
            send_data += move_new_arm_home
        send_data += "end \n"
        # print("-----------------------------------------------------------\n" + send_data)
        rtnVal = self._ur_ctrl.send_command(send_data)
        if (rtnVal != 0):
            message = "URrobot action failed"
        total_time = round(time.time() - start_time, 2)
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       pick_from_pose_name + "@" + str(total_time))
        return message

    def place_to_name_(self, place_to_pose_name):
        send_data = "def Place():\n"
        place_config = self._cobot_cfg.get_arm_pose_factor(place_to_pose_name, "PlaceFactor")
        if len(place_config) < 0:
            message = f"{place_to_pose_name} configuration is empty."
            return message

        # marker name

        place_marker_pose_name = FuncObj.getDictVal(place_config, "marker_name", "")

        if not self.adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
            return message

        move_new_arm_home = self._ur_ctrl.new_move_to_by_pose_name(
            "new_arm_home")  # new_move_to_by_pose_name返回的是ur script字符串
        move_new_arm_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")

        move_place_pose = self._ur_ctrl.new_move_to_by_pose_name(place_to_pose_name)
        if move_new_arm_home is None:
            message = "arm move to home failed"
            return message
        if move_new_arm_back_home is None:
            message = "arm move to back home failed"
            return message

        if place_to_pose_name is None:
            message = "arm move to " + place_to_pose_name + " failed"
            return message
        # if not pick_from_pose_name.startswith('robot'):
        #     send_data += move_new_arm_home              #调整完marker后回home位
        start_time = time.time()
        # 执行动作
        if place_to_pose_name.startswith('robot'):

            # move to back home
            send_data += move_new_arm_back_home
            send_data += move_place_pose

            message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift

            send_data += self._ur_ctrl.gripper_open_data()

            message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            # move to back home
            send_data += move_new_arm_back_home

            if "" != message:
                return message

        # input Rack station and cnc                  place to cnc or rack
        else:

            send_data += move_new_arm_home

            # move to maker

            message, relative_marker_pos = self._get_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "relative_marker")
            if "" != message:
                return message
            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(place_marker_pose_name,
                                                                                 "CorrectXYZ")
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                if correct_marker_pose == '':
                    message = place_to_pose_name + ":correct_marker_pose is empty"
                    return message
                send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                relative_marker_pos[0])

            message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            # if 0 != self._gripper_ctrl.gripper_open():
            send_data += self._ur_ctrl.gripper_open_data()
            # error twic time
            message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            # move home
            send_data += move_new_arm_home
        send_data += "end \n"
        print("-----------------------------------------------------------\n" + send_data)
        rtnVal = self._ur_ctrl.send_command(send_data)
        if (rtnVal != 0):
            message = "URrobot action failed"
        total_time = round(time.time() - start_time, 2)
        LogObj.logInfo("E:/CobotHome/TestData", "place_testing",
                       place_to_pose_name + "@" + str(total_time))
        return message
    #---------------------------------------------------------------------------
    # once and speed up
    def new_pick_and_place_(self, pick_from_pose_name, place_to_pose_name):
        send_data="def pickAndPlace():\n"
        pick_config = self._cobot_cfg.get_arm_pose_factor(pick_from_pose_name, "PickFactor")
        place_config = self._cobot_cfg.get_arm_pose_factor(place_to_pose_name, "PlaceFactor")
        if len(pick_config) < 0 or len(place_config) < 0:
            message = f"{pick_from_pose_name} or {place_to_pose_name} configuration is empty."
            return message

        # marker name
        pick_marker_pose_name = FuncObj.getDictVal(pick_config, "marker_name", "")
        place_marker_pose_name = FuncObj.getDictVal(place_config, "marker_name", "")
        # if not self.new_adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
        #     message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
        #     return message
        # if not self.new_adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
        #     message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
        #     return message

        if not self.adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
            return message

        if not self.adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
            return message

        move_new_arm_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_home")     #new_move_to_by_pose_name返回的是ur script字符串
        move_new_arm_back_home = self._ur_ctrl.new_move_to_by_pose_name("new_arm_back_home")
        move_pick_pose = self._ur_ctrl.new_move_to_by_pose_name(pick_from_pose_name)
        move_place_pose = self._ur_ctrl.new_move_to_by_pose_name(place_to_pose_name)
        if move_new_arm_home is None:
            message = "arm move to home failed"
            return message
        if move_new_arm_back_home is None:
            message = "arm move to back home failed"
            return message
        if move_pick_pose is None:
            message = "arm move to " + pick_from_pose_name + " failed"
            return message
        if place_to_pose_name is None:
            message = "arm move to " + place_to_pose_name + " failed"
            return message
        # if not pick_from_pose_name.startswith('robot'):
        #     send_data += move_new_arm_home              #调整完marker后回home位
        # add camera detect action

        barcodeData = "-1"
        detect_data = "def detect():\n"
        # # 五面加工摆放检测-----------------------------------
        # detect_pick_data = "def detectPick():\n"
        # if pick_from_pose_name.startswith('input'):
        #     message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
        #                                                                  "relative_marker")
        #     if "" != message:
        #         return message
        #     if len(relative_marker_pos) == 1:
        #         correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name,
        #                                                                          "CorrectXYZ")
        #         correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
        #         if correct_marker_pose == '':
        #             message = pick_from_pose_name + ":correct_marker_pose is empty"
        #             return message
        #         detect_pick_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
        #                                                                           relative_marker_pos[0])
        #
        #     message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
        #                                                                  "xyz_shift_before_op(m)")
        #     if "" != message:
        #         return message
        #     detect_pick_data += do_relative_shift
        #     detect_pick_data += "end \n"
        #     rtnVal = self._ur_ctrl.send_command(detect_pick_data)
        #     if (rtnVal != 0):
        #         message = "URrobot action failed"
        #         return message
        #     result, barcodeData = self._barcode_Ctrl.detect(50, 20)
        #
        #     if (result is False):
        #         self._ur_ctrl.move_to_by_pose_name("new_arm_home")
        #         message = "detect nothing please detect again"
        #         return message
        #     elif('A' not in barcodeData):
        #         self._ur_ctrl.move_to_by_pose_name("new_arm_home")
        #         message = "please put the fixture to correct position"
        #         return message
        # # --------------------------------------------------
        if place_to_pose_name.startswith('outputUnable'):
            # detect_data += move_new_arm_back_home
            message, relative_marker_pos = self._get_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "relative_marker")
            if "" != message:
                return message
            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(place_marker_pose_name,
                                                                                 "CorrectXYZ")
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                if correct_marker_pose == '':
                    message = place_to_pose_name + ":correct_marker_pose is empty"
                    return message
                detect_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                relative_marker_pos[0])

            message, do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "xyz_shift_before_op(m)")
            if "" != message:
                return message
            detect_data += do_relative_shift
            detect_data += "end \n"
            rtnVal = self._ur_ctrl.send_command(detect_data)
            if (rtnVal != 0):
                message = "URrobot action failed"
                return message
            result, barcodeData = self._barcode_Ctrl.detect(70,5)
            # result equal true mean that have fixture below the camera
            if (result):
                self._ur_ctrl.move_to_by_pose_name("new_arm_home")
                message = "place position have fixture"
                return message
            if pick_from_pose_name.startswith('robot'):
                self._ur_ctrl.move_to_by_pose_name("new_arm_home")

        if place_to_pose_name.startswith('robotUnable'):
            detect_data += move_new_arm_back_home
            detect_data += move_place_pose
            #make the fixture align the camera
            detect_data += self._ur_ctrl.new_move_tool([0.025, 0, 0, 0, 0, 0], 30, False, False, 1, 5)
            detect_data += "end \n"
            rtnVal = self._ur_ctrl.send_command(detect_data)
            if (rtnVal != 0):
                message = "URrobot action failed"
                return message
            result,barcodeData = self._barcode_Ctrl.detect(70)
            #result equal true mean that have fixture below the camera
            if(result):
                self._ur_ctrl.move_to_by_pose_name("new_arm_back_home")
                message = "place position have fixture"
                return message
            if pick_from_pose_name.startswith('input'):
                self._ur_ctrl.move_to_by_pose_name("new_arm_back_home")





        start_time = time.time()
        # 执行动作
        # pick
        if pick_from_pose_name.startswith('robot'):  # TODO 配置一个flag    pickFromRobot


            send_data += move_new_arm_back_home
            # 移动到机器人的托盘抓取夹具

            send_data += move_pick_pose



            send_data += self._ur_ctrl.gripper_open_data()


            message,do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                   "xyz_shift_do_operation(m)")

            if "" != message:
                return message
            send_data +=do_relative_shift

            send_data += self._ur_ctrl.gripper_close_data()

            message,do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                   "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data +=do_relative_shift
            send_data += move_new_arm_back_home

            # input Rack station and cnc                pick from cnc or rack
        else:
            # send_data += move_new_arm_home
            # # TODO 优化调用方法
            self._ur_ctrl.move_to_by_pose_name("new_arm_home")
            # 五面加工摆放检测-----------------------------------
            detect_pick_data = "def detectPick():\n"
            if pick_from_pose_name.startswith('input'):
                message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "relative_marker")
                if "" != message:
                    return message
                if len(relative_marker_pos) == 1:
                    correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name,
                                                                                     "CorrectXYZ")
                    correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                    if correct_marker_pose == '':
                        message = pick_from_pose_name + ":correct_marker_pose is empty"
                        return message
                    detect_pick_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose,
                                                                                           relative_marker_pos[0])

                message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                                             "xyz_shift_before_op(m)")
                if "" != message:
                    return message
                detect_pick_data += do_relative_shift
                detect_pick_data += "end \n"
                rtnVal = self._ur_ctrl.send_command(detect_pick_data)
                if (rtnVal != 0):
                    message = "URrobot action failed"
                    return message
                result, barcodeData = self._barcode_Ctrl.detect(30, 5)

                if (result is False):
                    message = ""
                    # self._ur_ctrl.move_to_by_pose_name("new_arm_home")
                    # message = "detect nothing please detect again"
                    # return message
                elif ('A' not in barcodeData):
                    self._ur_ctrl.move_to_by_pose_name("new_arm_home")
                    message = "please put the fixture to correct position"
                    return message
            # --------------------------------------------------


            message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "relative_marker")
            if "" != message:
                return message

            # move to maker 2023/6/2
            # if 0 != self._ur_ctrl.move_to_by_pose_name(pick_marker_pose_name):
            #     message = "arm move to " + pick_from_pose_name + " failed"
            #     return message

            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name, "CorrectXYZ")
                # print("CorrectXYZ:" + correct_marker_pose_factor)
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                # print("correct_marker_pose(rad):" + correct_marker_pose)
                if correct_marker_pose == '':
                    message = pick_from_pose_name + ":correct_marker_pose is empty"
                    return message
                send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose, relative_marker_pos[0])

            send_data += self._ur_ctrl.gripper_open_data()

            message, do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor","xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift

            send_data += self._ur_ctrl.gripper_close_data() #加紧

            message,do_relative_shift = self.new_do_tool_relative_shift(pick_from_pose_name, "PickFactor","xyz_shift_safe_leaving(m)") #离开动作
            if "" != message:
                return message
            send_data += do_relative_shift
            # move to home
            send_data += move_new_arm_home

        #place--------------------------------------------------------------------------------------------------------------------------------------------------

        if place_to_pose_name.startswith('robot'):

            # move to back home 2023/6/2
            send_data += move_new_arm_back_home
            send_data += move_place_pose


                # robot这里继续使用 relative shift 做相对移动，因为调试时使用了base坐标系
                # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                #                                                   "xyz_shift_do_operation(m)")
            message,do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift

            send_data += self._ur_ctrl.gripper_open_data()
            # if 0 != self._gripper_ctrl.gripper_open():

                # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                #                                                   "xyz_shift_safe_leaving(m)")
            message,do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
                # move to back home 2023/6/2
            send_data += move_new_arm_back_home

            if "" != message:
                return message

        # input Rack station and cnc                  place to cnc or rack
        else:

            # remove 2023/6/2
            # if pick_from_pose_name.startswith('robot'):
            #     if 0 != self._ur_ctrl.move_to_by_pose_name("temp_pose"):
            #         message = "arm move to temp pose failed"
            #         return message

            # move home 2023/6/2
            send_data += move_new_arm_home

            # move to maker 2023/6/2


            message, relative_marker_pos = self._get_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "relative_marker")
            if "" != message:
                return message
            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(place_marker_pose_name,
                                                                                 "CorrectXYZ")
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                if correct_marker_pose == '':
                    message = place_to_pose_name + ":correct_marker_pose is empty"
                    return message
                send_data += self._ur_ctrl.new_tool_relative_move_from_rad_pose(correct_marker_pose, relative_marker_pos[0])

            message,do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_do_operation(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            # if 0 != self._gripper_ctrl.gripper_open():
            send_data += self._ur_ctrl.gripper_open_data()
            # error twic time
            message,do_relative_shift = self.new_do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message
            send_data += do_relative_shift
            # move home 2023/6/2
            send_data += move_new_arm_home
        send_data += "end \n"
        print("-----------------------------------------------------------\n"+send_data)
        rtnVal = self._ur_ctrl.send_command(send_data)
        if(rtnVal != 0):
            message = "URrobot action failed"
        total_time = round(time.time() - start_time, 2)
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       pick_from_pose_name + "@" + place_to_pose_name + "@" + str(total_time))

        return message,barcodeData







    # add by Hui Zhi 2022/9/6
    # Modify Paris with 4 Axis (0,90,180 and 270) with 4 times to pickup with each layer 2023/05/31
    def pick_and_place_(self, pick_from_pose_name, place_to_pose_name):

        message = ''

        # 获取配置
        pick_config = self._cobot_cfg.get_arm_pose_factor(pick_from_pose_name, "PickFactor")
        place_config = self._cobot_cfg.get_arm_pose_factor(place_to_pose_name, "PlaceFactor")
        if len(pick_config) < 0 or len(place_config) < 0:
            message = f"{pick_from_pose_name} or {place_to_pose_name} configuration is empty."
            return message

        # marker name
        pick_marker_pose_name = FuncObj.getDictVal(pick_config, "marker_name", "")
        place_marker_pose_name = FuncObj.getDictVal(place_config, "marker_name", "")

        # 调整位姿

        # Move to back home 2023/6/2
        # if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
        #     message = "arm move to home failed"
        #     return message

        if not self.adjust_correct_marker_pose(pick_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + pick_marker_pose_name + ") failed!"
            return message

        if not self.adjust_correct_marker_pose(place_marker_pose_name, "PickAndPlaceFactor"):
            message = "adjust marker pose(" + place_marker_pose_name + ") failed!"
            return message
        # else:
        #     if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
        #         message = "arm move home failed"
        #         return message

        start_time = time.time()
        # 执行动作
        # pick
        if pick_from_pose_name.startswith('robot'):  # TODO 配置一个flag    pickFromRobot

            # Move to back home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_back_home"):
                message = "arm move to back home failed"
                return message

            # 移动到机器人的托盘抓取夹具
            if 0 != self._ur_ctrl.move_to_by_pose_name(pick_from_pose_name):
                message = "arm move to " + pick_from_pose_name + " failed"
                return message
            # if not self._new_gripper.is_GripperReset:
            #     if 0 != self._new_gripper.reset():
            #         message = "gripper_reset failed"
            #         return message
            # if not self._new_gripper.is_GripperOpen:

            if 0 != self._new_gripper.open():
                message = "gripper_open failed"
                return message

            # robot这里继续使用 relative shift 做相对移动，因为调试时使用了base坐标系
            # 移动至夹位
            # message, xyz_shift = self._do_pose_relative_shift(pick_from_pose_name, "PickFactor",
            #                                                   "xyz_shift_do_operation(m)")
            message = self._do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                              "xyz_shift_do_operation(m)")

            if "" != message:
                return message
            # if 0 != self._gripper_ctrl.gripper_close():
            if 0 != self._new_gripper.close():
                message = "gripper_close failed"
                return message
            # 抓取至安全位置
            # message, xyz_shift = self._do_pose_relative_shift(pick_from_pose_name, "PickFactor",
            #                                                   "xyz_shift_safe_leaving(m)")
            message = self._do_tool_relative_shift(pick_from_pose_name, "PickFactor",
                                                              "xyz_shift_safe_leaving(m)")

            if "" != message:
                return message

            # Move to back home
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_back_home"):
                message = "arm move to " + pick_from_pose_name + " failed"
                return message

        # input Rack station and cnc                pick from cnc or rack
        else:
            # if self._ur_ctrl.get_curr_pose_name().startswith("robot"):
            #     # TODO need to read from config Hui Zhi 2022/9/6
            #     if 0 != self._ur_ctrl.move_to_by_pose_name("temp_pose"):
            #         message = "arm move to temp pose failed"
            #         return message
            # # TODO 优化调用方法

            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
                message = "arm move to home failed"
                return message

            message, relative_marker_pos = self._get_pose_relative_shift(pick_from_pose_name, "PickFactor",
                                                                         "relative_marker")
            if "" != message:
                return message

            # move to maker 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name(pick_marker_pose_name):
                message = "arm move to " + pick_from_pose_name + " failed"
                return message

            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(pick_marker_pose_name, "CorrectXYZ")
                print("CorrectXYZ:"+correct_marker_pose_factor)
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                print("correct_marker_pose(rad):"+correct_marker_pose)
                if correct_marker_pose == '':
                    message = pick_from_pose_name + ":correct_marker_pose is empty"
                    return message
                if 0 != self._ur_ctrl.tool_relative_move_from_rad_pose(correct_marker_pose, relative_marker_pos[0]):
                    message = "move to " + pick_from_pose_name + " failed"
                    return message

            if 0 != self._new_gripper.open():
                message = "gripper_open failed"
                return message

            message = self._do_tool_relative_shift(pick_from_pose_name, "PickFactor", "xyz_shift_do_operation(m)")
            if "" != message:
                return message

            # if 0 != self._gripper_ctrl.gripper_close():
            if 0 != self._new_gripper.close():
                message = "gripper_close failed"
                return message

            message = self._do_tool_relative_shift(pick_from_pose_name, "PickFactor", "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message

            # move to home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
                message = "arm move to home failed"
                return message

        # place
        if place_to_pose_name.startswith('robot'):

            # move to back home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_back_home"):
                message = "arm move to back home failed"
                return message

            if 0 != self._ur_ctrl.move_to_by_pose_name(place_to_pose_name):
                message = "arm move to " + place_to_pose_name + " failed"
                return message

            # robot这里继续使用 relative shift 做相对移动，因为调试时使用了base坐标系
            # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
            #                                                   "xyz_shift_do_operation(m)")
            message = self._do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_do_operation(m)")
            if "" != message:
                return message

            # if 0 != self._gripper_ctrl.gripper_open():
            if 0 != self._new_gripper.open():
                message = "gripper_open failed"
                return message

            # message, xyz_shift = self._do_pose_relative_shift(place_to_pose_name, "PlaceFactor",
            #                                                   "xyz_shift_safe_leaving(m)")
            message = self._do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_safe_leaving(m)")

            # move to back home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_back_home"):
                message = "arm move to back home failed"
                return message

            if "" != message:
                return message
        # input Rack station and cnc                  place to cnc or rack
        else:
            # TODO need to read from config Hui Zhi 2022/9/6
            # remove 2023/6/2
            # if pick_from_pose_name.startswith('robot'):
            #     if 0 != self._ur_ctrl.move_to_by_pose_name("temp_pose"):
            #         message = "arm move to temp pose failed"
            #         return message

            # move home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
                message = "arm move to home failed"
                return message

            # move to maker 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name(place_marker_pose_name):
                message = "arm move to " + place_marker_pose_name + " failed"
                return message

            message, relative_marker_pos = self._get_pose_relative_shift(place_to_pose_name, "PlaceFactor",
                                                                         "relative_marker")
            if "" != message:
                return message
            if len(relative_marker_pos) == 1:
                correct_marker_pose_factor = self._cobot_cfg.get_arm_pose_factor(place_marker_pose_name, "CorrectXYZ")
                correct_marker_pose = correct_marker_pose_factor["correct_marker_pose(rad)"]
                if correct_marker_pose == '':
                    message = place_to_pose_name + ":correct_marker_pose is empty"
                    return message
                if 0 != self._ur_ctrl.tool_relative_move_from_rad_pose(correct_marker_pose, relative_marker_pos[0]):
                    message = "move to " + place_to_pose_name + " failed"
                    return message

            message = self._do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_do_operation(m)")
            if "" != message:
                return message

            # if 0 != self._gripper_ctrl.gripper_open():
            if 0 != self._new_gripper.open():
                message = "gripper_open failed"
                return message
            #error twic time
            message = self._do_tool_relative_shift(place_to_pose_name, "PlaceFactor", "xyz_shift_safe_leaving(m)")
            if "" != message:
                return message

            # move home 2023/6/2
            if 0 != self._ur_ctrl.move_to_by_pose_name("new_arm_home"):
                message = "arm move to home failed"
                return message

        total_time = round(time.time() - start_time, 2)
        LogObj.logInfo("E:/CobotHome/TestData", "pick_and_place_testing",
                       pick_from_pose_name + "@" + place_to_pose_name + "@" + str(total_time))
        return message
    
    def _cmd_is_valid(self, cmd):
        bValid = False
        vc = "NG"
        if not cmd is None and len(cmd) > 0:
            cmds = cmd.split(",")
            iLen = len(cmds)
            if iLen > 1:
                s = ""
                for i in range(iLen - 1):
                    s = s + cmds[i] + ","
                vc = FuncObj.calcValidationCode(s).upper()
                vcGet = cmds[iLen - 1].replace("\r", "").replace("\n", "").upper()
                if vc == vcGet: bValid = True
        return bValid, vc

    def _dock_to(self, positionName):  # add by Hui Zhi 2022/5/16
        bOK = False
        currArmPoseName = self._ur_ctrl.get_curr_pose_name()
        if not currArmPoseName == "robot_pallet_a_slot_2":
            if 0 != self._ur_ctrl.move_to_by_pose_name("robot_pallet_a_slot_2"):
                bOK = True
        if self._agv_ctrl.dock_to(positionName):
            bOK = True
        return bOK

    def _execute_cmd(self, conn, addr):
        print('Accept new connection from {0}'.format(addr))
        bVirtualCMR = False
        while (True):
            if self._stop_daemon: break
            bOK, data = self._socket_recv(conn)
            if not bOK: break
            print('{0} client send data is {1}'.format(addr,
                                                       data))  # b'\xe8\xbf\x99\xe6\xac\xa1\xe5\x8f\xaf\xe4\xbb\xa5\xe4\xba\x86'
            LogObj.logInfo(self._home_dir, "Receive_",
                           '{0} client send data is {1}'.format(addr, data))  # add by Hui Zhi 2022/5/19
            if data == 'exit' or not data:
                print('{0} connection close'.format(addr))
                break
            startT = time.time()
            bValid, vc = self._cmd_is_valid(data)
            if not bValid:
                if not self._socket_send(conn,
                                         "NG," + data + ",Validation Code Mismatched(!=" + vc + ")"): break  # Connection broken
            else:
                args = data.split(",")
                bExecuteOK = False
                rtnMsg = ""
                if bVirtualCMR:
                    time.sleep(2)
                    if args[0] == "get_battery": rtnMsg = "95"
                    bExecuteOK = True
                else:
                    try:
                        # 1.Gripper Command
                        if args[0] == "gripper_init":
                            # if 0 == self._gripper_ctrl.gripper_init(): bExecuteOK = True
                            if 0 == self._new_gripper.reset(): bExecuteOK = True
                            # if (not bExecuteOK) and 0 == self._gripper_ctrl.gripper_init(): bExecuteOK = True
                            if (not bExecuteOK) and 0 == self._new_gripper.reset(): bExecuteOK = True
                            # if bExecuteOK: self._gripper_ctrl.gripper_open()
                            if bExecuteOK: self._new_gripper.open()
                        if args[0] == "gripper_close":
                            # if 0 == self._gripper_ctrl.gripper_close(): bExecuteOK = True
                            if 0 == self._new_gripper.close(): bExecuteOK = True
                        if args[0] == "gripper_open":
                            # if 0 == self._gripper_ctrl.gripper_open(): bExecuteOK = True
                            if 0 == self._new_gripper.open(): bExecuteOK = True

                        # 2.AGV Command
                        if args[0] == "move_to":
                            if len(args) == 3:
                                bExecuteOK = self._agv_ctrl.move_to(args[1])
                                self._cobot_cfg.reset_arm_pose_corr_factor()
                            # testing. 2022/6/23 Hui Zhi
                            # if self._status == "Running":
                            #     self._socket_send(conn, "NG,CMR is running\n")
                            # else:
                            #     self._status = "Running"
                            #     self._socket_send(conn, "OK,start\n")
                            #     bExecuteOK = self._agv_ctrl.move_to(args[1])
                            #     self._cobot_cfg.reset_arm_pose_corr_factor()
                            #     if bExecuteOK:
                            #         self._status = "Ready"
                            #         print("finish")
                            #     else:
                            #         self._status = "Error"
                            # break
                            # end
                            else:
                                rtnMsg = "Wrong input paras"

                        if args[0] == "dock_to":  # add by Hui Zhi 2022/5/16
                            if len(args) == 3:
                                bExecuteOK = self._dock_to(args[1])
                                self._cobot_cfg.reset_arm_pose_corr_factor()
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "enable_charging":
                            if len(args) == 2:
                                bExecuteOK = self._agv_ctrl.enable_charging()
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "go_to_charge":
                            if len(args) == 3:
                                if args[1] == "1":
                                    bExecuteOK = self._agv_ctrl.go_to_charge(-1, True)  # Wait for charging starts
                                else:
                                    bExecuteOK = self._agv_ctrl.go_to_charge(-1, False)
                                self._cobot_cfg.reset_arm_pose_corr_factor()
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "read_barcode":
                            result, barcodeData = self._barcode_Ctrl.detect(50, 5)
                            if result == True:
                                bExecuteOK = True
                                rtnMsg = barcodeData
                            else:
                                rtnMsg = "Camera dose not scan the barcode "
                        if args[0] == "get_battery":
                            rtn = self._agv_ctrl.get_agv_battery()
                            rtnMsg = str(rtn)
                            if rtn > 0: bExecuteOK = True
                        if args[0] == "get_agv_position":
                            rtnMsg = self._agv_ctrl.get_curr_position()
                            bExecuteOK = True
                        if args[0] == "agv_is_ready":
                            bExecuteOK = self._agv_ctrl.agv_is_ready()
                            if not bExecuteOK:
                                rtnMsg = "AGV host is not reachable"
                            else:
                                bExecuteOK = self._agv_ctrl.mission_continue()
                                if not bExecuteOK: rtnMsg = "AGV mission fails to start"
                        if args[0] == "agv_move_backward":
                            bExecuteOK = self._agv_ctrl.move_backward()
                            if not bExecuteOK: rtnMsg = "AGV fails to unpark"

                        # 3.Arm command
                        if args[0] == "arm_move_to":
                            if len(args) == 3:
                                if 0 == self._ur_ctrl.move_to_by_pose_name(args[1]): bExecuteOK = True
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "relative_shift":
                            if len(args) == 3:
                                offset = float(args[1])
                                posDelta = []
                                if args[0] == "relative_shift_x": posDelta = [offset, 0, 0, 0, 0, 0]
                                if args[0] == "relative_shift_y": posDelta = [0, offset, 0, 0, 0, 0]
                                if args[0] == "relative_shift_z": posDelta = [0, 0, offset, 0, 0, 0]
                                if len(posDelta) > 0:
                                    if 0 == self._ur_ctrl.relative_shift(posDelta): bExecuteOK = True
                                else:
                                    rtnMsg = "Invalid command " + args[0]
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "pick_and_place":
                            if len(args) == 4:
                                pickFromPose = args[1]
                                placeToPose = args[2]

                                # old version from ZL
                                # rtnMsg = self._pick_and_place(pickFromPose, placeToPose)
                                # New version from HZ 2023/5/16
                                rtnMsg,barcode = self.new_pick_and_place_(pickFromPose, placeToPose)
                            else:
                                rtnMsg = "Wrong input paras"
                            if "" == rtnMsg:
                                bExecuteOK = True
                                rtnMsg = barcode
                        if args[0] == "pick_from_name":
                            if len(args) == 3:
                                pickFromPose = args[1]
                                rtnMsg = self.pick_from_name_(pickFromPose)
                            else:
                                rtnMsg = "Wrong input paras"
                            if "" == rtnMsg: bExecuteOK = True
                        if args[0] == "place_to_name":
                            if len(args) == 3:
                                placeToPose = args[1]
                                rtnMsg = self.place_to_name_(placeToPose)
                            else:
                                rtnMsg = "Wrong input paras"
                            if "" == rtnMsg: bExecuteOK = True
                        if args[0] == "adjust_gripper_pose":
                            if len(args) == 4:
                                pnpPose = args[1]
                                pnpType = args[2]
                                rtnMsg = self._calc_arm_corr_factor(pnpPose, pnpType, False)
                            else:
                                rtnMsg = "Wrong input paras"
                            if "" == rtnMsg: bExecuteOK = True
                        if args[0] == "reset_arm_pose_corr_factor":
                            self._cobot_cfg.reset_arm_pose_corr_factor()
                            bExecuteOK = True
                        if args[0] == "get_arm_position":
                            rtnMsg = self._ur_ctrl.get_curr_pose_name()
                            bExecuteOK = True
                        if args[0] == "calc_arm_corr_factor":
                            if len(args) == 4:
                                poseName = args[1];
                                factorType = args[2]
                                rtnMsg = self._calc_arm_corr_factor(poseName, factorType)
                                if "" == rtnMsg: bExecuteOK = True
                            else:
                                rtnMsg = "Wrong input paras"

                        # 4.Camera command
                        if args[0] == "open_camera":
                            if len(args) == 2:
                                self._open_camera()
                                bExecuteOK = True
                            else:
                                rtnMsg = "Wrong input paras"
                        if args[0] == "close_camera":
                            if len(args) == 2:
                                self._close_camera()
                                bExecuteOK = True
                            else:
                                rtnMsg = "Wrong input paras"
                    except Exception as e:
                        LogObj.logSystemError()
                        rtnMsg = "Daemon Exception"

                # add by Hui Zhi 2022/6/22
                if args[0] == "get_status":
                    if not self._socket_send(conn, self._status + ",testing\n"):
                        break
                    continue

                if args[0] != "get_last_cmd_state":
                    if bExecuteOK:
                        if "" == rtnMsg: rtnMsg = str(round(time.time() - startT, 2)) + "s"
                        # data = data.replace('\r\n','')
                        self._last_cmd_state = "OK" + "," + rtnMsg + "," + data
                    else:
                        if "" == rtnMsg: rtnMsg = "Check command log for details"
                        self._last_cmd_state = "NG" + "," + data + "," + rtnMsg

                if not self._socket_send(conn, self._last_cmd_state):  # Connection broken
                    LogObj.logInfo(self._home_dir, "Error_", "Feedback \"" + self._last_cmd_state + "\" failed")
                    break
                else:
                    LogObj.logInfo(self._home_dir, "Feedback_",
                                   'send data to {0} client : {1}'.format(addr, self._last_cmd_state))
                    # LogObj.logInfo(self._home_dir, "Feedback_", self._last_cmd_state)
        self._socket_close(conn)
        print("CMR Service Done")
