import threading
import time
import LogHelper as LogObj
import UtilsFunc as FuncObj
from ClassConfig import CobotCfg

#Singleton class
class AgvMIR(object):
    _instance_lock = threading.Lock()
    _cobot_cfg = CobotCfg()
    _home_dir = 'E:/CobotHome/ControllerAGV'
    _agv_ip = "192.168.12.20"
    _url = 'http://192.168.12.20/api/v2.0.0'
    _curr_pos = "Unkown"
    _pos_ids = {}
    _mis_ids = {
        "move_to_mis_id": "",
        "dock_to_mis_id": "",  # add by Hui Zhi 2022/5/10
        "dock_to_charger_mis_id": "",
        "enable_charging_mis_id": "",
        "move_backward_mis_id": ""
    }
    _headers = {
        "Content-Type": "application/json",
        "Accept-Language": "en_US",
        "Authorization": "Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
    }
    _move_to_para = {
        "mission_id": "",
        "parameters": [{
            "input_name": "moveTo",
            "value": ""
        }]
    }
    _dock_to_para = {  # add by Hui Zhi  2022/5/10
        "mission_id": "",
        "parameters": [{
            "input_name": "dockTo",
            "value": ""
        }]
    }

    _dock_to_charger_para = {
        "mission_id": "0fa91658-2612-11eb-821a-000129998290"
    }
    _enable_charging_para = {
        "mission_id": "b5807020-23f8-11eb-ab6b-000129998290"
    }
    _move_backward_para = {
        "mission_id": "2eb8b7a7-5002-11eb-8056-000129998290"
    }

    def __init__(self, *args, **kwargs):
        with AgvMIR._instance_lock:
            self._curr_pos = "Unkown"
            self._pos_ids = self._cobot_cfg.get_agv_pos_ids()
            self._mis_ids = self._cobot_cfg.get_agv_mis_ids()

    def __new__(cls, *args, **kwargs):
        with AgvMIR._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    AgvMIR._instance = super().__new__(cls)
            return AgvMIR._instance

    def clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)

    def get_curr_position(self):
        return self._curr_pos

    def get_agv_status(self, iRetryCnt=0):
        status = FuncObj.get(self._url + '/status', {}, self._headers)
        if status is None and iRetryCnt>0:
            for i in range(iRetryCnt):
                status = FuncObj.get(self._url + '/status', {}, self._headers)
                if not status is None: break
        return status

    def get_agv_info(self, info='/positions'):
        status = FuncObj.get(self._url + info, {}, self._headers)
        return status

    def agv_is_ready(self):
        ready = FuncObj.ping(self._agv_ip,3)
        return ready

    def clear_error(self):
        rtn = FuncObj.put(self._url + '/status', {"clear_error": True}, self._headers)
        if rtn is None:
            return False
        else:
            return True

    def set_mission_state(self, state):
        rtn = FuncObj.put(self._url + '/status', {"state_id": state}, self._headers)
        if rtn is None:
            return False
        else:
            return True

    def mission_pause(self):
        return self.set_mission_state(4)

    def mission_continue(self):
        return self.set_mission_state(3)

    def get_agv_battery(self, bLogBattery=False):
        status = self.get_agv_status(3)
        battery = FuncObj.getDictVal(status, "battery_percentage", None)
        if not battery is None:
            if bLogBattery:
                if FuncObj.getDictVal(status, "mission_text", "") == "Charging... Waiting for new mission...":
                    LogObj.logInfo(self._home_dir, "cobotCharging", str(round(battery,2)))
                else:
                    LogObj.logInfo(self._home_dir, "cobotDischarging", str(round(battery, 2)))
        if not battery is None:
            return round(battery, 2)
        else:
            return -1

    def _get_mission_err(self):
        status = self.get_agv_status(3)
        err = FuncObj.getDictVal(status, "errors", None)
        return err

    def _execute_mission(self, input_paras, mission_name="execute_mission", timeout_s=-1, bWaitForChargingStart=True, bRetryFlag=False):
        print(mission_name + " start")
        startT = time.time()
        bOK = False

        try:
            # Add mission
            rtnData = FuncObj.post_json(self._url + '/mission_queue', input_paras, self._headers)
            if rtnData is None:
                print(mission_name + " failed!")
                LogObj.logAppError("AGV", mission_name + " mission failed!")
            else:
                # Check mission status
                lastBattery = -1
                while (True):
                    if mission_name == "enable_charging":
                        status = self.get_agv_status(3)
                    else:
                        status = FuncObj.get(self._url + '/mission_queue/' + str(rtnData["id"]), {}, self._headers)
                    passT = time.time() - startT
                    if not status is None and ((mission_name != "enable_charging" and status["state"] == "Done") or (
                        mission_name == "enable_charging" and (bWaitForChargingStart and status["mission_text"] == "Charging... Waiting for new mission..."
                        or (not bWaitForChargingStart) and status["state_text"] == "Executing"))):
                        print(mission_name + " takes " + str(time.time() - startT) + "s")
                        LogObj.logInfo(self._home_dir, "missionTime", mission_name + "," + str(round(time.time() - startT, 3)))
                        bOK = True
                        break
                    else:
                        if (timeout_s > 0 and passT > timeout_s):
                            LogObj.logAppError("AGV", mission_name + " mission timeout("+str(round(passT,3))+">" + str(timeout_s) + "s)")
                            break
                        else:
                            err = self._get_mission_err()
                            if not err is None and len(err) > 0:
                                errCode = FuncObj.getDictVal(err[0], "code", 0)
                                errMsg = FuncObj.getDictVal(err[0], "description", "none")
                                LogObj.logAppError("AGV", mission_name + " mission error(" + str(errCode) + "):"+errMsg)
                                self.clear_error()
                                self.mission_continue() # Mission paused after error happens
                                if not bRetryFlag:
                                    bOK = self._execute_mission(input_paras, mission_name, timeout_s, bWaitForChargingStart, True)
                                break
                            else:
                                if status is None and passT > 120:
                                    if FuncObj.ping(self._agv_ip, 3):
                                        if not bRetryFlag:
                                            bOK = self._execute_mission(input_paras, mission_name, timeout_s, bWaitForChargingStart, True)
                                    else:
                                        LogObj.logAppError("AGV", mission_name + " mission error(Ping AGV NG)")
                                    break
                                else:
                                    if mission_name != "enable_charging": status = self.get_agv_status(3)
                                    battery = FuncObj.getDictVal(status, "battery_percentage", None)
                                    if not battery is None and (abs(battery - lastBattery) >= 1):
                                        LogObj.logInfo(self._home_dir, "cobotBattery", str(round(battery,2)) + "," + mission_name)
                                        lastBattery = battery
                                time.sleep(1)
        except Exception as e:
            LogObj.logSystemError()
        print(mission_name + " stop")
        return bOK

    def add_mission(self, mission_name): #For debug only
        input_paras = None
        status = None
        if mission_name == "move_backward": input_paras = self._move_backward_para
        if mission_name == "dock_to_charger": input_paras = self._dock_to_charger_para
        if mission_name == "enable_charging": input_paras = self._enable_charging_para
        if input_paras is None:
            posID = FuncObj.getDictVal(self._pos_ids, mission_name, None)
            if not posID is None:
                self._move_to_para["parameters"][0]["value"] = posID
                input_paras = self._move_to_para
        if not input_paras is None:
            rtnData = FuncObj.post_json(self._url + '/mission_queue', input_paras, self._headers)
            if not rtnData is None:
                status = FuncObj.get(self._url + '/mission_queue/' + str(rtnData["id"]), {}, self._headers)
        else:
            status = FuncObj.get(self._url + '/mission_queue/' + mission_name, {}, self._headers)
        return status

    def move_to(self, posName, timeout_s = -1):
        bOK = False
        posID = FuncObj.getDictVal(self._pos_ids, posName, None)
        misID = FuncObj.getDictVal(self._mis_ids, "move_to_mis_id", None)
        if posID is None or misID is None:
            print("move_to " + posName + " failed: " + posName + " or move_to_mis_id is not existing")
            LogObj.logAppError("AGV", "move_to(" + posName + ") failed: " + posName + " or move_to_mis_id is not existing")
        else:
            if self._curr_pos == "Charger": self.move_backward(timeout_s)
            self._move_to_para["parameters"][0]["value"] = posID
            self._move_to_para["mission_id"] = misID
            bOK = self._execute_mission(self._move_to_para, "move_to("+posName+")", timeout_s)
        if bOK: self._curr_pos = posName
        return bOK

    # add by Hui Zhi 2022/5/10
    def dock_to(self, posName, timeout_s = -1):
        bOK = False
        posID = FuncObj.getDictVal(self._pos_ids, posName, None)
        misID = FuncObj.getDictVal(self._mis_ids, "dock_to_mis_id", None)
        if posID is None or misID is None:
            print("dock_to_%s failed: mission id is not existing" % posName)
            LogObj.logAppError("AGV", "dock_to_%s failed: pos id or mission id is not existing" % posName)
        else:
            if self._curr_pos == "Charger": self.move_backward(timeout_s)
            self._dock_to_para["parameters"][0]["value"] = posID
            self._dock_to_para["mission_id"] = misID
            bOK = self._execute_mission(self._dock_to_para, "dock_to("+posName+")", timeout_s)
            if bOK: self._curr_pos = posName
        return bOK

    def dock_to_charger(self, timeout_s=-1):
        bOK = False
        if self._curr_pos == "Charger":
            if not self.move_backward(timeout_s):
                print("dock_to_charger failed: move_backward fails")
                LogObj.logAppError("AGV", "dock_to_charger failed: move_backward fails")
                return bOK
        misID = FuncObj.getDictVal(self._mis_ids, "dock_to_charger_mis_id", None)
        if misID is None:
            print("dock_to_charger failed: mission id is not existing")
            LogObj.logAppError("AGV", "dock_to_charger failed: mission id is not existing")
        else:
            self._dock_to_charger_para["mission_id"] = misID
            bOK = self._execute_mission(self._dock_to_charger_para, "dock_to_charger", timeout_s)
            if bOK: self._curr_pos = "Charger"
        return bOK

    def move_backward(self, timeout_s=-1):
        bOK = False
        misID = FuncObj.getDictVal(self._mis_ids, "move_backward_mis_id", None)
        if misID is None:
            print("move_backward failed: mission id is not existing")
            LogObj.logAppError("AGV", "move_backward failed: mission id is not existing")
        else:
            self._move_backward_para["mission_id"] = misID
            bOK = self._execute_mission(self._move_backward_para, "move_backward", timeout_s)
            if bOK: self._curr_pos = "Unparking"
        return bOK

    def enable_charging(self, timeout_s=-1, bWaitForChargingStart=True):
        bOK = False
        misID = FuncObj.getDictVal(self._mis_ids, "enable_charging_mis_id", None)
        if misID is None:
            print("enable_charging failed: mission id is not existing")
            LogObj.logAppError("AGV", "enable_charging failed: mission id is not existing")
        else:
            self._enable_charging_para["mission_id"] = misID
            bOK = self._execute_mission(self._enable_charging_para, "enable_charging", timeout_s, bWaitForChargingStart)
            if bOK: self._curr_pos = "Charger"
        return bOK

    def go_to_charge(self, timeout_s=-1, bWaitForChargingStart=True, bSkipCharging=False):
        for i in range(3):
            bOK = self.dock_to_charger(timeout_s)
            if bOK and (not bSkipCharging):
                self._curr_pos = "Charger"
                bOK = self.enable_charging(timeout_s, bWaitForChargingStart)
            if bOK: break
        return bOK

    def wait_for_charging_start(self, timeout_s=-1):
        startT = time.time()
        bOK = False
        while(True):
            status = self.get_agv_status()
            if(status["mission_text"] == "Charging... Waiting for new mission..."):
                bOK = True
                break
            else:
                if (timeout_s > 0 and (time.time() - startT) > timeout_s):
                    break
                else:
                    err = FuncObj.getDictVal(status, "errors", None)
                    if not err is None and len(err) > 0:
                        break
                    else:
                        time.sleep(1)
        return bOK

    # add by Hui Zhi 2022/9/22
    def get_state_text(self):
        status = self.get_agv_status(3)
        state_text = FuncObj.getDictVal(status, "state_text", '')
        return state_text

    # add by Hui Zhi 2022/9/22
    def get_mission_text(self):
        status = self.get_agv_status(3)
        mission_text = FuncObj.getDictVal(status, "mission_text", '')
        return mission_text

    # add by Hui Zhi 2022/9/22
    def _get_missions(self, iRetryCnt=0):
        missions = FuncObj.get(self._url + '/missions', {}, self._headers)
        if missions is None and iRetryCnt>0:
            for i in range(iRetryCnt):
                missions = FuncObj.get(self._url + '/missions', {}, self._headers)
                if not missions is None: break
        return missions

    # add by Hui Zhi 2022/9/22
    def get_guid_by_mission_name(self,mission_name):
        missions = self._get_missions()
        guid = ''
        if missions is not None:
            for mission in missions:
                if mission['name'] == mission_name:
                    guid = mission['guid']
        return guid
