import threading
import UtilsFunc as FuncObj

#Singleton class
class CobotCfg(object):
    _instance_lock = threading.Lock()
    _cfg_dir = "E:/CobotHome/Config"
    _factor_names = ["PickFactor", "PlaceFactor", "CorrectXYZ","PickAndPlaceFactor"]
    _config_arm_pose = {}
    _config_arm_pfactor = {}
    _config_marker = {}
    _config_agv_pos_ids = {}
    _config_agv_mis_ids = {}

    def __init__(self, *args, **kwargs):
        with CobotCfg._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with CobotCfg._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    CobotCfg._instance = super().__new__(cls)
            return CobotCfg._instance

    def get_agv_pos_ids(self, bReload = False):
        if bReload or len(self._config_agv_pos_ids) <= 0:
            self._config_agv_pos_ids = FuncObj.readConfig(self._cfg_dir, "PositionIDs", "agv_cfg.ini")
        return self._config_agv_pos_ids

    def get_agv_mis_ids(self, bReload = False):
        if bReload or len(self._config_agv_mis_ids) <= 0:
            self._config_agv_mis_ids = FuncObj.readConfig(self._cfg_dir, "MissionIDs", "agv_cfg.ini")
        return self._config_agv_mis_ids

    def get_arm_pose_cfg(self, bReload = False):
        if bReload or len(self._config_arm_pose) <= 0:
            self._config_arm_pose = FuncObj.readConfig(self._cfg_dir, "ArmPose", "arm_pose.ini")
        return self._config_arm_pose

    def get_arm_pose_factor(self, posName, factorName="PickFactor", bReload = False):
        rtn_factor = {}
        if factorName in self._factor_names:
            pose_factor = FuncObj.getDictVal(self._config_arm_pfactor, posName, None)
            rtn_factor = FuncObj.getDictVal(pose_factor, factorName, {})
            if bReload or pose_factor is None or len(rtn_factor)<=0:
                pose_factor = {}
                pose_factor["PickFactor"] = FuncObj.readConfig(self._cfg_dir, "PickFactor", posName + ".ini")
                pose_factor["PlaceFactor"] = FuncObj.readConfig(self._cfg_dir, "PlaceFactor", posName + ".ini")
                pose_factor["PickAndPlaceFactor"] = FuncObj.readConfig(self._cfg_dir, "PickAndPlaceFactor", posName + ".ini")#add by Hui Zhi 2022/9/15
                pose_factor["CorrectXYZ"] = FuncObj.readConfig(self._cfg_dir, "CorrectXYZ", posName + ".ini")
                self._config_arm_pfactor[posName] = pose_factor
            rtn_factor = FuncObj.getDictVal(pose_factor, factorName, {})
        return rtn_factor

    def save_arm_pose_factor(self, poseName, factorName, dictFactor):
        if not dictFactor is None:
            if factorName in self._factor_names and len(dictFactor) > 0:
                FuncObj.writeConfig(self._cfg_dir, factorName, dictFactor, poseName + ".ini")

    def refresh_arm_pose_factor(self, poseName, factorName="CorrectXYZ", dictFactor={"corr_x": 0, "corr_y": 0, "corr_rz": 0}):
        try:
            if factorName in self._factor_names:
                self._config_arm_pfactor[poseName][factorName] = dictFactor
        except Exception as e:
            pass

    def reset_arm_pose_corr_factor(self):
        try:
            for k in self._config_arm_pfactor.keys():
                self._config_arm_pfactor[k]["CorrectXYZ"] = {}
        except Exception as e:
            pass

    def get_marker_cfg(self, markerName = "CircleBasedMkr_01", bReload = False):
        cfg = FuncObj.getDictVal(self._config_marker, markerName, None)
        if bReload or cfg is None:
            cfg = FuncObj.readConfig(self._cfg_dir, markerName, "position_marker.ini")
            if len(cfg) <= 0:
                cfg = {"camera_index": 0,
                       "proc_method": "circles_01",
                       "roi_x_start(%)": 5,
                       "roi_x_stop(%)": 85,
                       "roi_y_start(%)": 35,
                       "roi_y_stop(%)": 75,
                       "gray_target": 100,
                       "circle_min_r(pixel)": 8,
                       "circle_max_r(pixel)": 24,
                       "circle_qty": 8,
                       "marker_centerx_rate": 0.6
                       }
                FuncObj.writeConfig(self._cfg_dir, markerName, cfg, "position_marker.ini")
            self._config_marker[markerName] = cfg
        return cfg