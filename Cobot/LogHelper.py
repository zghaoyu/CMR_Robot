import pathlib
import time
import os
import traceback

def getCurrentTime():
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))

def getCurrTime():
    return time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))

def getCurrTimeStr(fmt = '%Y%m%d'):
    return time.strftime(fmt, time.localtime(time.time()))

def deleteFile(filePath):
    try:
        os.remove(filePath)
    except Exception as e:
        print("deleteFile error: ", e)

def clearDir(dirPath, fileExt, maxDiffDays):
    try:
        files = list(pathlib.Path(dirPath).glob('*.' + fileExt))
        now_unixtime = time.time()
        for f in files:
            date_unixtime = os.path.getmtime(f)
            diffDays = round((now_unixtime-date_unixtime)/(3600*24),2)
            if diffDays > maxDiffDays:
                os.remove(f)
    except Exception as e:
        print("clearDir error: ", e)

def makeDir(dir):
    try:
        my_file = pathlib.Path(dir)
        if not my_file.is_dir():
            os.makedirs(dir)
    except Exception as e:
        print("makeDir error: ", e)

def logInfo(logDir, logFilePrefix, logContent):
    makeDir(logDir)
    f = None
    try:
        logFile = logDir + "/" + logFilePrefix + "_" + getCurrTimeStr() + ".log"
        with open(logFile, 'a') as f:
            f.write(getCurrentTime() + "," + str(logContent) + '\n')
    except Exception as e:
        print("logInfo error: ", e)
    finally:
        if (not f is None):
            f.close()

def logSystemError(logDir = "E:/CobotHome/Daemon"):
    makeDir(logDir)
    f = None
    try:
        logContent = traceback.format_exc()
        logFile = logDir + "/" + "SysErr_" + getCurrTimeStr() + ".log"
        with open(logFile, 'a') as f:
            f.write(getCurrentTime() + "," + str(logContent) + '\n')
    except Exception as e:
        print("logInfo error: ", e)
    finally:
        if (not f is None):
            f.close()

def logAppError(appName, logContent, logDir = "E:/CobotHome/Daemon"):
    makeDir(logDir)
    f = None
    try:
        logFile = logDir + "/" + "AppErr_" + getCurrTimeStr() + ".log"
        with open(logFile, 'a') as f:
            f.write(getCurrentTime() + "," + appName + "," + str(logContent) + '\n')
    except Exception as e:
        print("logInfo error: ", e)
    finally:
        if (not f is None):
            f.close()