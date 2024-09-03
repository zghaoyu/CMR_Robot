import configparser
import requests
import json
import LogHelper as LogObj
import os
import socket
import time

def readConfig(configDir, section, cfgFileName = "config.ini"):
    try:
        cfgFile = configDir + '/' + cfgFileName
        con = configparser.ConfigParser()
        con.read(cfgFile, encoding='utf-8')
        items = dict(con.items(section))
    except Exception as e:
        items = {}
    return items

def writeConfig(configDir, section, dictCfg, cfgFileName = "config.ini"):
    LogObj.makeDir(configDir)
    cfgFile = configDir + '/' + cfgFileName
    config = configparser.ConfigParser()
    config.read(cfgFile)
    sections = config.sections()
    if not section in sections:
        config.add_section(section)
    for key in dictCfg.keys():
        config.set(section, key, str(dictCfg[key]))
    fo = open(cfgFile, 'w+', encoding='utf-8')
    config.write(fo )
    fo.close()

def getDictVal(dictData, key, defaultVal):
    try:
        val = dictData[key]
    except Exception as e:
        val = defaultVal
    return val

def put(url, para, headers, bPrintData = False):
    json_r = None
    try:
        data = para
        data = json.dumps(data)  # python => json format
        r = requests.put(url, data=data, headers=headers)
        if r.status_code == 200:
            json_r = r.json()
            if bPrintData:
                print("return data: ", json_r)
        else:
            print("request failed: ", r.status_code)
            LogObj.logAppError("httpPut", str(r.status_code) + ",PUT," + url)
    except BaseException as e:
        LogObj.logSystemError()
    return json_r

def get(url, para, headers, bPrintData = False):
    json_r = None
    try:
        r = requests.get(url, params=para, headers=headers)
        if r.status_code == 200:
            json_r = r.json()
            if bPrintData:
                print("return data: ", json_r)
        else:
            print("request failed: ", r.status_code)
            LogObj.logAppError("httpGet",str(r.status_code)+",GET,"+url)
    except BaseException as e:
        LogObj.logSystemError()
    return json_r

def post(url, para, headers, bPrintData = False):
    json_r = None
    try:
        r = requests.post(url, data=para, headers=headers)
        if r.status_code == 201:
            json_r = r.json()
            if bPrintData:
                print("return data: ", json_r)
        else:
            print("request failed: ", r.status_code)
            LogObj.logAppError("httpPost", str(r.status_code) + ",POST," + url)
    except BaseException as e:
        LogObj.logSystemError()
    return json_r

def post_json(url, para, headers, bPrintData = False):
    json_r = None
    try:
        data = para
        data = json.dumps(data)  # python => json format
        r = requests.post(url, data=data, headers=headers)
        if r.status_code == 201:
            json_r = r.json()
            if bPrintData:
                print("return data: ", json_r)
        else:
            print("request failed: ", r.status_code)
            LogObj.logAppError("httpPostJs", str(r.status_code) + ",POST_JSON," + url)
    except BaseException as e:
        LogObj.logSystemError()
    return json_r

def ping(ip, iRetryCnt = 1):
    bOK = False
    if iRetryCnt <= 0: iRetryCnt = 1
    for i in range(iRetryCnt):
        try:
            rtn = os.popen("ping " + ip).read()
            if "= 0 (0% " in rtn:
                bOK = True
            else:
                print(rtn)
        except Exception as e:
            LogObj.logSystemError()
        if bOK: break
    return bOK

def get_host_ip(method=0):
    if method == 0:
        try:
            s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(('8.8.8.8',80))
            ip=s.getsockname()[0]
        except Exception as e:
            LogObj.logSystemError()
            ip = ""
        finally:
            s.close()
    else:
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname) #Local network IP
    return ip

def calcValidationCode(strData):
    vcode = ""
    iChrCnt = len(strData)
    if iChrCnt > 0:
        iCode = ord(strData[0])
        if iChrCnt > 1:
            for i in range(1, iChrCnt):
                iCode = iCode ^ ord(strData[i])
        #vcode = (hex(iCode)).replace("0x","")
        vcode = "{:2X}".format(iCode)
    return vcode

def swapVal(a, b):
    return b, a

def getCurrTimeStr(fmt = '%Y%m%d'):
    return time.strftime(fmt, time.localtime(time.time()))