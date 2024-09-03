import threading
import socket
import time
import LogHelper as LogObj

#Singleton class
class GripperSMC(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerGripper'
    _target_ip = ("192.168.12.40", 30001)
    _status_ip = ("192.168.12.40", 29999)

    def __init__(self, *args, **kwargs):
        with GripperSMC._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with GripperSMC._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    GripperSMC._instance = super().__new__(cls)
            return GripperSMC._instance

    def clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)

    def gripper_close(self):
        print("gripper_close start")
        rtn = self.gripper_motion(False, 10, 8.0)
        print("gripper_close end")
        return rtn

    def gripper_open(self):
        print("gripper_open start")
        rtn = self.gripper_motion(True, 10, 8.0)
        print("gripper_open end")
        return rtn

    def gripper_init(self, bRetry=False):
        if not bRetry: GripperSMC._instance_lock.acquire()
        timeout_s = 10
        rtnVal = -1
        print("gripper_init start")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            send_data1 = 'def func():\n'
            send_data1 = send_data1 + ' set_standard_digital_out(2,False)\n'
            send_data1 = send_data1 + ' set_standard_digital_out(3,False)\n'
            send_data1 = send_data1 + ' set_standard_digital_out(4,False)\n'
            send_data1 = send_data1 + ' set_standard_digital_out(5,True)\n'
            send_data1 = send_data1 + ' sleep(1.0)\n'

            send_data1 = send_data1 + ' set_standard_digital_out(4,True)\n'
            send_data1 = send_data1 + ' while (not get_standard_digital_in(1)):\n'
            send_data1 = send_data1 + '  sleep(0.1)\n'
            send_data1 = send_data1 + ' end\n'
            send_data1 = send_data1 + ' set_standard_digital_out(4,False)\n'
            send_data1 = send_data1 + ' sleep(2.0)\n'

            send_data1 = send_data1 + ' set_standard_digital_out(2,True)\n'
            send_data1 = send_data1 + ' while(not get_standard_digital_in(0)):\n' #UR5 is DI2, UR16e is DI0
            send_data1 = send_data1 + '  sleep(0.25)\n'
            send_data1 = send_data1 + ' end\n'
            send_data1 = send_data1 + ' set_standard_digital_out(2,False)\n'
            if bRetry: send_data1 = send_data1 + ' sleep(1)\n'
            send_data1 = send_data1 + 'end\n'
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
                            rtnVal = self.gripper_init(True)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Gripper", "Init start,timeout("+str(round(passT,3))+">5s)")
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
                        LogObj.logInfo(self._home_dir, "gripperMotionTime", "Init," + str(round(time.time() - startT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Gripper", "Init stop,timeout("+str(round(passT,3))+">" + str(timeout_s) + "s)")
                            rtnVal = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: GripperSMC._instance_lock.release()
            return rtnVal

        s.close()
        statusSck.close()
        print("gripper_init end")
        if not bRetry: GripperSMC._instance_lock.release()
        return rtnVal

    def gripper_motion(self, bOpen, timeout_s, payload=5.0,bRetry=False):
        if not bRetry: GripperSMC._instance_lock.acquire()
        rtnVal = -1
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)

            send_data1 = 'def func():\n'
            send_data1 = send_data1 + ' set_standard_digital_out(3, False)\n'
            send_data1 = send_data1 + ' while(get_standard_digital_out(3)):\n'
            send_data1 = send_data1 + '  sleep(0.01)\n'
            send_data1 = send_data1 + ' end\n'
            send_data1 = send_data1 + ' set_standard_digital_out(0, False)\n'

            if bOpen:
                send_data1 = send_data1 + ' set_standard_digital_out(1, True)\n'
                send_data1 = send_data1 + ' while(not get_standard_digital_out(1)):\n'
                send_data1 = send_data1 + '  sleep(0.01)\n'
                send_data1 = send_data1 + ' end\n'
            else:
                send_data1 = send_data1 + ' set_standard_digital_out(1, False)\n'
                send_data1 = send_data1 + ' while(get_standard_digital_out(1)):\n'
                send_data1 = send_data1 + '  sleep(0.01)\n'
                send_data1 = send_data1 + ' end\n'

            send_data1 = send_data1 + ' set_standard_digital_out(3, True)\n'
            send_data1 = send_data1 + ' while(not get_standard_digital_out(3)):\n'
            send_data1 = send_data1 + '  sleep(0.01)\n'
            send_data1 = send_data1 + ' end\n'
            send_data1 = send_data1 + ' sleep(0.25)\n'
            send_data1 = send_data1 + ' while(not get_standard_digital_in(0)):\n'
            send_data1 = send_data1 + '  sleep(0.01)\n'
            send_data1 = send_data1 + ' end\n'
            send_data1 = send_data1 + ' set_standard_digital_out(3, False)\n'
            #send_data1 = send_data1 + ' while(not get_standard_digital_in(2) or get_standard_digital_out(3)):\n' #This line is for UR5
            send_data1 = send_data1 + ' while(get_standard_digital_out(3)):\n'
            send_data1 = send_data1 + '  sleep(0.01)\n'
            send_data1 = send_data1 + ' end\n'

            if bOpen:
                send_data1 = send_data1 + ' set_payload('+str(payload)+')\n'
            else:
                send_data1 = send_data1 + ' set_payload('+str(payload)+')\n'
            send_data1 = send_data1 + ' sleep(0.5)\n'
            if bRetry: send_data1 = send_data1 + ' sleep(1)\n'
            send_data1 = send_data1 + 'end\n'
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
                            rtnVal = self.gripper_motion(bOpen, timeout_s, True)
                        else:
                            print("waitForRunning: timeout")
                            LogObj.logAppError("Gripper", ("Open" if bOpen else "Close") + " start,timeout("+str(round(passT,3))+">5s)")
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
                        LogObj.logInfo(self._home_dir, "gripperMotionTime",("Open" if bOpen else "Close") + "," + str(round(time.time() - startT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Gripper", ("Open" if bOpen else "Close") + " stop,timeout("+str(round(passT,3))+">" + str(timeout_s) + "s)")
                            rtnVal = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            if not bRetry: GripperSMC._instance_lock.release()
            return rtnVal

        s.close()
        statusSck.close()
        if not bRetry: GripperSMC._instance_lock.release()
        return rtnVal