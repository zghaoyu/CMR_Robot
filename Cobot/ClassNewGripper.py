import threading
import socket
import LogHelper as LogObj
import time


class NewGripper(object):
    _instance_lock = threading.Lock()
    _home_dir = 'E:/CobotHome/ControllerGripper'
    _target_ip = ("192.168.12.40", 30001)
    _status_ip = ("192.168.12.40", 29999)
    is_GripperOpen = False
    is_GripperReset = False

    def __init__(self, *args, **kwargs):
        with NewGripper._instance_lock:
            pass

    def __new__(cls, *args, **kwargs):
        with NewGripper._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    NewGripper._instance = super().__new__(cls)
            return NewGripper._instance

    def clear_log(self):
        LogObj.clearDir(self._home_dir, "*", 90)

    # TODO 优化重复代码 2023/3/31
    def close(self, payload=5.0, timeout_s=5):
        NewGripper._instance_lock.acquire()
        result = -1
        print("Gripper close start")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            # DO0:close, DO1:reset, DO2:open
            # DI0:alarm, DI1:status
            send_data1 = 'def func():\n'
            send_data1 += ' set_standard_digital_out(0,False)\n'
            send_data1 += ' set_standard_digital_out(1,False)\n'
            send_data1 += ' set_standard_digital_out(2,False)\n'
            send_data1 += ' set_payload(' + str(payload) + ')\n'

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
            send_data1 += 'end\n'
            s.send(send_data1.encode('utf8'))
            print(send_data1)
            # Wait for running
            startT = time.time()
            is_running = False
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                # print(recvData)
                if recvData.__contains__("running: true"):
                    print("program running")
                    is_running = True
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        print("waitForRunning: timeout")
                        self.error = LogObj.logAppError("Gripper",
                                                        "close start,timeout(" + str(round(passT, 3)) + ">5s)")
                        result = 1
                        break

            if is_running:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):

                        result = 0  # Success
                        self.is_GripperOpen = False
                        print("success")
                        LogObj.logInfo(self._home_dir, "gripperMotionTime",
                                       "Close," + str(round(time.time() - startT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Gripper", "Init stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            result = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            NewGripper._instance_lock.release()
            return result
        s.close()
        statusSck.close()
        print("gripper_close end")
        NewGripper._instance_lock.release()
        return result

    def open(self, payload=5.0, timeout_s=5):
        NewGripper._instance_lock.acquire()
        result = -1
        print("Gripper open start")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            # DO0:close, DO1:reset, DO2:open
            # DI0:alarm, DI1:status
            send_data1 = 'def func():\n'
            send_data1 += ' set_standard_digital_out(0,False)\n'
            send_data1 += ' set_standard_digital_out(1,False)\n'
            send_data1 += ' set_standard_digital_out(2,False)\n'
            send_data1 += ' set_payload(' + str(payload) + ')\n'

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
            send_data1 += 'end\n'
            s.send(send_data1.encode('utf8'))
            print("send open data:"+send_data1)
            # Wait for running
            startT = time.time()
            is_running = False
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                # print(recvData)
                if recvData.__contains__("running: true"):
                    print("program running")
                    is_running = True
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        print("waitForRunning: timeout")
                        self.error = LogObj.logAppError("Gripper",
                                                        "open start,timeout(" + str(round(passT, 3)) + ">5s)")
                        result = 1
                        break

            if is_running:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        result = 0  # Success
                        self.is_GripperOpen = True
                        print("success")
                        LogObj.logInfo(self._home_dir, "gripperMotionTime",
                                       "Open," + str(round(time.time() - startT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Gripper", "Init stop,timeout(" + str(round(passT, 3)) + ">" + str(
                                timeout_s) + "s)")
                            result = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            NewGripper._instance_lock.release()
            return result
        s.close()
        statusSck.close()
        print("gripper_open end")
        NewGripper._instance_lock.release()
        return result

    def reset(self, timeout_s=5):
        NewGripper._instance_lock.acquire()
        result = -1
        print("Gripper reset start")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(self._target_ip)
            statusSck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            statusSck.connect(self._status_ip)
            # DO0:close, DO1:reset, DO2:open
            # DI0:alarm, DI1:status
            send_data1 = 'def func():\n'
            send_data1 += ' set_standard_digital_out(0,False)\n'
            send_data1 += ' set_standard_digital_out(1,False)\n'
            send_data1 += ' set_standard_digital_out(2,False)\n'

            # when alarming
            # send_data1 += ' while (get_standard_digital_in(0)):\n'
            # send_data1 += '  sleep(0.1)\n'
            # send_data1 += ' end\n'
            # when gripper is not ready
            # send_data1 += ' while (not get_standard_digital_in(1)):\n'
            # send_data1 += '  sleep(0.1)\n'
            # send_data1 += ' end\n'
            # if ready, execute reset command

            # 无条件执行复位
            send_data1 += ' set_standard_digital_out(1,True)\n'
            send_data1 += ' sleep(0.2)\n'
            send_data1 += ' set_standard_digital_out(1,False)\n'
            # if status is false, gripper is running
            send_data1 += ' while (not get_standard_digital_in(1)):\n'
            send_data1 += '  sleep(0.1)\n'
            send_data1 += ' end\n'
            send_data1 += 'end\n'
            s.send(send_data1.encode('utf8'))

            # Wait for running
            startT = time.time()
            is_running = False
            while (True):
                statusSck.send("running\n".encode())
                recvData = (statusSck.recv(1024)).decode()
                # print(recvData)
                if recvData.__contains__("running: true"):
                    print("program running")
                    is_running = True
                    break
                else:
                    time.sleep(0.01)
                    passT = time.time() - startT
                    if passT > 5:
                        print("waitForRunning: timeout")
                        self.error = LogObj.logAppError("Gripper",
                                                        "Init start,timeout(" + str(round(passT, 3)) + ">5s)")
                        result = 1
                        break

            if is_running:
                startT = time.time()
                while (True):
                    statusSck.send("running\n".encode())
                    recvData = (statusSck.recv(1024)).decode()
                    if recvData.__contains__("running: false"):
                        result = 0  # Success
                        self.is_GripperReset = True
                        print("success")
                        LogObj.logInfo(self._home_dir, "gripperMotionTime", "Init," + str(round(time.time() - startT, 3)))
                        break
                    else:
                        time.sleep(0.01)
                        passT = time.time() - startT
                        if passT > timeout_s:
                            print("waitForFinishing: timeout")
                            LogObj.logAppError("Gripper", "Init stop,timeout("+str(round(passT,3))+">" + str(timeout_s) + "s)")
                            result = 2
                            break
        except socket.error as msg:
            LogObj.logSystemError()
            NewGripper._instance_lock.release()
            return result
        s.close()
        statusSck.close()
        print("gripper_init end")
        NewGripper._instance_lock.release()
        return result
