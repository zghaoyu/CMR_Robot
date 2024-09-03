import threading
import time
import win32api
import win32con
import win32gui

def show_message_box(title, msg, bAutoClose = True):
    if bAutoClose: close_message_box(title)
    win32api.MessageBox(0, msg, title, win32con.MB_OK)

def close_message_box(title):
    t = threading.Thread(target=close_msg_box, args=(title,))  # Response to every connection by thread
    t.start()

def close_msg_box(title):
    startT = time.time()
    while(True):
        hwnd = win32gui.FindWindow(0, title)
        if hwnd > 0:
            time.sleep(2)
            win32gui.PostMessage(hwnd, win32con.WM_CLOSE, 0, 0)
            break
        waitT = time.time() - startT
        if waitT > 2: break

show_message_box("Compile Testing", "Compile OK. Congratulations...")