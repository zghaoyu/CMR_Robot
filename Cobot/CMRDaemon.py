import os
import win32api
import win32con
import win32gui_struct
import win32gui_struct
import threading
import time

from ClassGUI import DaemonStatus
from ClassDaemon import Daemon
daemon = Daemon()
statusUI = DaemonStatus()

class SysTrayIcon(object):
    QUIT = 'QUIT'
    SPECIAL_ACTIONS = [QUIT]
    FIRST_ID = 1023

    def __init__(self, icon, hover_text, menu_options, on_quit=None, default_menu_index=None, window_class_name=None):
        self.icon = icon
        self.hover_text = hover_text
        self.on_quit = on_quit

        menu_options = menu_options + (('Quit', None, self.QUIT),)
        self._next_action_id = self.FIRST_ID
        self.menu_actions_by_id = set()
        self.menu_options = self._add_ids_to_menu_options(list(menu_options))
        self.menu_actions_by_id = dict(self.menu_actions_by_id)
        del self._next_action_id

        self.default_menu_index = (default_menu_index or 0)
        self.window_class_name = window_class_name or "SysTrayIconPy"
        message_map = {win32gui_struct.RegisterWindowMessage("TaskbarCreated"): self.restart,
                       win32con.WM_DESTROY: self.destroy,
                       win32con.WM_COMMAND: self.command,
                       win32con.WM_USER + 20: self.notify, }
        # Register the Window class.
        window_class = win32gui_struct.WNDCLASS()
        hinst = window_class.hInstance = win32gui_struct.GetModuleHandle(None)
        window_class.lpszClassName = self.window_class_name
        window_class.style = win32con.CS_VREDRAW | win32con.CS_HREDRAW
        window_class.hCursor = win32gui_struct.LoadCursor(0, win32con.IDC_ARROW)
        window_class.hbrBackground = win32con.COLOR_WINDOW
        window_class.lpfnWndProc = message_map  # could also specify a wndproc.
        classAtom = win32gui_struct.RegisterClass(window_class)
        # Create the Window.
        style = win32con.WS_OVERLAPPED | win32con.WS_SYSMENU
        self.hwnd = win32gui_struct.CreateWindow(classAtom, self.window_class_name, style, 0, 0, win32con.CW_USEDEFAULT,
                                          win32con.CW_USEDEFAULT, 0, 0, hinst, None)
        win32gui_struct.UpdateWindow(self.hwnd)
        self.notify_id = None
        self.refresh_icon()
        win32gui_struct.PumpMessages()

    def _add_ids_to_menu_options(self, menu_options):
        result = []
        for menu_option in menu_options:
            option_text, option_icon, option_action = menu_option
            if callable(option_action) or option_action in self.SPECIAL_ACTIONS:
                self.menu_actions_by_id.add((self._next_action_id, option_action))
                result.append(menu_option + (self._next_action_id,))
            elif self.non_string_iterable(option_action):
                result.append((option_text, option_icon, self._add_ids_to_menu_options(option_action), self._next_action_id))
            else:
                print('Unknown item', option_text, option_icon, option_action)
            self._next_action_id += 1
        return result

    def refresh_icon(self):
        # Try and find a custom icon
        hinst = win32gui_struct.GetModuleHandle(None)
        if os.path.isfile(self.icon):
            icon_flags = win32con.LR_LOADFROMFILE | win32con.LR_DEFAULTSIZE
            hicon = win32gui_struct.LoadImage(hinst, self.icon, win32con.IMAGE_ICON, 0, 0, icon_flags)
        else:
            print("Can't find icon file - using default.")
            hicon = win32gui_struct.LoadIcon(0, win32con.IDI_APPLICATION)

        if self.notify_id:
            message = win32gui_struct.NIM_MODIFY
        else:
            message = win32gui_struct.NIM_ADD
        self.notify_id = (self.hwnd, 0, win32gui_struct.NIF_ICON | win32gui_struct.NIF_MESSAGE | win32gui_struct.NIF_TIP,
                          win32con.WM_USER + 20, hicon, self.hover_text)
        win32gui_struct.Shell_NotifyIcon(message, self.notify_id)

    def restart(self, hwnd, msg, wparam, lparam):
        self.refresh_icon()

    def destroy(self, hwnd, msg, wparam, lparam):
        if self.on_quit: self.on_quit(self)
        nid = (self.hwnd, 0)
        win32gui_struct.Shell_NotifyIcon(win32gui_struct.NIM_DELETE, nid)
        win32gui_struct.PostQuitMessage(0)  # Terminate the app.

    def notify(self, hwnd, msg, wparam, lparam):
        if lparam == win32con.WM_LBUTTONDBLCLK:
            self.execute_menu_option(self.default_menu_index + self.FIRST_ID)
        elif lparam == win32con.WM_RBUTTONUP:
            self.show_menu()
        elif lparam == win32con.WM_LBUTTONUP:
            pass
        return True

    def show_menu(self):
        menu = win32gui_struct.CreatePopupMenu()
        self.create_menu(menu, self.menu_options)
        pos = win32gui_struct.GetCursorPos()
        win32gui_struct.SetForegroundWindow(self.hwnd)
        win32gui_struct.TrackPopupMenu(menu, win32con.TPM_LEFTALIGN, pos[0], pos[1], 0, self.hwnd, None)
        win32gui_struct.PostMessage(self.hwnd, win32con.WM_NULL, 0, 0)

    def create_menu(self, menu, menu_options):
        for option_text, option_icon, option_action, option_id in menu_options[::-1]:
            if option_icon:
                option_icon = self.prep_menu_icon(option_icon)

            if option_id in self.menu_actions_by_id:
                item, extras = win32gui_struct.PackMENUITEMINFO(text=option_text, hbmpItem=option_icon, wID=option_id)
                win32gui_struct.InsertMenuItem(menu, 0, 1, item)
            else:
                submenu = win32gui_struct.CreatePopupMenu()
                self.create_menu(submenu, option_action)
                item, extras = win32gui_struct.PackMENUITEMINFO(text=option_text, hbmpItem=option_icon, hSubMenu=submenu)
                win32gui_struct.InsertMenuItem(menu, 0, 1, item)

    def prep_menu_icon(self, icon):
        # First load the icon.
        ico_x = win32api.GetSystemMetrics(win32con.SM_CXSMICON)
        ico_y = win32api.GetSystemMetrics(win32con.SM_CYSMICON)
        hicon = win32gui_struct.LoadImage(0, icon, win32con.IMAGE_ICON, ico_x, ico_y, win32con.LR_LOADFROMFILE)

        hdcBitmap = win32gui_struct.CreateCompatibleDC(0)
        hdcScreen = win32gui_struct.GetDC(0)
        hbm = win32gui_struct.CreateCompatibleBitmap(hdcScreen, ico_x, ico_y)
        hbmOld = win32gui_struct.SelectObject(hdcBitmap, hbm)
        # Fill the background.
        brush = win32gui_struct.GetSysColorBrush(win32con.COLOR_MENU)
        win32gui_struct.FillRect(hdcBitmap, (0, 0, 16, 16), brush)
        # unclear if brush needs to be feed.  Best clue I can find is:
        # "GetSysColorBrush returns a cached brush instead of allocating a new
        # one." - implies no DeleteObject
        # draw the icon
        win32gui_struct.DrawIconEx(hdcBitmap, 0, 0, hicon, ico_x, ico_y, 0, 0, win32con.DI_NORMAL)
        win32gui_struct.SelectObject(hdcBitmap, hbmOld)
        win32gui_struct.DeleteDC(hdcBitmap)
        return hbm

    def command(self, hwnd, msg, wparam, lparam):
        id = win32gui_struct.LOWORD(wparam)
        self.execute_menu_option(id)

    def execute_menu_option(self, id):
        menu_action = self.menu_actions_by_id[id]
        if menu_action == self.QUIT:
            win32gui_struct.DestroyWindow(self.hwnd)
        else:
            menu_action(self)

    def non_string_iterable(self, obj):
        try:
            iter(obj)
        except TypeError:
            return False
        else:
            return not isinstance(obj, "basestring")

def show_message_box(title, msg, bAutoClose = True):
    if bAutoClose: close_message_box(title)
    win32api.MessageBox(0, msg, title, win32con.MB_OK)

def close_message_box(title):
    t = threading.Thread(target=close_msg_box, args=(title,))  # Response to every connection by thread
    t.start()

def close_msg_box(title):
    startT = time.time()
    while(True):
        hwnd = win32gui_struct.FindWindow(0, title)
        if hwnd > 0:
            time.sleep(2)
            win32gui_struct.PostMessage(hwnd, win32con.WM_CLOSE, 0, 0)
            break
        waitT = time.time() - startT
        if waitT > 2: break

def start_daemon(sysTrayIcon):
    if daemon.start_daemon():
        show_message_box("Start CMR Daemon", "CMR Daemon starts OK")
    else:
        show_message_box("Start CMR Daemon", "CMR Daemon starts failed")

def close_daemon(sysTrayIcon):
    statusUI.close_win()
    daemon.stop_daemon()
    show_message_box("Close CMR Daemon", "CMR Daemon is closing")

def show_daemon_status(sysTrayIcon):
    statusUI.show_status(daemon.get_status_info())

if daemon.start_daemon():
    statusUI.show_status(daemon.get_status_info())
else:
    show_message_box("Start CMR Daemon", "CMR Daemon starts failed")
menu_options = (('Start Daemon', "CMR.ico", start_daemon), ('Show Daemon Status', "CMR.ico", show_daemon_status))
SysTrayIcon("CMR.ico", "CMR Daemon", menu_options, on_quit=close_daemon, default_menu_index=1)