import sys
import threading
import time
import UtilsFunc as FuncObj
from PyQt5.QtCore import QThread, pyqtSignal, QDateTime, QObject
from PyQt5.QtWidgets import QWidget, QDesktopWidget, QLabel, QTextEdit, QGridLayout, QApplication
from PyQt5.QtGui import QIcon
from ClassDaemon import Daemon

class BackendThread(QObject):
    update_date = pyqtSignal(str) # 通过类成员对象定义信号
    # 处理业务逻辑
    def run(self):
        while True:
            data = QDateTime.currentDateTime()
            currTime = data.toString("yyyy-MM-dd hh:mm:ss")
            self.update_date.emit(str(currTime))
            time.sleep(1)

class DaemonStatus(QWidget):
    _instance_lock = threading.Lock()
    _app = QApplication(sys.argv)
    _daemon = Daemon()

    def __init__(self):
        with DaemonStatus._instance_lock:
            super().__init__()
            self._init_UI()

    def __new__(cls, *args, **kwargs):
        with DaemonStatus._instance_lock:
            if not hasattr(cls, '_instance'):
                if not hasattr(cls, '_instance'):
                    DaemonStatus._instance = super().__new__(cls)
            return DaemonStatus._instance

    def _init_UI(self):
        self.setWindowTitle('CMR Daemon Status')
        self.setWindowIcon(QIcon('CMR.ico'))

        status = QLabel('Status')
        self.status_info = QTextEdit()
        grid = QGridLayout()
        grid.setSpacing(10)
        grid.addWidget(status, 1, 0)
        grid.addWidget(self.status_info, 1, 1, 5, 1)
        self.setLayout(grid)
        self.setGeometry(100, 100, 450, 300)
        self._center()

        # 创建线程
        self.backend = BackendThread()
        self.backend.update_date.connect(self._set_status_text)
        self.thread = QThread()
        self.backend.moveToThread(self.thread)
        self.thread.started.connect(self.backend.run)
        self.thread.start()

    def _center(self):
        qr = self.frameGeometry() # 获得窗口
        cp = QDesktopWidget().availableGeometry().center() # 获得屏幕中心点
        qr.moveCenter(cp) # 显示到屏幕中心
        self.move(qr.topLeft())

    def _set_status_text(self):
        self.setWindowTitle('CMR Daemon ' + FuncObj.getCurrTimeStr("%H:%M:%S"))
        self.status_info.setText(self._daemon.get_status_info())

    def show_status(self, status_text):
        self.show()
        self.status_info.setText(status_text)
        self._app.exec_()

    def close_win(self):
        self._app.closeAllWindows()