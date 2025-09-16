# -*- coding: utf-8 -*-
import os
import sys
import time
import re
import json

from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QMessageBox, QWidget, QHeaderView, QTableWidgetItem, \
    QAbstractItemView
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QCoreApplication, QIODevice
from PyQt5.QtSerialPort import QSerialPortInfo, QSerialPort

import cv2
import numpy as np
from PIL import ImageFont
from ultralytics import YOLO

sys.path.append('UIProgram')
from UIProgram.UiMain import Ui_MainWindow
from UIProgram.QssLoader import QSSLoader
from UIProgram.precess_bar import ProgressBar
import detect_tools as tools
import Config


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.initMain()
        self.signalconnect()

        self.timer_detection = QTimer(self)
        self.timer_detection.setInterval(500)  # 设置为500毫秒（0.5秒）
        self.timer_detection.timeout.connect(self.detect_frame)

        # 串口相关初始化，只连接一次 readyRead 信号
        self._serial = QSerialPort(self)
        self._ports = {}
        self.getAvailablePorts()

        style_file = 'UIProgram/style.css'
        qssStyleSheet = QSSLoader.read_qss_file(style_file)
        self.setStyleSheet(qssStyleSheet)

        self.buffer = ""
        self.in_json = False

        self.ui.textBrowser.setReadOnly(False)

        self.ui.pushButton.clicked.connect(self.on_buttonConnect_clicked)
        self._serial.readyRead.connect(self.read_serial_data)  # 仅连接一次

        self.weizhi_X = None  # 存储 X 数据
        self.weizhi_Y = None  # 存储 Y 数据
        # 初始化车身角度相关变量，确保解析 JSON 与正则时均有定义
        self.weizhi_A = None
        self.weizhi_A_x = None
        self.weizhi_A_y = None
        self.weizhi_A_z = None

        self.at_commands = [
            "AT+CWMODE=1",
            'AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com "',
            'AT+CWJAP="DCY","12345678"',
            'AT+MQTTUSERCFG=0,1,"NULL","PYQT&iggisf93NrW","3870efe14bdc3ebccae0d6778404635cbcdb79eb06273b0224bdb1321eb5ffd6",0,0,""',
            'AT+MQTTCLIENTID=0,"iggisf93NrW.PYQT|securemode=2\\,signmethod=hmacsha256\\,timestamp=1741496379578|"',
            'AT+MQTTCONN=0,"iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com",1883,1',
            r'AT+MQTTPUB=0,"/iggisf93NrW/PYQT/user/update","{\"params\":{\"HUO\":0}}",1,0'
        ]
        self.current_cmd_index = 0  # 当前发送的指令索引
        self.HUO = [
            {"cmd": "AT+MQTTPUB=0,\"/sys/iggisf93NrW/PYQT/thing/event/property/post\",\"{\\\"params\\\":{\\\"HUO\\\":1}}\",1,0",
            "delay": 1000},
            {"cmd": "AT+MQTTPUB=0,\"/iggisf93NrW/PYQT/user/update\",\"{\\\"params\\\":{\\\"HUO\\\":1}}\",1,0",
             "delay": 1000}
        ]
        self.HUOlistID = 0

    def HUO_start_send_at_commands(self):
        if self._serial.isOpen() and self.HUOlistID < len(self.HUO):
            current_cmd = self.HUO[self.HUOlistID]
            HUO_cmd = current_cmd["cmd"]
            HUO_delay = current_cmd["delay"]
            self.HUO_send_at_command(HUO_cmd)
            self.HUOlistID += 1
            QTimer.singleShot(HUO_delay, self.HUO_start_send_at_commands)
        else:
            self.HUOlistID = 0

    def HUO_send_at_command(self, HUO_cmd):
        if self._serial.isOpen():
            HUO_full_cmd = HUO_cmd + "\r\n"
            HUO_bytes_written = self._serial.write(HUO_full_cmd.encode())
            if HUO_bytes_written == len(HUO_full_cmd):
                self.ui.textBrowser.append(f'[发送成功] {HUO_cmd}')
            else:
                self.ui.textBrowser.append(f'[发送失败] {HUO_cmd}')

    def start_send_at_commands(self):
        if self._serial.isOpen() and self.current_cmd_index < len(self.at_commands):
            cmd = self.at_commands[self.current_cmd_index]
            delay = 5000  # 设置一个统一的间隔，比如2秒
            self.send_at_command(cmd)
            self.current_cmd_index += 1
            QTimer.singleShot(delay, self.start_send_at_commands)
        else:
            self.current_cmd_index = 0

    def detect_frame(self):
        if self.cap:  # 检查是否打开了摄像头
            ret, frame = self.cap.read()
            if ret:
                # 在这里处理检测逻辑，例如调用 YOLO 模型
                t1 = time.time()
                results = self.model(frame)[0]
                t2 = time.time()
                take_time_str = '{:.3f} s'.format(t2 - t1)
                self.ui.time_lb.setText(take_time_str)

                # 处理检测结果并更新界面
                location_list = results.boxes.xyxy.tolist()
                self.location_list = [list(map(int, e)) for e in location_list]
                cls_list = results.boxes.cls.tolist()
                self.cls_list = [int(i) for i in cls_list]
                conf_list = results.boxes.conf.tolist()

                # 更新检测结果
                self.draw_rect_and_tabel(results, frame)

    def send_at_command(self, cmd):
        if self._serial.isOpen():
            full_cmd = cmd + "\r\n"
            bytes_written = self._serial.write(full_cmd.encode())
            if bytes_written == len(full_cmd):
                self.ui.textBrowser.append(f'[发送成功] {cmd}')
            else:
                self.ui.textBrowser.append(f'[发送失败] {cmd}')

    def read_serial_data(self):
        while self._serial.bytesAvailable():
            data_bytes = self._serial.readAll().data()
            try:
                new_data = data_bytes.decode('utf-8', errors='replace')
                self.buffer += new_data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.parse_serial_data(line.strip())
            except Exception as e:
                print(f"解码错误: {e}")
                self.buffer = ""

    def parse_serial_data(self, line):
        if not line:
            return

        try:
            if "+MQTTSUBRECV" in line and "params" in line:
                json_start = line.find('{')
                json_str = line[json_start:]
                json_data = json.loads(json_str)

                params = json_data.get("params", {})
                if "DCYX" in params:
                    self.weizhi_X = params["DCYX"]
                    self.ui.weizhi_X.setText(f"{self.weizhi_X}")
                if "DCYY" in params:
                    self.weizhi_Y = params["DCYY"]
                    self.ui.weizhi_Y.setText(f"{self.weizhi_Y}")
                if "AX" in params:
                    self.weizhi_A_x = params["AX"]
                    self.ui.weizhi_A_x.setText(f"{self.weizhi_A_x}")
                if "AY" in params:
                    self.weizhi_A_y = params["AY"]
                    self.ui.weizhi_A_y.setText(f"{self.weizhi_A_y}")
                if "AZ" in params:
                    self.weizhi_A_z = params["AZ"]
                    self.ui.weizhi_A_z.setText(f"{self.weizhi_A_z}")

                print(
                    f"收到 MQTT 数据: X={self.weizhi_X}, Y={self.weizhi_Y}, A_x={self.weizhi_A_x}, A_y={self.weizhi_A_y}, A_z={self.weizhi_A_z}")
            else:
                pattern = r'(DCYX|DCYY|Angle)[:\s]*([+-]?\d+)'
                matches = re.findall(pattern, line)

                for keyword, val in matches:
                    if keyword == 'DCYX':
                        self.weizhi_X = val
                        self.ui.weizhi_X.setText(f"{val}")
                    elif keyword == 'DCYY':
                        self.weizhi_Y = val
                        self.ui.weizhi_Y.setText(f"{val}")
                    elif keyword == 'Angle':
                        self.weizhi_A = val
                        self.ui.weizhi_A.setText(f"{val}")
                print(f"收到数据: {line} -> X={self.weizhi_X}, Y={self.weizhi_Y}, A={self.weizhi_A}")
        except Exception as e:
            print(f"解析数据错误: {e}")

    def getAvailablePorts(self):
        self._ports.clear()
        self.ui.comboBox11.clear()
        for info in QSerialPortInfo.availablePorts():
            self._ports[info.portName()] = info
            self.ui.comboBox11.addItem(info.portName())

    def closeEvent(self, event):
        if self._serial.isOpen():
            self._serial.close()
        super(QMainWindow, self).closeEvent(event)

    def on_buttonConnect_clicked(self):
        if self._serial.isOpen():
            self._serial.close()
            self.ui.textBrowser.append('串口已关闭')
            self.ui.pushButton.setText('打开串口')
            return
        name = self.ui.comboBox11.currentText()
        if not name or name not in self._ports:
            QMessageBox.critical(self, '错误', '没有选择有效的串口')
            return
        port = self._ports[name]
        self._serial.setPortName(port.systemLocation())
        try:
            baud_rate = getattr(QSerialPort, 'Baud' + self.ui.comboBoxbaud.currentText(), None)
            parity = getattr(QSerialPort, self.ui.comboBoxparity.currentText() + 'Parity', None)
            data_bits = getattr(QSerialPort, 'Data' + self.ui.comboBoxdata.currentText(), None)
            stop_bits = getattr(QSerialPort, self.ui.comboBoxstop.currentText(), None)
            if None in (baud_rate, parity, data_bits, stop_bits):
                raise AttributeError('无效的串口参数')
            self._serial.setBaudRate(baud_rate)
            self._serial.setParity(parity)
            self._serial.setDataBits(data_bits)
            self._serial.setStopBits(stop_bits)
        except AttributeError as e:
            QMessageBox.critical(self, '配置错误', f'无效的串口参数: {str(e)}')
            return
        self._serial.setFlowControl(QSerialPort.NoFlowControl)

        if self._serial.open(QIODevice.ReadWrite):
            self.ui.textBrowser.append('打开串口成功')
            self.ui.pushButton.setText('关闭串口')
            self.current_cmd_index = 0
            self.start_send_at_commands()
        else:
            QMessageBox.critical(self, '错误', f'打开串口失败: {self._serial.errorString()}')

    def signalconnect(self):
        self.ui.PicBtn.clicked.connect(self.open_img)
        self.ui.comboBox.activated.connect(self.combox_change)
        self.ui.VideoBtn.clicked.connect(self.vedio_show)
        self.ui.CapBtn.clicked.connect(self.camera_show)
        self.ui.SaveBtn.clicked.connect(self.save_detect_video)
        self.ui.ExitBtn.clicked.connect(QCoreApplication.quit)
        self.ui.FilesBtn.clicked.connect(self.detact_batch_imgs)

    def initMain(self):
        self.show_width = 770
        self.show_height = 480
        self.org_path = None
        self.is_camera_open = False
        self.cap = None
        self.model = YOLO(Config.model_path, task='detect')
        self.model(np.zeros((48, 48, 3)))  # 预热加载模型
        self.fontC = ImageFont.truetype("Font/platech.ttf", 25, 0)
        self.colors = tools.Colors()
        self.timer_camera = QTimer()
        self.timer_save_video = QTimer()
        self.ui.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.ui.tableWidget.verticalHeader().setDefaultSectionSize(40)
        self.ui.tableWidget.setColumnWidth(0, 80)
        self.ui.tableWidget.setColumnWidth(1, 200)
        self.ui.tableWidget.setColumnWidth(2, 150)
        self.ui.tableWidget.setColumnWidth(3, 90)
        self.ui.tableWidget.setColumnWidth(4, 230)
        self.ui.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.ui.tableWidget.verticalHeader().setVisible(False)
        self.ui.tableWidget.setAlternatingRowColors(True)

    def open_img(self):
        if self.cap:
            self.video_stop()
            self.is_camera_open = False
            self.ui.CaplineEdit.setText('摄像头未开启')
            self.cap = None

        file_path, _ = QFileDialog.getOpenFileName(self, '打开图片', './', "Image files (*.jpg *.jpeg *.png)")
        if not file_path:
            return

        self.ui.comboBox.setDisabled(False)
        self.org_path = file_path
        self.org_img = tools.img_cvread(self.org_path)

        t1 = time.time()
        self.results = self.model(self.org_path)[0]
        t2 = time.time()
        take_time_str = '{:.3f} s'.format(t2 - t1)
        self.ui.time_lb.setText(take_time_str)

        location_list = self.results.boxes.xyxy.tolist()
        self.location_list = [list(map(int, e)) for e in location_list]
        cls_list = self.results.boxes.cls.tolist()
        self.cls_list = [int(i) for i in cls_list]
        self.conf_list = self.results.boxes.conf.tolist()
        self.conf_list = ['%.2f %%' % (each * 100) for each in self.conf_list]

        total_nums = len(location_list)
        cls_percents = []
        for i in range(6):
            res = self.cls_list.count(i) / total_nums if total_nums else 0
            cls_percents.append(res)
        self.set_percent(cls_percents)
        now_img = self.results.plot()
        self.draw_img = now_img

        self.img_width, self.img_height = self.get_resize_size(now_img)
        resize_cvimg = cv2.resize(now_img, (self.img_width, self.img_height))
        pix_img = tools.cvimg_to_qpiximg(resize_cvimg)
        self.ui.label_show.setPixmap(pix_img)
        self.ui.label_show.setAlignment(Qt.AlignCenter)
        self.ui.PiclineEdit.setText(self.org_path)
        target_nums = len(self.cls_list)
        self.ui.label_nums.setText(str(target_nums))

        choose_list = ['全部']
        target_names = [Config.names[id] + '_' + str(index) for index, id in enumerate(self.cls_list)]
        choose_list += target_names
        self.ui.comboBox.clear()
        self.ui.comboBox.addItems(choose_list)

        if target_nums >= 1:
            self.ui.type_lb.setText(Config.CH_names[self.cls_list[0]])
            self.ui.label_conf.setText(str(self.conf_list[0]))
            self.ui.label_xmin.setText(str(self.location_list[0][0]))
            self.ui.label_ymin.setText(str(self.location_list[0][1]))
            self.ui.label_xmax.setText(str(self.location_list[0][2]))
            self.ui.label_ymax.setText(str(self.location_list[0][3]))
        else:
            self.ui.type_lb.setText('')
            self.ui.label_conf.setText('')
            self.ui.label_xmin.setText('')
            self.ui.label_ymin.setText('')
            self.ui.label_xmax.setText('')
            self.ui.label_ymax.setText('')

        self.ui.tableWidget.setRowCount(0)
        self.ui.tableWidget.clearContents()
        self.tabel_info_show(self.location_list, self.cls_list, self.conf_list, path=self.org_path)

    def detact_batch_imgs(self):
        if self.cap:
            self.video_stop()
            self.is_camera_open = False
            self.ui.CaplineEdit.setText('摄像头未开启')
            self.cap = None

        directory = QFileDialog.getExistingDirectory(self, "选取文件夹", "./")
        if not directory:
            return
        self.org_path = directory
        img_suffix = ['jpg', 'png', 'jpeg', 'bmp']
        for file_name in os.listdir(directory):
            full_path = os.path.join(directory, file_name)
            if os.path.isfile(full_path) and file_name.split('.')[-1].lower() in img_suffix:
                img_path = full_path
                self.org_img = tools.img_cvread(img_path)
                t1 = time.time()
                self.results = self.model(img_path)[0]
                t2 = time.time()
                take_time_str = '{:.3f} s'.format(t2 - t1)
                self.ui.time_lb.setText(take_time_str)

                location_list = self.results.boxes.xyxy.tolist()
                self.location_list = [list(map(int, e)) for e in location_list]
                cls_list = self.results.boxes.cls.tolist()
                self.cls_list = [int(i) for i in cls_list]
                self.conf_list = self.results.boxes.conf.tolist()
                self.conf_list = ['%.2f %%' % (each * 100) for each in self.conf_list]
                total_nums = len(location_list)
                cls_percents = []
                for i in range(6):
                    res = self.cls_list.count(i) / total_nums if total_nums else 0
                    cls_percents.append(res)
                self.set_percent(cls_percents)
                now_img = self.results.plot()
                self.draw_img = now_img
                self.img_width, self.img_height = self.get_resize_size(now_img)
                resize_cvimg = cv2.resize(now_img, (self.img_width, self.img_height))
                pix_img = tools.cvimg_to_qpiximg(resize_cvimg)
                self.ui.label_show.setPixmap(pix_img)
                self.ui.label_show.setAlignment(Qt.AlignCenter)
                self.ui.PiclineEdit.setText(img_path)
                target_nums = len(self.cls_list)
                self.ui.label_nums.setText(str(target_nums))
                print("检测到的类别：", [Config.CH_names[cls] for cls in self.cls_list])
                choose_list = ['全部']
                target_names = [Config.names[id] + '_' + str(index) for index, id in enumerate(self.cls_list)]
                choose_list += target_names
                self.ui.comboBox.clear()
                self.ui.comboBox.addItems(choose_list)

                if target_nums >= 1:
                    self.ui.type_lb.setText(Config.CH_names[self.cls_list[0]])
                    self.ui.label_conf.setText(str(self.conf_list[0]))
                    self.ui.label_xmin.setText(str(self.location_list[0][0]))
                    self.ui.label_ymin.setText(str(self.location_list[0][1]))
                    self.ui.label_xmax.setText(str(self.location_list[0][2]))
                    self.ui.label_ymax.setText(str(self.location_list[0][3]))
                else:
                    self.ui.type_lb.setText('')
                    self.ui.label_conf.setText('')
                    self.ui.label_xmin.setText('')
                    self.ui.label_ymin.setText('')
                    self.ui.label_xmax.setText('')
                    self.ui.label_ymax.setText('')
                self.tabel_info_show(self.location_list, self.cls_list, self.conf_list, path=img_path)
                self.ui.tableWidget.scrollToBottom()
                QApplication.processEvents()

    def draw_rect_and_tabel(self, results, img):
        now_img = img.copy()
        location_list = results.boxes.xyxy.tolist()
        self.location_list = [list(map(int, e)) for e in location_list]
        cls_list = results.boxes.cls.tolist()
        self.cls_list = [int(i) for i in cls_list]
        self.conf_list = results.boxes.conf.tolist()
        self.conf_list = ['%.2f %%' % (each * 100) for each in self.conf_list]

        for loacation, type_id, conf in zip(self.location_list, self.cls_list, self.conf_list):
            type_id = int(type_id)
            color = self.colors(int(type_id), True)
            now_img = tools.drawRectBox(now_img, loacation, Config.CH_names[type_id], self.fontC, color)

        self.img_width, self.img_height = self.get_resize_size(now_img)
        resize_cvimg = cv2.resize(now_img, (self.img_width, self.img_height))
        pix_img = tools.cvimg_to_qpiximg(resize_cvimg)
        self.ui.label_show.setPixmap(pix_img)
        self.ui.label_show.setAlignment(Qt.AlignCenter)
        self.ui.PiclineEdit.setText(self.org_path)
        target_nums = len(self.cls_list)
        self.ui.label_nums.setText(str(target_nums))
        if target_nums >= 1:
            self.ui.type_lb.setText(Config.CH_names[self.cls_list[0]])
            self.ui.label_conf.setText(str(self.conf_list[0]))
            self.ui.label_xmin.setText(str(self.location_list[0][0]))
            self.ui.label_ymin.setText(str(self.location_list[0][1]))
            self.ui.label_xmax.setText(str(self.location_list[0][2]))
            self.ui.label_ymax.setText(str(self.location_list[0][3]))
        else:
            self.ui.type_lb.setText('')
            self.ui.label_conf.setText('')
            self.ui.label_xmin.setText('')
            self.ui.label_ymin.setText('')
            self.ui.label_xmax.setText('')
            self.ui.label_ymax.setText('')
        self.ui.tableWidget.setRowCount(0)
        self.ui.tableWidget.clearContents()
        self.tabel_info_show(self.location_list, self.cls_list, self.conf_list, path=self.org_path)
        return now_img

    def combox_change(self):
        com_text = self.ui.comboBox.currentText()
        if com_text == '全部':
            cur_box = self.location_list
            cur_img = self.results.plot()
            self.ui.type_lb.setText(Config.CH_names[self.cls_list[0]])
            self.ui.label_conf.setText(str(self.conf_list[0]))
        else:
            index = int(com_text.split('_')[-1])
            cur_box = [self.location_list[index]]
            cur_img = self.results[index].plot()
            self.ui.type_lb.setText(Config.CH_names[self.cls_list[index]])
            self.ui.label_conf.setText(str(self.conf_list[index]))

        self.ui.label_xmin.setText(str(cur_box[0][0]))
        self.ui.label_ymin.setText(str(cur_box[0][1]))
        self.ui.label_xmax.setText(str(cur_box[0][2]))
        self.ui.label_ymax.setText(str(cur_box[0][3]))

        resize_cvimg = cv2.resize(cur_img, (self.img_width, self.img_height))
        pix_img = tools.cvimg_to_qpiximg(resize_cvimg)
        self.ui.label_show.clear()
        self.ui.label_show.setPixmap(pix_img)
        self.ui.label_show.setAlignment(Qt.AlignCenter)

    def get_video_path(self):
        file_path, _ = QFileDialog.getOpenFileName(self, '打开视频', './', "Video files (*.avi *.mp4 *.mpeg *.mov)")
        if not file_path:
            return None
        self.org_path = file_path
        self.ui.VideolineEdit.setText(file_path)
        return file_path

    def video_start(self):
        self.ui.tableWidget.setRowCount(0)
        self.ui.tableWidget.clearContents()
        self.ui.comboBox.clear()
        self.timer_detection.start()
        self.timer_camera.start(1)
        self.timer_camera.timeout.connect(self.open_frame)

    def tabel_info_show(self, locations, clses, confs, path=None):
        for location, cls, conf in zip(locations, clses, confs):
            row_count = self.ui.tableWidget.rowCount()
            self.ui.tableWidget.insertRow(row_count)
            item_id = QTableWidgetItem(str(row_count + 1))
            item_id.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            item_path = QTableWidgetItem(str(path))
            item_cls = QTableWidgetItem(str(Config.CH_names[cls]))
            item_cls.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            item_conf = QTableWidgetItem(str(conf))
            item_conf.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            item_location = QTableWidgetItem(str(location))
            self.ui.tableWidget.setItem(row_count, 0, item_id)
            self.ui.tableWidget.setItem(row_count, 1, item_path)
            self.ui.tableWidget.setItem(row_count, 2, item_cls)
            self.ui.tableWidget.setItem(row_count, 3, item_conf)
            self.ui.tableWidget.setItem(row_count, 4, item_location)
        self.ui.tableWidget.scrollToBottom()

    def video_stop(self):
        if self.cap:
            self.cap.release()
        self.timer_camera.stop()

    def open_frame(self):
        ret, now_img = self.cap.read()
        if ret:
            try:
                t1 = time.time()
                results = self.model(now_img)[0]
                t2 = time.time()
                take_time_str = '{:.3f} s'.format(t2 - t1)
                self.ui.time_lb.setText(take_time_str)

                location_list = results.boxes.xyxy.tolist()
                self.location_list = [list(map(int, e)) for e in location_list]
                cls_list = results.boxes.cls.tolist()
                self.cls_list = [int(i) for i in cls_list]
                conf_list = results.boxes.conf.tolist()

                filtered_locations = []
                filtered_cls_list = []
                filtered_conf_list = []
                sendnum_list = []
                for i in range(len(conf_list)):
                    if conf_list[i] >= 0.7:
                        sendnum_list.append(self.cls_list[i])
                    filtered_locations.append(self.location_list[i])
                    filtered_cls_list.append(self.cls_list[i])
                    filtered_conf_list.append(conf_list[i])

                self.location_list = filtered_locations
                self.cls_list = filtered_cls_list
                self.conf_list = filtered_conf_list

                textsend = ','.join(map(str, sendnum_list))
                if textsend:
                    self.HUOlistID = 0
                    self.HUO_start_send_at_commands()

                total_nums = len(self.location_list)
                cls_percents = []
                for i in range(6):
                    res = self.cls_list.count(i) / total_nums if total_nums else 0
                    cls_percents.append(res)
                self.set_percent(cls_percents)

                now_img = results.plot()

                self.img_width, self.img_height = self.get_resize_size(now_img)
                resize_cvimg = cv2.resize(now_img, (self.img_width, self.img_height))
                pix_img = tools.cvimg_to_qpiximg(resize_cvimg)
                self.ui.label_show.setPixmap(pix_img)
                self.ui.label_show.setAlignment(Qt.AlignCenter)
                target_nums = len(self.cls_list)
                self.ui.label_nums.setText(str(target_nums))
                choose_list = ['全部']
                target_names = [Config.names[id] + '_' + str(index) for index, id in enumerate(self.cls_list)]
                choose_list += target_names
                self.ui.comboBox.clear()
                self.ui.comboBox.addItems(choose_list)
                if target_nums >= 1:
                    self.ui.type_lb.setText(Config.CH_names[self.cls_list[0]])
                    self.ui.label_conf.setText(str(self.conf_list[0]))
                    self.ui.label_xmin.setText(str(self.location_list[0][0]))
                    self.ui.label_ymin.setText(str(self.location_list[0][1]))
                    self.ui.label_xmax.setText(str(self.location_list[0][2]))
                    self.ui.label_ymax.setText(str(self.location_list[0][3]))
                else:
                    self.ui.type_lb.setText('')
                    self.ui.label_conf.setText('')
                    self.ui.label_xmin.setText('')
                    self.ui.label_ymin.setText('')
                    self.ui.label_xmax.setText('')
                    self.ui.label_ymax.setText('')
                self.tabel_info_show(self.location_list, self.cls_list, self.conf_list, path=self.org_path)

            except Exception as e:
                print(f"Error occurred: {str(e)}")
        else:
            self.cap.release()
            self.timer_camera.stop()

    def send_data_over_serial(self, textsend):
        send_str = textsend + '\n'
        print(f"准备发送的数据: {send_str}")
        if self._serial.isOpen():
            bytes_written = self._serial.write(send_str.encode())
            if bytes_written == len(send_str):
                self.ui.textBrowser.append(f'发送数据: {send_str}')
            else:
                self.ui.textBrowser.append('发送数据失败')
        else:
            self.ui.textBrowser.append('串口未打开')

    def vedio_show(self):
        if self.is_camera_open:
            self.is_camera_open = False
            self.ui.CaplineEdit.setText('摄像头未开启')

        video_path = self.get_video_path()
        if not video_path:
            return None
        self.cap = cv2.VideoCapture(video_path)
        self.video_start()
        self.ui.comboBox.setDisabled(True)

    def camera_show(self):
        self.is_camera_open = not self.is_camera_open
        if self.is_camera_open:
            self.ui.CaplineEdit.setText('正在连接RTSP...')
            rtsp_url = "rtsp://admin:admin@192.168.0.198/12"
            self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
            if not self.cap.isOpened():
                self.ui.CaplineEdit.setText('连接RTSP地址')
                self.is_camera_open = False
                return
            self.ui.CaplineEdit.setText('RTSP连接成功')
            self.video_start()
            self.ui.comboBox.setDisabled(True)
        else:
            self.ui.CaplineEdit.setText('摄像头未开放')
            self.ui.label_show.clear()
            if self.cap:
                self.cap.release()
            self.timer_camera.stop()

    def get_resize_size(self, img):
        _img = img.copy()
        img_height, img_width, _ = _img.shape
        ratio = img_width / img_height
        if ratio >= self.show_width / self.show_height:
            self.img_width = self.show_width
            self.img_height = int(self.img_width / ratio)
        else:
            self.img_height = self.show_height
            self.img_width = int(self.img_height * ratio)
        return self.img_width, self.img_height

    def save_detect_video(self):
        return

    def update_process_bar(self, cur_num, total):
        if cur_num == 1:
            self.progress_bar = ProgressBar(self)
            self.progress_bar.show()
        if cur_num >= total:
            self.progress_bar.close()
            QMessageBox.about(self, '提示', '视频保存成功!\n文件在{}目录下'.format(Config.save_path))
            return
        if not self.progress_bar.isVisible():
            self.btn2Thread_object.stop()
            return
        value = int(cur_num / total * 100)
        self.progress_bar.setValue(cur_num, total, value)
        QApplication.processEvents()

    def set_percent(self, probs):
        items = [self.ui.progressBar, self.ui.progressBar_2, self.ui.progressBar_3,
                 self.ui.progressBar_4, self.ui.progressBar_5, self.ui.progressBar_6]
        labels = [self.ui.label_20, self.ui.label_21, self.ui.label_22,
                  self.ui.label_23, self.ui.label_24, self.ui.label_25]
        prob_values = [round(each * 100) for each in probs]
        label_values = ['{:.1f}%'.format(each * 100) for each in probs]
        for i in range(len(probs)):
            items[i].setValue(prob_values[i])
            labels[i].setText(label_values[i])


class btn2Thread(QThread):
    update_ui_signal = pyqtSignal(int, int)

    def __init__(self, path, model, com_text):
        super(btn2Thread, self).__init__()
        self.org_path = path
        self.model = model
        self.com_text = com_text
        self.colors = tools.Colors()
        self.is_running = True

    def run(self):
        cap = cv2.VideoCapture(self.org_path)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = cap.get(cv2.CAP_PROP_FPS)
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        fileName = os.path.basename(self.org_path)
        name, end_name = fileName.split('.')
        save_name = name + '_detect_result.avi'
        save_video_path = os.path.join(Config.save_path, save_name)
        out = cv2.VideoWriter(save_video_path, fourcc, fps, size)

        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print("[INFO] 视频总帧数：{}".format(total))
        cur_num = 0

        while cap.isOpened() and self.is_running:
            cur_num += 1
            print('当前第{}帧，总帧数{}'.format(cur_num, total))
            ret, frame = cap.read()
            if ret:
                results = self.model(frame)[0]
                frame = results.plot()
                out.write(frame)
                self.update_ui_signal.emit(cur_num, total)
            else:
                break
        cap.release()
        out.release()

    def stop(self):
        self.is_running = False


if __name__ == "__main__":
    QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
