from PyQt5.QtCore import pyqtSlot, QIODevice, QByteArray
from PyQt5.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt5.QtWidgets import QWidget, QMessageBox, QMainWindow

from untitled import Ui_MainWindow  # @UnresolvedImport


class Window(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super(Window, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self._serial = QSerialPort(self)
        self._ports = {}  # 确保 _ports 变量初始化
        self.getAvailablePorts()

        # 允许 textBrowser 输入
        self.textBrowser.setReadOnly(False)

        # 按钮连接
        self.pushButton.clicked.connect(self.on_buttonConnect_clicked)
        #self.pushButtonsend.clicked.connect(self.on_buttonSend_clicked)

    @pyqtSlot()
    def on_buttonConnect_clicked(self):
        if self._serial.isOpen():
            self._serial.close()
            self.textBrowser.append('串口已关闭')
            self.pushButton.setText('打开串口')
            return

        name = self.comboBox.currentText()
        if not name or name not in self._ports:
            QMessageBox.critical(self, '错误', '没有选择有效的串口')
            return
        port = self._ports[name]
        self._serial.setPortName(port.systemLocation())

        try:
            baud_rate = getattr(QSerialPort, 'Baud' + self.comboBoxbuad.currentText(), None)
            parity = getattr(QSerialPort, self.comboBoxparity.currentText() + 'Parity', None)
            data_bits = getattr(QSerialPort, 'Data' + self.comboBoxdata.currentText(), None)
            stop_bits = getattr(QSerialPort, self.comboBoxstop.currentText(), None)

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
            self.textBrowser.append('打开串口成功')
            self.pushButton.setText('关闭串口')
        else:
            QMessageBox.critical(self, '错误', f'打开串口失败: {self._serial.errorString()}')

    @pyqtSlot()
    def on_buttonSend_clicked(self):
        text = self.textBrowser.toPlainText().strip()

        if not self._serial.isOpen():
            self.textBrowser.append('串口未连接，发送失败')
            return

        if not text:
            self.textBrowser.append("发送内容为空")
            return

        try:
            text = QByteArray(text.encode('utf-8'))
        except Exception as e:
            self.textBrowser.append(f"编码错误: {e}")
            return

        bytes_written = self._serial.write(text)
        if bytes_written == -1:
            self.textBrowser.append(f"发送失败: {self._serial.errorString()}")

    def getAvailablePorts(self):
        self._ports.clear()
        self.comboBox.clear()
        for info in QSerialPortInfo.availablePorts():
            self._ports[info.portName()] = info
            self.comboBox.addItem(info.portName())

    def closeEvent(self, event):
        if self._serial.isOpen():
            self._serial.close()
        super(Window, self).closeEvent(event)
