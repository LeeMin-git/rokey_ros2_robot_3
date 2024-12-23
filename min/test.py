from PyQt5.QtWidgets import QApplication,QWidget
import serial.serialutil
from ui_view import Ui_Form
import sys
import serial
import time
import threading

port = "/dev/ttyACM0"

class UI_image_test(QWidget):
    def __init__(self):
        super().__init__()
        self.ui=Ui_Form()
        self.ui.setupUi(self)
        try:
            self.ser = serial.Serial(port, 115200)
        except serial.serialutil.SerialException:
            print('선 뽑힘') 
            

    def send(self):
        data = self.ser.write(self.ui.lineedit_input_tic.text().encode('utf-8'))
        print(data)
        time.sleep(0.1)
    
    def stop(self):
        pass

    def debug(self):
        val = ser.read()
        print(val)
        # debug_list = [] 
        while True:
            self.ui.textbrowser_show.setText(val.decode('utf-8')+'\n')



def main():
    app=QApplication([])
    window=UI_image_test()
    # window.ui.textbrowser_show.setText(lst.decode('utf-8')+'\n')
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()


