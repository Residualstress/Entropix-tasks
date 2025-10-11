from uwb360_receiver import UWB360Receiver
import time, math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from ukf_filter import PositionUKF

uwb_position = None

def uwb_cb(pkt):
    global uwb_position
    uwb_position = pkt
    x,y,z = pkt
    print(f'x:{x} y:{y}')

ukf_filter = PositionUKF(dt=0.025, win_size=1) # 50Hz data rate
uwb = UWB360Receiver("/dev/ttyUSB0", uwb_cb, ukf_filter)
uwb.start()

# # ukf_filter = PositionUKF(dt=0.02, win_size=5) # 50Hz data rate
# uwb = UWBReceiver("COM6", 115200)
# uwb.start()

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="UWB Real-time Position")
plot = win.addPlot()
plot.setXRange(-2, 2)
plot.setYRange(-2, 2)
plot.setAspectLocked(True)
scatter = plot.plot([], [], pen=None, symbol='o', symbolBrush='r')

# 主循环用定时器驱动，不要 while True 死循环
def update():
    if uwb_position is not None:
        x,y,z = uwb_position
        scatter.setData([x], [y])
        

    # while uwb.has_data():
    #     x_meas, y_meas, x_filt, y_filt = uwb.pop_data()
    #     x = x_filt
    #     y = y_filt
    # scatter.setData([x], [y])

    # x,y = uwb.get_filtered_location()
    # print(f'x: {x}, y: {y}')
    # # print(uwb.thread.is_alive())
    # scatter.setData([x], [y])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(2) # ms

app.exec()