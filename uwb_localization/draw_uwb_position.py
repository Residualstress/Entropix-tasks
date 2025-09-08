from uwb_receiver import UWBReceiver
# from range_angle_filter import RangeBearingAB
from ukf_filter import PositionUKF
import time, math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

ab_filter = PositionUKF(dt=0.02, win_size=2) # 50Hz data rate
uwb = UWBReceiver("COM7", 115200, ab_filter)
uwb.start()

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="UWB Real-time Position")
plot = win.addPlot()
plot.setXRange(-200, 200)
plot.setYRange(-200, 0)
plot.setAspectLocked(True)
scatter = plot.plot([], [], pen=None, symbol='o', symbolBrush='r')

# 主循环用定时器驱动，不要 while True 死循环
def update():
    if uwb.has_data():
        x_meas, y_meas, x_filt, y_filt = uwb.pop_data()
        x = x_filt
        y = y_filt
        scatter.setData([x], [y])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(2) # ms

app.exec_()
