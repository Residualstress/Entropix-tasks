# usage_example.py
import pandas as pd
import numpy as np
from trans_est import OnlineTransformEstimator   # 若直接在同一文件则不用 import

df = pd.read_csv("data5.csv")  # 你上传的文件
est = OnlineTransformEstimator(window_size=100, use_ransac=True, ransac_tol=0.05)

res = est.batch_process_dataframe(df)
hist = res['history']
errs = res['errors']

print("RMSE:", np.sqrt((errs**2).mean()))
print("mean err:", errs.mean())
print("median err:", np.median(errs))
# 若要画图（可视化），用 matplotlib 绘制 hist.theta / hist.tx / hist.ty 随时间变化