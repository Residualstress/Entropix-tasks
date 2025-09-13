# BEACON LANDING 功能解释

1. 与ESP32摄像头连接，HttpAprilResolver模块识别apriltag，并通过回调函数更新apriltag像素坐标

注意：这个视频流稳定性很差！！！！！！！！！！！！需要解决WIFI连接稳定性问题

```python
    ip = "10.201.171.40"
    april_beacon = HttpAprilResolver(ip, callback=beacon_resolver_callback)
```

1. apriltag回调函数

回调函数接收BEACON中心像素位置，并转换为地面坐标（单位：m）

```python
def beacon_resolver_callback(center):
    target_pixel_position = center
    center_pixel_position = (160,120)
    rel_pixel_position = np.array(target_pixel_position) - np.array(center_pixel_position) # relative to center (pixel position)
    alpha = 320.1 # 相机标定的结果
    z = position_estimate[2]
    rel_position = tuple(rel_pixel_position / alpha * z) # 投影到实际距离
    rel_position[1] = -rel_position[1] # y轴翻转 
    global rel_postion2beacon # 相机中心相对于beacon的实际距离
    rel_postion2beacon = rel_position
```

    # resolution = [320 240]
    # cx = 146.35090
    # cy = 125.99543
    # alpha_x = 316.92703   alpha = f / d(pixel_width)
    # alpha_y = 326.38666

3. 核心功能：beacon_landing(scf)，飞机控制的主循环

设置两个PID控制器

state == 'ALIGNING'负责将飞机与BEACON对准

state == "DESCENDING"负责对准并下降

state == 'LANDING'负责降落

target_point=(0.0,0.0,0.25) 的x，y坐标是飞机相对信标的实际坐标（由视觉计算得到），z坐标由飞控给出

```python
def beacon_landing(scf):
    pid_aligning = PIDController2D(target_point=(0.0,0.0), kp=1.0, ki=0.3, kd=0.0, output_limit=0.2)
    pid_descending = PIDController3D(target_point=(0.0,0.0,0.25), kp=0.5, ki=0.2, kd=0.0, output_limit=0.2)

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(5.0)
        while True:
            print(f'state: {state}') # log current state
            if state == 'ALIGNING': # align with beacon
                aligning(mc, pid_aligning)
            elif state == "DESCENDING": # descending to 0.25m
                descending(mc, pid_descending)
            elif state == 'LANDING':
                mc.land()
                break

            time.sleep(0.05)
```

4. PID控制：

先通过pid控制将飞机稳在信标上方

注意这个PID控制有个致命缺陷，因为AprilTag的视觉识别回报虽然可靠，但是很不连续，如果一段时间不会报，PID就会持续使用之前的状态调整飞机，导致飞机持续往一个方向飞！！！

可以考虑使用发点的形式控制飞机！！！！！！！！！

```python
def aligning(mc: MotionCommander, pid: PIDController2D):

    if rel_postion2beacon is None:
        return
    
    vel_x, vel_y = pid.compute_velocity(rel_postion2beacon[0], rel_postion2beacon[1])

    print(f' m: {rel_postion2beacon[0]}, y: {rel_postion2beacon[1]}, vel_x: {vel_x}, vel_y: {vel_y}')

    mc.start_linear_motion(vel_x, vel_y, 0)

    # 如果稳定度满足要求，进入下降阶段
    stability = pid.get_stability() # 注意：stability的度量需要测试和调整
    print(stability)
    if stability > 0.9:
        global state
        state = 'DESCENDING'

```

实时监控历史数据的稳定性+误差，如果达标，进入下一阶段

这里明显有个bug，就是只监控了误差的稳定性，没有监控误差的大小！！！！！！！！！！！！！！！！！！！！！！！！

```python
    def get_stability(self):
        """
        衡量稳定性：
        - 根据误差标准差评估
        - 返回一个 0~1 的稳定性指标
        """
        if len(self.error_history_x) < 5:
            return 0.0

        std_x = np.std(self.error_history_x)
        std_y = np.std(self.error_history_y)
        std_z = np.std(self.error_history_z)

        stability_x = 1.0/(1.0+std_x)
        stability_y = 1.0/(1.0+std_y)
        stability_z = 1.0/(1.0+std_z)

        return (stability_x + stability_y + stability_z) / 3.0

```