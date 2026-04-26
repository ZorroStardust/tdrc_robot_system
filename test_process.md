**先做实机几何/方向验证，再做系统架构解耦。**


### 1. 电机理想编号与实际编号

目标是确定：

```python
MOTOR_INDEX_MAP = {1: ?, 2: ?, 3: ?, 4: ?}
```

测试方法：

每次只动一个理想电机，例如：

```text
motor 1 delta = +2 deg，其余 0
motor 2 delta = +2 deg，其余 0
motor 3 delta = +2 deg，其余 0
motor 4 delta = +2 deg，其余 0
```

观察实际哪根驱动丝/哪个方向发生变化，记录成表：

```text
ideal motor 1 -> real motor ?
ideal motor 2 -> real motor ?
ideal motor 3 -> real motor ?
ideal motor 4 -> real motor ?
```

这个测试应该先于方向测试。

---

### 2. 电机理想方向与实际方向

确定编号后，再测试方向：

```text
ideal motor i +delta
```

观察它对应的驱动丝是收紧还是放松，以及连续体弯曲方向是否符合模型预期。

最后得到：

```python
MOTOR_DIRECTION_MAP = {
    1: +1 or -1,
    2: +1 or -1,
    3: +1 or -1,
    4: +1 or -1,
}
```

注意：方向测试时 delta 不要太大，建议 `1~3 deg` 起步。

---

### 3. 关节空间建系验证

这一步测试：

```text
phi_a / theta_a
phi_c / theta_c
```

是否和你物理建系一致。

建议先只测近端：

```text
theta_a = 5 deg
phi_a = 0 / 90 / 180 / -90 deg
theta_c = 0
```

再只测远端：

```text
theta_a = 0
theta_c = 5 deg
phi_c = 0 / 90 / 180 / -90 deg
```

记录实际弯曲方向是否符合你定义的 x/y 方向。

---

## 你还可以补充测试什么

### 4. 零点重复性测试

流程：

```text
调直 -> Set Current As Zero
给一个小角度目标
回到 joint 全 0
观察是否回到调直状态
```

反复做 3~5 次，看是否存在明显漂移。

这能验证：

```text
zero_pulses 是否稳定
机械回差是否明显
驱动丝松弛是否影响零点
```

---

### 5. 正负对称性测试

例如：

```text
theta_a = +5 deg, phi_a = 0
theta_a = +5 deg, phi_a = 180
```

或者等价测试左右方向。

你要观察正反方向的弯曲幅度是否大致对称。
如果明显不对称，可能是：

```text
驱动丝预紧不同
零点不准
电机方向/映射仍有问题
机械摩擦或回差
```

---

### 6. 小角度线性区测试

连续体在小角度下应该比较接近线性。建议测试：

```text
theta = 2 / 4 / 6 / 8 / 10 deg
```

观察弯曲量是否近似单调、平滑。

这对后续轨迹生成很重要。

---

### 7. 安全边界测试

不是让你冲极限，而是先验证程序保护：

```text
theta_a 最大限制
theta_c 最大限制
电机 delta 最大限制
```

建议后续加：

```python
MOTOR_ANGLE_LIMIT_DEG
PULSE_DELTA_LIMIT
JOINT_THETA_LIMIT
```

避免 GUI 手滑。

---

## 关于后续两条主线

你的两条主线都对，但优先级建议这样排：

## 主线 A：先优化电机控制

这条应该先做，因为它决定实机是否平滑、安全。

建议分三层：

```text
目标点控制
→ 插值轨迹控制
→ streaming 实时控制
```

### A1. 目标点控制

当前本质是：

```text
joint target -> motor target -> pulse target -> PMAC move_joints
```

下一步要加：

```text
速度限制
加速度限制
最大 pulse step 限制
目标点到达判断
```

### A2. 插值轨迹控制

给定起点和终点，不要直接跳到目标，而是生成：

```text
q0, q1, q2, ..., qN
```

每个点再发给 PMAC。

先用简单线性插值即可：

```text
motor angle interpolation
```

后面再升级为：

```text
梯形速度
S 曲线
minimum jerk
```

### A3. Streaming / Omega7 遥操作

遥操作不要用“每条命令都必须执行”的思路，而要用：

```text
只保留最新目标
旧目标过期丢弃
固定频率下发
本地限速/滤波
```

也就是：

```text
Omega7 高频输入
→ target buffer
→ rate limiter
→ 50/100Hz executor
→ PMAC
```

这里一定要有：

```text
TTL
seq
timestamp
watchdog
```

如果超过一段时间没有新主手数据，就停止或保持当前位置。

---

## 主线 B：多 package 解耦

这条也重要，但建议在实机方向验证之后做。

原因是：
如果你现在还没确认 motor map、direction map、joint frame，过早把系统拆成多进程，会让调试复杂度上升。

建议顺序：

```text
1. 当前 GUI 直连 PMAC，完成硬件方向和建系验证
2. 把验证好的参数写入 tdrc.yaml / pmac.yaml
3. 再接入 robot_msgs + ZMQ
4. 用 demo sender 替代 GUI 直连
5. 最后让 GUI 只发消息，不直接控制 PMAC
```

最终结构应该变成：

```text
GUI / teleop / trajectory_node
        |
        v
motion_coordinator
        |
        v
pmac_bridge_node
        |
        v
pmac_sdk
        |
        v
PMAC
```

而当前脚本只是临时的：

```text
GUI -> model -> pmac_sdk -> PMAC
```

这是合理的实机验证工具，不需要长期保留为主控制架构。

---

## 我建议你的近期开发路线

### 第 1 阶段：实机参数确认

完成并记录：

```text
MOTOR_INDEX_MAP
MOTOR_DIRECTION_MAP
R_HOLE
D_SPOOL
joint frame convention
safe theta range
safe motor delta range
```

### 第 2 阶段：配置化

把这些从代码里移到：

```text
configs/tdrc.yaml
configs/pmac.yaml
configs/limits.yaml
```

### 第 3 阶段：轨迹生成基础版

先实现：

```text
motor angle linear interpolation
fixed update rate
max step limit
```

### 第 4 阶段：ZMQ 解耦

把当前 GUI 改成：

```text
GUI publishes JointTarget
tdrc_model_node publishes MotorTarget
motion_coordinator publishes filtered MotorTarget
pmac_bridge_node executes
```

### 第 5 阶段：遥操作 streaming

最后接 Omega7：

```text
omega7_node -> Cartesian/Joint target stream -> coordinator -> PMAC
```

## 最关键建议

现在不要急着追求完整架构。
你当前最重要的是得到一组可信的实机参数：

```text
电机编号
电机方向
关节坐标系
零点流程
安全范围
```

这些没确定前，轨迹、ZMQ、遥操作都会建立在不稳定基础上。
