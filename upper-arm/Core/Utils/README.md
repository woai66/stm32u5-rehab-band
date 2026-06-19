# Core/Utils

本目录用于放置项目内可复用的轻量工具函数，避免在 IMU、EMG、评分和显示逻辑中重复实现基础算法。

当前包含：

- `Rehab_ClampFloat` / `Rehab_ClampInt32`：限幅。
- `Rehab_MapFloat`：线性映射。
- `Rehab_DeadbandFloat`：死区处理。
- `Rehab_FirstOrderFilter`：一阶低通滤波。
- `Rehab_MovingAverage`：滑动平均。
- `Rehab_RmsFloat`：RMS 计算。

建议只放无业务含义的通用函数。具体的肘关节角度、动作识别、肌电参与度等逻辑仍放在各自模块中。
