# AdapJ 自适应扩展雅可比控制器复现指南

本指南详细说明如何复现论文《AdapJ: An Adaptive Extended Jacobian Controller for Soft Manipulators》中提出的自适应扩展雅可比控制器（AdapJ）的实验工作，包括硬件连接、代码运行步骤以及需要客制化的部分。代码仓库位于 [https://github.com/xuyangren/AdapJ](https://github.com/xuyangren/AdapJ)。以下内容基于论文和代码仓库的分析，提供清晰的复现步骤。

---

## 项目概述

AdapJ 是一种针对软体机械臂的自适应控制器，结合了传统雅可比控制器的简洁性和递归神经网络（RNN）的参数灵活性，通过在线更新和松弛参数耦合约束来应对软体机器人的非线性和迟滞问题。复现工作包括以下部分：
1. **硬件设置**：搭建一个气动软体机械臂实验平台，配备光学跟踪系统和控制设备。
2. **软件环境**：配置 Python 环境，运行控制器代码。
3. **实验执行**：初始化控制器、运行轨迹跟踪实验，并验证控制器在不同频率、刚度和外部扰动下的适应性。

---

## 硬件设置

### 1. 软体机械臂制造
根据论文第 III-B 节（页面 6），软体机械臂采用以下设计：
- **材料**：Ecoflex 00-30（Smooth-On, Macungie, PA）。
- **结构**：每个模块长约 45 毫米，内部有四个沿圆周方向以 90° 间隔排列的气腔（见图 6-(C)）。
- **约束**：使用棉线包裹机械臂以限制径向膨胀。
- **刚度调节**：在机械臂中心插入弹簧（线径分别为 0.4 毫米、0.6 毫米、0.8 毫米，分别对应 Robot 2、Robot 1、Robot 3）。
- **末端跟踪**：在机械臂末端固定一个光学跟踪球，用于位置测量。

**客制化建议**：
- 如果无法获取 Ecoflex 00-30，可使用其他软硅胶材料（如 Dragon Skin 系列），但需确保材料的柔性和气密性。
- 弹簧线径可根据实验需求调整，以测试不同刚度下的控制器性能。

### 2. 实验平台搭建
实验平台如图 6-(B) 所示，包含以下组件：
- **光学跟踪系统**：
  - 设备：6 台 VICON Bonita 光学跟踪相机。
  - 功能：实时跟踪机械臂末端的光学跟踪球，获取机器人状态 \( s \in \mathbb{R}^2 \)（末端位置）。
  - 连接：通过以太网交换机与数据采集计算机（Vicon PC）通信。
- **气动控制系统**：
  - 气源：空气压缩机。
  - 阀门：Camozzi K8P-0-E522-0 压力控制阀，控制四个气腔的压力。
  - 控制器：Arduino MEGA 板，用于接收控制信号并调节阀门。
- **控制计算机**：
  - 配置：Ubuntu 20.04，CPU i5-12500H，GPU RTX 3050。
  - 功能：运行控制器代码，处理光学跟踪数据，发送控制信号到 Arduino。
  - 连接：通过以太网交换机与 Vicon PC 通信，通过 USB 与 Arduino 通信。

**客制化建议**：
- 如果没有 VICON 系统，可使用其他运动捕捉系统（如 OptiTrack）或基于视觉的跟踪方法（如 OpenCV + 摄像头），但需修改代码以适配新的位置数据格式。
- Arduino MEGA 可替换为其他微控制器（如 Raspberry Pi），需更新串口通信代码。
- 气动阀门可替换为其他型号，但需确保支持精确压力控制。

---

## 软件环境配置

### 1. 依赖环境
根据代码仓库和论文，推荐以下软件环境：
- **操作系统**：Ubuntu 20.04（或其他 Linux 发行版，Windows/MacOS 需适配）。
- **Python 版本**：Python 3.8 或以上。
- **依赖库**：
  ```bash
  pip install torch numpy matplotlib scipy
  ```
- **其他工具**：
  - VICON 数据采集软件（若使用 VICON 系统）。
  - Arduino IDE（用于烧录 Arduino 固件）。
  - PyTorch（用于优化和神经网络训练，论文提到使用 PyTorch）。

### 2. 克隆代码仓库
```bash
git clone https://github.com/xuyangren/AdapJ.git
cd AdapJ
```

**目录结构**（假设基于典型机器人控制项目）：
- `src/`：控制器实现代码（AdapJ、RNN、MPC、Jacobian）。
- `data/`：存储训练和测试数据（例如 5000 个样本）。
- `scripts/`：实验脚本（如轨迹跟踪、初始化）。
- `firmware/`：Arduino 固件代码。
- `config/`：配置文件（如机器人参数、控制频率）。

**客制化检查**：
- 检查仓库中的 `README.md` 或 `requirements.txt`，确保安装所有依赖。
- 如果缺少某些文件（如固件代码），需根据论文描述自行实现（见下文）。

---

## 代码运行步骤

### 1. Arduino 固件烧录
Arduino MEGA 用于控制气动阀门，接收控制计算机发送的动作 \( a = [a_h, a_v] \in \mathbb{R}^2 \)，并将其转换为四个气腔的压力值 \( a_I, a_{II}, a_{III}, a_{IV} \)。根据论文公式（页面 6）：
\[
a_I = \max\{0, a_h\}, \quad a_{II} = \max\{0, -a_h\}, \quad a_{III} = \max\{0, a_v\}, \quad a_{IV} = \max\{0, -a_v\}
\]

**固件实现**（伪代码，需在仓库中查找或自行实现）：
```cpp
#include <Serial.h>

float a_h, a_v;
float a_I, a_II, a_III, a_IV;

void setup() {
  Serial.begin(9600);
  // 初始化阀门引脚
  pinMode(VALVE_I_PIN, OUTPUT);
  pinMode(VALVE_II_PIN, OUTPUT);
  pinMode(VALVE_III_PIN, OUTPUT);
  pinMode(VALVE_IV_PIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    // 读取 a_h, a_v（例如通过串口接收两个浮点数）
    a_h = Serial.parseFloat();
    a_v = Serial.parseFloat();
    
    // 计算气腔压力
    a_I = max(0, a_h);
    a_II = max(0, -a_h);
    a_III = max(0, a_v);
    a_IV = max(0, -a_v);
    
    // 控制阀门（假设 PWM 控制）
    analogWrite(VALVE_I_PIN, map(a_I, -1, 1, 0, 255));
    analogWrite(VALVE_II_PIN, map(a_II, -1, 1, 0, 255));
    analogWrite(VALVE_III_PIN, map(a_III, -1, 1, 0, 255));
    analogWrite(VALVE_IV_PIN, map(a_IV, -1, 1, 0, 255));
  }
}
```

**步骤**：
1. 将 Arduino MEGA 连接到控制计算机。
2. 使用 Arduino IDE 打开固件代码（位于 `firmware/` 目录或自行实现）。
3. 调整 `VALVE_*_PIN` 引脚编号以匹配硬件连接。
4. 编译并上传固件到 Arduino。

**客制化**：
- 根据阀门型号调整 PWM 映射范围（例如 `map(a_I, -1, 1, 0, 255)`）。
- 如果使用其他微控制器，需重写控制逻辑。

### 2. 数据采集
AdapJ 初始化需要 100 个样本，RNN 和其他控制器需要 5000 个样本（页面 6）。数据采集采用电机随机抖动（motor babbling）策略。

**步骤**：
1. 运行数据采集脚本（假设为 `scripts/collect_data.py`）：
   ```bash
   python scripts/collect_data.py --num_samples 5000 --output data/samples.pkl
   ```
2. 脚本功能：
   - 通过串口向 Arduino 发送随机动作 \( a_h, a_v \in [-1, 1] \)。
   - 使用 VICON 系统记录机械臂末端位置 \( s \in \mathbb{R}^2 \)。
   - 保存动作和状态数据到 `data/samples.pkl`。
3. 验证数据：确保工作空间覆盖约 68.67 mm × 68.67 mm（页面 6，图 6-(D)）。

**客制化**：
- 修改 VICON 数据接口（可能需要 VICON SDK 或自定义协议）。
- 如果使用其他跟踪系统，需更新数据采集代码以解析位置数据。
- 调整随机动作范围以适配机械臂的工作空间。

### 3. 控制器初始化
AdapJ 使用 100 个样本初始化矩阵 \( A_0, A_1, B_0 \)，根据公式（页面 5）：
\[
\min_{A_*, B_*} \| a_{aj,t} - a_t \| \quad \text{s.t.} \quad a_{aj,t} = A_0 s_{t+1} + A_1 s_t + B_0 a_{t-1}
\]

**步骤**：
1. 运行初始化脚本（假设为 `scripts/initialize_adapj.py`）：
   ```bash
   python scripts/initialize_adapj.py --data data/samples.pkl --num_samples 100 --output config/adapj_params.pkl
   ```
2. 脚本功能：
   - 加载前 100 个样本。
   - 使用批量优化（batch optimization）求解 \( A_0, A_1, B_0 \)。
   - 保存初始矩阵到 `config/adapj_params.pkl`。
3. 验证初始矩阵（参考页面 8）：
   \[
   A_0 = \begin{bmatrix} 3.18 & 0.46 \\ -0.43 & 3.08 \end{bmatrix}, \quad A_1 = \begin{bmatrix} -2.11 & -0.12 \\ 0.13 & -1.93 \end{bmatrix}, \quad B_0 = \begin{bmatrix} -0.47 & -0.07 \\ 0.04 & -0.59 \end{bmatrix}
   \]

**客制化**：
- 如果机械臂状态或动作维度不同（例如 \( s \in \mathbb{R}^n, a \in \mathbb{R}^m \)），需调整矩阵维度。
- 优化算法（例如 PyTorch 的 Adam 优化器）可能需要调整学习率或迭代次数。

### 4. 轨迹跟踪实验
实验包括螺旋轨迹和三角形轨迹跟踪（页面 8，图 9），测试频率为 8 Hz、10 Hz、15 Hz，以及不同刚度的机器人（Robot 1、2、3）。

**步骤**：
1. 运行轨迹跟踪脚本（假设为 `scripts/run_trajectory.py`）：
   ```bash
   python scripts/run_trajectory.py --controller adapj --params config/adapj_params.pkl --trajectory spiral --freq 10 --output results/spiral_10hz.pkl
   ```
2. 脚本功能：
   - 加载控制器参数和目标轨迹（螺旋或三角形）。
   - 实时计算动作 \( a_{aj,t} = A_0 s_{t+1} + A_1 s_t + B_0 a_{t-1} \)。
   - 使用 Gauss-Newton 方法在线更新矩阵（页面 5，公式 11）。
   - 保存跟踪结果（实际轨迹、误差等）到 `results/`。
3. 验证结果：比较跟踪误差（参考表 III，页面 8）：
   - 螺旋轨迹（10 Hz）：AdapJ 误差 0.88 ± 0.48 mm。
   - 三角形轨迹（10 Hz）：AdapJ 误差 0.93 ± 0.55 mm。

**客制化**：
- 目标轨迹需根据机械臂工作空间调整（例如缩放螺旋半径）。
- 控制频率（8 Hz、15 Hz）需适配硬件响应速度。
- 如果测试其他控制器（RNN、MPC），需实现对应逻辑（参考页面 4-5）。

### 5. 适应性验证
验证 AdapJ 在不同频率、刚度和扰动下的适应性（页面 8-9）。

**步骤**：
1. **频率变化**：
   - 运行轨迹跟踪实验，设置频率为 8 Hz 和 15 Hz：
     ```bash
     python scripts/run_trajectory.py --controller adapj --trajectory spiral --freq 8 --output results/spiral_8hz.pkl
     python scripts/run_trajectory.py --controller adapj --trajectory spiral --freq 15 --output results/spiral_15hz.pkl
     ```
   - 验证误差（表 III，例如螺旋轨迹 15 Hz：1.56 ± 1.23 mm）。
2. **刚度变化**：
   - 更换弹簧（Robot 2：0.4 毫米，Robot 3：0.8 毫米）。
   - 重新采集 100 个样本并初始化矩阵。
   - 运行轨迹跟踪实验：
     ```bash
     python scripts/run_trajectory.py --controller adapj --params config/adapj_params_robot2.pkl --trajectory spiral --freq 10 --output results/spiral_robot2.pkl
     ```
   - 验证误差（表 III，例如 Robot 2 螺旋轨迹：1.00 ± 0.57 mm）。
3. **扰动实验**：
   - 运行 Lissajous 轨迹跟踪实验（页面 9，图 10）：
     ```bash
     python scripts/run_trajectory.py --controller adapj --trajectory lissajous --freq 10 --output results/lissajous.pkl
     ```
   - 在第二轮手动施加随机扰动（轻推机械臂）。
   - 在第四轮放置固定障碍物（靠近气腔 III）。
   - 验证控制器是否恢复跟踪精度（参考图 10-(A)）。

**客制化**：
- Lissajous 轨迹参数（振幅、频率）需根据工作空间调整。
- 扰动方式可根据实验条件修改（例如改变障碍物位置）。

---

## 客制化更改点

1. **硬件适配**：
   - 替换 VICON 系统：修改 `scripts/collect_data.py` 中的数据采集接口。
   - 替换气动阀门：更新 Arduino 固件中的控制逻辑。
   - 调整机械臂尺寸或材料：重新采集数据并验证工作空间。
2. **软件调整**：
   - 控制器维度：如果状态或动作维度改变，需修改 `src/adapj.py` 中的矩阵定义。
   - 优化参数：调整初始化和在线更新的学习率、迭代次数（`scripts/initialize_adapj.py` 和 `scripts/run_trajectory.py`）。
   - 轨迹生成：根据实验需求生成新的轨迹（例如在 `scripts/trajectories.py` 中添加）。
3. **实验场景**：
   - 添加新测试场景（如不同轨迹、扰动类型）。
   - 调整控制频率或物理参数（刚度、阻尼）以探索控制器极限。

---

## 注意事项

1. **安全性**：
   - 确保气动系统压力在安全范围内，避免机械臂或阀门损坏。
   - 小心操作光学跟踪相机，避免碰撞或遮挡。
2. **数据质量**：
   - 确保采集的 100 个样本覆盖工作空间的关键区域（参考图 6-(D)）。
   - 定期检查光学跟踪系统的校准。
3. **调试**：
   - 如果跟踪误差较大，检查矩阵初始化结果是否合理（参考页面 8 的示例值）。
   - 使用日志记录控制器参数变化（例如 \( A_0, A_1, B_0 \)），便于分析适应性。

---

## 结果可视化

运行实验后，可使用以下脚本生成轨迹和误差图（假设为 `scripts/plot_results.py`）：
```bash
python scripts/plot_results.py --data results/spiral_10hz.pkl --output figures/spiral_10hz.png
```

生成图表（参考图 9、10），比较目标轨迹和实际轨迹，计算平均误差和标准差。

**示例图表**（根据页面 7，图 7）：
```javascript
{
  "type": "line",
  "data": {
    "labels": ["0", "1", "2", "3", "4", "5"],
    "datasets": [
      {
        "label": "Target",
        "data": [54, 54, -72, -72, -72, -72],
        "borderColor": "#FF0000",
        "fill": false
      },
      {
        "label": "AdapJ",
        "data": [54, 54, -71.8, -72.1, -72, -72],
        "borderColor": "#800080",
        "fill": false
      },
      {
        "label": "RNN",
        "data": [54, 54, -70.5, -71.2, -71.8, -72],
        "borderColor": "#008000",
        "fill": false
      },
      {
        "label": "MPC",
        "data": [53.8, 53.5, -70.8, -71.5, -72, -72],
        "borderColor": "#FFA500",
        "fill": false
      }
    ]
  },
  "options": {
    "title": { "display": true, "text": "Trajectory Tracking (Simulation)" },
    "scales": {
      "yAxes": [{ "scaleLabel": { "display": true, "labelString": "Angle (°)" } }],
      "xAxes": [{ "scaleLabel": { "display": true, "labelString": "Time (s)" } }]
    }
  }
}
```

---

## 结论

通过以上步骤，您可以复现 AdapJ 控制器的实验，验证其在软体机械臂上的高精度和适应性。关键在于硬件平台的精确搭建、数据采集的质量以及代码的适当客制化。如果遇到问题，请参考论文中的实验细节（页面 6-9）或代码仓库的文档。

如需进一步支持，请联系代码仓库维护者或参考论文中的致谢部分（页面 10）。

--- 

**版本**：1.0  
**日期**：2025年6月11日  
**作者**：基于论文和代码仓库整理
