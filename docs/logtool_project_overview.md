# Logtool 项目说明文档

> 版本：草稿  
> 适用工程：本仓库当前最新代码  
> 面向读者：算法工程师 / 机器人软件工程师 / 测试工程师

---

## 1. 项目定位与目标

`logtool` 是一套围绕「扫地机器人运行日志」构建的离线分析与可视化工具链，主要目标：

1. 定义一套稳定的**二进制日志协议**，覆盖清扫过程中的关键信息：
   - 运行状态：清扫状态、异常状态、运动状态；
   - 传感器：IMU、里程计、滑移判定、碰撞、悬空、红外、声呐、虚拟墙、悬崖、基站等；
   - 地图类：清扫栅格地图、探索栅格地图；
   - 轨迹与路径：实时轨迹、全局路径、局部路径、覆盖点、调试点集等；
   - 雷达：机器人本体激光扫描。

2. 提供一套**可重复的仿真数据生成链路**，基于 floorplan 生成：
   - 栅格地图 (`grid_map.npy`)；
   - 轨迹 (`trajectory.csv`)；
   - 完整清扫日志 (`fake_cleaning.log` + `fake_cleaning.log.idx`)。

3. 提供**离线检查与统计能力**：
   - `logdump`：解析 `.log` 文件，检查协议与内容；
   - `scanStats`：统计各 type 的帧率、长度，用于对齐设计文档与实际实现。

4. 提供一个**Web 可视化播放器**（`log_web_viewer` + 前端）：
   - 后端 HTTP 服务：从 `.log + .idx` 中按时间对齐读取多源数据；
   - 前端 2D 视图：地图、轨迹、激光、路径、机器人姿态，可交互缩放 / 平移 / 旋转；
   - 支持播放、拖拽时间轴、倍速、测量、录制等。

整体上，这是一个从「环境 floorplan → 仿真清扫 log → 检查/统计 → Web 回放」的闭环系统，同时为**接入真实机器人日志**预留好协议与能力。

---

## 2. 目录结构

工程根目录结构如下（核心部分）：

```text
.
├─ CMakeLists.txt          # C++ 工程构建脚本
├─ readme.md               # 快速上手说明
├─ include/                # 公共头文件(协议、IO、索引、统计、仿真)
├─ src/                    # C++ 源码，可执行程序
├─ tools/                  # Python 工具脚本 (floorplan → assets)
├─ web/                    # Web 前端 (HTML/CSS/JS)
└─ docs/                   # 文档目录 (可放本说明)
```

### 2.1 include/

- **`log_types.hpp`**  
  日志协议定义：
  - 日志数据结构：`Pose2D`、`GridMap`、`LaserScan`、`SensorData` 等；
  - 状态枚举：`CleanState`、`ExceptionCode`、`MotionState` 等；
  - type 常量：`TYPE_CLEAN_STATE / TYPE_SENSOR_DATA / TYPE_GRID_MAP / TYPE_ROBOT_LIDAR / ...`；
  - “type → 含义 / 预期频率 / 对应结构体” 的中心定义处。

- **`log_io.hpp`**  
  日志文件读写基础设施：
  - `FileHeader`：魔数、版本、flags；
  - `RecordHeader`：type / timestamp_ns / length / crc32；
  - `Record`：在内存中的抽象 (type + ts + payload)；
  - `LogWriter`：顺序写 `FileHeader + RecordHeader + payload`；
  - `LogReader`：顺序遍历 / 按 offset 读取记录。

- **`codec.hpp`**  
  通用二进制编解码器：
  - `appendScalar / readScalar`：POD 基本类型读写；
  - `appendVector / readVector`：容器读写；
  - `encodePayload<T>(const T&)`：结构体编码为 payload；
  - `parseRecordPayload<T>(const Record&, T&)`：payload 解码为结构体；
  - 通过“显式编码字段顺序”避免结构体 padding / 对齐差异。

- **`index.hpp`**  
  日志索引管理：
  - `IndexItem`：offset、type、timestamp_ns；
  - `IndexDB`：
    - `std::vector<IndexItem> all`：按时间排序的全局记录列表；
    - `std::unordered_map<uint32_t, std::vector<IndexItem>> by_type`：按 type 的索引视图；
    - 查询接口：
      - `nearest(type, ts)`：给定时间戳找到最近的某 type 记录。
  - `.idx` 文件的读写。

- **`stats.hpp`**  
  type 统计：
  - `TypeStats`：count / len(min/max/avg) / 时间跨度 / 帧率估算；
  - `accumulate()`：扫描日志构建统计信息；
  - `printStatsTable()`：输出人类可读的统计表。

- **`fake_data.hpp`**  
  仿真数据生成：
  - 从 `assets/grid_map.npy` 和 `assets/trajectory.csv` 读取地图与轨迹；
  - 模拟扫地机器人在该环境中的运动过程；
  - 按预设频率生成各类 type 的记录并写入 `LogWriter`；
  - 同时记录索引数据供 `IndexDB` 使用。

### 2.2 src/

- **`main.cpp` → 可执行 `logtool`**  
  仿真日志生成工具 + 自检（统计、demo 拖拽）。

- **`logdump.cpp` → 可执行 `logdump`**  
  日志解析与检查工具（对齐协议与实现）。

- **`log_web_viewer.cpp` → 可执行 `log_web_viewer`**  
  Web 可视化后端 HTTP server。

### 2.3 tools/

- **`floorplan_to_map_and_traj.py`**  
  - 输入：`data/floorplan.jpg` 等；
  - 输出：
    - `assets/grid_map.npy`（Int16 栅格地图）；
    - `assets/trajectory.csv`（轨迹点，含时间和姿态）。

- **`visualize_assets.py`**  
  - 读取 `grid_map.npy + trajectory.csv`；
  - 使用 Matplotlib 将地图与轨迹可视化，用于资产检查。

### 2.4 web/

- `index.html`：页面结构与控件布局；
- `style.css`：样式定义，整体深色 / RViz 风格；
- `app.js`：前端逻辑与绘图实现。

---

## 3. 日志协议与数据模型

### 3.1 文件结构

- **文件头 `FileHeader`**：
  - `uint32_t magic`：固定魔数，识别文件类型；
  - `uint32_t version`：协议版本；
  - `uint32_t flags`：预留字段，例如是否启用 CRC32。

- **记录头 `RecordHeader`**：
  - `uint32_t type`：记录类型；
  - `uint64_t timestamp_ns`：记录时间戳（纳秒）；
  - `uint32_t length`：payload 长度（字节）；
  - `uint32_t crc32`：校验码（目前可为 0，占位）。

- **记录体 `Record`**：
  - `RecordHeader header`；
  - `std::vector<uint8_t> data`：payload。

日志物理布局：

```text
[FileHeader]
[RecordHeader0][payload0]
[RecordHeader1][payload1]
...
```

### 3.2 关键结构体与语义（摘要）

> 细节以 `log_types.hpp` 为准，这里只列核心设计。

- **姿态与轨迹**

```cpp
struct Pose2D {
    float x;
    float y;
    float angle; // yaw
};

struct Pose2DSet {
    std::vector<Pose2D> pose_set;
};
```

- **IMU**

```cpp
struct Imu {
    float pitch;
    float roll;
    float yaw;
    float imu_acc_x;
    float imu_acc_y;
    float imu_acc_z;
    float imu_yaw_vel;
};
```

- **里程计**

```cpp
struct Odom {
    Pose2D raw_pose;   // 原始里程计
    Pose2D fuse_pose;  // 融合后的里程计
};
```

- **栅格地图**

```cpp
struct GridMap {
    int32_t width;
    int32_t height;
    float   resolution;
    float   origin_x;
    float   origin_y;
    std::vector<int8_t> data; // occupancy grid
};
```

- **激光扫描**

```cpp
struct LaserScan {
    uint64_t stamp_ns;
    float angle_min;
    float angle_increment;
    std::vector<float> beams; // 距离数组
};
```

- **综合传感器数据（节选）**

```cpp
struct SensorData {
    float ctrl_v;
    float ctrl_w;

    Odom odom;
    Imu  imu;

    // 各类滑移判定
    bool slip_line_front;
    bool slip_line_back;
    bool slip_rot_left;
    bool slip_rot_right;
    // ...

    // 碰撞、悬空、红外、声呐、虚拟墙等
    bool bumper_front;
    bool bumper_left;
    bool bumper_right;
    bool wheel_drop_left;
    bool wheel_drop_right;
    // ...

    // 悬崖 / 基站红外 / 电池电压等
    bool cliff_front;
    bool cliff_left;
    bool cliff_right;
    bool dock_ir_front;
    bool dock_ir_back;
    float battery_voltage;
};
```

- **状态枚举**

```cpp
enum class CleanState : uint8_t {
    // selfcheck, checkdock, findwall, followwall, coverage, gohome, end, ...
};

enum class ExceptionCode : uint8_t {
    // low_power, nai_fail, go_home_fail, end, ...
};

enum class MotionState : uint8_t {
    // rotate, line, dock, cpp, npp, end, ...
};
```

### 3.3 Type 编号与典型频率

> 非完整列表，仅作示例，详细见 `log_types.hpp`。

| Type 常量                    | ID  | 对应结构     | 典型频率 | 说明                 |
|------------------------------|-----|--------------|----------|----------------------|
| `TYPE_CLEAN_STATE`           |  99 | CleanState   | 5 Hz     | 清扫状态             |
| `TYPE_EXCEPTION_DATA`        | 100 | ExceptionCode| 5 Hz     | 异常状态             |
| `TYPE_MOTION_STATE`          | 101 | MotionState  | 5 Hz     | 运动状态             |
| `TYPE_SENSOR_DATA`           | 102 | SensorData   | 50 Hz    | 综合传感器与控制量   |
| `TYPE_GRID_MAP`              | 103 | GridMap      | 1 Hz     | 清扫栅格地图         |
| `TYPE_EXPROATION_GRID_MAP`   | 104 | GridMap      | 1 Hz     | 探索栅格地图         |
| `TYPE_ROBOT_LIDAR`           | 105 | LaserScan    | 6 Hz     | 机器人激光扫描       |
| `TYPE_ROBOT_POSE`            | 106 | Pose2D       | 6 Hz     | 激光坐标系的 pose    |
| `TYPE_ROBOT_REALTIME_POSE`   | 107 | Pose2D       | 20 Hz    | 机器人实时 pose      |
| `TYPE_GLOBAL_PATH_SET`       | 108 | Pose2DSet    | <5 Hz    | 全局规划路径         |
| `TYPE_LOCAL_PATH_SET`        | 109 | Pose2DSet    | <5 Hz    | 局部规划路径         |
| `TYPE_COVER_POINT_SET`       | 112 | Pose2DSet    | 不定     | 覆盖点               |
| `TYPE_POINT_DEBUG1/2/3`      |116+ | Pose2DSet    | 不定     | 调试点集             |

机器人侧写日志时，建议尽量遵守上述语义与频率，以便本工具链可以直接消费真实数据。

---

## 4. C++ 可执行程序说明

### 4.1 `logtool`：仿真日志生成工具

**目标：**

- 从 floorplan 生成的地图与轨迹出发，合成一段完整的清扫日志；
- 同时生成索引 `.idx`，并做一遍统计与 demo。

**主要流程：**

1. 解析命令行参数：
   - `--from_floorplan`（当前实现必选）；
   - `--img`：floorplan 图像路径；
   - `--duration`：仿真时长（秒）；
   - `--out`：输出日志路径（默认 `fake_cleaning.log`）。

2. 调用 `fake_data`：
   - 解析 `assets/grid_map.npy` 得到 `GridMap`；
   - 解析 `assets/trajectory.csv` 得到轨迹点序列；
   - 按时间推进（0 → duration），在各 type 上采样并写入 `LogWriter`；
   - 写入过程中，在 `IndexDB` 中同步记录 offset / type / ts。

3. 构建索引：
   - 基于 `IndexDB` 中记录构建 `all` 和 `by_type` 视图；
   - 写出 `.idx` 文件（`<log>.idx`）。

4. 统计 & Demo：
   - `scanStats(log_path, stats)`：再次读取 `.log`，统计各 type 的长度与帧率；
   - `printStatsTable(stats)`：打印统计结果；
   - `demoDragPreview()`：演示一个“拖拽预览”的简单用例（从中间时间点抽一帧）。

**典型用法：**

```bash
./logtool --from_floorplan           --img data/floorplan.jpg           --duration 120           --out data/fake_cleaning.log
```

---

### 4.2 `logdump`：日志检查工具

**目标：**

- 面向开发调试，检查 `.log` 的结构与内容；
- 对齐协议设计与实际记录。

**主要功能：**

- 打印文件头与时间信息：
  - 起始时间 `t0`、结束时间 `t1`、总时长；
- 遍历记录：
  - 显示 `#idx, offset, type, type_name, ts, len`；
- 支持过滤：
  - `--type <id>`：只查看某一 type；
  - `--limit <N>`：限制显示记录数；
- payload 解码 / 十六进制查看：
  - `--parse`：按照类型解码并以人类可读形式输出；
  - `--hex N`：以十六进制打印 payload 前 N 字节。

**典型用法：**

```bash
./logdump --log data/fake_cleaning.log --limit 50
./logdump --log data/fake_cleaning.log --type 102 --limit 10 --parse
./logdump --log data/fake_cleaning.log --type 103 --limit 3 --hex 64
```

---

### 4.3 `log_web_viewer`：Web 可视化后端

**目标：**

- 提供极简 HTTP Server；
- 为前端提供：
  - 元信息：`/api/meta`；
  - 单帧数据：`/api/frame?ts_ns=...`；
  - 轨迹抽样：`/api/traj?ts_ns=...&step_ms=...&max_points=...`；
  - 静态资源服务：HTML/CSS/JS。

**核心逻辑：**

1. **初始化与启动：**
   - 解析参数：`--log`、`--web`、`--port`；
   - 使用 `LogReader` 打开日志文件；
   - 使用 `IndexDB` 加载 `.idx`；
   - 从 `TYPE_ROBOT_REALTIME_POSE` 记录构建轨迹列表 `poses`，用于 `/api/traj` 快速抽样。

2. **HTTP 路由：**
   - 静态资源：
     - `/`、`/index.html` → 前端主页面；
     - `/app.js`、`/style.css` 等。
   - JSON API：
     - `/api/meta` → 整体元信息；
     - `/api/frame` → 指定时间戳附近的一帧“聚合视图”；
     - `/api/traj` → 轨迹前缀抽样。

3. **`/api_meta`：元信息**

   - 基于 `IndexDB` 和最新 `GridMap` 记录生成：
     - `t0_ns / t1_ns / duration_sec`；
     - 是否存在 map / pose / lidar / sensor 等；
     - 地图分辨率 / 大小 / 原点等；
   - 供前端初始化 slider 范围、开关状态等。

4. **`/api/frame`：单帧聚合数据**

   - 输入：`ts_ns`（目标时间戳，纳秒）。
   - 对齐逻辑：
     - 以 `TYPE_ROBOT_REALTIME_POSE` 为主时间线；
     - 对齐最近的：
       - `TYPE_ROBOT_LIDAR`；
       - `TYPE_GRID_MAP`；
       - `TYPE_GLOBAL_PATH_SET / TYPE_LOCAL_PATH_SET`；
       - `TYPE_CLEAN_STATE / TYPE_EXCEPTION_DATA / TYPE_MOTION_STATE / TYPE_SENSOR_DATA`；
     - 解码 payload，组装成一个聚合 JSON。

   - 输出字段包含：
     - `pose`、`scan`、`map`；
     - `imu`、`odom`、`slip`、`bumper`、`cliff`、`dock`；
     - `clean_state`、`exception`、`motion_state`；
     - 以及必要的时间戳等信息。

5. **`/api/traj`：轨迹抽样**

   - 输入：
     - `ts_ns`：目标时间；
     - `step_ms`：抽样时间步长（毫秒）；
     - `max_points`：最大点数。
   - 从 0 → `ts_ns` 时间段按 `step_ms` 抽样 `Pose2D`；
   - 将 `{x,y}` float32 序列编码为 base64，返回：
     - `xy_f32_b64`；
     - `count`；
     - `step_ms`。

---

## 5. Web 前端设计

### 5.1 布局概览

前端界面由三部分组成：

1. **顶部工具栏（播放控制区）**
   - 播放 / 暂停：`#btnPlay`；
   - 录制：`#btnRecord`（全屏录制）；
   - 轨迹重置：`#btnResetTraj`；
   - 倍速选择：`#selSpeed`；
   - 当前时间显示：`#txtTime`；
   - 状态文本：`#txtState`；
   - 异常文本：`#txtException` 等。

2. **中间显示区（主画布）**
   - 主画布：`#canvas`：
     - 绘制内容：
       - 栅格地图；
       - 激光点云；
       - 机器人轨迹；
       - 全局路径、局部路径；
       - 机器人图标 + 朝向；
       - RViz 风格网格、坐标轴、比例尺等；
     - 支持交互：
       - 放大缩小（滚轮）；
       - 任意平移（拖动）；
       - 绕光标旋转。

   - 视图控制按钮：
     - `#btnFitTraj`：视图自适应轨迹；
     - `#btnResetView`：重置视图；
     - 测量工具按钮：通过 JS 动态插入，开启/关闭测距模式。

3. **底部/右侧信息区**
   - 时间 slider：`#slider`；
   - 时间数值显示：`#tsec`；
   - IMU 信息：`#imu_pitch / #imu_roll / #imu_yaw`；
   - Odom 信息：`#odom_txt` 等；
   - 传感器综合信息：
     - 加速度、角速度；
     - 各种碰撞 / 悬空 / 红外 / 声呐 / 虚拟墙 / 悬崖 / 基站等标志；
     - 电池电压；
   - 显示开关：
     - `#ckMap / #ckScan / #ckTraj / #ckRobot / #ckAxes / #ckGrid / #ckScale`；
     - 路径 / 调试点集开关：如 `#ckGlobalPath / #ckLocalPath / #ckDebugPoints` 等。

### 5.2 交互与数据流

1. **初始化：**
   - 页面加载后调用 `/api/meta` 获取元数据；
   - 根据 `t0_ns / duration_sec` 初始化时间轴与内部 time 基准；
   - 若存在地图，则优先加载最新 `GridMap` 作为初始背景。

2. **播放与时间轴：**
   - 手动拖拽 `#slider`：
     - 计算对应秒数 sec；
     - 转为 `ts_ns`（BigInt）；
     - 调用 `/api/frame?ts_ns=...` 更新画面。
   - 播放模式：
     - 使用定时器 / `requestAnimationFrame` 按当前倍速推进时间；
     - 每次迭代调用 `/api/frame`；
     - 同时维护一条前缀轨迹 `trajPts` 用于实时绘制。

3. **轨迹抽样 `/api/traj`：**
   - 在需要高质量录制或快速重绘轨迹时调用；
   - 后端返回压缩的 float32 序列，前端解码后绘制轨迹，提高连续性、减小接口压力。

4. **视图控制：**
   - 内部维护 world → screen 变换参数：平移、缩放、旋转；
   - 鼠标操作更新这些参数；
   - 所有绘图使用统一变换，保证视图交互一致性。

5. **测量工具：**
   - 进入测量模式后，捕获两次鼠标点击；
   - 在世界坐标系下计算两点距离，并在 UI 中展示；
   - 用于评估地图尺度 / 路径长度等。

---

## 6. 典型工作流

### 6.1 从 floorplan 仿真到 Web 回放

1. 准备 Python 环境（推荐虚拟环境）；
2. 使用 `floorplan_to_map_and_traj.py` 生成：
   - `assets/grid_map.npy`；
   - `assets/trajectory.csv`；
3. 使用 `logtool` 基于 assets 生成：
   - `data/fake_cleaning.log`；
   - `data/fake_cleaning.log.idx`；
4. 使用 `logdump` 做基本检查与统计；
5. 使用 `log_web_viewer` 启动 HTTP server；
6. 浏览器打开 `http://127.0.0.1:<port>/`，执行时间拖拽、播放、录制等操作。

### 6.2 接入真实机器人日志（建议路径）

1. **机器人侧实现兼容的日志写入：**
   - 按 `log_types.hpp` 定义的结构体与 type 编号编码；
   - 保证 `timestamp_ns` 单调递增；
   - 尽量遵守预期频率。

2. **生成或拷贝日志：**
   - 将真实 `.log`/`.idx` 拷贝到 PC；
   - 若只有 `.log`，可使用本工程的索引工具生成 `.idx`。

3. **使用 `logdump` 检查真实日志：**
   - 验证 type 分布、帧率、payload 解码是否正常；
   - 快速发现协议不匹配或字段错误。

4. **使用 `log_web_viewer` 可视化真实日志：**
   - 协议兼容时，前端可直接复用；
   - 若新增 type：
     - `log_types.hpp`：新增结构体与 type；
     - `codec.hpp`：增加编码 / 解码支持；
     - `logdump.cpp`：新增解析输出；
     - `log_web_viewer.cpp`：在 `/api/frame` 或新增 API 中暴露数据；
     - `app.js`：新增绘图与 UI 控件。

---

## 7. 后续扩展建议

基于当前设计，后续可以考虑的扩展方向：

1. **多日志文件管理**
   - 支持一次加载多段 log，在前端通过下拉列表切换。

2. **场景标注 / 事件标记**
   - 增加“事件 type”，用于标记卡死、定位失败、重新构图等时间点；
   - 前端支持事件列表与一键跳转。

3. **更细粒度的统计与导出**
   - 在 `stats.hpp` 基础上扩展：
     - 指定 type 的分段统计；
     - 结果导出为 CSV/JSON，供后续分析。

4. **前端多视图布局**
   - 在现有单 canvas 之上扩展为多视图（地图 / 激光 / 传感器状态）；
   - 采用 RViz 式 docking / tab 布局。

5. **自动回放脚本**
   - 增加“脚本模式”：
     - 指定时间范围、图层组合；
     - 自动播放并配合录制，生成标准演示视频。

---
