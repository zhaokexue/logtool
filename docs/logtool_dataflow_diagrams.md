# Logtool 数据流流程图

本文件描述 Logtool 从数据生成、日志写入、索引建立、到 Web 可视化回放的整体数据流。

---

## 1. 总体数据流

```mermaid
flowchart LR

A[Floorplan 图像] --> B[Python: floorplan_to_map_and_traj.py]
B --> C[grid_map.npy]
B --> D[trajectory.csv]

C --> E[仿真数据生成 fake_data]
D --> E

E --> F[LogWriter 写入 .log]
E --> G[生成索引 IndexDB]

F --> H[fake_cleaning.log]
G --> I[fake_cleaning.log.idx]

H --> J[logdump 日志检查]
I --> J

H --> K[log_web_viewer 后端服务]
I --> K

K --> L["/api/meta"]
K --> M["/api/frame"]
K --> N["/api/traj"]

L --> O[Web 前端初始化页面]
M --> O
N --> O

O --> P[Canvas 渲染: 地图/轨迹/激光/机器人]
```

---

## 2. 运行时数据流（Web 回放路径）

```mermaid
sequenceDiagram
participant FE as 前端 Web
participant API as log_web_viewer
participant IDX as IndexDB
participant LOG as LogReader

FE->>API: GET /api/meta
API->>IDX: 收集类型与时间范围
API->>FE: 返回元信息

loop 播放或拖拽时间条
    FE->>API: GET /api/frame?ts_ns=...
    API->>IDX: 查询各 type 最近时间
    API->>LOG: 读取具体记录 payload
    LOG-->>API: 解码后的结构体
    API-->>FE: 聚合 JSON
    FE->>FE: 更新画布渲染
end

opt 需要完整轨迹
    FE->>API: GET /api/traj
    API->>IDX: 读取轨迹 Pose 索引
    API-->>FE: base64 压缩轨迹点
    FE->>FE: 轨迹解码并绘制
end
```

---

## 3. .log 文件内部数据流

```mermaid
flowchart TB

subgraph 写入端
    S1["各模块产生数据: pose / lidar / map / sensor"]
    S2["编码: encodePayload"]
    S3["构建 RecordHeader"]
    S4["LogWriter 顺序写入"]
    S1 --> S2 --> S3 --> S4
end

subgraph 存储文件
    F1["FileHeader"]
    F2["RecordHeader + payload"]
end

subgraph 读取端
    R1["LogReader 顺序读取"]
    R2["按 type 分类"]
    R3["parseRecordPayload 解码"]
    R4["IndexDB 建立按时间索引"]
end

S4 --> F1
S4 --> F2
F1 --> R1
F2 --> R1
R1 --> R2 --> R3 --> R4
```

---

## 4. 核心 API 数据结构流向

```mermaid
flowchart LR

LOG[".log 文件"] -->|"Record"| LR["LogReader"]
LR -->|"IndexItem"| IDX["IndexDB"]

IDX -->|"nearest ts"| AGG["Aggregator 聚合器"]

AGG -->|"decode"| Pose["Pose2D / Pose2DSet"]
AGG --> Grid["GridMap"]
AGG --> Lidar["LaserScan"]
AGG --> Sensor["SensorData"]
AGG --> State["Clean / Motion / Exception"]

Pose --> JSON["聚合 JSON 返回前端"]
Grid --> JSON
Lidar --> JSON
Sensor --> JSON
State --> JSON
```

---

以上四个图涵盖：

- 仿真生成链路
- 日志文件内部流转
- Web API 调用序列
- 数据结构聚合关系

可直接嵌入到项目 `docs/` 或 README 中使用。
