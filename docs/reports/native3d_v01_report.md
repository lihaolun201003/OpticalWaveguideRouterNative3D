# Native3D v0.1 实施与验收报告（PART B）

> ## ⚠️ 勘误（v0.1.1）：本报告以下结论已被推翻，请勿继续引用
>
> **勘误与重新验收报告：[native3d_v011_correctness_report.md](native3d_v011_correctness_report.md)**
>
> 本文件保留 v0.1 的原始结果与历史，不改写正文。以下四项在 v0.1.1 中被证明是错的：
>
> 1. **"A* 最优性"**：v0.1 在候选生成循环内做目标判定并立即返回，终止条件不正确。
>    有限图反例（S→G 代价 100 先生成，S→A→G 代价 2）在欧氏启发式与 h=0 下都返回 100，正确答案是 2。
> 2. **"量化去重键保证量化图连通性与最优性"**：同名 key 合并了**后继不同**的真实 pose。
>    同桶反例 A=(10,0,0)、B=(10.4,0,0)（S→A=10、S→B=10.4、B→G=9.6）中量化版本报 `search_exhausted`，安全模式下最优代价为 20。
> 3. **"连续 clearance 认证下界"**：v0.1 把有限采样 + 局部搜索得到的**上界**当作认证下界，
>    出现 下界 5.820272826928609 > 采样最小 5.773917590493442 这种数学上不可能的关系；
>    两条真实相交的中心线被报成相距 3.125 µm 且 `verdict=pass`。
> 4. **终端连接器 LSL/RSR 第二弧扫角反向**：合法 LSL（总长 142.987184 µm）被端点检查丢弃，
>    v0.1 只生成约 204.374715 µm 的 LSR/RSL。
>
> 因此本报告中的 "64 passed"、"2D 模式 2241 次扩展真正搜索耗尽"、"1722 次扩展得到 521.722 µm 路径"、
> "认证下界 5.820 µm / 34.480 µm" 等数字，都是在**错误的 A* 终止条件**与**不安全的量化去重**下得到的，
> 不能作为正确性证据。v0.1 原始证据已冻结保留：
> `OpticalWaveguideRouterNative3D/artifacts/cases/`、`OpticalWaveguideRouterNative3D/artifacts/v01_preserved/`；
> v0.1.1 的新证据在 `OpticalWaveguideRouterNative3D/artifacts/v011/`。

项目目录：`C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D`    
报告日期：2026-10-06（北京时间）。运行环境：Windows，Python 3.10.11（项目内 `.venv`），pytest 9.1.1。  
本报告只覆盖 PART B（Native3D v0.1）。PART A（官方 LiDAR baseline）由另一份报告 `docs/reports/lidar_reproduction_baseline.md` 负责。

**一句话结论**：同一套新引擎、同一份几何输入，在 `mode="2d"` 下 2241 次扩展后**真正搜索耗尽**（非预算停止）且不可达；在 `mode="native3d"` 下 1722 次扩展即找到一条 521.722 µm 的连续三维路径，其 z 变化完全来自搜索期生成的三维动作原语（初始状态即存在 13 个可行的改变 z/俯仰的候选动作），全程 R ≥ Rmin、连续避障 clearance 认证下界 5.820 µm ≥ 5 µm、无碰撞、不越界、端点与端点切向误差 ~1e-13 µm / 0°，路径 C1 连续。**没有先生成二维路线再抬升 z 的任何步骤**。

---

## 0. 范围、声明与红线

本轮实现并验证：

* 单条 net、纯几何层面的 **native 3D** 波导寻路：路径由三维搜索本身产生（三维位置 + 三维方向 + 空间圆弧）。
* 五类硬约束（Rmin、三维障碍 keep-out、中心线 spacing、chip volume 边界、端点位置/切向容差）在**完整连续 primitive** 上检查，且用有误差界的保守细分，不可判定即拒绝。
* 独立的第二实现校验入口（只读 JSON，可脱离路由代码复核）。
* 机器可读证据：`artifacts/cases/*.json`、原始 pytest 日志 `artifacts/logs/pytest_full_run.log`。

本轮**没有**做、也**不声称**：

* 没有多网联合路由、没有 3D 占用更新、没有 rip-up & reroute、没有 history congestion、没有 1024 通道、没有 ML/BPM。
* **没有任何光学损耗/串扰/dB 结果**：引擎只输出几何量（µm、rad、无量纲等效长度）。代码中不存在 loss/dB 标识符（已用 AST 扫描测试强制），也不存在"未知光学参数生成 dB"的路径。
* 所有数值都是**合成几何参数**，不是任何论文的材料参数；不冒充 LiDAR 论文原算法，全部标为"本课题扩展"。
* 有限动作 + 量化去重图上的最优性**不等于**连续空间全局最优或完备；有限图上的最优性有精确对照，连续空间只有"找到的路径合法"这一结论。
* 搜索期没有强制"路径对自身的非相邻近接 spacing"（该量依赖整条路径历史，会破坏 pose-only 状态空间），列为限制并在独立校验中后验检查。

---

## 1. 架构与文件职责

| 文件 | 职责 |
|---|---|
| `src/native3d/config.py` | `RouterConfig`：单位、数值容差、三种距离口径（中心线 dmin / 障碍 keep-out / 波导包络半径）、Rmin、搜索分辨率、目标容差、动作参数、预算。 |
| `src/native3d/geometry/vec.py` | 纯 Python 三维向量工具（float 三元组）。 |
| `src/native3d/geometry/primitives.py` | `Pose3D`、`LineSegment3D`、`CircularArc3D`（任意三维平面圆弧）。 |
| `src/native3d/geometry/path3d.py` | `Path3D`：有序分段、累计长度、位置/切向求值、C1 报告（非 C1 尖角与曲率突变分开记录）。 |
| `src/native3d/search/state.py` | `State3D=(x,y,z,direction)`，direction 为真正三维单位向量；去重键（位置+方向）。 |
| `src/native3d/search/actions.py` | Phase 3 动作：straight / yaw left-right / pitch up-down / skew（倾斜轴）；局部框架、旋转轴与正负号定义。 |
| `src/native3d/search/connector.py` | 终端连接器：精确 pose-to-pose 平面 Dubins（LSL/RSR/LSR/RSL）+ 退化直线，搜索期生成、同样全约束校验。 |
| `src/native3d/search/congestion.py` | 固定拥塞场接口（零场、盒区密度场），单位是等效长度 µm，恒 ≥ 0。 |
| `src/native3d/search/astar.py` | native 3D A*：edge_cost / g / f、trace、初始状态候选探针、显式状态返回。 |
| `src/native3d/search/router.py` | 装配：场景+模式 → 候选生成 → 入 open 集前的完整约束校验 → 目标判定（位置+切向）与启发式。 |
| `src/native3d/constraints/checks.py` | Phase 5 硬约束与连续几何检查方法。 |
| `src/native3d/validation/validate.py` | **独立**校验：只从序列化 JSON 重新实现曲线求值与距离查询，不 import 路由代码。 |
| `src/native3d/io/caseio.py`、`io/artifacts.py` | `Scene`（chip volume / 障碍 / 固定参考中心线 / 源目标 pose / 配置）与证据写出。 |
| `src/native3d/cases.py` | 合成案例定义（核心案例 + Phase 6 其余 8 例）。 |
| `tests/` | 64 个 pytest 用例（Phase 1–6 + 独立校验）。 |
| `examples/run_core_case.py` | 重新生成核心案例证据 JSON。 |

---

## 2. 输入 / 输出契约

输入（`Scene`，JSON，单位 µm）：`chip_volume`（AABB）、`obstacles`（AABB 列表）、`reference_centerlines`（固定已有波导中心线，直线/圆弧）、`source_pose`、`target_pose`（position + 单位切向）、`config`。

输出（`SearchResult` → 证据 JSON）：

* `status` ∈ `success` | `invalid_source_pose` | `invalid_target_pose` | `search_exhausted` | `budget_exhausted`；
* `path`：**原始搜索得到的**连续 `Path3D`（分段参数 + 累计长度），无任何后处理；
* `trace`：父节点链（动作名、x/y/z、direction、azimuth、elevation、g）——z 演化证据；
* `initial_candidates`：初始状态生成的全部候选（含 z/俯仰变化量、校验结论）；
* `segment_provenance`：路径每一段由哪次搜索动作产生；
* `cost_um_equivalent`、`length_um`、`congestion_cost_um_equivalent`、`expansions`、`generated`、`rejected`、`rejection_summary`、`runtime_s`；
* `independent_validation`：第二实现复核结果（最小半径、clearance 认证下界、spacing、边界/碰撞违规计数、端点误差、z 剖面、判定）。

---

## 3. Phase 1 几何：公式与单位

单位全部 µm，几何内部角度用 rad（配置文件用 deg）。

* `Pose3D(position, tangent)`：tangent 强制为**三维单位向量**（非单位自动归一化；零向量/非有限值抛 `ValueError`）。
* `LineSegment3D(p0,p1)`：长度 = |p1−p0|；零长度抛错；半径 = ∞；曲率 = 0。
* `CircularArc3D(center, u, v, radius, theta)`：存圆心、**正交单位平面基** (u,v)、半径 R、带符号转角 theta；平面法向 n = u×v。
  * P(φ) = C + R(cos φ · u + sin φ · v)，φ ∈ [0, θ]（θ<0 时反向参数化）
  * T(φ) = sign(θ)·(−sin φ · u + cos φ · v)
  * **L = R·|θ|**，曲率 = 1/R，最小半径 = R
  * 坐标极值解析可得（`aabb()` 精确，不是采样）
  * 非法输入明确行为：R ≤ 0、R 非有限、θ = 0、基非单位/非正交、平面法向不垂直切向 → `ValueError`
  * `from_pose(pose, normal, R, sweep)`：正 sweep 表示绕 +normal 按右手法则旋转切向（切向转向 normal × t）；负 sweep 通过翻转平面基实现，起点切向严格等于 pose.tangent。
* `Path3D`：有序分段、累计长度、`point_at/tangent_at`；`continuity_report()` 分别给出**非 C1 尖角**（位置断裂/切向断裂，判为非法）与**曲率突变**（line↔arc 必然存在，记录但不作为 C1 违规）。

---

## 4. Phase 2 native 3D 状态

```text
State3D = (x, y, z, direction)     direction = 三维单位向量（非"二维朝向 + layer"）
```

* 去重键：`(round(x/Δp), round(y/Δp), round(z/Δp), azimuth 扇区, elevation 扇区)`，Δp = 20 µm，Δθ = 30°（可配置）。**方向必须参与键**（同一位置不同方向永不共用键，已单测）。
* 键只用于重复检测；节点保存**真实连续位姿**，圆弧端点从不 snap 到网格，半径与切向不受量化影响。
* 同格不同真实位姿：两条都进入搜索图；只有当新位姿 g 不优于已记录的该格 g 时才剪枝（并在报告中说明这使最优性只在"量化图"意义上成立）。
* 状态充分性：全部动作只依赖 pose（straight/yaw/pitch/skew 与终端连接器都只需要 pose + 固定目标），因此没有额外的历史变量；唯一依赖历史的可行量是"路径对自身的非相邻近接"，它**没有**进入状态，列为限制并由独立校验后验检查。

---

## 5. Phase 3 三维动作原语

局部框架（`local_frame(t)`，以全局竖直轴 ẑ 为锚，明确写入代码与文档）：

* `left = normalize(ẑ × t)`：竖直平面内的侧向轴（t ∥ ẑ 时取确定性 fallback ±ŷ）
* `n_yaw = t × left` = `normalize(ẑ − (ẑ·t)t)`：局部"平面内竖直轴"；**t 水平时严格等于全局 ẑ**
* `a_up = −left`：抬头轴

| 动作 | 旋转轴 | 正 sign 含义 | 说明 |
|---|---|---|---|
| `straight(L)` | — | 沿当前三维切向直行 L | 只改变位置 |
| `yaw_left/right(θ)` | ±n_yaw | +θ = 绕 +n_yaw，切向转向 n_yaw × t | t 水平时就是全局 ẑ 方位角转弯且 z 不变；t 倾斜时弧面是**倾斜平面**（旋转轴必须垂直切向，水平圆弧无法从倾斜切向出发），因此仰角也变化——这是如实定义并在代码 docstring/报告中写明，不冒充"纯方位角" |
| `pitch_up(θ)` | a_up = −left | +θ = 抬头 | 竖直平面内改变仰角 |
| `pitch_down(θ)` | left | +θ = 低头 | 与上镜像 |
| `skew(tilt α, ±θ)` | cos α · n_yaw + sin α · a_up | ±θ | α∈(0°,90°) 时旋转轴既非 ẑ 也非 left，弧面同时倾斜于水平面与竖直面，切向同时改变方位角与仰角——**真正的斜向三维动作**；轴退化为零向量时该动作直接跳过 |

每个动作返回**完整连续 primitive**（直线或空间圆弧）与其精确端点 pose；**三维位置与三维方向都在搜索扩展时改变**，不存在"先定二维节点再补 z"。半径、边界、障碍、spacing 在加入 open 集之前对完整 primitive 校验（终端连接器使用同一套规则）。

---

## 6. Phase 4 A*：cost / heuristic 与证明范围

```text
edge_cost = primitive_length + congestion_penalty      (congestion_penalty >= 0，单位=等效 µm)
g         = 路径上真实 edge_cost 之和
f         = g + h
h(s)      = max(0, |target_position − s.position| − eps_pos)
```

* 拥塞场在一次搜索期间**固定**（接口 `CongestionField`；零场与盒区密度场），不与硬 spacing 混用（dmin 始终是硬约束）；硬约束永不退化为 soft penalty。
* **admissibility 论证**：任何合法路径长度 ≥ 起点到目标点的直线距离；目标允许停在 eps_pos 容差内，故剩余代价 ≥ max(0, dist − eps_pos)；每条边代价 = primitive 长度（≥ 其弦长 = 端点欧氏距离）+ 非负拥塞项；约束只删路径不增路径。因此 h 从不高估，且 `|h(u) − h(v)| ≤ dist(u,v) ≤ cost(u,v)` 使 h 一致。没有加入任何未证明的方向/弯曲罚项。
* 目标判定**同时**检查位置与切向（`pos_err ≤ eps_pos` 且 `ang_err ≤ eps_ang`）；同一位置切向错误不得判成功（有专门案例）。
* 状态：`success` / `invalid_source_pose` / `invalid_target_pose` / `search_exhausted`（open 集真空，有限图真正耗尽）/ `budget_exhausted`（扩展数或运行时间上限，reason 中明确写 "this is NOT a proof of unreachability"）。
* 最优性范围：对**A* 实际搜索的量化图**成立；在显式有限图上与 h=0 统一代价搜索（同引擎）以及测试内独立实现的 Dijkstra 三者代价完全一致（见 §9 案例 8）。
* 受限的终端连接器族（平面 Dubins，要求起终 pose 共面）会影响完备性，不影响正确性：任何被接受的连接器都经过完整数值校验。

---

## 7. Phase 5 连续硬约束与检查方法

三种距离口径**显式分开配置**，绝不暗中混用：

| 配置字段 | 含义 |
|---|---|
| `dmin_centerline_um` | 波导中心线之间的最小距离（对固定已有参考波导） |
| `obstacle_clearance_um`（可叠加 `wg_envelope_radius_um`，需显式打开开关） | 中心线到障碍实体的 keep-out |
| `wg_envelope_radius_um` | 波导实体半径/工艺包络；默认作为中心线到 chip volume 边界的 margin（`effective_boundary_margin_um`） |

检查方法（**禁止只采样就宣称连续无碰撞**）：

* **直线 vs 盒**：精确。slab 相交测试给出精确相交参数；不相交时距离 = min(两端点到盒距离, 线段到盒 12 条棱的精确线段-线段距离)——数学上精确，无采样。
* **圆弧 vs 盒**：保守细分。半径 R、扫角 Δφ 的弦与弧的偏差上界为弦高 s = R(1 − cos(Δφ/2))，于是
  `clearance ≥ min_i dist(chord_i, box) − s`（认证下界）且 `clearance ≤ min_i dist(chord_i, box) + s`（上界）。下界 ≥ 要求 → safe；上界 < 要求 → violation；否则继续细分；到达 `max_subdiv_level` 仍未判定 → **undecided → 拒绝**。
* **spacing（primitive 对）**：同一细分同时给出 LB/UB（两侧弦高都扣除/加上），覆盖**所有参数对**（弦对遍历两个参数区间），绝不比较"相同参数/相同传播坐标"的点。包围盒只用于**排除**（正的 AABB 间隙是合法下界）；包围盒重叠**不**判为碰撞。
* **边界**：解析坐标极值（直线的端点、圆弧的解析极值）——整个 primitive 精确包含判定，不是只查首尾。
* **相邻段豁免**：只有共享端点的相邻段对（|i−j| = 1）在自接近检查中被豁免；其余段对全部检查，因此路径回绕后的真实近接会被发现（有单测）。搜索期不检查自接近，独立校验后验报告（限制）。
* **端点/目标**：位置容差 eps_pos、切向容差 eps_ang 显式配置；目标方向定义为**期望传播切向**（端口外法向需自行转换，本轮不涉及端口约定）。
* 候选在加入 open 集之前做上述全部检查；接受后**绝不**调用任何 elevation repair。

---

## 8. 独立验证入口

`src/native3d/validation/validate.py` 是独立的第二实现：只 import 标准库（有 AST 级别的测试断言），从**序列化 JSON** 重新实现曲线求值、点到盒/点到段/段到段距离与保守细分，输出：

最小半径（用三点外接圆**重新拟合**，与存储半径比对）、长度（用弦长反解 R·Δφ 重新积分）、接缝位置/切向（解析重算，另附有限差分交叉检查）、边界违规采样数、碰撞/clearance 的采样值与认证下界、spacing 采样值与认证下界、端点误差、z 剖面、违规计数与 `verdict`。

可脱离路由代码直接运行（`-m` 形式需先设 `PYTHONPATH`；模块本身只 import 标准库）：

```powershell
$env:PYTHONPATH = "src"
.venv\Scripts\python.exe -m native3d.validation.validate artifacts\cases\core_case_native3d_mode.json
```

实测（项目根目录）：`min_radius_um_refit = 49.9999999999999`、`conservative_clearance_lower_bound_um = 5.820272826928609`、`z_varies = true`、`verdict = "pass"`。

---

## 9. Phase 6 核心验收案例：二维不可达 / Native3D 可达

### 9.1 输入（两模式完全相同）

* chip volume：x ∈ [0, 600]，y ∈ [0, 300]，z ∈ [−60, 120] µm（**z = 0 是 volume 内部的端口平面**，下方 60 µm 属于 volume）。
* 源 pose：(60, 150, 0)，切向 (1,0,0)；目标 pose：(540, 150, 0)，切向 (1,0,0)（**两者 z = 0**）。
* 障碍墙：x ∈ [290, 310]，y ∈ [−50, 350]（**完全覆盖 chip 的 y 范围，无侧绕缝隙**），z ∈ [−70, 40]（**延伸到 chip 底板以下，无下绕缝隙**）。
* 固定参考波导中心线：(150,120,0) → (450,120,0)（距 y = 150 走廊 30 µm）。
* 约束：Rmin = 50 µm，dmin(中心线) = 5 µm，obstacle keep-out = 5 µm，波导包络半径 = 2 µm（同时是边界 margin），eps_pos = 2 µm，eps_ang = 3°。
* 动作集：straight {10,20,40} µm；yaw/pitch/skew 角度 {30°,45°,90°}，半径 {50,75} µm；skew tilt {45°,135°}；终端连接器半径 {50,100} µm。
* 去重分辨率：位置 20 µm，方向 30°；预算 50000 次扩展 / 300 s。

### 9.2 z = 0 平面不可达的**几何**论证

墙在 z = 0 切片中覆盖全部可行横向宽度（y 范围超出 chip 边界），且 z 范围包含 0 并延伸到 volume 以下；chip volume 边界禁止绕行。因此 z = 0 平面上，源区域 {x ≤ 290 − 5} 与目标区域 {x ≥ 310 + 5} 之间**不存在任何可行中心线**：任何在该平面内的路径必须穿过墙的 keep-out。这不是"搜索预算不够"，而是平面几何隔离事实。墙顶 z = 40 以上留有 120 − 40 = 80 µm 高度（中心线可用 z ∈ [45, 118]），且源侧/目标侧各有 ≥ 2·Rmin 的纵向空间，所以三维路径在给定 Rmin 与 clearance 下确实存在。

### 9.3 结果（真实运行，证据见 `artifacts/cases/`）

| 项 | 2D mode | Native3D mode |
|---|---|---|
| 状态 | **search_exhausted** | **success** |
| 扩展 / 生成 | 2241 / 18549 | 1722 / 51516 |
| 是否预算停止 | 否（`budget_exhausted=false`；预算 50000） | 否 |
| 运行时间 | 5.07 s | 7.92 s |
| 路径 | 无（不可达） | 6 段，521.722 µm，cost = 521.722163（拥塞项 0） |
| 拒绝原因统计 | boundary 6，obstacle_clearance 2 | boundary 29，obstacle_clearance 2 |

Native3D 搜索父节点轨迹（`trace`，z 真实变化）：

```text
<source>       (60.0, 150.0,  0.00)
straight_10    (70.0, 150.0,  0.00)
pitch_up_30_50 (95.0, 150.0,  6.70)      # 俯仰 +30°，z 由搜索动作抬升
pitch_up_45_50 (118.3,150.0, 37.06)      # 俯仰再 +45°（仰角 75°）
connector_R50  (540.0,150.0,  0.00)      # 终端连接器（3 段，全程校验）
```

* `trace_z_summary`：z 取值 [0, 0, 6.699, 37.059, −9.9e−14]，`z_varies = true`，单步最大 z 变化 37.06 µm，改变 z 的动作数 3（pitch ×2 + connector），仰角范围 [0°, 75°]。
* 路径最大高度 **73.109 µm**（独立校验 z 剖面），远高于墙顶 40 + 5 = 45 µm；5 段离开 z = 0。
* 每段来源（`segment_provenance`）：`0:straight_10, 1:pitch_up_30_50, 2:pitch_up_45_50, 3..5:connector_R50`——**路径每一段都对应一次搜索动作，没有任何后置处理**；非零 z 段来自搜索期生成的空间圆弧原语。
* 路径 6 段：直线 10.00 µm；R = 50 圆弧 30°、45°（法向 (0,−1,0)）；R = 50 圆弧 86.5°（法向 (0,1,0)）；直线 360.70 µm；R = 50 圆弧 11.5°。所有圆弧法向均非 ±ẑ，即**不是 XY 平面圆弧**（满足"非 XY 平面圆弧"要求）。
* **初始状态（扩展前）候选探针**：2D 模式 17 个候选、其中 0 个可行且改变 z；Native3D 模式 53 个候选、其中 **13 个可行且改变 z/俯仰**，例如 `pitch_up_30_50 (Δz=+6.70, Δ仰角=+30°)`、`pitch_up_45_50 (+14.64, +45°)`、`skew_tilt45_plus_30_50 (+4.74, +20.7°)`。即：**搜索的第一步就具有 z 自由度**（最终路线并非必须先升高，此处事实证明它恰好升高）。

独立校验（`independent_validation`，第二实现，只看 JSON）：

| 量 | 值 | 判定 |
|---|---|---|
| verdict | **pass** | — |
| 最小半径（三点重拟合） | 49.9999999999999 µm（相对误差 2.6e−14） | ≥ Rmin = 50（在 1e−6 容差内） |
| 长度（解析 / 独立重积分） | 521.722162838514 / 521.722162838514 µm（相对误差 0） | 一致 |
| 障碍 clearance（采样最小 / 认证下界） | 5.7739 / **5.8203** µm | 认证下界 ≥ 5 µm |
| 参考波导 spacing（采样最小 / 认证下界） | 34.4726 / **34.4799** µm | ≥ dmin = 5 µm |
| 边界 | 2096 采样点，0 违规，最小余量 43.88 µm | 通过 |
| 端点误差 | 起点 0 µm / 0°；终点 9.9e−14 µm / 0° | ≤ 2 µm / 3° |
| C1 连续性 | 最大接缝位置误差 1.4e−14 µm，最大接缝切向误差 0° | C1 连续，非 C1 接缝 0 |
| 违规计数 | radius 0；boundary 0；碰撞 0；spacing 0；端点 0 | 全部为 0 |

**Lead 复核补充（关于 boundary 的保守下界字段，属披露而非结论变更）**：
`independent_validation.boundary` 中还有一个 `conservative_margin_lower_bound_um = -0.22257226463781032`（**负数**）。
该字段**不代表边界违规**：其构造为"弦包围盒侵入内体积的量 − 弦高界"，当弦包围盒完全落在内部时侵入量为 0，
故结果恒等于 `−sagitta(max) = −R·(1 − cos(θ/16))`（nch = 8）。本路径最大扫角弧 R = 50 µm、θ = 1.510242666 rad，
代入得 `−50·(1 − cos(1.510242666/16)) = −0.22257226463781032`，与 JSON 逐位一致（Lead 已数值复核，绝对差 0.0）。
因此它是一个**由该保守构造决定的固有负值**，只说明 8 段弦的包围盒均在 volume 内部，**不携带安全裕度信息**。
边界安全的实际证据仍是：采样 2096 点 0 违规、最小采样余量 43.88 µm，以及引擎在搜索期对完整 primitive 做的解析坐标极值包含判定。

---

## 10. Phase 6 其余必修案例（真实结果）

| # | 案例 | 构造 | 真实结果 |
|---|---|---|---|
| 1 | 纯直线 | (0,0,0)+(1,0,0) → (100,0,0)+(1,0,0)，无障碍 | success；长度 = **100.0 µm** = 欧氏距离（1e−9 内）；全部为直线段；首尾方向一致 |
| 2 | 空间弯曲 | 倾斜平面圆弧几何测试 + 引擎路径 (0,0,0)+x → (120,40,25)+x（平行切向 ⇒ 连接器平面由切向与位移张成，是**倾斜平面**） | 几何测试：任意法向 (0,1,−1)/√2 平面圆弧满足 |P−C| = R、点积平面法向 = 0、L = R|θ|、端点切向 = 绕法向旋转 θ；引擎路径 success，含法向非 ±ẑ 的圆弧（非 XY 平面），且中段某曲线真的离开 z = 0，独立校验 pass |
| 3 | 封闭障碍不可达 | chip 200×200×60；四壁(延伸到 chip 底板以下)+顶盖围出 x∈[90,130], y∈[90,130], z∈[0,40] 的空腔；目标 (110,110,10)+x 位于腔内（源/目标各自合法，`pose_invalid_reason` 均为 None） | 状态 = **search_exhausted**，417 次扩展 / 2278 生成 / 2.24 s，`budget_exhausted=false`，预算 20000；对照组：同一引擎在核心案例上 `max_expansions=30` 返回 **budget_exhausted** 且 reason 明写"NOT a proof of unreachability" |
| 4 | 半径边界 | Rmin = 50，ε = 1.0 µm（≫ 判定容差 1e−6 µm） | R = 49 → 违规；R = 50 → 合法；R = 51 → 合法；Rmin − 1e−9（容差内）→ 合法（容差内行为已文档化） |
| 5 | spacing 边界 | 平行线段间距 dmin ± 0.01 µm；另有最近点位于**采样节点之间**的斜交对（最近点在 x = 53，节点每 10 µm） | 5.01 → safe（认证界 = 5.01）；4.99 → violation（认证界 = 4.99）；斜交对认证界 = **30.000000** µm（解析值），而仅比较采样节点会得到 30.15 µm（测试中显式对照）；29.95 → safe，30.05 → violation；场景级：中心线距参考波导 4 µm → 拒绝，6 µm → 接受 |
| 6 | 曲线中段越界/碰撞 | 端点均为合法 pose（`pose_invalid_reason` 为 None），但圆弧中点位于墙体内 / 圆弧中段凸出 chip volume | 两者都 **被拒绝**：前者 `obstacle_clearance` 判 violation（认证上界 < 要求），后者 `boundary` 判 violation（解析极值越界）；补充：细分深度不足时返回 **undecided 并拒绝**（不能判定的不允许通过） |
| 7 | 终端切向 | 死巷口袋：自由中心线区 x∈[160,175], y∈[45,55], z∈[2,38]（y 向仅 10 µm 宽）；同一位置两种目标切向 | 正确切向 (+x)：**success**，1 次扩展，132.0 µm，端点误差 0 µm / 0°；错误切向 (+y)：**search_exhausted**（9 次扩展，非预算），因为 90° 转弯需 R ≥ Rmin = 50 µm = 5 倍走廊宽度，几何上不可能；合法连接器必须通过同一套完整约束校验 |
| 8 | admissibility / cost | 3×3×2 有限图（6 邻接 + 2 条昂贵捷径，权重 = 1.4×欧氏距离） | A*(欧氏 h)、同引擎 h = 0 统一代价、测试内独立 Dijkstra **三者代价完全一致**；h ≤ 精确 cost-to-go（用反向 Dijkstra 逐点验证）；返回路径的 g 等于真实边代价之和；几何长度等于各边欧氏长度之和；拥塞案例：非零盒区密度 0.5/µm 时 congestion_cost > 0，cost = length + congestion 精确成立，零场时 cost = length；拥塞项恒 ≥ 0；AST 扫描断言包内不存在 loss/dB/crosstalk 标识符，输出载荷也不含此类键 |

另有 Phase 5 的边界接触、AABB 只用于排除（重叠不等于碰撞）、线段-盒距离与稠密采样 oracle 对照（±0.02 µm）等测试。

---

## 11. 测试执行记录（真实）

```text
工作目录: C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D
命令    : .venv\Scripts\python.exe -m pytest -v
解释器  : C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D\.venv\Scripts\python.exe
          (Python 3.10.11, pytest 9.1.1, pluggy 1.6.0, platform win32)
配置    : pyproject.toml [tool.pytest.ini_options] testpaths=["tests"] pythonpath=["src"]
结果    : collected 64 items -> 64 passed, 0 failed, 0 skipped, 0 xfail, 0 error
pytest 报告耗时: 22.60 s      进程墙钟: 23.12 s      退出码: 0
原始日志: artifacts\logs\pytest_full_run.log （含每用例 ID、耗时与上面的汇总行）
```

按文件分布（均为真实收集数）：`test_phase1_geometry.py` 11、`test_phase2_state.py` 5、`test_phase3_actions.py` 7、`test_phase4_astar.py` 7、`test_phase5_constraints.py` 10、`test_phase6_cases.py` 9、`test_phase6_core_acceptance.py` 10、`test_independent_validation.py` 5。

* 核心验收测试（2D 不可达 / Native3D 可达、z 自由度、连续约束、原始路径来源、证据文件可复核）全部为普通测试，**没有** skip/xfail 掩盖。
* 会话级 fixture 会真实运行两次核心搜索（2D 与分析性 Native3D），并**在测试运行时重写** `artifacts/cases/*.json`（因此证据与测试结果始终同源）。
* 路线成功率：核心案例 Native3D 1/1 成功；2D 对照 0/1（几何不可达）；其余案例见 §10。

### 11.1 Lead 独立复跑记录（数字对账）

Lead 在交付前于同一工作目录**独立复跑**了同一命令，用以确认上面的结果不是单次偶然：

```text
命令    : .venv\Scripts\python.exe -m pytest -q
结果    : 64 passed, 0 failed, 0 skipped, 0 xfail      退出码: 0
pytest 报告耗时: 19.15 s      进程墙钟: 19.80 s
日志    : artifacts\logs\lead_verification_pytest.log
```

由于 §11 提到的 fixture 会在测试运行时重写 `artifacts/cases/*.json`，Lead 的这次复跑**替换了磁盘上的证据文件**。因此：

| 字段 | 报告正文（首次运行） | 磁盘 JSON 现值（Lead 复跑） | 是否一致 |
|---|---|---|---|
| collected / passed / failed / skipped / xfail | 64 / 64 / 0 / 0 / 0 | 64 / 64 / 0 / 0 / 0 | **一致** |
| 2D 扩展数 / 生成数 | 2241 / 18549 | 2241 / 18549 | **一致** |
| Native3D 扩展数 / 生成数 | 1722 / 51516 | 1722 / 51516 | **一致** |
| 拒绝原因（2D / Native3D） | boundary 6, clearance 2 / boundary 29, clearance 2 | 同 | **一致** |
| 路径长度 / 最小半径 / clearance 下界 / spacing 下界 / verdict | 521.722 µm / 49.9999999999999 / 5.8203 / 34.4799 / pass | 同 | **一致** |
| 2D / Native3D 单次搜索耗时 | 5.07 s / 7.92 s | **4.3033 s / 6.7616 s** | **不同（仅墙钟抖动）** |
| pytest 耗时 | 22.60 s（墙钟 23.12 s） | **19.15 s（墙钟 19.80 s）** | **不同（仅墙钟抖动）** |

**唯一随运行变化的是墙钟时间**；扩展数、生成数、拒绝分类、路径几何、半径、clearance/spacing 认证下界与 `verdict` 在两次运行中逐位相同。`generated_at_utc` 显示当前磁盘 JSON 为 Lead 复跑所生成（`2026-10-06T16:27:00Z`）。本节显式记录此对账，避免报告引用的耗时与 JSON 中的耗时被误读为矛盾。

### 核心案例证据文件

```text
artifacts/cases/core_case_native3d_mode.json        （输入几何/配置、状态、原始路径分段、父节点 trace、段来源、独立校验）
artifacts/cases/core_case_2d_mode.json              （同几何 2D 模式：search_exhausted，无路径）
artifacts/cases/core_case_initial_action_probe.json （初始状态候选动作探针：2D 0 个改变 z / Native3D 13 个）
artifacts/logs/pytest_full_run.log                  （完整 pytest 原始输出）
```

---

## 12. 复跑命令

```powershell
cd "C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D"

# 全部测试（会同时重写 artifacts/cases 下的证据）
.venv\Scripts\python.exe -m pytest -v

# 只重跑核心案例并重新生成证据 JSON
.venv\Scripts\python.exe examples\run_core_case.py

# 只用独立实现复核已保存的证据文件（不加载路由代码）
$env:PYTHONPATH = "src"
.venv\Scripts\python.exe -m native3d.validation.validate artifacts\cases\core_case_native3d_mode.json

# 环境（若 .venv 不存在）
C:\Users\lihao\AppData\Local\Programs\Python\Python310\python.exe -m venv --system-site-packages .venv
.venv\Scripts\python.exe -m pip install pytest
```

---

## 13. 已知限制（如实列出）

1. **单网**：没有多网顺序、没有 3D 占用随路由更新、没有冲突检测与 rip-up & reroute、没有 history congestion；拥塞场在一次搜索内固定（若动态修改，g 与 admissibility 论证需重新说明）。
2. **完备性**：有限动作 + 量化去重 ⇒ 只在"量化图"意义上讨论最优性；动作集与去重分辨率是配置项，改变它们会改变可达集合。终端连接器只覆盖共面 pose 对，非共面情形需要搜索先到达共面位姿（这是完备性限制，不是正确性限制）。
3. **自接近 spacing**：搜索期未强制路径对自身的非相邻近接（需要路径历史进入状态，会破坏 pose-only 状态空间）。独立校验会报告该量并给出相邻段豁免范围（仅共享端点的相邻对豁免），但核心验收结论基于的硬约束不含此项。
4. **障碍模型**：仅轴对齐盒体（AABB）。倾斜墙/任意实体需要新的距离例程；不过检查框架（精确直线-盒 + 保守弧细分 + LB/UB 判定）可直接推广。
5. **圆弧-盒/圆弧-圆弧**判定是保守细分：存在性判定依赖细分深度；不可判定时**拒绝**，因此可能比真实几何更保守（宁可拒绝也不放过）。sagitta 弦高界只对圆弧成立。
6. **拥塞 soft cost**的弧长积分用均匀细分近似（容差 1e−6 µm 级别，仅影响软代价，不影响任何硬判定）。
7. **未做的事**：1024 通道、16 通道联合、ML、BPM、GDS/GUI、光学损耗/串扰模型、端口外法向约定、制造约束（写入顺序、深度、最小间距工艺窗口）。
8. 2D 模式并非"旧的二维 Router"：它是同一引擎在 `allow_out_of_plane=False` 下只生成不离开 z = 常数平面的 primitive（直线 + yaw + 平面内连接器），代码中不存在对旧项目的任何调用。

---

## 14. 四个问题的回答（本报告负责 Q2、Q3）

**Q2 哪些是本课题自己的 native3D 扩展？**（逐项列出，均不冒充 LiDAR 论文贡献）

1. **三维 pose 与三维方向状态**：`State3D=(x,y,z,direction)`，direction 是真正三维单位向量（非 (x,y,orientation) + layer）。
2. **任意三维平面圆弧几何**：圆心 + 正交基 + 半径 + 带符号转角，L = R|θ|，解析坐标极值与精确 C1 报告；不是"XY 圆弧 + layer 标签"。
3. **三维动作原语族**：straight / yaw left-right / pitch up-down / **skew（倾斜轴圆弧）**，轴与正负号显式定义，每个原语在搜索期同时改变三维位置与三维方向。
4. **native 3D A***：edge_cost = 长度 + 非负拥塞等效长度、g/f、max(0, dist − eps) 可采纳启发式、位置+切向双条件目标判定、区分预算耗尽与有限图耗尽。
5. **连续硬约束与保守判定**：精确直线-盒、弦高界保守细分的弧-盒/弧-弧 clearance 与 spacing（覆盖所有参数对）、解析边界包含、Rmin/端点容差。
6. **三维终端连接器**：搜索期生成的平面 Dubins（弧-直-弧）pose-to-pose 连接，与普通动作同样全约束校验。
7. **独立第二实现校验入口**（JSON-only）与机器可读证据链。

**Q3 能否证明搜索从第一步就具有 z 自由度？** 能，且给出四类证据（针对本实现与验收案例，不是对任意输入的完备性证明）：

1. **状态定义**：状态显式含 z 与三维方向（§4），方向参与去重键；没有任何"layer/层号"变量。
2. **初始邻居**：`artifacts/cases/core_case_initial_action_probe.json` 记录初始状态在**任何扩展之前**生成的候选：Native3D 模式 53 个候选中 **13 个可行且改变 z/俯仰**（例如 `pitch_up_30_50` Δz = +6.70 µm、Δ仰角 = +30°；`pitch_up_45_50` Δz = +14.64 µm；`skew_tilt45_plus_30_50` Δz = +4.74 µm）；同几何 2D 模式 17 个候选中 0 个改变 z。
3. **搜索 trace**：核心案例 success 的父节点链 z 从 0 → 6.699 → 37.059 → 0（µm），改变 z 的动作 pitch×2 + connector，仰角范围 [0°, 75°]；最大高度 73.109 µm。
4. **原始路径证据**：`core_case_native3d_mode.json` 的 `search.path.segments` 与 `segment_provenance` 表明每一段都来自一次搜索动作（straight/pitch/connector），**不存在任何后置抬升或 z 修复步骤**；同几何 2D 模式在 2241 次扩展后 `search_exhausted`（非预算停止），而 z = 0 平面被墙几何隔离，二者共同证明三维可达性来自 z 自由度本身。

（Q1 由 PART A 报告 `docs/reports/lidar_reproduction_baseline.md` 回答；Q4 "进入 16 通道还缺什么" 见 §13 限制清单：多网调度、3D 占用更新、冲突检测、局部 rip-up & reroute、history congestion、顺序依赖、整体独立验证、失败预算/恢复、空间索引性能与制造约束。）