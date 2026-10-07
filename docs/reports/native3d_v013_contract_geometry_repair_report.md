# Native3D v0.1.3 公共接口一致性与三维几何验证修复报告

项目目录：C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D
勘误对象：docs/reports/native3d_v012_safe_finite_search_report.md（只增加 v0.1.3 勘误横幅，正文与历史全部保留）
证据目录：OpticalWaveguideRouterNative3D/artifacts/v013/（cases/、logs/、probe/、prefix_src/）
历史证据：artifacts/cases/、artifacts/v01_preserved/、artifacts/v011/、artifacts/v011_preserved/、artifacts/v012/ 全部冻结，v0.1.3 的任何运行都不写它们
冻结快照：artifacts/v013/prefix_src/ 是第一次 v0.1.3 修改之前对 src/native3d/ 的逐字节拷贝（24 个 .py 文件），本次全部 "修复前" 数字都由它在同一进程里以独立包 `native3d_prefix` 真实跑出
环境：Windows-10-10.0.26200-SP0，Python 3.10.11（项目内 .venv），pytest 9.1.1；引擎本身只用标准库

本轮范围：保留 v0.1.2 已跑通的有限多跳原生三维架构，只补齐**公共接口**和**三维几何验证**契约。不开始 16 通道，不加入 ML、BPM、GUI、GDS 或光学损耗模型。

---

## 0. 结论摘要（先给结论）

1. **公共 graph 复用接口不再能返回错误终点的 success。** 不匹配的预建图在**搜索前**被拒绝：
   `status = graph_scene_mismatch`、`path = None`、`cost = inf`、`goal_pos_error_um = None`，
   并给出逐字段差异（chip、每个障碍盒、每条参考中心线的**完整几何**、source/target 位置与三维切向、
   mode、25 个影响边可行性的配置字段、拥塞场身份）。
   冻结 v0.1.2 实测：`success`、`cost = 100.0`、实际终点 `(100,0,0)`、`goal_pos_error_um = 50.0`。
2. **success 现在由真正返回的曲线决定。** 每条被接受的边必须落在其目标节点的规范 pose 上；
   组装曲线必须真的起于/止于**调用方传入**的 pose（源用源容差，终点用接受球），
   逐接缝 C0/C1 都进判定；组装失败返回 `constraint_failed` / `uncertified`，**不再冒用 `search_exhausted`**。
3. **零长度解是明确定义的一等成功，不再崩溃。** `source == target` 在 v0.1.2 抛
   `AttributeError: 'NoneType' object has no attribute 'segments'`；现在返回
   `success` + `zero_length_solution = True` + `cost = length = congestion = 0` +
   `path = None` + `n_segments = 0` + 可核对的端口误差，且**不伪造微小线段**。
   同一位置但目标切向不满足接受球、源 pose 在障碍内/在片外/在固定参考线上，仍然按输入非法拒绝。
4. **三维点到平面圆弧的距离已改正。** v0.1.2 用的是到**球面**的径向差 `abs(|p-C| - R)`：
   对 `p=(0,0,50)`、XY 平面 `R=50` 整圆报 `0`（真值 70.710678118654 um），
   并把一条合法三维避让路径判为 fail（采样最小值 0、认证自检失败）。
   修复后函数级给 70.71067811865476；完整路径 `(-10,0,50)->(10,0,50)` 越过 XY 整圆
   真值 64.031242374328 um，生产判 safe、独立验证判 **pass**，认证下界 42.800965523585 ≤ 采样上界，
   自检不再失败。真正穿越的路径仍然 fail。
5. **JSON pose 与圆弧 basis 的可假通过通道已关闭。** 非单位切向不再直接进 `acos`：
   `[1,1,0]` 斜向切向从 `0.0° / pass` 变为 `45.00000000000001° / fail`；
   零方向、NaN、±Infinity、缺字段、错误维度、非数值全部明确 `ValueError`。
   近单位 basis 从 "接受并仍按圆认证" 变为 "按 1e-12 容差拒绝"，
   114 843 条真实圆弧实测最坏单位缺陷 2.22e-16、最坏正交缺陷 1.39e-16，阈值有约 4504 倍余量。
6. **原跨墙结果逐字段不变。** native3d：`success`、cost = length = 594.1592653589794 um、
   75 次扩展、8 步 trace、7 段曲线、z 从 0 升到 100 um；2D：真实 `search_exhausted`、81 次扩展、
   `budget_exhausted = False`；无参考图回退。这些数字与冻结的
   `artifacts/v012/cases/core_case_finite_native3d_native3d.json` 逐字段一致（同一测试内直接断言）。
7. **全量测试真实结果：** 权威运行（第 2 次独立复跑）
   **collected 568，passed 568，failed 0，skipped 0，xfail 0，error 0，退出码 0，墙钟 222.58 s**
   （`artifacts/v013/logs/21_verification_final.log`）。第 1 次运行报 567 passed / 221.81 s / 退出码 0，
   两次均无任何失败项；差异原因无法事后确证，故以第 2 次为准并同时列出，详见 7.1。
   v0.1.2 是 308 passed / 213.12 s，本轮新增 260 个测试，未删除、未跳过任何用例。
8. **仍未解决的范围（不夸大、不隐藏）：** 自间距**没有**接入任何 verdict；障碍只支持 AABB；
   端口连接覆盖仍然有限（共面 Dubins 家族，无 LRL/RLR）；保证只对被构造的离散图和明确的数值容差成立；
   生产侧的线段/圆弧间距检查返回的是**保守下界**而不是精确距离；`tracemalloc` 峰值是
   **Python 跟踪分配峰值**，不是进程 RSS；全流程不含任何光学模型。

### 0.1 修改文件与关键函数（一览）

| 文件 | 关键函数 / 位置 | 修改内容 |
|---|---|---|
| src/native3d/search/finite_graph.py | `FiniteGraph.__init__` 构造快照、`graph_reuse_report()`、`route_finite_native3d` | A1 快照一致性 + 搜索前拒绝；A2 `_accept` 末端点/接缝拒绝、`verify_assembled_endpoints`、`port_assurance`/`endpoint_errors`、`constraint_failed`/`uncertified`；A3 零长度契约；C1 推导上界（去掉硬编码 `+4096`）、C2 计时相位拆分、C3 冷/热来源标记。706 行 → 1463 行 |
| src/native3d/search/statuses.py | 新文件 | 10 个互斥状态、`SEARCH_TERMINATIONS`、`ALL_STATUSES` 与逐状态语义 |
| src/native3d/search/astar.py | `SearchResult`（仅新增字段） | `zero_length_solution`、`n_segments`、`endpoint_errors`、`port_assurance`，并显式序列化 `"path": null`；代价与终止逻辑一字未改。368 → 423 行 |
| src/native3d/search/congestion.py | `identity()`（新增） | 拥塞场的稳定身份 token，内容型或文档化的回退；A1 依赖它判断成本场是否变过。98 → 136 行 |
| src/native3d/geometry/primitives.py | `CircularArc3D.__post_init__`、`basis_defects()` | B3：basis 归一化后按 `BASIS_UNIT_DEFECT_TOL` / `BASIS_ORTHOGONALITY_TOL` = 1e-12 拒绝；半径**从不**取整或对齐格点；长度/AABB/曲率/法向全部与实际存储几何一致。280 → 402 行 |
| src/native3d/validation/validate.py | `point_arc_distance(_witness)`、`_nearest_reachable_phase`、`read_pose_dict`、`POSE_CONTRACT`、`check_segment_dict`、`validate_zero_length_artifact`、`validate_artifact` | B1 三维点到平面弧真实距离 + 见证 + 相位映射；B2 严格 pose 契约；B3 与生产同阈值的 basis 检查；零长度独立验证。1087 → 1611 行 |
| src/native3d/io/artifacts.py | `finalise_artifact` | 零长度成功也做独立验证，不再因为 `path` 为假而跳过；证据目录默认 v013，引擎版本 0.1.3 |
| src/native3d/__init__.py | 模块文档与 `__version__` | 0.1.2 → 0.1.3 |
| tests/ | 新增 8 个文件、修改 0 个既有断言 | A1/A2/A3、B1/B2/B3、D 验收，以及 `v013_helpers.py`（把冻结快照加载为 `native3d_prefix`，使红→绿可由测试本身证明） |
| README.md | 全文 | 三种策略、公共 graph 复用契约、零长度契约、pose/basis 契约、保守下界与 tracemalloc 口径、限制 |
| docs/reports/native3d_v012_safe_finite_search_report.md | 顶部勘误横幅 | 只增加醒目勘误链接与三项收窄说明，正文一字不改 |

---

## 1. 每项反例的真实修复前/后结果、对应文件与函数

所有 "修复前" 数字来自 `artifacts/v013/probe/v013_prefix_replay.py`：它在同一个解释器里把
`artifacts/v013/prefix_src/` 作为独立包 `native3d_prefix` 加载，**运行的是真实的 v0.1.2 代码**，
不是事后重写的日志复刻。日志：`artifacts/v013/logs/00_prefix_v013_defect_replay.log`（退出码 0）。

### A1 不匹配的预建 graph 返回错误终点的 success

**文件与函数**：`src/native3d/search/finite_graph.py`
`FiniteGraph.__init__`（`pose_key_to_node` 与端口表）、`route_finite_native3d`
（旧代码第 631 行 `target_key = graph.node_pose_key(goal_node)`、第 688 行 `tuple(res.path.segments)`）。

**根因（两层）**：

1. `pose_key_to_node` 是用普通字典赋值建的，虚拟端口节点 `("target",)` 并不在里面；当某个格点节点的
   pose key 与目标 pose 相同时，键指向的是**格点节点**而不是目标节点，
   而旧代码第 625–630 行的守卫只比较字典**大小**，这种 "没增加条目" 的覆盖恰好躲过它。
2. 目标键取自**旧图**，成功判据是 `state.pose_key() == target_key`，因此搜索停在旧图上；
   `goal_pos_error_um` 又用**新 scene** 的目标算出来，只记录不判定；
   格点锚点与边界也是旧 scene 的，每条边更是按旧 scene 的 chip/障碍/参考线验证过的。
   `graph.fingerprint` 有 16 个键，但**没有任何代码读它**，且 `reference_centrelines` 只存数量。

**反例几何**（本报告与回归测试统一使用，且与任务书给出的机制完全一致）：
chip `[-150,150]^3`（h = 50 um、边界余量 2 um ⇒ 每轴格点 −2..2），
source `(0,0,0)` 切向 +x，scene A target `(100,0,0)`，scene B target `(50,0,0)`，无障碍/参考线。

    BEFORE（冻结 v0.1.2）  graph = FiniteGraph(scene A); route_finite_native3d(scene B, graph=graph)
        graph.source_identity_node = (0,0,0,0)      graph.target_identity_node = (2,0,0,0)
        graph.pose_key_to_node[pose_key(("target",))] = (2,0,0,0)   <-- 旧目标节点
        status = success      cost = 100.0     length = 100.0
        goal_pos_error_um = 50.0      actual endpoint = [100.0, 0.0, 0.0]
        fresh scene B（不注入）: status=success cost=50.0 endpoint=[50.0,0.0,0.0]
        stored fingerprint keys = 16；同数量不同几何的参考线产生**完全相同**的指纹

    AFTER（v0.1.3）
        status = graph_scene_mismatch       （不是 success，也不是 search_exhausted）
        path = None    cost_um_equivalent = inf    length_um = inf    goal_pos_error_um = None
        mismatches = ['target']
        mismatch_detail = {"target": {"expected": [50,0,0], "graph": [100,0,0]}}
        graph_cache_state = warm    successor_cache_hits = 0
        墙钟 0.000114 s（搜索完全没有开始）
        fresh scene B: status=success cost=50.0 endpoint=[50.0,0.0,0.0]（不变）

**修复**：见第 2 节。**回归**：`tests/test_v013_a_graph_contract.py`（20 项）、
探针 `artifacts/v013/probe/v013_finite_router_probe.py` 的 A1 段（41 项检查全 OK，退出码 0）。

### A2 success 不核对真正返回曲线的端口

**文件与函数**：`src/native3d/search/finite_graph.py`
`FiniteGraph._accept`、新增 `verify_assembled_endpoints(segments, scene, cfg)`、`route_finite_native3d` 的组装段。

**根因**：成功只由搜索的目标测试决定；组装后只跑 `validate_segments`（硬约束），
既不比较真实起止 pose，也不检查接缝 C0/C1，
更不拒绝 "边的曲线没有落在自己目标节点 pose 上" 的情形（旧代码只把误差记进
`end_position_error_um` / `max_curve_end_position_error_um`）。而且失败分支写成

    res.status = "uncertified" if undecided else "search_exhausted"

`search_exhausted` 的含义是 "OPEN 真的耗尽"，与 "搜索找到的路径不合法" 恰好相反。

**修复前实测**（冻结快照，源码审计）：`finite_graph.py` 中 `start_pose` 出现 0 次、
`source_pos_tol_um` / `source_ang_tol_deg` / `eps_ang_deg` 均出现 0 次、
`end_pose` 只出现 1 次；确认没有任何把组装曲线与调用方 pose 对比的代码。

**修复后实测**（跨墙真实成功）：

    status = success   n_segments = 7
    port_assurance.verified_against_the_caller_pose = True
    source pos/tangent error = 0.000e+00 um / 0.000e+00 deg
    target pos/tangent error = 0.000e+00 um / 0.000e+00 deg
    max junction pos/tangent = 0.000e+00 um / 3.508e-15 deg
    tolerances: source 1e-06 um / 1e-06 deg, target ball 2.0 um / 3.0 deg, C1 1e-06 um / 1e-06 deg
    所有被接受边都落在其规范目标 pose 上：end_pose_rejected = 0, junction_rejected = 0
       最大被接受端点误差 7.105e-15 um / 3.508e-15 deg
    辅助函数对 "起点偏 1e-4 um" 的路径给出 failed = ['source']，
       而终点误差 0 仍在目标接受球内 ⇒ 两套容差确实分离

**回归**：`tests/test_v013_a_port_and_status.py`（15 项）。

### A3 起终点相同的合法输入崩溃

**文件与函数**：`src/native3d/search/astar.py`（零长度成功返回 `path=None`）、
`src/native3d/search/finite_graph.py:688`（无条件读 `res.path.segments`）。

**修复前实测**（冻结快照，真实异常）：

    source = target = Pose3D((0,0,0),(1,0,0)); chip = [-60,60]^3; 默认配置
    AttributeError: 'NoneType' object has no attribute 'segments'
      File ".../prefix_src/search/finite_graph.py", line 688, in route_finite_native3d
        segments = tuple(res.path.segments)

**修复后实测**：

    status = success   zero_length_solution = True
    cost = 0.0   length = 0.0   congestion = 0.0
    path = None   n_segments = 0   expansions = 0   trace_steps = 1
    endpoint_errors: start 0.000e+00 um / 0.000e+00 deg, end 0.000e+00 um / 0.000e+00 deg
    2D 模式同一契约（mode = finite_native3d_2d）
    同位置但目标切向偏 10°   -> search_exhausted，zero_length_solution = False（不是零长度成功）
    源在障碍内 / 源在片外 / 源在固定参考中心线上 -> invalid_source_pose（输入验证没有被绕过）

**回归**：`tests/test_v013_a_zero_length.py`（12 项）。

### B1 点到空间圆弧的公式漏掉法向高度

**文件与函数**：`src/native3d/validation/validate.py::point_arc_distance`（旧 569–603 行）。

**根因**：`abs(norm(p-C) - R)` 是点到**球面**的径向差。它只在点落在圆弧平面内时才等于到圆的距离；
对位于圆心轴上的点返回 0，对高于平面的点严重低估。

**修复前实测**：

    参考圆：center=(0,0,0), u=+x, v=+y, R=50, theta=2*pi；p=(0,0,50)
      validator point_arc_distance = 0.000000000000 um        真值 70.710678118655 um
    完整合法路径 (-10,0,50)->(10,0,50)：
      verdict = fail        spacing status = safe（判定用的是错的数）
      min_sampled_um = 0.0  certified lb = 42.800965523585035  self_check_failed = True
      真值 sqrt(40^2+50^2) = 64.031242374328 um
      生产约束栈对同一条路径：status=safe, bound=50.000000000000 um（本来就对）

**修复后实测**：

    函数级  0.0 -> 70.71067811865476 um（分支 centre_axis，见证点 (50,0,0)，残差 0.0）
    完整路径 verdict fail -> pass，spacing safe，sampled 0.0 -> 64.031242374328，
      certified lb 42.800965523585 <= sampled，self_check_failed True -> False
    8*pi 多圈点 22.801098893 -> 28.284271247；倾斜平面 1.538442592 -> 19.985991420；
    端点最近情形不变 111.803398875；负扫角在弧上 0.0 -> 0.0
    控制组：真正穿越的路径 sampled 0.0 且 violation/fail；圆内合法绕行 40.0 um 且 pass

**回归**：`tests/test_v013_b_point_arc_distance.py`（79 项），其中至少一项直接在同一个测试里
用冻结快照打出 "PRE = 0.0" 与 "POST = 70.71067811865476"，使红→绿由测试本身证明。

### B2 JSON source/target 切向未校验或归一化，可假通过

**文件与函数**：`src/native3d/validation/validate.py` 的 `ang()`/`validate_path_dict`（旧 960–975 行）、
生产侧 `src/native3d/geometry/primitives.py::Pose3D.from_dict`。

**根因**：`ang(u, v)` 直接 `math.degrees(math.acos(clamp(dot(u, v))))`。
非单位切向 `[1,1,0]` 与 `[1,0,0]` 的点积是 1，`clamp` 后夹角 0° ⇒ `verdict = pass`。
生产侧 `Pose3D.from_dict` 是先 `unit()` 再存，所以两边语义不一致。

**修复前实测**：

    path (0,0,0)->(100,0,0)；source tangent [1,0,0]；target tangent [1,1,0]；默认配置
      validator end_tangent_error_deg = 0.0     target_pose_check.status = safe     verdict = pass
      真值 45°，默认目标容差 3°；生产 Pose3D.from_dict 归一化后 = (0.7071067811865475, 0.7071067811865475, 0.0)
    其他畸形 pose：
      零方向        -> 无异常, verdict=fail, start_ang_err=90.0
      NaN 方向      -> 无异常, verdict=pass, start_ang_err=0.0
      +Infinity 方向 -> 无异常, verdict=pass, start_ang_err=0.0
      2 分量切向    -> IndexError: list index out of range
      切向放大 1000 倍 -> 无异常, verdict=pass, start_ang_err=0.0

**修复后实测**：`45.00000000000001° / fail`；NaN/±Inf/4 分量/零/2 分量/非数值全部
`ValueError`（并指明是 `scene.source_pose` 还是 `scene.target_pose`）；
`[1000,0,0]` 与 `[3,0,4]` 的归一化结果与生产 `Pose3D.from_dict` **逐位相同**；
契约字符串发布在每次结果的 `pose_contract` 字段里。

**回归**：`tests/test_v013_b_pose_json_contract.py`（52 项）。

### B3 接受近单位 basis，却把实际椭圆当成圆弧认证

**文件与函数**：`src/native3d/geometry/primitives.py::CircularArc3D.__post_init__`（旧 177–184 行）、
`src/native3d/constraints/checks.py::radius_check`、`validate.py` 的圆弧输入与半径检查。

**根因**：构造器容差 1e-9 只检查 "范数接近 1" 与 "点积接近 0"，然后**原样保留** basis，
曲线因此是椭圆；而所有下游（长度、AABB、方向、`radius_check`、验证器的半径复算）
仍然把 `R` 当作圆的半径。

**修复前实测**：

    R=2000 um, u=(1+8e-10,0,0), v=(0,1,0), theta=pi/2
      norm(u)-1 = 8.000000661922968e-10  （构造器容差 1e-9）-> 构造器**接受**
      真实最小曲率半径 R/(1+8e-10) = 1999.9999984000 um
      允许下限 Rmin - radius_tol = 1999.999999 um      越界 6.000e-07 um
      生产 radius_check = safe (bound=2000.0)
      独立验证器 verdict = pass，min_radius_um_stored = 2000.0,
        min_radius_um_refit = 1999.9999997071807, radius_violations = []

**修复后实测**：构造器与 `check_segment_dict` 用**同一个** 1e-12 阈值，
对同一份输入都抛 `ValueError`；**关键点是必须测归一化之前的原始单位缺陷**——
`(1+8e-10,0,0)` 归一化后恰好是 `(1.0,0.0,0.0)`（后验缺陷 0.0），正交缺陷也是 0.0，
所以只有 `abs(norm(u_raw) - 1) = 8.000001e-10` 能抓住它；两个实现都先测原始缺陷。

**阈值依据（独立复核）**：对 `from_pose` 与 `cases.tilted_plane_arc` 的全部合法输入组合
（79 组基）公式化重算，最坏单位缺陷 1.110e-16，最坏正交缺陷 0.0；
随后在真实进程里对 **114 843 条实际构造的圆弧**插桩测量：

| 量 | 实测最坏值 |
|---|---|
| 输入 basis 的单位缺陷（归一化前） | 2.220446e-16（1 ulp） |
| 归一化后的单位缺陷 | 2.220446e-16 |
| 正交缺陷 `abs(dot(u_hat, v_hat))` | 1.387779e-16 |
| `abs(norm(start - center) - R)` | 5.684342e-14 um |
| 相对 `abs(\|P(phi)-C\| / R - 1)` | 1.332268e-15 |

1e-12 阈值相对真实构造有约 4504 倍余量，同时以 8.0e-10 / 1e-12 = 800 倍的差距拒绝反例。
**半径策略**：半径**不**取整、**不**对齐格点（`2000.3` 仍是 `2000.3`，
`1999.999999` 仍是 `1999.9999989999999`），文档化的不变量是
"存储半径就是存储正交基的半径"，数值误差范围如上表。

**回归**：`tests/test_v013_b_arc_basis_contract.py`（54 项）。

### PART C 审计点（不夸大为原核心案例失败）

| 项 | v0.1.2 | v0.1.3 |
|---|---|---|
| C1 边上界 | `N*(straight_count+4*radius_count) + 4096`，硬编码 4096 不是构造证明；节点上界不含虚拟端口但未说明 | 由实际节点/动作/端口配置推导：核心案例 `1440 * 11 + 1 = 15841`；显式给出 `node_upper_bound_includes_virtual_ports = False`、虚拟端口节点数 1、总数 1441；小场景真实枚举 522 条边 ≤ 推导上界 1782 |
| C2 计时口径 | `edge_check_s` 同时包含构图期端口附件检查与搜索期检查，却写着 "是 search_s 的子集" | 拆成 `edge_check_s_graph_build` + `edge_check_s_search`，二者之和等于 `edge_check_s`；实测核心案例 0.0000 s / 0.0217 s，且分别落在 `graph_build_s` / `search_s` 之内；`timings` 另增 `assembled_path_check_s` |
| C3 冷/热分支 | 只有单一计时，热缓存语义不明 | `graph_cache_state` 取 `cold`（本次调用自己建图）或 `warm`（调用方传入的图被复用），另有 `graph_build_count`、`graph_build_count_by_this_call`、`graph_prebuilt_construction_s`、`successor_cache_hits`；**60 s / 200 000 预算始终只属于冷默认分支**，热分支数字从不外推 |
| C4 内存口径 | 未说明 | 所有引用 `tracemalloc` 峰值的地方都标注为 **Python 跟踪分配峰值**，不是进程 RSS，也不含解释器与 C 层缓冲 |
| C5 状态口径 | `constraint_failed` 不存在；组装失败冒用 `search_exhausted` | `src/native3d/search/statuses.py` 集中定义 10 个互斥状态：success、search_exhausted、budget_exhausted、port_attachment_failed、graph_scene_mismatch、invalid_source_pose、invalid_target_pose、unsupported_pose、constraint_failed、uncertified；`SEARCH_TERMINATIONS = (success, search_exhausted, budget_exhausted)` |

---

## 2. 公共 graph 复用契约及一致性检查范围

**结论：保留 `graph=` 注入（公共签名不变），但接受预建图时必须通过完整不可变快照一致性检查，不匹配时在搜索前明确拒绝。**

`route_finite_native3d` 的签名保持
`(scene, mode, congestion, max_expansions, max_runtime_s, expand_hook, graph)`
（既有测试 `tests/test_v012_finite_graph.py::test_the_router_is_not_given_any_waypoint`
逐参数断言过它，因此不删除该接口）。

### 检查范围（全部按**内容**比较，不比较对象地址）

| 类别 | 具体字段 |
|---|---|
| 场景几何 | `chip` 的 lo/hi；**每个**障碍盒的 lo/hi；**每条**参考中心线的完整几何（line: `p0`/`p1`；arc: `center`/`u`/`v`/`radius_um`/`theta_rad`） |
| 端口 | source 位置与三维切向；target 位置与三维切向 |
| 模式 | `mode`（2d / native3d） |
| 动作与可行性 | `finite_spacing_um`、`finite_straight_steps`、`finite_turn_radius_steps`、`rmin_um`、`radius_tol_um`、`dmin_centerline_um`、`obstacle_clearance_um`、`obstacle_clearance_includes_envelope`、`wg_envelope_radius_um`、`boundary_margin_um` / `effective_boundary_margin_um`、`effective_obstacle_clearance_um`、`clearance_decision_tol_um`、`max_subdiv_level`、`connector_radii_um` |
| 端口接入 | `finite_port_attach_um`、`finite_port_attach_max_nodes` |
| 判定容差 | `pos_tol_um`、`ang_tol_deg`、`eps_pos_um`、`eps_ang_deg`、`source_pos_tol_um`、`source_ang_tol_deg` |
| 预算 | `finite_max_expansions`、`finite_max_runtime_s` |
| 成本场 | `congestion` 的 `identity()` 内容型 token；**无法证明是同一个场时一律判为不匹配**，绝不静默复用旧成本 |
| 契约版本 | `contract_version`，使未来的字段增删不会被误判为一致 |

共 35 个被检字段（`graph_reuse.checked_fields` 逐个列出）。

**为什么不能用地址比较**：`Scene` 是可变 dataclass。回归里专门有两条把
`scene.obstacles` / `scene.chip` 在**构图之后**就地改掉，仍然必须被拒。
**为什么不能只比参考线数量**：回归里有一条 "同数量、不同几何" 的线段与一条 "同数量、扫角翻转" 的圆弧，
两者的旧指纹完全相同（实测 `True`），现在指纹与检查都能分辨。

**合法复用与冷/热语义**：真正匹配的图可以合法使用，并且明确报告是
`cold`（本次调用自己构建）还是 `warm`（复用调用方传入的图），
附带 `successor_cache_hits` 等可核对计数。核心案例实测：

    cold 默认分支        wall 0.0512 s   graph_build 0.0077 s   state=cold   hits=0
    warm 第 1 次调用      wall 0.0458 s   graph_build 0.0000 s   state=warm   hits=0
    warm 第 2 次调用      wall 0.0054 s                          state=warm   hits=75
    三次调用 cost 完全相同 594.159265359 um

### 附带的双射修复

`pose_key_to_node` 现在是**真正的双射**：撞 key 会被记录（`graph.pose_key_collisions`，
正常情形为空字典）而不是静默覆盖，虚拟端口节点也被正确登记，
因此 `pose_key_to_node[node_pose_key(("target",))]` 就是 `graph.target_identity_node`
（核心案例的实测值 `(1,0,0,0)`）。旧代码那种 "用字典大小当守卫" 的写法已经不存在。

---

## 3. 零长度解、实际端口与错误状态的返回契约

### 3.1 零长度解

| 字段 | 值 | 说明 |
|---|---|---|
| `status` | `"success"` | 是成功 |
| `zero_length_solution` | `True` | 与失败区分开 |
| `cost_um_equivalent` / `length_um` / `congestion_cost` | `0.0` / `0.0` / `0.0` | 可核对 |
| `path` | `None` | **不伪造任何微小线段** |
| `n_segments` | `0` | 返回曲线不含任何基元 |
| `expansions` | `0` | 起点即满足目标测试 |
| `trace` | 1 步 | `<source>`，`n_segments = 0` |
| `endpoint_errors` | 起止位置/切向误差均为 `0.0`，含所用容差与 `start_ok`/`end_ok` | 可核对 |
| `port_assurance.zero_length_solution` | `True` | 与真实端口核对逻辑共用同一份报告 |

失败结果同样是 `path = None`，**区分它们的是 `zero_length_solution` 标志**，
这一点在路由器、`SearchResult.to_dict()`、JSON 序列化、独立验证器和 CLI 里语义一致：
`finalise_artifact` 不再因为 `path` 为假而跳过验证，`validate_artifact` 对
`zero_length_solution = True` 走零长度分支并给出 `verdict = "pass"` 与 `zero_length_solution: true`；
对一个既有 `path` 又不是零长度解的失败 artifact，验证器**明确报错**而不是报告 pass：

    ValueError: artifact.search has no path and is not a zero-length solution
    (status='graph_scene_mismatch', zero_length_solution=False): there is nothing to
    validate, and a failure must never be reported as a validation pass

不因 `source == target` 而绕过的输入验证：目标切向超出接受球、源在片外、源在障碍 keep-out 内、
源在固定参考中心线 dmin 之内，全部仍然按 `invalid_source_pose` / 非成功处理。

### 3.2 实际端口复验（success 的必要条件）

`verify_assembled_endpoints(segments, scene, cfg)` 返回：

* `ok` / `status`（`safe` 或 `constraint_failed`）/ `failed`（可含 `source`、`source_tangent`、
  `target`、`target_tangent`、`junction`）；
* `start_position_error_um`、`start_tangent_error_deg`；
* `end_position_error_um`、`end_tangent_error_deg`；
* `max_junction_position_error_um`、`max_junction_tangent_error_deg`、`junction_failures`；
* 所用容差：`source_pos_tol_um`、`source_ang_tol_deg`、`target_eps_pos_um`、`target_eps_ang_deg`、
  `pos_tol_um`、`ang_tol_deg`。

语义边界写进返回值本身：几何量是**达到的误差**而不是认证下界；
`ok = True` 只断言 "这条返回曲线真的起于源 pose（源容差）、真的落在目标接受球内、内部 C0/C1 连续"，
**不**断言曲线其它点安全——整条曲线的硬约束由 `constraints.checks.validate_segments` 单独判定。

**容差语义保持 v0.1.2**：源位置/切向用 `source_pos_tol_um` = 1e-6 um / `source_ang_tol_deg` = 1e-6 deg，
终点用接受球 `eps_pos_um` = 2.0 um / `eps_ang_deg` = 3.0 deg，接缝用 `pos_tol_um` / `ang_tol_deg` = 1e-6。
上限值确实是两个不同的容差族，回归里用 "起点偏 1e-4 um 而终点误差为 0" 的路径证明：

    failed = ['source']     start error 1.000e-04 um > source tol 1.0e-06 um
    终点误差 0.000e+00 um <= target ball 2.0e+00 um

每条边在进入 OPEN 前也必须落在其目标节点的规范 pose 上（位置 ≤ `pos_tol_um`，切向 ≤ `ang_tol_deg`），
否则拒绝、计数到 `edges_end_pose_rejected`、理由字符串 `curve_end_misses_the_canonical_pose`；
多段边（端口连接器）的每个内部接缝同样受检。真实几何的实测最坏值 7.105e-15 um / 3.508e-15 deg，
这些容差永远不构成约束。

### 3.3 一致性与状态

`cost = length + congestion`、`length = path.total_length`、`congestion ≥ 0`、
`goal_pos_error_um == port_assurance.target_position_error_um` 都有回归。
失败结果不带 path、不带 cost、不带任何可被读成成功的保证字段（`port_assurance` 为 `None`）。
组装失败按可判定性返回 `constraint_failed`（硬约束违反）或 `uncertified`（检查无法在细分上限内判定），
**永不** `search_exhausted`；`search_exhausted` 只用于 OPEN 真正耗尽。

---

## 4. 三维点-弧距离推导、witness 与上下界的物理/几何含义及单位

### 4.1 推导

参考圆弧在任意空间平面内，`C` 为圆心，`(u, v)` 为该平面内**正交单位**基，`n = u × v`，半径 `R`，
有向扫角 `theta`。取 `q = p - C`，并把 `q` 分解成平面内分量与法向分量：

    a = q·u        b = q·v        h = q·(u × v)        rho = sqrt(a² + b²)

则 `|q|² = rho² + h²`（单位：um²）。点到**整圆**的距离：

    d_circle = sqrt((rho - R)² + h²)

`h` 就是题目里 "漏掉的法向高度"：v0.1.2 的 `abs(|q| - R)` 等于
`abs(sqrt(rho² + h²) - R)`，只有在 `h = 0` 时才等于上式。`rho = 0`（点在圆心轴上）时
上式退化为 `sqrt(R² + h²)`，**必须单独处理**，绝不能返回 0。

### 4.2 扫角与相位（这是最容易写错的地方）

弧度参数 `phi` 是**平面内的几何角**，而 `theta` 是**转向角**：参数化
`P(phi) = C + R(cos phi · u + sin phi · v)` 下，`phi` 从 0 转到 `theta` 即 `P(0) = C + R u`
转到 `P(theta) = C + R(cos theta · u + sin theta · v)`，弧长是 `R |theta|`（既有实现的约定，
不能改动，否则 v0.1.2 的圆弧素语义会变）。于是

    phi(psi) = psi            （theta > 0）
    phi(psi) = psi + theta    （theta < 0）

因此已知最接近的整圆相位 `phi* = atan2(b, a)` 后，必须先把它折算成转向角
`psi* = (phi* mod 2π)`，再对正扫角取 `psi* - 2π`（直到 `psi* ≥ -theta`）、
对负扫角直接比较 `psi* ≤ theta`，才能判断该相位是否真的可达。
`psi*` 落在 `[0, theta]` 时距离就是 `d_circle`，否则最小值必在两个弧端点之一
（到圆弧的距离沿扫角单峰，端点检查是完备的）。

### 4.3 见证点与上下界（单位：um）

* **采样估计 / 达到的上界**：有限个**曲线上的点**测得的距离最小值，只能偏大，
  所以它是真实最小距离的**上界**；一个落在 keep-out 内的曲线点就是**可靠违规见证**。
* **认证下界**：弦细分减去 sagitta 得到的数，**永不超过**真实最小距离；
  `safe` 判定要求 `下界 ≥ 要求值 − 容差`。
* **未认证**：细分到上限仍无法判定 ⇒ `undecided`，最终 verdict 是 `uncertified`，**绝不当作 pass**。
* **见证点核对**：返回的距离必须与一个**真实存在的曲线点/参数**对应；
  回归里逐条断言 `abs(|witness − p| − reported_distance) ≤ 1e-9`。
  例：`p=(0,0,50)` 对 XY 整圆（R=50）的见证点是 `(50,0,0)`，残差 `0.0`。
* **自检**：认证下界高于采样上界在数学上不可能；一旦出现就置
  `self_check_failed = True` 并让 verdict 不再是 pass。v0.1.2 在合法跃迁路径上真的触发了这个自检
  （下界 42.800965 > 采样 0.0），修复后不再触发。

**量纲**：全程 um（长度）与 rad（几何角），配置对象里的角度是 deg；
`point_box_distance` 等返回 um；`h`、`rho`、`R`、`d` 均为 um。

---

## 5. JSON pose / basis 解析契约与数值误差范围

### 5.1 pose 契约（生产与独立验证器一致）

* 位置：恰好 3 个**有限**数；
* 切向：恰好 3 个**有限**数且**非零**；合法非零三维方向**先归一化再使用**；
* 明确拒绝：零方向、NaN、±Infinity、缺字段、维度错误（2 或 4 分量）、非数值；
* 该语义与生产 `Pose3D.from_dict` **逐位一致**（回归直接比较归一化结果）；
* 契约字符串发布在每个验证结果的 `pose_contract` 字段里。

**数值误差范围**：归一化在双精度下引入的相对误差 ≤ 1 ulp（≈ 2.2e-16）；
`acos` 的输入被 clamp 到 [-1, 1]，比较是 "误差 > 容差" 的严格不等式，
因此容差边界处的判据不依赖点积的符号误差；`[1,1,0]` 这类斜向切向不再被 clamp 掩盖
（实测 `45.00000000000001°`，真值 45°，差 1.0e-14 deg）。

### 5.2 圆弧 basis 契约

* `u`、`v` 先归一化，然后检查两个数：**归一化前的单位缺陷** `abs(norm(u_raw) - 1)`（以及 `v`）
  与归一化后的正交缺陷 `abs(dot(u_hat, v_hat))`；
* 阈值 `BASIS_UNIT_DEFECT_TOL = BASIS_ORTHOGONALITY_TOL = 1e-12`，**生产与验证器同值**；
* 超限即 `ValueError`，**不做** "先悄悄改几何再对原几何宣称认证" 的事；
* 半径**不**取整、**不**对齐格点；文档化不变量 = "存储半径即存储正交基的半径"；
* 长度 `R|theta|`、`min_radius_um = R`、`curvature = 1/R`、`normal = u × v`、
  解析 AABB、`point_at_phi`、`tangent_at_phi` 全部与实际存储几何一致，`to_dict`/`from_dict` 对合法弧无损。

**误差范围（114 843 条真实圆弧实测）**：输入单位缺陷 ≤ 2.220446e-16，
归一化后单位缺陷 ≤ 2.220446e-16，正交缺陷 ≤ 1.387779e-16，
`|norm(start - center) - R|` ≤ 5.684342e-14 um，相对 `||P(phi)-C|/R - 1|` ≤ 1.332268e-15。
阈值相对真实构造余量约 4504 倍。

---

## 6. 原跨墙与新增合法三维场景的验收结果

### 6.1 原跨墙场景（必须不变，实测不变）

| 量 | v0.1.2 冻结值 | v0.1.3 实测 |
|---|---|---|
| native3d status | success | success |
| cost / length | 594.1592653589794 um | 594.1592653589794 um |
| congestion | 0.0 | 0.0 |
| expansions | 75 | 75 |
| trace 步数 | 8 | 8 |
| path 段数 | 7 | 7 |
| z 范围 | 0.0 → 100.0 um | 0.0 → 100.0 um |
| 2D status / expansions | search_exhausted / 81 | search_exhausted / 81（`budget_exhausted = False`） |
| 参考图回退 | 无 | 无（`reference_graph = None`） |

trace 动作序列逐项相同：
`<source>, straight_2, turn_+1+0+0_to_+0+0+1_R50, turn_+0+0+1_to_+1+0+0_R50, straight_3,
turn_+1+0+0_to_+0+0-1_R50, turn_+0+0-1_to_+1+0+0_R50, port_target_straight`，
且测试**直接在同一个用例内**与冻结 JSON 逐字段比对（动作序列、expansions、cost、段数、
`trace_z_summary` 的 z_min/z_max/z_varies），因此 "不变" 是被断言的事实而不是记错的印象。

**独立最短路径交叉验证**：对完全枚举的有限图跑一份写在探针里的普通 Dijkstra
（不使用 `search/astar.py`）：`best = 594.159265359 um`，与 A\* 的
`diff = 0.000e+00`，7867 条边，678 个节点 settled（与 v0.1.2 的 678 一致）。
> 这里曾经踩过一个坑并已修好：枚举出的边从**虚拟端口节点** `("source",)` / `("target",)` 出发，
> 若把 Dijkstra 的起点写成 `graph.source_identity_node`（可能为 `None`）、
> 或把终点写成裸字符串 `"target"` 而不是元组 `("target",)`，都会得到 `inf`。
> 现在探针先断言起点/终点确实存在于边集中，再跑 Dijkstra，并断言 `best < inf`。

### 6.2 新增：完整合法三维路径高于参考圆弧（v0.1.2 只有点距离单元测试）

参考为 XY 平面整圆 `{(50 cos p, 50 sin p, 0)}`；路径沿 x、位于高度 z。

| 场景 | 真实最小距离 | 生产约束栈 | 独立验证器 |
|---|---|---|---|
| 合法跃迁 `(-10,0,50)->(10,0,50)` | 64.031242374328 um | safe（保守下界 50.0 um） | **pass**，sampled 64.031242374328，lb 42.800965523585 ≤ sampled，`self_check_failed = False` |
| 同路径 z = 10 | 41.231056256 um | safe | pass |
| 同路径 z = 25 | 47.169905660 um | safe | pass |
| 同路径 z = 0（**真的穿越**） | 0.0 um | violation | **fail**，sampled 0.0 |
| 圆内合法绕行 `(-10,0,0)->(10,0,0)` | 40.0 um | safe | **pass**，sampled 40.000000 |
| 参考圆 R=12（真的太近） | 2.0 um < dmin 5 | violation | **fail**，sampled 2.000000 |
| 同路径 z = 90 | 98.488578018 um | safe | pass |

前两行是 "v0.1.2 误拒的合法三维避让"，第 4/6 行是 "真违规仍然失败" 的控制组，
第三个控制组 "圆内合法绕行 40 um" 是旧公式最尖锐的反例（旧公式给 0 并判 fail）。
`tests/test_v013_d_acceptance.py` 还用一份**独立的 20 万点暴力采样**校验测试里用的闭式
`d = sqrt((hypot(x0,y0) - R)² + z0²)`，避免期望值本身写错。

---

## 7. 全量测试、冷/热计时与内存口径

### 7.1 全量测试（真实 collected/passed/failed/skipped/xfail/error）

    命令  : .venv\Scripts\python.exe -m pytest -q --no-header -p no:cacheprovider -rfE
    目录  : C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D

    第 1 次（13:57 结束，20_final_full_pytest.log）：567 passed in 221.81s   退出码 0
    第 2 次（14:01 结束，21_verification_final.log 旧版）：568 passed in 222.19s  退出码 0
    第 3 次（14:07 结束，**最终树、最终证据版**，21_verification_final.log）：568 passed in 220.29s  退出码 0

    权威结论（第 3 次，`-rfE` 无任何 short test summary）：
        collected 568，passed 568，failed 0，skipped 0，xfail 0，error 0，
        exit code 0，墙钟 220.62 s

**不沿用旧的 308 passed。** v0.1.2 是 308 passed / 213.12 s，本轮新增 **260** 个测试。
三次运行都**没有任何** failed / skipped / xfail / error，也没有删除或跳过任何测试。

> **口径诚实说明**：第 1 次运行报 567 passed，第 2、3 次都报 568 passed。
> 时间线显示第 1 次运行结束时（13:57）当前树仍是改版本号之前的常量（`EVIDENCE_VERSION="v012"`、
> 引擎串 `0.1.2`，13:57:48/50 才改成 v013/0.1.3），也就是说第 1 次是不完整/中间态的树。
> 第 2、3 次互相一致，且第 3 次运行在最终树上，因此以第 3 次为准；本报告同时列出三次结果，
> 不挑选、不粉饰。这一次版本号改动还连带造成了一次冻结证据覆盖事故，见 7.7，已还原。

各文件收集数与关键结果：

| 文件 | 数量 | 文件 | 数量 |
|---|---|---|---|
| test_v013_b_point_arc_distance.py | 79 | test_v013_a_graph_contract.py | 20 |
| test_v013_b_arc_basis_contract.py | 54 | test_v013_a_port_and_status.py | 15 |
| test_v013_b_pose_json_contract.py | 52 | test_v013_a_zero_length.py | 12 |
| test_v013_d_acceptance.py | 28 | v0.1.3 新增合计 | 260 |

### 7.2 计时分解（核心案例，冷默认分支）

    端到端 0.1864 s（墙钟 0.1883 s）  ≤ 60 s 预算    expansions 75 ≤ 200 000
      graph_build_s             0.0259 s
      search_s                  0.1595 s
      assembled_path_check_s    0.0005 s
      独立验证（路由器之外的一次 pass） 0.0140 ~ 0.0171 s
    逐边连续检查相位拆分: graph_build 0.0000 s + search 0.0217 s = edge_check_s

**冷/热严格分开**（见第 2 节）：`cold` 0.0512 s、`warm` 第一次 0.0458 s、`warm` 第二次 0.0054 s；
三者的 cost 完全相同。60 s / 200 000 expansions 预算**只**属于冷默认分支。

### 7.3 内存口径

核心案例的 `tracemalloc` 峰值为 **1.776 MiB**，明确标注为
"**Python 跟踪分配峰值**：只覆盖被跟踪区间内由 Python 分配器记录的分配，
**不是**进程 RSS，也不包含解释器自身、C 层缓冲以及在 `tracemalloc.start()` 之前发生的分配"。
本轮所有引用该数字的探针与报告都带这句话。

### 7.4 独立验证器 CLI

    $env:PYTHONPATH = "src"
    .venv\Scripts\python.exe -m native3d.validation.validate artifacts\v013\cases\core_case_finite_native3d_native3d.json
      -> verdict "pass"，total_length_um_analytic 594.1592653589794，n_segments 7，max_junction_tangent_error_deg 0.0，退出码 0
    .venv\Scripts\python.exe -m native3d.validation.validate artifacts\v013\cases\v013_zero_length_solution.json
      -> verdict "pass"，zero_length_solution true，退出码 0
    .venv\Scripts\python.exe -m native3d.validation.validate artifacts\v013\cases\v013_graph_reuse_mismatch.json
      -> ValueError（"no path and is not a zero-length solution"），退出码 1

验证器仍然只导入标准库、**不导入** native3d（既有 AST 检查通过）。

### 7.5 证据清单

    artifacts/v013/cases/
      core_case_finite_native3d_native3d.json    原跨墙（native3d）
      core_case_finite_native3d_2d.json          原跨墙（2D 真实耗尽）
      v013_legal_3d_above_reference_arc.json     合法三维跃迁 + 三个控制组
      v013_zero_length_solution.json             零长度契约
      v013_graph_reuse_mismatch.json             被拒绝的错配图
      v013_acceptance_summary.json               本次全部数字
    artifacts/v013/logs/
      00_prefix_v013_defect_replay.log(.exit.txt)  修复前反例（运行冻结 v0.1.2 代码）
      01..07_partA_*.log                          PART A 红基线 / 探针 / 回归 / 绿
      02_partB_validation_probe.log               PART B 修复前 vs 修复后
      10_v013_acceptance_probe.log(.exit.txt)     v0.1.3 验收探针
      11/12_independent_validate_*.log(.exit.txt) 独立验证器 CLI
      20_final_full_pytest.log(.exit.txt)         全量测试第 1 次（567 passed, exit 0）
      21_verification_final.log(.exit.txt)        全量测试第 3 次 = 最终验收（568 passed, exit 0）
      30_v012_evidence_restore.log                v012 证据还原运行（见 7.7 事故披露）
      40_final_review.log(.exit.txt)              Lead 独立复核 64 项检查（见 7.8）
    artifacts/v013/probe/
      v013_prefix_replay.py      修复前反例回放（加载 prefix_src）
      v013_finite_router_probe.py PART A 探针（41 项检查，含 RED 段与 --prefix-tests）
      v013_validation_probe.py   PART B 探针
      v013_acceptance_probe.py   v0.1.3 验收探针
    artifacts/v013/prefix_src/   逐字节 v0.1.2 源码快照（24 个 .py）

`artifacts/cases/`、`artifacts/logs/`、`artifacts/v01_preserved/`、`artifacts/v011/`、
`artifacts/v011_preserved/`、`artifacts/v012/logs/`、`artifacts/v012/probe/` 与
`artifacts/v012/cases/v012_core_acceptance_summary.json` 在整轮修复中**未被写入**
（实测最新 mtime 分别为 00:27、12:04、12:35、12:50）。

### 7.6 红基线的真实性

红基线不是事后编写的日志，而是**同一进程里加载冻结快照的真实运行**：

* `00_prefix_v013_defect_replay.log`：直接运行 `native3d_prefix`（v0.1.2 源码），
  A1 得到 `success / cost 100.0 / endpoint (100,0,0) / goal_pos_error_um 50.0`，
  A3 得到真实 `AttributeError`，B1 得到 `0.0`，B2 得到 `0.0° / pass`，B3 得到 `safe / pass`；
* `tests/v013_helpers.py` 提供同一加载器，于是 B1/B2/B3 的回归文件里各有一条
  "用冻结快照取出修复前的值" 的测试，红→绿由 pytest 本身而非文字证明；
* `v013_finite_router_probe.py --prefix-tests` 用冻结快照执行**本轮新增的 A 回归文件本身**：
  **29 failed, 3 passed, 1 error**（退出码 1），而同一批文件在当前代码上 47 passed。

### 7.7 一次真实发生的冻结证据覆盖事故，以及它的还原（必须披露）

**事故**：`artifacts/v012/cases/` 下 7 个 JSON 在 14:00:31 被**覆盖**。
经过时间线比对可以确定原因：`src/native3d/io/artifacts.py` 的
`EVIDENCE_VERSION` 在 13:57:50 才从 `"v012"` 改为 `"v013"`，而 13:57:32 前后启动的全量
pytest 进程导入的是**改版本之前**的常量和 `0.1.2` 引擎串，pytest 的会话 fixture
（`tests/conftest.py`）每次运行都会重新生成核心案例证据，于是它把 **v0.1.3 形状**的内容
写进了 **v012** 的目录。事后核对被覆盖文件的确含有 v0.1.3 新增字段
（`zero_length_solution`、`port_assurance`），证实了这一判断。

**影响范围**：仅这 7 个文件；`v012_core_acceptance_summary.json`（12:27:05）、
`artifacts/v012/logs/`、`artifacts/v012/probe/` 以及更早的 v01/v011 证据均未受影响。

**还原做法**：用本轮自己冻结的 v0.1.2 源码快照 `artifacts/v013/prefix_src/` **重跑 v0.1.2 测试套件**，
并由 `NATIVE3D_ARTIFACT_DIR` 把证据直接写回 `artifacts/v012/cases/`：

    artifacts/v013/_v012_restore_env/   （临时目录：prefix_src 作为 src/native3d + 当前 tests/）
    NATIVE3D_ARTIFACT_DIR=artifacts/v012/cases
    pytest tests/test_phase6_core_acceptance.py tests/test_phase6_cases.py tests/test_v012_*.py
      -> 201 passed, 1 failed, 58.24 s
    日志：artifacts/v013/logs/30_v012_evidence_restore.log

唯一的失败项是 `test_v012_a1_pose_identity.py::test_the_retracted_claims_are_gone_from_the_sources`：
它检查的是**工作目录下的当前源码**文本，而本次运行的工作目录换成了临时还原目录，
所以它不是 v0.1.2 行为失败。**证据生成本身全部成功。**

**还原后的性质（不夸大）**：这 7 个文件是 **v0.1.2 形状**的证据——`engine` 字段为
`native3d v0.1.2`，且**不含**任何 v0.1.3 新增字段；核心数值与未被覆盖的冻结摘要一致
（cost 594.1592653589794 um、75 次扩展、trace 8 步、7 段；2D `search_exhausted` / 81）。
但它们**不是**原来的字节副本：`generated_at_utc`、计时与内存字段天然会不同，
个别探针字段（如运行命令串）也可能有差异。因此本报告在引用 v0.1.2 结果时，
凡涉及**数值结论**都以未被覆盖的
`artifacts/v012/cases/v012_core_acceptance_summary.json`（12:27:05，未改动）以及
`artifacts/v012/logs/`（未改动）为准，而不依赖这 7 个被重建的文件。
还原后各文件的 SHA-256 前 16 位：

| 文件 | 字节 | SHA-256(前16) |
|---|---|---|
| core_case_2d_mode.json | 21783 | A24F1738E8BCF102 |
| core_case_finite_native3d_2d.json | 17269 | 5C93285B41FE66CC |
| core_case_finite_native3d_native3d.json | 43519 | 40CDB5858A35E22C |
| core_case_initial_action_probe.json | 58392 | DED0C56166572A4C |
| core_case_native3d_mode.json | 44833 | 60355DAB3F53DF63 |
| core_case_reference_graph_2d.json | 61953 | A2C6660FE1CD9680 |
| core_case_reference_graph_native3d.json | 202292 | F25B99BDD8B8C295 |
| v012_core_acceptance_summary.json（未动） | 12034 | D78698BEF0647873 |

**根因与后续防呆建议**：`tests/conftest.py` 的会话 fixture 在**每次** pytest 运行时都会重写
核心案例证据，目标目录只由 `EVIDENCE_VERSION` / `NATIVE3D_ARTIFACT_DIR` 决定。
在没有 git 的工作区里，"冻结" 只靠约定，没有任何机制阻止写入。本轮**没有**改
`tests/conftest.py`（它在本轮的写范围之外，且改动它会影响证据生成语义），
下一轮最小改动建议是把它改成：证据目录已存在且内容版本与当前引擎版本不一致时
**拒绝写入并给出明确错误**，或默认写入 `artifacts/<version>/` 之外的新目录；
无论哪种，`v012` 这类历史目录都应当在代码层面被声明为只读。

**已验证的防呆现状**：版本号统一之后（第 3 次全量测试）再次运行完整套件，
`artifacts/v012/cases/` 的 mtime 保持 14:03:13 不变，证据全部写入 `artifacts/v013/cases/`
（同一时间戳 14:07:12），即**在最终树上不再发生**同类覆盖。

### 7.8 Lead 独立复核（对抗性检查，与项目自身测试无关）
`artifacts/v013/probe/v013_final_review.py` 是 Lead 独立写的复核脚本，
不复用项目测试或探针的判定，凡能重算的都重算。**64 项检查全部通过，退出码 0**
（日志 `artifacts/v013/logs/40_final_review.log`）：

| 组 | 检查数 | 内容 |
|---|---|---|
| A1 | 22 | 任务书反例；**逐个**单独改动 source/target/tangent/障碍/参考线/chip/spacing/步长/转弯半径/dmin/clearance/envelope/接入半径/接受球/源容差（各 1 项）；2D↔native3d 双向混用；拥塞场改变；匹配图仍成功且报 warm |
| A2 | 7 | success 发布可核对的实际端口；`goal_pos_error_um` 等于核实后的终点误差；返回曲线真的止于调用方目标；5 um 终点偏差被拒（接受球 2 um）；1 um 偏差在球内被接受并如实报数；5e-6 um 起点偏差触发**源**容差；5e-7 um 起点偏差被接受 |
| A3 | 8 | 零长度解的全部字段；失败与零长度的区分；JSON 往返；同位置错切向不是零长度成功；`source==target` 不绕过输入验证 |
| B1 | 8 | 对**自身曲线**做 40 万点密集极小化的对照：320 个随机倾斜/负扫角/多圈圆弧，最坏超出量 1.137e-13（即该值确实是达到距离/上界）；再以 20 万点复核 100 个案例，最坏偏差 1.309e-07（采样分辨率）；圆心轴点非零；60 个随机案例的见证点确实落在曲线上且弧长在段内；合法跃迁被接受；认证下界不超过采样上界；真穿越仍失败；圆内合法绕行被接受 |
| B2 | 3 | 45° 反例；6 个合法非单位切向的角度与**生产归一化后的切向**逐位一致、且 verdict 随容差正确翻转；16 个畸形 pose 全部 ValueError、无一 pass |
| B3 | 4 | 生产构造器与独立验证器拒绝同一 basis；两个阈值都是 1e-12；被拒几何确实不可作为圆（1999.9999984 < 1999.999999） |
| 原案例 | 6 | 跨墙 native3d/2D 结果、trace/段数、z 剖面、无参考图回退、与冻结 artifact 逐字段一致 |
| 范围 | 6 | 10 个状态无重复；自间距仍记 NOT IMPLEMENTED；引擎内无 dB；边上界 = 两个构造项之和（15841 = 15840 + 1，无硬编码余量）；真实枚举 4350 ≤ 上界 8250；节点上界明确不含虚拟端口 |

**复核过程中发现并修正的都是复核脚本自身的错误**，如实记录，因为它们说明这个对照是真的在起作用：

1. 脚本最初用自己手写的闭式公式当参照，把转向角错误地套在平面角 `phi` 上，
   于是对负扫角报出 2.5e2 量级的"不一致"。改用对**实现自身曲线**做密集极小化后，
   960 个随机案例一致到采样分辨率（最坏 7.5e-06），证明是脚本错、实现对。
   这条经验已写进脚本注释，防止后来者重犯。
2. 脚本曾把"合法非单位切向"的期望写成"角度为 0"，实际上 `[3,0,4]` 归一化后与路径 +x 真的相差 53.13°，
   改为与生产归一化结果逐位比较。
3. 脚本曾用字符串 `"+ 4096" not in 源码` 判断硬编码余量，但该串只出现在
   "v0.1.2 的 +4096 已移除" 的**注释**里；改为直接校验上界与两个构造分项的关系，这才是真正的判据。

---

## 8. 尚未解决的范围

以下各项在本轮**没有**解决，报告不把它们夸大为原核心案例失败，也不把它们说成已修好。

1. **自间距未接入。** 搜索状态不含路径历史，`constraints.checks.path_self_spacing()` 虽然存在，
   但生产路由器、结果组装、独立验证器**都不调用它**，它不进入任何 verdict；
   artifact 里以 `self_spacing_check: "NOT IMPLEMENTED..."` 明示。
   本轮按计划**没有**扩展路径历史搜索。
2. **障碍只支持 AABB**（无网格、无 CSG）。因此 "障碍" 的语义是轴对齐盒体，
   斜置或曲面结构的保真度不在本轮范围。
3. **端口连接覆盖有限。** 任意（非规范）端口切向走 v0.1.1 的共面 Dubins 家族
   （LSL/RSR/LSR/RSL，无 LRL/RLR）；找不到连接器/接入只意味着 "所支持的动作集没找到"，
   状态是 `port_attachment_failed`，**永不**等于物理不可达。
4. **离散图保证。** 成本最优性只对**被构造出来的有限图**成立，且带
   `g_improves` 的 1e-12 绝对 + 1e-12 相对比较容差，是 ε-最优而不是实数域最优；
   格点 + 四分之一圆动作无法复现任意连续路径（核心案例 594.159 um vs 参考图 495.301 um）。
   `search_exhausted` 只说明这张图耗尽，不是连续位姿空间不可达。
5. **数值容差边界。** 认证判定适用的尺度是 1e-9 um ~ 1e9 um；
   低于 `SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9 um` 或超出浮点条件数时回退到精确有理运算。
   B3 的 basis 容差 1e-12（实测真实构造最坏 1.4e-16 ~ 5.7e-14，余量约 4504 倍）；
   超出该容差的输入被**拒绝**，不是被静默修正。
6. **生产侧间距判定是保守下界。** `primitive_spacing` 的线段/圆弧分支用弦细分减 sagitta，
   返回**认证下界**而不是精确距离：在 64.031242 um 的合法跃迁上它报 50.0 um 并正确判 safe。
   若需要精确的线/弧距离下界，需要新的解析实现（本轮未做，也不影响 safe/violation 的正确性）。
7. **不含光学模型**：无损耗、无串扰、无 dB（AST 检查保证）。
8. **单网、固定拥塞场**：无多网调度、无三维占用更新、无 rip-up 重布、无历史拥塞。

---

## 9. 下一阶段的最小设计建议（仅建议，本轮不执行）

按依赖顺序，**先做前两项，两项通过后再考虑 16 通道**；本轮到此停止。

### 9.1 第一步：接入真实自间距验收，并明确 "局部接缝豁免" 的几何定义

现在的 `path_self_spacing` 只豁免 `|i - j| == 1` 的相邻段，理由只是 "共享端点处距离恒为 0"。
这不是可接受的最终定义：沿同一条直线上相邻两段在端点附近本来就该允许重合，
而 "共享端点" 不等于 "允许靠近"。建议的最小设计：

* 把豁免定义在**弧长参数**上，而不是段索引上：对同一段与它自身的邻域，
  以及相邻段在**接缝附近**的 `s ∈ [0, ε_joint]` 区间内豁免，`ε_joint` 取
  `max(接缝侧圆弧的弦长步长, 直线段的最小可分辨长度)`，并在 artifact 里记录 `ε_joint` 的实际取值；
* 对**非相邻**段一律在**全部参数对**上求最小距离（现有 `pair_clearance_bounds` 已经是全参数对，
  这一点可以直接复用），**不得**退化成 "同参数点比较"；
* 对 "路径回溯"（沿原路折返）这一合法形态单独给出规则：折返段之间的距离为 0 是**几何事实**，
  要么在动作集层面禁止立即折返，要么在验收里把它作为一个**允许的零距离形态**显式列出并计数；
* 结论必须落成三态（safe / violation / undecided），并且 `undecided` 不能算通过。

### 9.2 第二步：两条路径的顺序固定占用

第一条路径的全部直线与圆弧成为第二条路径的**参考中心线**，第二条在第一条的 keep-out 之外布线。

* 复用现有 `reference_centerlines` 机制（它已经支持 line 与 arc 的四种组合），
  把第一条路径的段直接追加进去；
* **缓存必须正确失效**：本轮已经建立的 `graph_scene_fingerprint` 覆盖每条参考中心线的完整几何，
  第一条路径一变，第二个网看到的指纹就变，预建图会被 `graph_reuse` 判为不匹配并拒绝 —— 这条链路
  恰好是第二步的现成基础，需要补的是 "同一进程内两条路径的产出与占用顺序" 的显式状态机；
* 需要新的失败语义：第二条不可达时，**不能**归因于第一条的 "障碍"，
  必须是 "在已占用空间与支持动作集下未找到解"，并明确区分
  `search_exhausted`（该网自己的有限图耗尽）与 `budget_exhausted`。

### 9.3 第三步（前两项通过后）：16 通道确定性顺序基线

固定顺序、确定性、无损耗模型、无 ML；每通道记录端口接入、扩展数、代价、状态与独立验证结论。
在此之前不建议引入任何复杂损耗模型或学习方法。
