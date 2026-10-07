# Native3D v0.1.2 残余正确性修复与有限多跳三维搜索报告

> ## ⚠️ 勘误（v0.1.3）：本报告以下结论已被收窄，请勿按原文范围引用
>
> **勘误与后续报告：[native3d_v013_contract_geometry_repair_report.md](native3d_v013_contract_geometry_repair_report.md)**
>
> 本文档保留 v0.1.2 的原始结果与历史，正文一字不改。以下三项在 v0.1.3 中被真实复现并修复：
>
> 1. **公共 graph 复用接口会返回错误的 success（PART A1）。** 把 scene A 预建的 `FiniteGraph`
>    传给 `route_finite_native3d(scene B, graph=graph)` 时，目标节点键来自旧图，成功判据只看旧图的目标键，
>    组装复验又从不核对真正输入的 source/target pose。实测反例：scene A target=(100,0,0)、scene B target=(50,0,0)，
>    得到 `status=success`、`cost=50`、实际终点 `(100,0,0)`、`goal_pos_error_um=50`。
>    复现日志：`artifacts/v013/logs/00_prefix_v013_defect_replay.log`；冻结快照：`artifacts/v013/prefix_src/`。
>    本报告"图/成本指纹已保证缓存安全"的说法**只覆盖同一 scene 内部的边缓存**，从未覆盖跨 scene 的图复用。
> 2. **零长度解会让公共入口崩溃（PART A3）。** `source == target`（位置与切向相同、默认配置、无障碍/参考线）
>    在 v0.1.2 抛出 `AttributeError: 'NoneType' object has no attribute 'segments'`；
>    原因是 A\* 的零长度成功返回 `path=None`，而有限 Router 无条件读取 `res.path.segments`。
> 3. **独立验证器的三维几何与 JSON 契约不闭合（PART B1/B2/B3）。**
>    `validation/validate.py::point_arc_distance` 用的是到球面的径向差 `abs(|p-C|-R)`，不是到平面圆弧的距离；
>    对 `p=(0,0,50)`、XY 平面 R=50 整圆报 `0`（真值 70.710678），并把合法三维避让路径判为 fail；
>    JSON source/target 切向未归一化即进入 `acos`，`[1,1,0]` 斜向切向得到 0° 与 `pass`（真值 45°）；
>    `CircularArc3D` 以 1e-9 容差接受近单位 basis 后仍按圆认证，实际曲线是椭圆。
>
> 本报告其余结论（A\* 终止条件、有限多跳整数节点图、六轴向方向、跨墙场景 594.159265 μm / 75 次扩展、
> 2D 真实耗尽、自间距**未实现**、离散动作与数值容差的范围限制）在 v0.1.3 中仍然成立，并有回归保护。
>
> 勘误对象：docs/reports/native3d_v011_correctness_report.md（只增加 v0.1.2 勘误横幅，正文与历史全部保留）

项目目录：C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D
勘误对象：docs/reports/native3d_v011_correctness_report.md（只增加 v0.1.2 勘误横幅，正文与历史全部保留）
证据目录：OpticalWaveguideRouterNative3D/artifacts/v012/（cases/、logs/、probe/）
历史证据：artifacts/cases/、artifacts/v01_preserved/、artifacts/v011/ 全部冻结；artifacts/v011_preserved/ 是 v0.1.2 首次运行前对 v0.1.1 证据做的逐字节快照
环境：Windows-10-10.0.26200-SP0，Python 3.10.11（项目内 .venv），pytest 9.1.1；引擎本身只用标准库

---

## 0. 结论摘要（先给结论）

1. **PART A 五项残余缺陷全部修复**，每一项都有可在最终代码树上随时重跑的 before/after 回放
   （artifacts/v012/probe/partA_prefix_replay.py，日志 00_prefix_v012_defect_replay.log）：
   舍入状态合并、source pose 漏检、参考圆弧漏检、大扫角 sagitta 界失效、近乎平行数值消减。
2. **PART B 有限多跳原生三维 Router 完成**：整数节点 ID (i,j,k,方向)、六轴向方向、任意多跳、
   端口通用接入、每条边进 OPEN 前做完整连续曲线检查、节点/边上界由构造计算。
3. **PART C 原跨墙案例由新公共入口 route_finite_native3d(scene) 真实跑通**：
   native3d=success，代价 594.159265 µm，75 次扩展，冷缓存端到端 0.165 s（预算 60 s / 200 000 扩展）；
   2D 模式真实 search_exhausted（81 次扩展，非预算停止）。全程没有回退到两跳参考图。
   独立 Dijkstra 对同一枚举图给出完全相同的代价（差 0.000e+00）。
4. **全量测试：collected 308，passed 308，failed 0，skipped 0，xfail 0，error 0，退出码 0，墙钟 213.12 s**
   （日志 20_final_full_pytest.log）。新增 181 个测试。修复前这 5 个新回归文件是
   **27 failed / 5 passed / 2 collection error**（日志 01_prefix_partA_regressions_red.log，真实红色基线）。
5. **必须继续披露的代价与限制**：新图是离散动作图（更长的合法路线 594.159 µm vs 参考图 495.301 µm）；
   最优性只对被构造的有限图成立且带 1e-12 量级的比较容差；自间距**没有**实现后验检查（v0.1.1 的说法已勘误）；
   生产约束与独立验证仍会在无法认证时返回 undecided/uncertified（保守拒绝，不允许假通过）。

### 0.1 修改文件与关键函数（一览）

| 文件 | 关键函数 / 位置 | 修改内容 |
|---|---|---|
| src/native3d/identity.py | 新文件 | 身份名称集中定义：pose_float（默认）、approximate_quantised、v0.1.1 只读别名 exact |
| src/native3d/search/state.py | State3D.pose_key()（新增）、key()、模块文档 | 删除 key_exact() 与全部舍入；默认身份=真实存储 pose；撤回 SAFE / Every distinct real pose 等声明 |
| src/native3d/config.py | state_identity 默认值、source_pos_tol_um、source_ang_tol_deg、finite_* | 默认身份改名并去掉舍入；源/目标容差分离；有限图参数与校验 |
| src/native3d/search/astar.py | g_improves / g_tolerance / G_COMPARISON_* | 把隐式 1e-12 改成**写明的**比较容差，并明确只声称 ε-最优 |
| src/native3d/search/router.py | route(..., strategy=...)、_route_continuous | 三种策略显式分派并回写 strategy |
| src/native3d/search/finite_graph.py | 新文件：FiniteGraph、route_finite_native3d、enumerate_graph_edges | v0.1.2 生产有限多跳 Router |
| src/native3d/constraints/checks.py | segment_segment_distance(_certified)、chord_count_for_domain、sagitta_bound_um、pair_clearance_bounds、arc_box_clearance、primitive_spacing | 稳定分母 + 明确误差界 + 精确有理回退；sagitta 适用域；违规见证优先 |
| src/native3d/validation/validate.py | 全文重写 | source/target 分别判定；line/arc 四种组合；输入显式拒绝；sagitta 域；见证优先；认证自检改变 verdict |
| src/native3d/geometry/primitives.py | 注释与支持范围 | 明确「大扫角受支持，但 sagitta 界只在 abs(delta_theta) <= pi 的已证明适用域内求值」 |
| src/native3d/search/reference_graph.py | _pose_key、key_fn、共面性注释 | 去掉查找键舍入；就地更正「精确整数共面判定」的错误注释 |
| src/native3d/io/artifacts.py | EVIDENCE_VERSION=v012、strategy、self_spacing_check | 证据目录参数化到 v012；记录策略与自间距未实现 |
| examples/run_core_case.py | main() | 三种策略全部运行并写 v012 证据（含计时与峰值内存） |
| tests/ | 8 个新增文件 + 6 个既有文件更新 | A1–A5 回归、有限图、Dijkstra 对照、一般性矩阵；既有文件只做身份改名/容差口径/夹具更新 |
| README.md | 全文 | 三种策略、保证范围、自间距未实现、离散动作限制 |
| docs/reports/native3d_v011_correctness_report.md | 顶部勘误横幅 | 只增加醒目勘误链接，正文不改 |

---

## 1. PART A：五项残余缺陷的修复

所有 before/after 数字都来自 artifacts/v012/probe/partA_prefix_replay.py 的真实运行
（在最终代码树上可随时重跑；其中 BEFORE 侧是**就地重建的 v0.1.1 代码路径**，
不是事后编写的日志，第一次在未修改的 v0.1.1 工作树上运行得到的数字与此完全一致）。

### A1 舍入状态合并

**根因**：key_exact() 把位置与方向各 round(...,9)。round(10.0000000002, 9) == round(10.0, 9)，
所以相距 2e-10 µm 的两个真实 pose 共享 key；后继是真实 pose 的函数，容差再小也不能证明同桶 pose 后继等价。

**修复前实测**（S->A=10 且 A 无后继、S->B=10.0000000002、B->G=9.9999999998）：

    BEFORE  v0.1.1 key_exact()（round 9）   status=search_exhausted  cost=inf  expansions=2
    AFTER   v0.1.2 pose_key()（存储 pose）  status=success           cost=20.0 expansions=3

**修复**：

* 默认身份改为 pose_key()：(position 三元组, direction 三元组) 原样作键，**没有任何舍入、量化或容差**；
* 删除 key_exact() 与 EXACT_KEY_DECIMALS / EXACT_KEY_POSITION_TOLERANCE_UM，因此不存在"名字叫 exact 却在舍入"的东西；
* 配置默认 state_identity="pose_float"；v0.1.1 的写法 "exact" 只作为**读取别名**解析到 pose_float，不再是另一个模式；
* 量化键 key(pos_res, dir_res) 保留，但**只**作为明确标注的近似探测模式（approximate_quantised），其耗尽不是不可达证明；
* 文档撤回 SAFE identity、Every distinct real pose、pruning on this key never merges 等声明，
  并写明无舍入浮点身份**不等于**连续空间完备搜索、枚举图一般无限。

**回归**：tests/test_v012_a1_pose_identity.py（9 项，含微小差值反例、10 与 10.4 旧反例、撤回声明检查、比较容差文档检查）。

### A2 独立验证器漏掉 source pose 违规

**根因**：validate.py 计算了起点位置/切向误差，但最终判定只统计终点误差。

**修复前实测**：path=(10,0,0)->(100,0,0)、source=(0,0,0) 切向 (0,1,0) -> 输出起点位置误差 10 µm、起始切向误差 90°，
同时 verdict=pass。

**修复**：起点与终点分别判定，容差显式且分离：

* source_pos_tol_um = 1e-6 µm、source_ang_tol_deg = 1e-6 deg（新增配置项），起点必须满足**源 pose 约束**；
* 终点的接受球仍是 eps_pos_um = 2 µm / eps_ang_deg = 3 deg；
* violation_counts 新增 source_position_out_of_tolerance / source_tangent_out_of_tolerance，
  原先的 endpoint_* 计数含义明确为**终点**；输出新增 source_pose_check / target_pose_check 两块；
* C1、最小半径、长度检查全部保留，端点正确不能替代全路径正确。

现在同一输入 verdict=fail，且 source 位置/切向各记 1 次越界。

**回归**：tests/test_v012_a2_source_pose.py（8 项，含容差边界、容差可配置、目标球不得复用、C1/半径仍生效）。

### A3 独立验证器忽略参考圆弧

**根因**：_certified_spacing 只筛 kind == "line"，Scene 已支持的圆弧参考被静默丢弃；
纯圆弧集合还会因 None 与 float 比较抛 TypeError。

**修复前实测**：path=(10,0,0)->(100,0,0)，参考=圆弧（过 (50,0,0)）+ 远处直线：

    PRODUCTION spacing：violation（bound 0）
    BEFORE v0.1.1 validator（arc+line）：safe，lower bound 100.000000 µm（圆弧被丢掉）
    BEFORE v0.1.1 validator（arc only）：TypeError: '<' not supported between 'NoneType' and 'float'

**修复**：验证器覆盖 line/line、line/arc、arc/line、arc/arc 四种组合；
两侧都是曲线时认证下界同时减去**双方**的几何近似误差；纯圆弧集合正常工作；
新增点的**解析**点-弧距离（径向投影或端点）用于采样估计与见证；
参考种类普查写入 JSON（n_reference_curves / reference_kinds）；
**未支持或畸形输入显式拒绝**（ValueError），不再忽略后给 pass。

现在同一输入（arc+line 与 arc only）都 verdict=fail，且不再抛 TypeError。

**回归**：tests/test_v012_a3_reference_arcs.py（15 项，含四种组合、纯弧集合、混合集合、6 类畸形输入显式拒绝）。

### A4 大扫角圆弧使弦误差界失效

**根因**：弦偏差界 R(1-cos(delta_theta/2)) 只在**每个子弧** abs(delta_theta) <= pi 的已证明适用域内成立；
v0.1.1 的细分从 n=1 或 2 开始，而 cos 是周期函数：8π 扫角、n=2 时 delta_theta=4π 给出 sagitta=0，
于是把四圈圆弧当成一个退化的"弦"并"认证"出 99.005050376 µm 的下界。

**修复前实测**：

    PRODUCTION spacing : safe，所谓下界 99.005050376 µm
    PRODUCTION obstacle: safe，所谓下界 99.000000 µm
    VALIDATOR          : sampled 0.022088195608 µm，claimed lb 99.005050376 µm，
                         一致性标志 False，verdict pass

**修复**：

* 新增 MAX_CHORD_SWEEP_RAD = pi、chord_count_for_domain(theta, requested)、sagitta_bound_um(seg, n)：
  **先把弦数抬进适用域**再求值，越域直接 AssertionError；生产与独立验证都走这一条路径；
* 不得用多圈弧端点重合推断偏差为零（新增端点重合反例测试）；
* **违规见证优先**：弦节点是曲线上的点，其到障碍/参考中心线的精确距离低于 keep-out 时一律判 violation，
  "下界看着安全"的分支不能覆盖它；
* 认证自检：下界 > 采样上界 + 容差 在数学上不可能，出现即 certification.self_check_failed=True
  且 verdict 不再可能是 pass；独立验证器新增 self_check_failed 字段。

现在同一输入：生产 spacing=violation(0)、obstacle=violation(0)、
独立验证 sampled 0.022088195608、status=violation、verdict=fail、自检未触发（下界 -50 < 采样）。

**回归**：tests/test_v012_a4_large_sweep_arc.py（79 项，含 ±扫角、跨 2π、适用域边界 ±1e-9、多圈、端点重合、
自检 monkeypatch 反例）。

### A5 近乎平行线段的浮点消减

**根因**：经典解算用 denom = a*e - b*b。两条 1000 µm、相距 5e-6 µm 的近平行线段上，
a*e 与 b*b 都是 ~1e12 而差值 ~1e-4，低于操作数分辨率，denom 与最近点参数被舍入误差支配。

**修复前实测**：真实距离 0（交于 (500,0,0)），

    BEFORE v0.1.1 经典公式            -> 9.039999999999993e-07 µm（生产判 safe、验证器判 pass）
    AFTER  v0.1.2 certified decision  -> violation，[0.000e+00, 0.000e+00]，method=exact_rational

**修复**：

* 用 Lagrange 恒等式 a*e - b*b == norm(d1 x d2)^2 计算同一个量，**不产生灾难性消减**；
* segment_segment_distance_certified() 返回**认证的** (lower, upper)：先减一个写明的保守误差界
  SEGMENT_DISTANCE_ERROR_FLOOR_UM + 1e4 * 2^-53 * scale * (1/sin theta)，误差界不足以判定时
  **回退到精确有理数运算**（fractions.Fraction，输入 double 是精确有理数，故判定是精确的），
  并用有理平方与整数 ulp 调整给出严格的 sqrt 浮点上下界；
* 支持数值尺度写明：坐标/长度 1e-9 µm ~ 1e9 µm；容差与单位在文档中明确；
* 生产与独立验证**都**修复（各自实现，验证器不导入生产代码）。

现在生产判 violation、验证器 analytic 8.470e-22 µm、verdict=fail。

**回归**：tests/test_v012_a5_near_parallel.py（34 项，含 x/÷1000、平移、旋转、独立精确公式对照、
见证优先、v0.1.1 良态用例不变）。

### A1–A5 修复前/后对照（一表）

| 缺陷 | 修复前（实测） | 修复后（实测） |
|---|---|---|
| A1 | key_exact：search_exhausted，2 次扩展 | pose_key：success，cost 20，3 次扩展 |
| A2 | verdict=pass（起点错 10 µm / 90°） | verdict=fail（source 位置/切向各 1 次越界） |
| A3 | pass，间距 100 µm（圆弧被丢）；纯弧集合 TypeError | fail（lb -50，采样 0）；纯弧集合 fail，无异常 |
| A4 | spacing safe 99.005050376；obstacle safe 99.0；validator pass | spacing/obstacle 均 violation；validator fail |
| A5 | 生产 safe 9.039999999999993e-07；validator pass | 生产 violation；validator analytic 8.470e-22，fail |

### A2' 生产约束与独立验证是否仍可能对已证实违规输入输出 safe/pass？

**不会。** 本轮把三类"假通过"通道全部关闭，并保留保守拒绝：

1. **见证优先**：曲线上的点进入 keep-out（点-障碍精确距离、点-中心线精确距离）就是 violation，
   任何"下界看起来安全"的分支都不能覆盖；
2. **认证自检**：下界 > 采样上界时 self_check_failed=True，verdict 不可能是 pass（只能 uncertified/fail）；
3. **判定三分 + 保守拒绝**：safe / violation / undecided，undecided 一律 **REJECT**（validate_segments 的 ok=False，
   verdict=uncertified），绝不当作通过；独立验证器的 undecided 也永不输出 pass；
4. **数值判定精确化**：line/line 的判定在误差界不足时用精确有理算术，因此不存在"算错但仍报 safe"的路径；
5. **输入显式拒绝**：不支持的 kind、缺失字段、非法半径/扫角、非有限数直接 ValueError。

仍然存在、且必须承认的边界：圆弧参与的**下界**是保守区间，极小临界情形会在层数上限内无法判定 → 返回
undecided/uncertified（保守误拒，不允许假通过）；本轮的证据里这类计数为 0（核心案例拒绝原因中 undecided=0）。

---

## 2. A6 文档保证范围的勘误

native3d_v011_correctness_report.md 顶部新增 v0.1.2 勘误横幅，正文一字不改。勘误内容：

1. **九位小数舍入不是无损状态身份**。round(10.0000000002,9)==round(10.0,9)，
   实测反例使 v0.1.1 默认身份返回 search_exhausted 而真实 pose 身份返回 success/cost 20。已删除该身份。
2. **127 passed 不能证明全部受支持输入都已认证安全**。同一份代码本轮又复现出 5 个残余缺陷，
   每一个都能让已认证输出产生假 pass/safe；测试数量不是覆盖性证明。
3. **参考图共面性不是精确整数测试**。代码中不存在 offset·(t x d)==0 的整数判定；
   connector.planar_connectors() 用浮点容差 abs(dot(dp,n)) > coplanar_tol_um = 1e-6 µm，
   方向也已先转成浮点单位向量。reference_graph.py 的相应注释已就地更正。
4. **1200 秒预算停止不能推出"不可有限完成"或"超指数增长"**。它只说明在该实现、该动作集、该预算下没有跑完；
   §6.5 的"前沿只从 486.4 走到 491.0"是 99 501 次扩展的实测曲线，不是增长率的理论结论。
   本轮用有限整数节点图证明同一场景可在 60 秒预算内真正跑通。
5. **"自间距已有后验检查"不成立**。constraints/checks.py 里确有 path_self_spacing()，但
   生产搜索、结果组装与独立验证器**都没有调用它**（只有 tests/test_phase5_constraints.py 直接调用），
   它不进入任何 verdict。本轮记它为**未实现**，并写入 README、报告与每份 v0.1.2 artifact 的
   self_spacing_check 字段。本轮不扩展路径历史搜索。

---

## 3. PART B：有限、多跳、原生三维单网搜索

### B1 旧参考图的真实范围（如实保留）

reference_graph.py 的中间节点是"以源点为锚的规则格点 × 26 个整数方向"，
后继严格限定为 source->target 与 source->mid->target（max_hops = 2），没有 mid->mid。
它作为**对照实现**原样保留，不改名、不冒充新生产 Router，也没有用固定 waypoint 模板替代多跳搜索。

### B2 新的有限状态图（src/native3d/search/finite_graph.py）

* **唯一规范 pose**：节点 ID 是整数四元组 (i, j, k, d)，d 属于 {+x,-x,+y,-y,+z,-z}；
  position(i,j,k) = anchor + h*(i,j,k)（anchor = 源点位置，h = finite_spacing_um），
  tangent(d) = 该轴向单位向量。一个 ID 对应一个规范 pose，一一对应。
* **去重依据**：A* 的 key_fn 就是**节点 ID**；后继状态一律由目标节点的规范 pose 构造，
  所以"同 ID = 同规范状态"是构造性事实，而不是"舍入后近似相同"。
  （构造期还断言 node 与 pose 是双射；若两个 ID 共享 pose 会直接 AssertionError。）
* **方向集合**：六个轴向方向，包含 ±z；转弯族里涉及 ±z 的动作改变 z（±R），
  两个水平方向之间的转弯 delta_z=0（水平 yaw）。不存在"二维 orientation + 层号"。
* **有限性由构造给出**：索引范围由 chip 体积收缩边界余量后算出，节点上界 = 位置数 × 6，
  边上界 = 节点上界 × (直线步数 + 4 × 半径步数) + 端口邻域。核心案例实测：
  索引范围 x 属于 [-1,10]、y 属于 [-2,2]、z 属于 [-1,2] -> 240 个位置 × 6 = **1440 节点上界**，
  边上界 **19936**；上界是构造计算的，不是"运行没继续扩展"推断的。
* **动作**：直线 n*h（n 属于 finite_straight_steps）；在**坐标平面**内的 ±90° 圆弧，半径 m*h
  （m 属于 finite_turn_radius_steps 且 m*h >= Rmin）。90° 弧从 d 到 d' 的位移恰为 R(d+d')，
  所以终点仍落在格点上、终点切向恰为轴向单位向量。核心案例里 R 属于 {50, 100} µm，Rmin=50 µm。
* **段间一致性**：曲线由规范 pose 用与其它 primitive 相同的 CircularArc3D.from_pose / LineSegment3D 生成，
  没有"先生成错端点再吸附/扭曲/elevation repair"。实测"曲线终点 vs 规范目标节点"最大误差：
  位置 7.105e-15 µm、切向 0 deg。
* **每条边进 OPEN 前**都过完整连续曲线检查（Rmin、边界、障碍 keep-out、中心线 spacing），
  组装后的整条路径再检查一次（assembled_path_constraint_check.ok=True）。
* **端口**：源/目标可以是虚拟节点。若端口 pose 本身就是某个规范 pose（源点必然是，因为格以源点为锚），
  该节点**就是**端口（零长度接入，kind="identity"）；否则在通用接入邻域
  （finite_port_attach_um 半径、finite_port_attach_max_nodes 上限）内用**已验证的直线接入**连接，
  端口切向不是轴向时回退到 v0.1.1 共面 Dubins 连接器族。没有接入时返回
  port_attachment_failed/unsupported_pose，并写明"这不是物理不可达"。
  接入邻域由通用配置决定，跨墙案例没有指定任何中点、层高或路线。

### B3 从第一步就有的 z 自由度

核心案例第一次扩展（初始状态）生成的候选里，**被接受且改变 z 的有 3 个**（z 变化 +50、+100、-50），
2D 模式同一次扩展为 **0 个**。trace 的 z 序列为 0 -> 50 -> 100 -> 100 -> 50 -> 0，全部来自搜索边。

### B4 多跳与"不是两跳模板"

* 每个规范节点的后继规则完全相同，没有 source/mid/target 分类，也没有跳数上限；
* 核心案例的路径动作序列是
  <source> -> straight_2 -> turn(+x 到 +z, R50) -> turn(+z 到 +x, R50) -> straight_3
  -> turn(+x 到 -z, R50) -> turn(-z 到 +x, R50) -> port_target_straight，
  即有 **5 个运动中间节点**、其中包含同一节点类型之间的**普通 mid->mid 转移**；
* 测试 test_v012_finite_graph.py::test_the_core_route_uses_mid_to_mid_transitions 断言内部动作 >= 3、
  至少 2 个转弯、至少 1 个直线，并断言动作名里不含 wall/core/mid/waypoint/layer/climb 等字样；
* test_multi_wall_case_needs_several_interior_nodes 用**两道墙**的案例要求路径 >= 8 个 trace 节点，
  证明一般局部后继能组成更长的路径。

### B5 A*、成本与状态语义

* 成本：cost = real_path_length + 非负拥塞惩罚；h = max(0, |target - pos| - eps_pos)（可采纳且一致）；
* 终止：目标**出堆**才返回（同一份 astar_search，v0.1.1 已修）；不重新引入贪心提前返回；
* **比较容差写明**：g_improves(new, old) := new < old - (1e-12 + 1e-12*max(|new|,|old|))；
  结论只声称到该容差为止的 ε-最优，不声称任意实数成本下的严格最优；
* 状态语义分开：success / search_exhausted（有限图真实耗尽）/ budget_exhausted /
  invalid_source_pose / invalid_target_pose / unsupported_pose / port_attachment_failed / uncertified；
* **独立 Dijkstra 对照**：enumerate_graph_edges() 枚举全图（核心案例 7867 条边），
  测试与探针各自用**独立实现**的朴素 Dijkstra 求解，代价与 A* 完全一致（差 0.000e+00），
  不是"同一个 A* 把 h 改成 0"；
* 边缓存：单次搜索内的静态边缓存，键包含影响几何/约束/代价的配置指纹（graph_config_fingerprint），
  不跨场景复用；证据里记录 cache_hits 与指纹。

---

## 4. PART C：实际验收与性能记录

### C1 原跨墙案例（输入几何一字未改）

chip (0,0,-60)–(600,300,120)，墙 (290,-50,-70)–(310,350,40)，源 (60,150,0)+x，目标 (540,150,0)+x，
Rmin=50 µm，障碍 keep-out 5 µm，dmin=5 µm，边界余量 2 µm，原参考中心线不变。

| 运行 | 状态 | 扩展 | 代价 / 长度 | 耗时 |
|---|---|---|---|---|
| **route_finite_native3d native3d（新公共入口）** | **success** | 75 | **594.159265 µm** | 端到端 **0.165 s**（建图 0.028 + 搜索 0.136，其中边检查 0.077） |
| route_finite_native3d 2d | **search_exhausted**（非预算） | 81 | — | 0.045 s |
| 独立 Dijkstra（同一枚举图） | 最优代价 | 678 节点 settled | 594.159265 µm（差 0.000e+00） | 0.003 s |
| 有限参考图 native3d（v0.1.1 对照） | success | 27 | 495.301314 µm | 1.372 s |
| 连续位姿调试搜索 native3d | budget_exhausted | 16 342 | — | 120.4 s |
| 连续位姿调试搜索 2d | budget_exhausted | 17 954 | — | 120.2 s |

（连续位姿调试搜索的两行是**预算停止**：扩展数由 120 s 墙钟决定，会在 16 000–18 000 之间随机器速度波动，
这不是可复现的确定性数字，也不是本轮的验收依据；它还明确写着预算停止不是不可达证明。）

* **预算**：单次搜索 60 s / 200 000 扩展（写进 RouterConfig，核心案例配置即默认值）；
  实测端到端 0.165 s、75 次扩展，均远在预算内；计时是**冷缓存**下"公共 Router 收到 Scene -> 返回结果"，
  包含本次建图、预计算、搜索与生产约束检查；独立验证与写盘另行计时（本报告单独列出）。
* **峰值内存**：tracemalloc 追踪峰值 **1.520 MiB**（探针）/ 1.685 MiB（证据脚本，含写盘前对象）。
* **图规模**：节点上界 1440，边上界 19936，访问节点 75，扩展调用 75，生成候选 305，拒绝 521，
  边验证 413（其中拒绝 108，undecided 0），缓存命中 0。
* **路径**：trace 为 (60,150,0) ->(straight_2)-> (160,150,0) ->(turn R50)-> (210,150,50)
  ->(turn R50)-> (260,150,100) ->(straight_3)-> (410,150,100) ->(turn R50)-> (460,150,50)
  ->(turn R50)-> (510,150,0) ->(port_target_straight, 30 µm)-> (540,150,0)。
  墙 keep-out 顶面 z=45 µm，路径在 x 属于 [285,315] 内 z >= 93.3 µm。
* **离散动作的代价（必须解释）**：新图代价 594.159265 µm > 参考图 495.301314 µm。
  原因是格距 50 µm 与 R=50 µm 的四分之一转把"爬升—平移—下降"限制成固定形状：
  一次上坡+下坡各消耗 2R 的水平位移与 2R 的高度，峰高只能是 h 的整数倍（这里 100 µm）。
  这不是缺陷，而是本轮自主离散方案的可解释限制。
* **没有参考图回退**：新入口只走有限图；结果里 reference_graph=None、state_identity="finite_lattice_node_id"，
  测试 test_the_production_strategy_does_not_fall_back_to_the_reference_graph 显式断言。

### C2 一般性与多跳测试矩阵

| # | 要求 | 测试 | 结果 |
|---|---|---|---|
| 1 | 纯直线 | test_v012_generality.py::test_case1_pure_straight_is_exactly_euclidean | success，长度 150.000 µm（全直线段） |
| 2 | 真实三维弯曲 | ::test_case2_a_genuinely_three_dimensional_target_is_reached | success，z/y 均变化，存在非 xy 平面圆弧 |
| 3 | 跨墙核心案例 | test_v012_finite_graph.py::test_core_case_is_solved_by_the_new_public_router 等 | success / 2D search_exhausted |
| 4 | 多个中间节点、三个以上动作 | ::test_the_core_route_uses_mid_to_mid_transitions | 5 个运动中间节点，含 mid 到 mid 转移 |
| 5 | 多墙/连续绕障 | ::test_multi_wall_case_needs_several_interior_nodes | success，>= 8 个 trace 节点，2D 耗尽 |
| 6 | 封闭障碍、端口合法但目标不可达 | ::test_sealed_target_exhausts_the_finite_graph；generality::test_case6 | search_exhausted（非预算）或显式 port_attachment_failed |
| 7 | 改墙位置/高度/源目标/格点相位的确定性变体 | generality::test_case7（7 个参数化变体） | 全部 success，且两次运行 trace 与代价逐位一致 |
| 8 | 起终点切向错误、Rmin/spacing 刚好合法与违规 | generality::test_case8 系列；finite_graph 中的 radius 用例 | 目标/源切向错误 -> port_attachment_failed；m*h>=Rmin 恰好接受、无法达到 Rmin 直接 ValueError |
| 9 | PART A 全部已确认反例 | tests/test_v012_a1..a5 | 145 项 A 回归全部通过 |

**没有向 Router 提供人工 waypoint、最高 z、换层位置或预制路径**：
公共入口签名就是 route_finite_native3d(scene, mode, congestion, max_expansions, max_runtime_s, expand_hook, graph)，
test_the_router_is_not_given_any_waypoint 断言该签名；算法实现不识别案例名、不硬编码任何坐标。

### C3 全量测试、证据与计时

**命令与结果**

| 用途 | 命令 | 退出码 | 日志 |
|---|---|---|---|
| 修复前反例回放（最终树上可重跑） | .venv Scripts python.exe artifacts\v012\probe\partA_prefix_replay.py | 0 | logs/00_prefix_v012_defect_replay.log |
| **修复前红色基线**（5 个新回归文件） | .venv Scripts python.exe -m pytest tests/test_v012_a*.py -q --continue-on-collection-errors | 1 | logs/01_prefix_partA_regressions_red.log |
| v0.1.2 验收探针（A 反例复跑 + 核心验收 + Dijkstra + 计时/内存） | .venv Scripts python.exe artifacts\v012\probe\v012_acceptance_probe.py | 0 | logs/10_v012_acceptance_probe.log |
| 证据重生成（三种策略） | .venv Scripts python.exe examples\run_core_case.py | 0 | logs/11_run_core_case.log |
| 独立验证器 CLI（只用标准库、只读 JSON） | .venv Scripts python.exe -m native3d.validation.validate artifacts\v012\cases\core_case_finite_native3d_native3d.json | 0 | logs/12_independent_validate_cli.log |
| **全量测试（第一次全绿）** | .venv Scripts python.exe -m pytest -v --durations=25 | 0 | logs/20_final_full_pytest.log |
| **全量测试（最终复核）** | .venv Scripts python.exe -m pytest -v --durations=25 | 0 | logs/21_verification_final.log |

**全量测试**：collecting ... collected **308** items -> **308 passed**，failed 0，skipped 0，xfail 0，error 0，
pytest 墙钟 **213.12 s（3 分 33 秒）**，退出码 0。最终复核（全部文件冻结后再跑一次）：
**308 passed in 213.71 s**，退出码 0（logs/21_verification_final.log）。本轮**没有任何 skip/xfail**，
v0.1.1 的 127 passed 只是历史基线，不作为本轮结论。

说明：cases/ 下的证据由"最后一次写入它的命令"产生。本报告引用的核心数字来自
artifacts/v012/probe/v012_acceptance_probe.py 与 examples/run_core_case.py（后者是最后一次写入者，
两项都写入 logs/10 与 logs/11）；pytest 会话夹具也会写同一目录，使用较小的连续搜索预算
（1500 次扩展 / 8 s，见 tests/conftest.py），因为那些运行只用于固定状态语义。

按文件分布（全部 passed）：

| 文件 | 数量 | 说明 |
|---|---|---|
| test_phase1_geometry.py | 11 | 几何 / primitive / 精确 AABB |
| test_phase2_state.py | 5 | 状态与身份函数 |
| test_phase3_actions.py | 7 | 三维动作族与符号约定 |
| test_phase4_astar.py | 7 | A*、拥塞、状态语义 |
| test_phase5_constraints.py | 10 | 五类硬约束 |
| test_phase6_cases.py | 9 | 必测案例 |
| test_phase6_core_acceptance.py | 12 | 核心案例（含 v0.1.2 有限图证据） |
| test_phase7_reference_graph.py | 7 | v0.1.1 有限参考图 |
| test_repair_a_astar_optimality.py | 12 | v0.1.1 修复 A 回归 |
| test_repair_b_state_identity.py | 10 | v0.1.1 修复 B 回归（身份改名后保留） |
| test_repair_c_validator_bounds.py | 22 | v0.1.1 修复 C 回归 |
| test_repair_d_connector_signs.py | 10 | v0.1.1 修复 D 回归 |
| test_independent_validation.py | 5 | 独立验证器 |
| **test_v012_a1_pose_identity.py** | **9** | 本轮 A1 |
| **test_v012_a2_source_pose.py** | **8** | 本轮 A2 |
| **test_v012_a3_reference_arcs.py** | **15** | 本轮 A3 |
| **test_v012_a4_large_sweep_arc.py** | **79** | 本轮 A4（含参数化扫角矩阵） |
| **test_v012_a5_near_parallel.py** | **34** | 本轮 A5 |
| **test_v012_dijkstra_crosscheck.py** | **4** | 独立 Dijkstra 对照 |
| **test_v012_finite_graph.py** | **18** | 新有限多跳 Router |
| **test_v012_generality.py** | **14** | 一般性矩阵 |

新增合计 **181** 个测试（127 -> 308）。

**修复前红色基线**（logs/01_prefix_partA_regressions_red.log）：
27 failed、5 passed、2 collection error（A4/A5 文件当时依赖尚未存在的公开 API），
失败信息是真实的行为差异（例如 assert 'pass' == 'fail'、cannot import name
'segment_segment_distance_certified'）。

**耗时构成**（--durations=25，区分搜索 / 验证 / pytest 墙钟）：
连续位姿调试搜索的拥塞用例 118.97 s（v0.1.2 取消舍入后同场景从约 35 s 增加到约 115 s，
已在该测试里写明并把夹具预算放宽到 600 s，断言不变）、核心验收夹具 20.36 s、
预算语义 9.63 s、封闭场景 9.41 s、终端切向 8.63 s、生产预算停止 8.08 s、
参考图最优性 6.12 s、近似模式 5.02 s、其余均 < 3.3 s；
独立验证与约束认证类用例全部 < 0.35 s；新有限图/一般性/Dijkstra 用例合计 < 3 s。

**核心搜索性能与峰值内存**（冷缓存，单进程）：

| 指标 | native3d | 2d |
|---|---|---|
| 状态 | success | search_exhausted（非预算） |
| 端到端 | 0.165 s | 0.045 s |
| 建图 | 0.028 s | 0.029 s |
| 搜索（含惰性边检查 0.077 s） | 0.136 s | — |
| 扩展 / 生成 / 拒绝 | 75 / 305 / 521 | 81 |
| 边验证 / 被拒 / undecided | 413 / 108 / 0 | — |
| 峰值 tracemalloc | 1.520 MiB | 1.064 MiB |
| 节点上界 / 边上界 | 1440 / 19936 | 同 |

**证据清单**

    artifacts/v012/
      cases/core_case_finite_native3d_native3d.json   新公共 Router native3d（success，含路径/溯源/独立验证）
      cases/core_case_finite_native3d_2d.json         新公共 Router 2d（search_exhausted，非预算）
      cases/core_case_reference_graph_native3d.json   v0.1.1 两跳参考图（对照）
      cases/core_case_reference_graph_2d.json         v0.1.1 两跳参考图 2D（耗尽）
      cases/core_case_native3d_mode.json              连续位姿调试搜索（budget_exhausted）
      cases/core_case_2d_mode.json                    连续位姿调试搜索 2D
      cases/core_case_initial_action_probe.json       四种策略的初始状态候选探针
      cases/v012_core_acceptance_summary.json         验收汇总（含 Dijkstra 对照与内存）
      logs/00_prefix_v012_defect_replay.log           A1–A5 before/after 回放
      logs/01_prefix_partA_regressions_red.log        修复前红色基线
      logs/10_v012_acceptance_probe.log               验收探针
      logs/11_run_core_case.log                       证据重生成
      logs/12_independent_validate_cli.log            独立验证器 CLI
      logs/20_final_full_pytest.log                   全量测试
      probe/partA_prefix_replay.py                    A1–A5 before/after 回放脚本
      probe/v012_acceptance_probe.py                  验收探针脚本

每份核心 JSON 记录 strategy、state_identity、数值容差、输入配置与约束认证
（assembled_path_constraint_check / independent_validation / self_spacing_check）。

**历史证据完整性（如实披露一次过程事故与修复）**：开发中期、在把证据版本切到 v012 之前，
有几次 pytest 运行仍然写到 artifacts/v011/cases/，把 4 份 v0.1.1 用例 JSON 覆盖成了 v0.1.2 代码的输出。
发现后立即从 artifacts/v011_preserved/（在任何源码修改之前、用未改动的 v0.1.1 代码跑出的逐字节快照）
恢复这 4 个文件，并对 artifacts/v011/ 的全部 33 个文件做了 SHA-256 与快照的逐一比对：
**差异 0**。此后 EVIDENCE_VERSION 默认值改为 v012，v0.1.2 的任何运行都不再写 v011 / v011_preserved /
cases / v01_preserved 这四棵历史目录。artifacts/v011_preserved/ 本身在本轮只被读取。

---

## 5. 保证范围：能保证什么、不能保证什么

**能保证**

1. 有限图策略：**在所构造的有限图上**找到的路径是该图上最优的（A* 正确终止 + 可采纳一致启发式，
   到写明的比较容差为止）；search_exhausted 是**该图**的真实耗尽，不是预算停止；
   节点/边上界由构造计算并随结果给出。
2. 每条边与最终路径都通过完整连续曲线检查（Rmin、边界、障碍 keep-out、中心线 spacing），
   组装后再次检查；路径端点严格落在规范节点/端口上（最大曲线末端误差 7.105e-15 µm，切向 0）。
3. 状态身份：pose_float **不合并任何不同的存储 pose**；有限图以整数节点 ID 去重，
   ID 与规范 pose 是双射（构造期断言）。
4. 生产约束与独立验证：见证优先、判定三分、不能认证即拒绝/uncertified；
   line/line 判定在必要时用精确有理算术；四种参考曲线组合全部覆盖；
   不支持/畸形输入显式拒绝。
5. 端口支持范围与失败语义明确：接入失败 = 受支持动作集没找到，**不写成物理不可达**。

**不能保证**

1. **不声称连续位姿空间的全局最优或完备**。有限图是离散动作图，能表达的路线是格点子集；
   连续位姿调试搜索的枚举图一般无限，其 budget_exhausted 永远不是不可达证明。
2. **不声称任意多跳一定可达**：可达性依赖格距 h、方向集、半径集与端口接入邻域；
   端口切向不是轴向且不满足共面 Dubins 条件时只能 port_attachment_failed。
3. **路径可能更长**：核心案例 594.159 µm vs 参考图 495.301 µm，是离散动作可解释的代价。
4. **自间距未实现**：搜索状态无路径历史，path_self_spacing() 不被任何生产/验证路径调用，
   不进入 verdict（v0.1.1 的相关说法已勘误）。
5. **障碍只有 AABB**，没有网格/CSG/曲面障碍。
6. **非共面端口连接器仍缺**：只有共面 arc-straight-arc 四族（无 LRL/RLR）。
7. **圆弧相关的临界判定**在层数上限内可能无法决定 -> undecided/uncertified（保守误拒）。
8. **没有光学模型**：没有任何损耗/串扰/dB 结果。

---

## 6. 最终问题回答

**1. 五项残余缺陷如何修复？原反例现在是什么结果？**
A1：删除 1e-9 舍入身份，默认改为存储 pose 身份 pose_key()（并删除 key_exact 与相关常量）；
微小差值反例现在 success、cost 20、3 次扩展（原 search_exhausted、2 次）。
A2：起点与终点分别判定，源 pose 有独立容差（1e-6 µm / 1e-6 deg），源越界进入计数与 verdict；
原输入现在 fail（原 pass）。
A3：验证器覆盖 line/line、line/arc、arc/line、arc/arc，双方误差都计入下界，纯弧集合不再 TypeError，
未支持输入显式拒绝；原输入现在 fail（原 pass / TypeError）。
A4：新增 sagitta 适用域（每弦 abs(delta_theta) <= pi，先抬弦数再求值），违规见证优先，认证自检失败改变 verdict；
原输入现在生产 spacing/obstacle 均 violation、验证器 fail（原 safe 99.005050376 / 99.0 / pass）。
A5：用 norm(d1 x d2)^2 稳定分母 + 写明误差界 + 精确有理回退；原输入现在 violation、验证器 analytic 8.470e-22、fail
（原 safe 9.039999999999993e-07 / pass）。

**2. 生产约束与独立验证是否仍可能对已证实违规输入输出 safe/pass？不能认证时如何处理？**
不会：见证优先、认证自检（下界 > 采样上界即 self_check_failed）、判定三分且 undecided 一律拒绝、
line/line 判定必要时精确有理、未支持输入显式 ValueError。不能认证时返回 undecided/uncertified 并 REJECT，
永不输出 pass；本轮核心案例的 undecided 计数为 0。

**3. 新有限状态的唯一规范 pose 是什么？为何按节点 ID 去重成立？**
节点 ID 是整数 (i,j,k,d)；规范 pose = (anchor + h*(i,j,k), 第 d 个轴向单位向量)。
一个 ID 唯一对应一个规范 pose；后继一律由该规范 pose 生成并把末端状态构造成目标节点 ID 的规范 pose，
所以"同 ID = 同规范状态"是构造事实而非近似，去重不会把后继不同的 pose 合并。
构造期还断言节点与规范 pose 双射（端口与规范 pose 重合时直接用该节点作为端口，不额外造虚拟节点）。

**4. 新图如何自动生成 mid 到 mid 后继？哪项测试证明它不再局限两跳？**
每个规范节点用同一套规则生成后继（直线 n*h、坐标平面内 ±90° 弧 m*h），没有节点分类与跳数上限；
核心案例路径含 5 个运动中间节点（straight_2、两个 R50 转弯、straight_3、两个 R50 转弯）
与普通 mid 到 mid 转移，test_the_core_route_uses_mid_to_mid_transitions 断言内部动作 >= 3；
test_multi_wall_case_needs_several_interior_nodes 用双墙案例要求 >= 8 个 trace 节点。

**5. 原跨墙案例是否由新公共 Router 跑通？是否有参考图回退？2D 是否真实耗尽？**
跑通：route_finite_native3d -> success，594.159265 µm，75 次扩展，端到端 0.165 s。
没有任何回退：reference_graph=None、state_identity=finite_lattice_node_id，并有专门测试断言。
2D 是真实耗尽：search_exhausted、budget_exhausted=False、81 次扩展 0.045 s；
独立 Dijkstra 在 2D 枚举图上同样得到 inf（图不连通）。

**6. 搜索从第一步是否具有 z 自由度？路径是否逐段对应三维动作？**
是。核心案例初始状态（端口节点）第一次扩展就生成 3 个被接受且改变 z 的候选（+50、+100、-50），
2D 模式同一次扩展为 0。路径 trace 为 0 -> 50 -> 100 -> 100 -> 50 -> 0，逐段动作名为
straight / turn(+x 到 +z) / turn(+z 到 +x) / straight / turn(+x 到 -z) / turn(-z 到 +x) / port attachment，
segment_provenance 与路径分段一一对应。

**7. 本轮真实测试数量、结果、耗时、核心搜索性能与峰值内存？**
collected 308 / passed 308 / failed 0 / skipped 0 / xfail 0 / error 0，退出码 0，墙钟 213.12 s；
新增 181 项。核心案例：单次搜索端到端 0.165 s（预算 60 s）、75 次扩展（预算 200 000）、
建图 0.028 s、搜索 0.136 s（其中惰性边检查 0.077 s）、峰值 tracemalloc 1.520 MiB、
节点上界 1440、边上界 19936；2D 0.045 s / 81 次扩展 / 1.064 MiB；
独立验证与写盘另行计时（pytest 全量墙钟里单列）。

**8. 当前能保证什么、不能保证什么？进入 16 通道前还缺什么？**
保证见第 5 节上半：有限图内最优与真实耗尽、完整连续约束检查、双射节点身份、见证优先与保守拒绝。
不保证：连续空间最优/完备、任意动作集下可达、路径最短、自间距、非 AABB 障碍、非共面连接器、光学量。
进入 16 通道前至少还需要：多网并发调度与 3D 占用（把已布路径作为动态 keep-out）、rip-up and reroute；
把自间距纳入搜索或给出有证明的顺序占位方案并重做支配性分析；
非 AABB 障碍与波导截面的保守距离界；非共面端口连接器；面向通道数的性能工程（当前纯 Python）；
以及"有限图抽象与连续问题关系"的正式论证。

---

## 7. 本轮明确未做

* 未进入多通道（16/1024）：没有多网调度、没有占用更新、没有 rip-up and reroute。
* 未加入任何光学损耗/串扰/dB 模型（AST 测试强制）。
* 未扩展路径历史搜索（自间距仍记为未实现）。
* 未修改旧 OpticalWaveguideRouter2D / OpticalWaveguideRouter3D，未重跑 LiDAR PART A，未改写文献审计报告。
* 未通过修改既有墙体、Rmin、dmin、芯片体积或数值容差让旧案例变容易：
  核心案例的 chip / wall / 源 / 目标 / Rmin / keep-out / dmin / 边界余量与原参考中心线一字未改；
  A2/A5 的容差是在**测试夹具**里显式给出的边界值，用于验证边界行为，不影响验收场景。
* 未做二维路线后抬升或 elevation repair：新图的 z 全部来自三维搜索边。

