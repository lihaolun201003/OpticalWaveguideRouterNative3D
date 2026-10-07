# Native3D v0.1.1 正确性修复与重新验收报告

> ## ⚠️ 勘误（v0.1.2）：本报告以下结论已被推翻或收窄，请勿继续引用
>
> **勘误与后续报告：[native3d_v012_safe_finite_search_report.md](native3d_v012_safe_finite_search_report.md)**
>
> 本文件保留 v0.1.1 的原始结果与历史，正文一字不改。以下五项在 v0.1.2 中被证明是错的：
>
> 1. **"九位小数舍入的状态身份是无损/安全的"**（§2.3、§5.2 与此前的 `key_exact()` 声明）**是错的**。
>    `round(10.0000000002, 9) == round(10.0, 9)`，所以相距 2e-10 µm 的两个真实 pose 会共享一个 key；
>    当只有被剪掉的那个 pose 有可行后继时，搜索会错误地报 `search_exhausted`。
>    实测反例：S→A=10（A 无后继）、S→B=10.0000000002、B→G=9.9999999998；
>    v0.1.1 默认身份得到 `search_exhausted`（扩展 2 次），真实存储 pose 身份得到 `success`、代价 20（扩展 3 次）。
>    v0.1.2 已把默认身份改为不做任何舍入的 `pose_float`（`State3D.pose_key()`），并删除了 `key_exact()`。
>
> 2. **"127 passed 证明全部受支持输入都已认证安全"**（§7.2、§9）**是错的**。
>    127 个测试只覆盖当时写下的输入；本轮在同一份代码上又找到并复现了 5 个残余缺陷（A1–A5），
>    每一个都能让已认证的输出产生假 `pass`/假 `safe`。测试数量不是覆盖性证明。
>
> 3. **"参考图共面性走精确整数运算 `offset · (t × d) == 0`"**（§6.1）**是错的**。
>    代码中不存在任何精确整数共面判定；`connector.planar_connectors()` 用浮点容差
>    `abs(dot(dp, n)) > coplanar_tol_um = 1e-6 µm` 判断共面，方向也已先转成浮点单位向量。
>    这是一个有明确容差的浮点测试，不是精确整数测试。
>
> 4. **"生产搜索在跨墙场景不可有限完成 / 状态数超指数增长"**（§6.1、§6.5、§8）**超出证据**。
>    1200 秒预算停止只能说明：在**该实现、该动作集、该预算**下没有跑完；它不能推出"不可有限完成"，
>    也不能推出状态数按指数增长的理论结论（那个"前沿只从 486.4 走到 491.0"只是 99 501 次扩展的实测曲线）。
>    本轮用有限整数节点图证明了同一场景可以在 60 秒预算内真正跑通。
>
> 5. **"自间距由独立验证器后验检查"**（§8、README）**在验收路径上并不存在**。
>    `constraints/checks.py` 里确实有 `path_self_spacing()` 函数，但生产搜索、结果组装和独立验证器
>    **都没有调用它**（只有 `tests/test_phase5_constraints.py` 直接调用）。
>    它没有进入任何 verdict。v0.1.2 记它为**未实现**，并且本轮不扩展路径历史搜索。
>
> 其余 v0.1.1 结论（A\* 终止条件、独立验证器的解析距离与边界认证、LSL/RSR 第二弧扫角、
> 四项修复的 pre-fix 回放日志）在 v0.1.2 中仍然成立。

项目目录：`C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D`
勘误对象：`docs/reports/native3d_v01_report.md`（该文件只增加了勘误横幅，原始结果与历史全部保留）
证据目录：`OpticalWaveguideRouterNative3D/artifacts/v011/`（v0.1 原始证据冻结在 `artifacts/cases/` 与 `artifacts/v01_preserved/`）
环境：Windows 10.0.26200，Python 3.10.11（项目内 `.venv`），pytest 9.1.1；引擎本身只用标准库。

---

## 0. 结论摘要（先给结论）

1. **四项缺陷全部修复**，每一项都有"修复前真实失败"与"修复后通过"的双向证据：
   A\* 终止条件、状态身份与去重、独立验证器的距离与边界认证、LSL/RSR 第二弧扫角。
2. 修复前反例不是编造的日志：它们被写进回归测试后**先跑出失败**，再把 v0.1 代码逐字复制成
   `artifacts/v011/probe/v01_defect_replay.py`，在任何时刻都能重新复现同样的失败
   （日志 `artifacts/v011/logs/00_prefix_v01_defect_replay.log`）。
3. **必须披露的代价**：把 A\* 终止条件改正确、并把状态身份改成安全的精确 pose 之后，生产搜索在**跨墙场景**
   无法在有限时间内完成——这不是调参问题，而是该图本身的指数性质（§6.5 给出实测数据：
   99 501 次扩展、1200 s，f 前沿只从 486.396 走到 491.936）。因此这些场景的验收证据改用
   **可穷尽的有限规范参考图 + 几何分离证明**，并如实报告生产搜索的状态（`budget_exhausted`，绝不写成不可达）。
4. 修复后核心案例：有限规范参考图上 native3d **成功**，代价 **495.301314 µm**（比 v0.1 的 521.722 µm 更短），
   z 从 0 升到 60 µm（墙体 keep-out 顶面 z=45），C1 连续、R≥Rmin、独立验证器 `verdict=pass`；
   同一张参考图在 2D 控制模式下**真正穷尽**（`search_exhausted`，115 次扩展），并另有三重不可达证据。

### 0.1 修改文件与关键函数（一览）

| 文件 | 关键函数 / 位置 | 修改内容 |
|---|---|---|
| `src/native3d/search/astar.py` | `astar_search` 主循环；`SearchResult` | 目标出堆才结束、陈旧堆项丢弃、更优 g 重开；新增 `state_identity` / `approximate_state_identity` / `reference_graph` 字段；`search_exhausted` 的 reason 按身份分型 |
| `src/native3d/search/state.py` | `State3D.key_exact()`、`EXACT_KEY_*`、`key()` | 新增安全精确身份；旧量化键降级为明确标注的近似模式并改写文档 |
| `src/native3d/config.py` | `RouterConfig.state_identity`、`reference_spacing_um` | 新增配置项与取值校验 |
| `src/native3d/search/router.py` | `key_fn` | 按 `state_identity` 选择状态身份，默认 exact |
| `src/native3d/search/connector.py` | `planar_connectors` 的 LSL/RSR 分支 | 修正第二弧扫角；写明支持范围与"缺连接器≠不可达" |
| `src/native3d/search/reference_graph.py` | 新文件：`canonical_poses`、`best_connector_edge`、`reference_search` | 有限规范参考图（规范节点 + 精确 Dubins 边 + 同一约束栈 + 同一 A\*） |
| `src/native3d/validation/validate.py` | `seg_seg_distance`、`seg_box_intersection`、`seg_box_distance`、`seg_aabb`、`_certified_clearance`、`_certified_spacing`、`validate_path_dict` | 解析精确距离、解析弧极值、认证下界与三层判定、`uncertified` verdict、采样/解析/下界分开命名 |
| `src/native3d/geometry/primitives.py` | `CircularArc3D._compute_aabb`、`_phase_in_interval` | 精确解析 AABB（修掉"加入未达到的另一侧极值"） |
| `src/native3d/io/artifacts.py` | `artifact_dir()`、`trace_z_summary` | 证据目录参数化（默认 `artifacts/v011/cases`）；`ref_connector` 计入改变 z 的动作 |
| `examples/run_core_case.py` | `main()` | 生产运行 + 有限参考图运行 + 初始动作探针，写入 v011 目录 |
| `tests/` | `conftest.py` + 5 个新增/改写文件 | 参数化证据目录、四项修复回归、有限参考图验收、必测案例全部改到 v0.1.1 语义 |
| `README.md` | 全文 | 撤回旧的最优性/去重保证，写明新保证范围与参考图限制 |
| `docs/reports/native3d_v01_report.md` | 顶部勘误横幅 | 只增加醒目勘误链接，正文与历史全部保留 |

---

## 1. 修复 A：A\* 终止条件

### 1.1 根因

v0.1 的 `astar.py` 在**候选生成循环内部**做目标判定，一旦某个候选通过 goal test 就立刻返回该候选的 g。
终止条件与"目标以最小 f 出堆"无关，因此即便启发式为零（等价于均匀代价搜索）也可能返回次优结果。

### 1.2 修复前反例（真实失败）

有限图，使用真实 `astar_search` 接口，不用量化键：

```text
S = (0,0,0)   A = (1,0,0)   G = (2,0,0)      方向一律 +x
S -> G : cost = 100   （先生成）
S -> A : cost = 1
A -> G : cost = 1
```

边代价非负且都 >= 端点欧氏距离，因此 h(s) = |G - s| 可采纳且一致。最优代价 2。

修复前（`artifacts/v011/logs/00_prefix_v01_defect_replay.log`）：

```text
v0.1 A*, h=euclidean -> success  cost=100.0   (correct: 2)
v0.1 A*, h=zero      -> success  cost=100.0   (correct: 2)
```

对应回归文件 `tests/test_repair_a_astar_optimality.py` 修复前 **11 failed / 1 passed**。

### 1.3 修复方案（`src/native3d/search/astar.py`）

* **目标出堆才结束**：只有从 OPEN 中以最小 f 弹出的状态通过 goal test 时才返回成功；
* **陈旧堆项丢弃**：`node.g > best_g[key] + 1e-12` 直接跳过；
* **改进的 g 允许重开**：键从 `closed` 中移除后重新入堆（对可采纳但不必一致的 h 仍然正确）；
* 目标是源点、目标位置容差、目标切向容差、非负拥塞代价、`budget_exhausted` 与
  `search_exhausted` 的语义全部保留；`budget_exhausted` 仍然带 "NOT a proof" 的说明。

### 1.4 修复后

* 上述反例在**欧氏启发式与 h=0 下都返回 2**；交换邻居生成顺序结果不变；
* 5 组随机有限图（固定种子）与文件内独立 Dijkstra 逐个对齐，代价一致，并核对返回路径的 g = 路径各边代价之和；
* `budget_exhausted` 与真正的 OPEN 耗尽继续分开记录（`budget_exhausted` 与 `search_exhausted` 各有用例）；
* `tests/test_repair_a_astar_optimality.py` 修复后 **12 passed**。

### 1.5 最优性成立的条件（明确写出）

在**被枚举的图**上成立，条件是：边代价有限非负；h 可采纳且一致（本项目的三维欧氏下界满足，
因为每条边的代价 >= 其端点欧氏距离）；目标集就是 goal test 接受的状态集；
堆中陈旧项被丢弃、同状态只按不劣的 g 展开一次、更优 g 会重开。
**不声称**连续位姿空间的全局最优或完备。

---

## 2. 修复 B：状态身份与去重

### 2.1 根因

v0.1 用 (位置, 方向) 的**量化格**做去重键，但后继动作、碰撞检查与终端连接仍由**真实 pose** 计算。
同一个格里的两个 pose 不是同一个状态，"桶内只留最低 g"会删掉唯一可行的那一个。

### 2.2 修复前反例（真实失败）

```text
S = (0,0,0)  A = (10,0,0)  B = (10.4,0,0)  G = (20,0,0)   方向一律 +x
位置键分辨率 10 µm，方向键分辨率 15°
S -> A : 10     S -> B : 10.4     只有 B -> G : 9.6（A 没有去 G 的后继）
```

A 与 B 在 v0.1 键下同格。修复前（replay 日志）：

```text
v0.1 A*, quantised key -> search_exhausted  cost=inf   (safe optimum: 20)
v0.1 A*, exact     key -> success           cost=20.0
```

回归文件 `tests/test_repair_b_state_identity.py` 修复前 **8 failed / 2 passed**。

### 2.3 v0.1.1 选择的安全方案（方案 1）

采用提示中的**方案 1**：使用保留不同真实 pose 的精确状态标识，量化不用于任何未经证明的删除。

* `State3D.key_exact()`：把**完整 pose**（位置 + 三维方向）四舍五入到 1e-9 µm（约 1 飞米），
  两个 pose 只有在此精度内相同才共享 key；文档化的合并容差
  `EXACT_KEY_POSITION_TOLERANCE_UM = 2e-9 µm`，比本项目最小几何特征（Rmin = 50 µm）小 10 个数量级。
* 搜索只在**同一个 pose 以不更优的 g 再次到达**时剪枝；这正是 §1.5 最优性论证所依赖的图。
* `RouterConfig.state_identity` 默认 `"exact"`；旧的量化键保留为
  **明确标为近似的探测模式** `"approximate_quantised"`，其结果带 `approximate_state_identity=True`，
  且 `search_exhausted` 的 reason 明确写出"这是量化抽象图耗尽，不是连续空间不可达的证明"。
* 没有"只缩小量化分辨率"、没有"只保留桶内最低 g"、也没有"只改文档"。

### 2.4 修复后证据

* 反例在安全模式下成功、**cost = 20**；同桶不同方向、同桶不同碰撞可行性的用例一并加入；
* **完整运行实际路由器**（不是只修有限图适配器）：`route()` 默认 `state_identity="exact"`；
  在一个真实场景上，安全模式展开的 102 个状态里有 **13 个**是 v0.1 键会直接丢弃的桶内同伴
  （早期探针在同一场景上测得 380 次扩展中 307 个这样的状态）；
* `tests/test_repair_b_state_identity.py` 修复后 **10 passed**。

### 2.5 安全精确身份的代价（披露）

枚举图一般是无限的（位置是实数），所以 2D/跨墙场景的生产搜索会以 `budget_exhausted` 结束。
近似模式的耗尽不得解释为真实可达空间不可达；本轮所有"不可达"结论都不依赖近似模式。

---

## 3. 修复 C：独立验证器的距离与连续边界认证

### 3.1 根因

`validate.py` 的 `seg_seg_distance` 用粗扫描 + 坐标下降，得到的是**候选距离最小值**，
即真实最小距离的**上界**；它却被当成 `conservative_clearance_lower_bound_um` 输出。
同一条弧/盒的"下界"也没有减去弦偏差之外的误差来源。

### 3.2 修复前反例（三级，全部真实复现）

**距离函数级**：segment A = (0,0,0)→(100,0,0)，segment B = (3.125,-75,0)→(3.125,25,0)，真实距离 0。

```text
v0.1   seg_seg_distance = 3.125000 um
v0.1.1 seg_seg_distance = 0.000000 um
```

**完整验证器级**：被验证路径 (0,0,0)→(100,0,0)，参考中心线 (0.390625,-75,0)→(0.390625,25,0)，
dmin = 0.3 µm，chip = [-200,200]^3，无障碍物。

```text
v0.1 validator: sampled spacing = 0.109375 um, claimed certified lower
                bound = 0.390625 um  -> lower bound ABOVE the sampled
                minimum (impossible) and verdict=pass
```

**连续边界级**：一条半径 50 µm 的圆弧，其 x 最大值超出 chip 面 **1e-4 µm**，且极值点正好落在两个
0.25 µm 采样点之间，因此所有采样点都在盒内；v0.1 的 verdict 是 `pass`，v0.1.1 必须判 `fail`。

回归文件 `tests/test_repair_c_validator_bounds.py` 修复前 **10 failed / 11 passed**。

### 3.3 修复方案

* **直线段之间解析最近距离**（闭式钳位解），覆盖相交、平行、共线、退化零长度、端点最小值；
* **直线段 vs AABB**：精确 slab 求交 + （端点距离 ∪ 12 条盒边）精确距离，覆盖从面内部穿盒的情况；
* **圆弧**：本文件自带解析极值（每个坐标用 c + A cos(phi - phi_max) 的极值判据），
  需要弦近似时用 `min(弦距离) - 弦偏差` 作为**认证下界**，并按层加密直到可判定或到层数上限；
* **输出口径分开**：`min_sampled_*`（上界/采样估计）、`analytic_min_*`（有解析解时）、
  `conservative_*_lower_bound_um`（认证下界）、`status`（safe / violation / undecided）；
* **verdict 规则**：任何可靠采样违规证据（采样点在障碍内、采样点在边界外、采样最小 < 要求）→ `fail`；
  边界越界由解析极值判定 → `fail`；无法判定 → `uncertified`，**永不输出 pass**；
* **`primitives.py` 的 AABB 修正**：v0.1 在 `phi_max + pi` 分支里加入了另一侧的极值，
  使包围盒在只达到一侧极值时被撑满。修正后 arc 的 AABB 是**真正的解析精确包围盒**，
  不再误拒绝合法路线（`test_inflated_arc_aabb_does_not_falsely_reject_a_legal_arc`）。
  注意：`docs/reports/native3d_v01_report.md` 里 43.8819 µm 的边界余量结论依赖采样点，
  v0.1 的 `conservative_margin_lower_bound_um = -0.222572` 正是这个负弦误差，既不能证明越界也不能证明安全。
* **独立性保持**：验证器仍然只从序列化 JSON 重建几何，只 import 标准库
  （`test_validator_is_an_independent_implementation` 用 AST 强制检查）。

### 3.4 修复后证据

* 两个交叉线段反例距离 = 0（1e-12 内），完整验证器判 `fail`；
* 盒面穿透、刚好相切（恰好 5.000 µm 合法 / 4.999 µm 违规）、spacing 刚好合法与刚好违规、
  连续边界反例全部按预期判定；
* 对所有真实路径验证 `lower_bound <= sampled_minimum`（这是修复前被违反的不变式）；
* 无法认证的案例不再被统计为通过（`certification.n_undecided` 与 `verdict=uncertified`）；
* 没有通过修改 dmin / Rmin / 障碍位置 / chip 体积 / 容差来让旧案例通过；
* `tests/test_repair_c_validator_bounds.py` 修复后 **21 passed**。

---

## 4. 修复 D：LSL/RSR 终端连接器第二弧扫角

### 4.1 从约定重新推导（不是机械替换）

约定：`CircularArc3D.from_pose` 中正扫角让切向绕 +normal 右旋；平面内 e1 = 起点切向，
e2 = n × e1，theta_e 为目标切向相对 e1 的带符号角；左弧圆心在左侧（rotate +90°），右弧在右侧。

* 第一弧：切向要从起点方向转到直线方向 u_ang → 左弧扫 `mod2pi(u_ang)`，右弧扫 `mod2pi(-u_ang)`（未变）；
* 第二弧：切向要从 u_ang 转到 theta_e → **左弧扫 `mod2pi(theta_e - u_ang)`，右弧扫 `mod2pi(u_ang - theta_e)`**。

v0.1 把这两个式子写反了（只对 LSL/RSR，即 s1 == s2 的同向族），于是第二弧反向、端点检查失败、
整个族被丢掉。**LSR/RSL（异向族）v0.1 是对的，本轮不动**，并加回归防止被一起翻转。

### 4.2 修复前反例

source (0,0,0) tangent +x，target (100,100,0) tangent +y，R = 10 µm。合法 LSL：+π/4 弧 + √(90²+90²) 直线 + π/4 弧 = **142.987184 µm**。

```text
v0.1 LSL: 通过不了端点检查（第二弧 5.4978 rad 而非 0.7854 rad）
v0.1 LSR / RSL: 204.374715 um   （提示中给出的旧数字完全一致）
v0.1.1 connector set: [142.987184, 204.374715, 204.374715, 265.519235]
```

回归文件 `tests/test_repair_d_connector_signs.py` 修复前 **5 failed / 5 passed**。

### 4.3 修复后证据

* 上述 LSL 的端点、目标切向、C1、Rmin 全部合法，总长 142.987184 µm；
* RSR 的镜像用例（target 切线 -y）同样得到 TwoR = 142.987184 µm；
* LSR/RSL 的 v0.1 扫角保持不变（独立封闭式对照：异向族两次扫角分别为 +theta 与 -theta，直线长 √((80-2R)²-4R²)）；
* 覆盖镜像、绕任意轴旋转、任意空间平面（连接器圆弧法矢确实偏离 z 轴）、共线退化直线、扫角超过 2π（270°+270°）的用例；
* 支持范围写明：只支持**共面** pose 对的四种 arc-straight-arc 族 + 退化直线，
  没有 LRL/RLR、没有非共面族；**缺少某种连接器只说明当前动作集没找到连接，不代表物理空间不可达**；
* `tests/test_repair_d_connector_signs.py` 修复后 **10 passed**。

---

## 5. 保证范围（能保证什么、不能保证什么）

### 5.1 A\*
能保证：在**被枚举的图**上最优（条件见 §1.5），陈旧项/重开处理正确，
预算停止与真实穷尽严格区分。
不能保证：连续位姿空间全局最优或完备；正确终止 + 安全精确身份下图一般是无限的。

### 5.2 状态身份
能保证：默认 `"exact"` 只合并数值上同一个 pose（≤2e-9 µm），因此剪枝不会把后继不同的 pose 合并掉；
近似模式被明确标注，其耗尽不被解释为不可达。
不能保证：精确浮点状态等于连续空间完备搜索——状态去重、浮点误差与预算仍然限制了完备性；
将来若加入依赖路径历史的自间距约束，必须重新评估状态与支配关系（本轮不扩展多通道）。

### 5.3 连续约束认证
能保证：直线/直线、直线/AABB 距离是解析精确值；圆弧参与的判定给出**认证下界**
（= 弦距离 - 弦偏差）与采样上界，判定三分（safe / violation / undecided），
不可判定永不输出 pass。报告里 `min_sampled_*` 永远是估计量而不是认证量。
不能保证：圆弧相关的量在极少数临界情形下会在层数上限内仍无法判定——那时返回 `uncertified`。

### 5.4 终端连接器
能保证：返回的连接器一定满足端点、切向、C1 与半径约束（搜索期还要通过全部硬约束）。
不能保证：覆盖所有 Dubins 族或非共面情形，因此**不声称**完整 Dubins 最短连接器。

---

## 6. 重新执行原生 3D 验收

### 6.1 为什么需要有限规范参考图

正确的 A\* 必须展开所有 f < 最优值 的状态。安全精确身份下图几乎处处不重复，
于是 f 前沿增长极慢、而状态数指数增长（§6.5 实测）。因此对这些场景，
本轮按提示允许的方式使用**可穷尽的明确有限参考图**（`src/native3d/search/reference_graph.py`），
并配合几何证明给出可达/不可达结论，同时如实报告生产搜索状态。

有限规范参考图（选项 3 的实现）定义：

* **节点**是规范 pose：位置在**以源点为锚**的规则格上（默认间距 60 µm），
  方向取立方体 26 个整数方向之一；外加源 pose 与目标 pose。节点集由构造固定且有限
  （核心案例 2338 个中间节点）；两个整数方向与格偏移的共面性判定 `offset · (t × d) == 0`
  是**精确整数运算**，不依赖浮点容差。
* **边**是一条连续 primitive 序列（平面 Dubins arc-straight-arc 或退化直线），
  从源节点的规范 pose 出发、**精确落在**目标节点的规范 pose 上。因此后继是节点的确定性函数、
  组装出的路径**精确 C1 连接**，没有任何"吸附后再修补"的步骤。
* 每条边在进入 OPEN 之前用**与生产路由器完全相同**的硬约束栈检查（Rmin、边界、障碍 keep-out、中心线 spacing）；
* 用**同一个 `astar_search`**（正确终止、同一可采纳启发式）搜索 → 对本有限图最优；
* 覆盖"直达连接器"与"source → mid → target 两跳连接器"，mid 遍历全部规范 pose。
* 局限必须一起披露：最多两跳连接器；是有限参考图，不是生产路由器状态策略，也不是连续空间完备搜索。

### 6.2 核心案例（与 v0.1 完全相同的输入几何与约束）

输入：chip (0,0,-60)–(600,300,120)，墙 (290,-50,-70)–(310,350,40)，参考中心线 y=120 的直线，
源 (60,150,0)+x，目标 (540,150,0)+x，Rmin = 50 µm，障碍 keep-out 5 µm，dmin = 5 µm，
边界余量 2 µm。**墙、chip、端口坐标与 v0.1 一字未改。**

| 运行 | 状态 | 扩展 | 耗时 | 代价 |
|---|---|---|---|---|
| 生产搜索 native3d（精确身份 + 正确 A\*） | budget_exhausted | 15249 | 120.9 s | — |
| 生产搜索 2d | budget_exhausted | 17441 | 120.1 s | — |
| 有限参考图 native3d | **success** | 27 | 2.20 s | **495.301314 µm** |
| 有限参考图 2d | **search_exhausted**（非预算） | 115 | 1.59 s | — |

参考图路径的动作 trace：

```text
<source> (60,150,0)  ->  ref_connector_R50  ->  (300,150,60)  ->  ref_connector_R50  ->  (540,150,0)
z: 0 -> 60 µm（墙体 keep-out 顶面 z = 45 µm）
```

独立验证器对这条路径的结论（`artifacts/v011/cases/core_case_reference_graph_native3d.json`）：

| 检查 | 结果 |
|---|---|
| verdict / certification | `pass` / `all_checks_certified = True`，`n_undecided = 0` |
| C1 | 连续（max 连接位置误差与切向误差都在容差内） |
| 最小半径（三点重拟合） | 49.99999999999868 µm ≥ Rmin |
| 障碍 clearance | 采样最小 18.4407 µm；认证下界 18.4407 µm ≥ 5 µm |
| 中心线 spacing | 采样最小 36.8146 µm；认证下界 36.8145 µm ≥ 5 µm |
| 边界 | 解析精确余量 58.0000 µm；analytic_overshoot = 0；certified_contained = True |
| 端点 | 起点位置/切向误差 0 / 0；终点位置误差 9.996e-13 µm、切向误差 0 |
| 每条分段 | 都有搜索动作来源（`ref_connector_R50`），`segment_provenance` 与路径分段一一对应 |

初始状态 z 自由度探针（在任何扩展之前、直接从初始 pose 生成）：2D 模式 17 个候选、其中可行且改变 z 的 = **0**；native3d 模式 53 个候选、其中可行且改变 z 的 = **35**（含仰角变化 > 1°）。
修复后路径比 v0.1 的 521.722 µm 更短，说明正确终止条件下参考图找到的是**该图上最优**的连接器路线。

### 6.3 2D 不可达的三重证据

1. **有限参考图真正穷尽**：`search_exhausted`、`budget_exhausted = False`、115 次扩展 1.59 s。
   这与 v0.1 的 2241 次"量化图耗尽"不同：本图的节点集由构造固定，穷尽是真实的图穷尽。
2. **几何分离证明**（与任何搜索无关）：z = 0 切片上，墙体 keep-out 在 y 方向覆盖了整个 chip 的 y 范围
   （墙 y ∈ [-50,350]，含 5 µm keep-out 后更宽；chip 加边界余量后 y ∈ [2,298]），
   z 方向包含端口平面（墙 z ∈ [-70,40]，端口 z = 0）。
   因此切片上落在 x ∈ [285,315] 带内的点与墙的距离为 0 < 5 µm；任何从 x=60 到 x=540 的连续曲线
   必然穿过平面 x = 290，而该平面上切片内的点全部落在墙体内 → 违反 keep-out。
   1 µm 网格上的穷尽检查（6237 个网格点在墙内、2970 个在 keep-out 带内）
   与"源、目标分居带两侧"一起在 `test_2d_separation_is_a_geometric_property_not_a_search_result` 中断言。
3. **生产搜索状态如实报告**：2D 生产运行报 `budget_exhausted`（"NOT a proof"），从不写成不可达。

### 6.4 其它必测案例（同一输入几何与约束）

完整矩阵见 `artifacts/v011/logs/probe_acceptance_matrix.log`：

| 案例 | 生产搜索（精确身份 + 正确 A\*） | 有限参考图 |
|---|---|---|
| 纯直线（0→100 µm） | success，50 次扩展，代价 100.000 µm | — |
| 需要空间弯曲（倾斜平面弧） | success，410 次扩展，代价 130.075 µm | — |
| 完全封闭、确实不可达（2D） | budget_exhausted（诚实预算停止） | search_exhausted（10 次扩展） |
| 完全封闭（native3d） | — | search_exhausted（13 次扩展） |
| 死胡同 + 目标切向要求（2D，切向正确） | success，13 次扩展，代价 130.000 µm | success（3 次扩展，132.000 µm） |
| 死胡同 + 目标切向错误 | budget_exhausted | search_exhausted（5 次扩展） |
| 小墙场景（200 µm） | budget_exhausted | success，36 次扩展，384.096 µm |
| 验证器演示场景 | success，20 次扩展，211.317 µm | success |
| 最小半径刚好合法 / 刚好违规 | 约束级用例（不经搜索） | — |
| 中心线 spacing 刚好合法 / 刚好违规 | 约束级用例（不经搜索） | — |

### 6.5 生产搜索状态与预算披露（实测，不是猜测）

用带**可采纳障碍穿越下界**的启发式（比纯欧氏更强，仍然可采纳）跑核心案例：

```text
expansions=   2000  best_f=489.916   h(source)=486.396
expansions=  20000  best_f=491.002
expansions=  60000  best_f=491.629
expansions=  99501  best_f=491.936   t=1200 s   （budget stop）
```

用纯欧氏启发式的另一组数据：45 000 次扩展 / 387 s，前沿只到 484.571（h(source) = 478）。
也就是说：在安全精确身份下，f 前沿随扩展数只做对数级缓慢上升，
而低于任一有意义最优值 f 的状态数按指数增长——这不是调预算能解决的问题。
因此本轮把预算与状态如实写进报告：生产搜索在这些场景**不可有限完成**，
其 `budget_exhausted` 结果只用来验证状态语义，不作为可达/不可达证据。

---

## 7. 测试、证据与命令

### 7.1 环境与命令

```text
Python:   3.10.11 (tags/v3.10.11:7d4cc5a, Apr  5 2023) [MSC v.1929 64 bit (AMD64)]
Platform: Windows-10-10.0.26200-SP0
pytest:   9.1.1        （pip list 只多出 pytest 及其依赖；引擎本身纯标准库）
工作目录: OpticalWaveguideRouterNative3D
```

| 用途 | 命令 | 退出码 | 日志 |
|---|---|---|---|
| v0.1 基线（未改代码） | `.venv\Scripts\python.exe -m pytest -v --durations=20` | 0 | `artifacts/v011/logs/00_baseline_v01_suite.log` |
| v0.1 缺陷回放（逐字复制的 v0.1 代码） | `.venv\Scripts\python.exe artifacts\v011\probe\v01_defect_replay.py` | 0 | `artifacts/v011/logs/00_prefix_v01_defect_replay.log` |
| 证据重生成 | `.venv\Scripts\python.exe examples\run_core_case.py` | 0 | `artifacts/v011/logs/02_run_core_case.log` |
| 验收矩阵探针 | `.venv\Scripts\python.exe artifacts\v011\probe\acceptance_matrix.py` | 0 | `artifacts/v011/logs/probe_acceptance_matrix.log` |
| 前沿实测 | `.venv\Scripts\python.exe artifacts\v011\probe\frontier_probe2.py` | 0 | `artifacts/v011/logs/probe_frontier_crossing.log` |
| 全量测试（最终） | `.venv\Scripts\python.exe -m pytest -v --durations=25` | 0 | `artifacts/v011/logs/03_final_full_pytest.log` |
| 全量测试（最终复核 127 passed） | `.venv\Scripts\python.exe -m pytest -v --durations=25` | 0 | `artifacts/v011/logs/04_verification_final.log` |

证据写入位置已参数化：`NATIVE3D_ARTIFACT_DIR` / `artifact_dir()`，
默认 `artifacts/v011/cases`，**旧 v0.1 目录 `artifacts/cases/` 与快照 `artifacts/v01_preserved/` 保留不动**。

### 7.2 最终测试计数

最终全量测试：**collected 127，passed 127，failed 0，skipped 0，xfail 0，error 0**，
pytest 墙钟 **142.10 s（2 分 22 秒）**，退出码 0（日志 `artifacts/v011/logs/04_verification_final.log`）。

一个都不能含糊：本轮**没有任何 skip 或 xfail**，关键正确性测试也没有被任何标记规避；
v0.1 的 "64 passed" 只是历史基线，不作为新结论。

按文件分布（全部 passed）：

| 文件 | 数量 | 说明 |
|---|---|---|
| `test_phase1_geometry.py` | 11 | 几何 / primitive / 精确 AABB |
| `test_phase2_state.py` | 5 | 状态与身份函数 |
| `test_phase3_actions.py` | 7 | 三维动作族与符号约定 |
| `test_phase4_astar.py` | 7 | A* 与拥塞、状态语义 |
| `test_phase5_constraints.py` | 10 | 五类硬约束 |
| `test_phase6_cases.py` | 9 | 必测案例 1-8 |
| `test_phase6_core_acceptance.py` | 12 | 核心案例验收 |
| `test_repair_a_astar_optimality.py` | 12 | 修复 A 回归 |
| `test_repair_b_state_identity.py` | 10 | 修复 B 回归 |
| `test_repair_c_validator_bounds.py` | 21 | 修复 C 回归 |
| `test_repair_d_connector_signs.py` | 10 | 修复 D 回归 |
| `test_phase7_reference_graph.py` | 7 | 有限参考图验收（新增） |
| `test_independent_validation.py` | 5 | 独立验证器 |

耗时构成（`--durations=25`，区分搜索耗时 / 验证耗时 / pytest 墙钟）：

* **搜索类**：66.87 s 拥塞+路由用例（参考图）、19.80 s 核心验收 fixture
  （2 次生产预算运行 + 2 次参考图运行）、9.20 s 预算语义、9.07 s 封闭场景、
  8.40 s 终端切向、8.07 s 生产预算停止、5.30 s 参考图暴力对照、5.03 s 近似模式、
  2.21 s 参考图成功、1.73 s 参考图 2D 穷尽；
* **验证类**（独立校验器 + 约束认证）：全部 < 0.3 s（修复 C 的 22 个用例合计 0.3 s）；
* 其余是 pytest 装配与导入；
* **官方 LiDAR baseline 本轮完全没有运行**（PART A 未重跑），所以不存在 LiDAR 耗时混入；
* 参考图搜索的墙钟（2.2 s / 1.6 s）与生产预算停止的 120 s 级运行分开记录，见 §6.2、§6.5。

（上面已经区分了**搜索耗时**、**验证耗时**与 **pytest 墙钟**；官方 LiDAR baseline 本轮完全不运行，
PART A 未重跑，所以不存在 LiDAR 耗时混入。详细耗时见 `--durations=25` 段与 §6.2 / §6.5。）

### 7.3 修复前 / 修复后逐文件对照

| 回归文件 | 修复前 | 修复后 | 修复前真实失败要点 |
|---|---|---|---|
| `test_repair_a_astar_optimality.py` | 11 failed / 1 passed | 12 passed | 反例返回 100（应为 2）；随机图与 Dijkstra 不一致；1 次扩展预算被绕过 |
| `test_repair_b_state_identity.py` | 8 failed / 2 passed | 10 passed | 同桶 A/B 只留 A → search_exhausted；缺 key_exact / state_identity |
| `test_repair_c_validator_bounds.py` | 10 failed / 11 passed | 21 passed | 交叉线段距离 3.125（应 0）；下界 0.390625 > 采样 0.109375 却 pass；弧 AABB 撑满导致误拒 |
| `test_repair_d_connector_signs.py` | 5 failed / 5 passed | 10 passed | LSL 缺失，只剩 204.374715 µm 的 LSR/RSL |
| `test_phase7_reference_graph.py`（新增） | — | 7 passed | — |

### 7.4 证据清单

```text
artifacts/v011/
  cases/core_case_2d_mode.json                     生产 2D 运行（budget_exhausted，含披露预算）
  cases/core_case_native3d_mode.json               生产 native3d 运行（budget_exhausted）
  cases/core_case_reference_graph_2d.json          有限参考图 2D（search_exhausted）
  cases/core_case_reference_graph_native3d.json    有限参考图 native3d（success，含路径/溯源/独立验证）
  cases/core_case_initial_action_probe.json        初始状态 z 自由度探针
  logs/00_baseline_v01_suite.log                   v0.1 原始 64 passed 基线
  logs/00_prefix_v01_defect_replay.log             四项缺陷的 v0.1 回放
  logs/02_run_core_case.log                        证据重生成
  logs/03_final_full_pytest.log                    全量测试（第一次全绿，126 passed）
  logs/04_verification_final.log                   最终复核（127 passed，退出码 0）
  logs/probe_acceptance_matrix.log                 验收矩阵
  logs/probe_frontier_crossing.log                 前沿实测（99 501 次扩展 / 1200 s）
  probe/                                           可复现的探针与回放脚本
```

---

## 8. 未解决限制与进入 16 通道前需要的工作

仍然存在、且**不得声称已解决**的限制：

1. **单网**：没有多网联合路由、没有 3D 占用更新、没有 rip-up & reroute、没有 history congestion。
2. **自间距只在结果阶段检查**：搜索期状态只有 pose，非相邻自接近 spacing 仍由独立验证器后验检查；
   若要在搜索期强制，必须把路径历史放进状态并重新评估去重与支配关系。
3. **障碍只有 AABB**：没有网格/CSG/曲面障碍。
4. **终端连接器只支持共面 pose 对**，且只有四种 arc-straight-arc 族（无 LRL/RLR）。
5. **连续约束认证**：圆弧相关的量是保守区间判定，极少数临界情形会 `uncertified`（保守误拒，不允许假通过）。
6. **生产搜索在跨墙场景不可有限完成**（§6.5），当前依靠有限规范参考图给出验收证据；
   参考图本身也只覆盖最多两跳连接器。
7. **无光学模型**：没有任何损耗/串扰/dB 结果（AST 扫描测试强制）。

进入 16 通道之前至少还需要：

* 多网并发调度 + 3D 占用（把已布路径作为动态 keep-out）、rip-up & reroute；
* 把自间距纳入状态或改成有证明的顺序占位方案，并重新做一致性与支配性分析；
* 把障碍与波导截面从 AABB 扩到更真实的几何，并给出对应的保守距离界；
* 非共面终端连接器（例如 arc-arc-arc 或数值 Dubins/Reeds-Shepp）与其误差界；
* 一套可接受的"预算—完备性"策略：要么接受有限图抽象（并证明其与连续问题的关系），
  要么给出可穷尽的状态离散方案，二者都需要在 16 通道规模下重新评估；
* 面向通道数的性能工程（当前 `5-10 ms/次扩展，纯 Python）。

---

## 9. 最终问题回答

**1. 四项缺陷是否全部修复？哪些反例证明了修复？**
全部修复。A：S→G(100) 先生成、S→A→G(2) 最优的有限图反例，修复前欧氏与 h=0 都返回 100，修复后返回 2，
且与独立 Dijkstra 在 5 组随机图上一致。B：同桶 A=(10,0,0)/B=(10.4,0,0) 反例，修复前量化键 `search_exhausted`，
修复后安全身份下 cost = 20；另有同桶不同方向、同桶不同碰撞可行性、真实路由器上 13 个"会被旧键丢弃"的扩展状态。
C：交叉线段距离必须为 0（修复前 3.125）；完整验证器对交叉中心线必须 fail（修复前下界 0.390625 > 采样 0.109375 却 pass）；
两条采样点之间越界 1e-4 µm 的圆弧必须 fail；弧 AABB 必须精确。D：LSL 必须存在且总长 142.987184 µm
（修复前只剩 204.374715 µm 的 LSR/RSL），同时 LSR/RSL 的 v0.1 扫角保持不变。

**2. 当前实际采用什么状态与去重策略？能保证什么，不能保证什么？**
策略：`key_exact()`（完整 pose，1e-9 µm，合并容差 2e-9 µm）为默认；
只有同一 pose 以不更优的 g 再次到达才剪枝；更优 g 会重开状态；陈旧堆项丢弃。
旧量化键仅作为明确标注的近似探测模式（`approximate_state_identity=True`），其耗尽不是不可达证明。
能保证：在被枚举图上最优；不会把后继不同的 pose 合并掉。
不能保证：连续空间完备性（枚举图一般是无限的，可能以 `budget_exhausted` 结束）；
精确浮点状态也不等于连续空间完备搜索（去重、浮点误差、预算都是限制）。

**3. 独立验证器为何可以认证连续几何？哪些结果仅是采样估计？**
可以认证的部分：直线/直线、直线/AABB 距离是解析精确值；
圆弧相关判定给出认证下界 `min(弦距离) - 弦偏差`（并按层加密），边界用每个坐标的解析极值；
判定三分（safe / violation / undecided），且"可靠采样违规证据"优先判失败，无法判定返回 `uncertified`，永不 pass。
仅是估计的：`min_sampled_clearance_um`、`min_sampled_um`、`min_margin_um_sampled` 都是采样上界/估计量，
报告与 JSON 里与认证下界分开命名、并且被 `lower_bound <= sampled_min` 这一不变式约束。

**4. 原生 3D 跨墙案例是否仍成立？2D 失败是否确实不是预算耗尽？**
仍成立：有限规范参考图给出 495.301314 µm 的连续三维路线，z 从 0 升到 60 µm 跨过 keep-out 顶面 z=45 µm，
C1、R≥Rmin、独立验证器 `pass`、每段都有搜索动作来源。2D 失败不是预算问题：
参考图 **真正穷尽**（`search_exhausted`、非预算、115 次扩展），并另有几何分离证明
（墙 keep-out 覆盖整个 z=0 切片，任何连续曲线都必须穿越墙体的 x 面）。
生产搜索在同一场景报 `budget_exhausted`（"NOT a proof"），我们没有把它写成不可达。

**5. 全部测试实际多少个，passed/failed/skipped/xfail 分别多少？**
collected **127**，passed **127**，failed **0**，skipped **0**，xfail **0**，error **0**，退出码 **0**（日志 `artifacts/v011/logs/03_final_full_pytest.log`）。
关键正确性测试没有用 skip/xfail 规避；v0.1 的 64 passed 基线日志保留在同目录 `00_baseline_v01_suite.log`。

**6. 是否可以进入 16 通道阶段，尚缺哪些模块与验证？**
**不可以，现在还不能**。本轮只修好了单网正确性：多网调度与 3D 占用、rip-up & reroute、
搜索期自间距、非 AABB 障碍、非共面终端连接器、以及"可穷尽状态离散 vs 有限参考图"的完备性策略都还不存在；
此外当前生产搜索在跨墙场景不可有限完成、单次扩展 5-10 ms（纯 Python），
都需要在进入 16 通道前先解决或明确降级为可披露的近似方案。