# Native3D v0.2 多通道顺序布线基线报告

项目目录：`C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D`
任务来源：`docs/prompts/deepseek_native3d_v02_multichannel_baseline_prompt.md`
证据目录：`OpticalWaveguideRouterNative3D/artifacts/v020/`（`cases/`、`logs/`、`metrics/`、`figures/`、`probe/`）
历史证据：`artifacts/cases/`、`artifacts/v01_preserved/`、`artifacts/v011/`、`artifacts/v011_preserved/`、`artifacts/v012/`、`artifacts/v013/` 全部**冻结**，v0.2 的任何代码路径都不写它们（精确定义见 §7.4）。
环境：Windows-10-10.0.26200-SP0，Python 3.10.11（项目内 `.venv`），pytest 9.1.1；引擎本身只用标准库（matplotlib/numpy 只用于整板图）

本轮范围：**2 → 4 → 8 → 16 通道原生三维顺序布线基线**。不重写几何内核与搜索架构；不涉及 1024、ML、BPM、GUI/GDS、光学损耗/串扰 dB、全局优化、hierarchy、rip-up & reroute。

---

## 0. 结论摘要（先给结论）

1. **多通道顺序基线真的跑通了，而且是真实几何结果。** 2 / 4 / 8 / 16 通道跨墙案例：`routed = verified = committed = N`，16 通道 **16/16** 全部通过所声明的单网、网间与自间距验收，总长度 9506.548246 μm，wall time 3.61 s（每网 ≤ 0.341 s，总扩展 1017 次）。
2. **独立 JSON 联合验收器与调度器零分歧。** 11 个案例全部 `overall_verdict = pass`，`n_disagreements = 0`。该验收器只读写入磁盘的 JSON，只 import 标准库（AST 断言在测试里），重做端口、C1、Rmin、边界、障碍、静态参考线间距、已发布占用间距、全局弧长自间距与全部网间曲线对。
3. **每条网都用新建有限图。** `graph=` 从不传入；每个 net 的 `graph_branch = fresh_cold_graph`、`caller_supplied_graph = False`、`graph_cache_state = "cold"` 都逐网记录在 artifact 里。已知的“图持有可变 Scene/config/成本场别名”问题由 **新建图 + 深拷贝输入** 隔离，而不是去改缓存机制。**报告明确声明 warm/prebuilt 图分支本轮仍未验证。**
4. **固定占用是硬约束，拥塞惩罚恒为 0。** `cost = path_length`，`congestion_cost_um_equivalent = 0.0`；避让靠把已验收网的全部真实直线/圆弧发布成参考中心线，进入现有 `centerline_spacing` 硬约束。拥塞惩罚从不被当成“可以支付的碰撞”。
5. **自间距检查真正接进主流程，且语义是全局弧长。** 定义域是 `|s-t| >= ell_local`（`P(s)` 为整条路径累计弧长），`ell_local = 2*dmin = 10 μm` 是本轮**基线局部几何豁免政策**（不是论文参数、不是光学安全距离）。它覆盖同段自交、多圈回返、相邻段远端与非相邻段；豁免带内的最近点永不作为违规。生产侧与独立 JSON 侧都实际执行，且两侧数值在直线上一致（9.999999999 vs 10.0）。
6. **“候选路径”与“合法路径”严格分开计数。** `routed_count`（搜到候选）、`verified_count`（通过全部声明验收）、`committed_count`（已发布占用）三个数分别报告；失败按类别统计，部分成功从不写成 `all_success`。压力案例正是这样：4 通道 4 个候选 **0 条合法**，8 通道 6 个候选 **5 条合法**。
7. **压力案例给出了真实失败原因，而且第一原因是自间距而不是拥塞。** `stress_swap_04` 4 条候选全部被后验自间距判为违规（brute-force 独立复核：0.021 / 0.044 / 0.597 / 3.861 μm，均 < dmin = 5 μm，witness 弧长间隔 ≈ 314 μm 完全在检查域内）；`stress_swap_08` 出现 3 类失败：`self_spacing_violation` ×1、`search_exhausted` ×1、`port_attachment_failed` ×1。
8. **全量测试真实结果：617 passed / 0 failed / 退出码 0 / 185.96 s**（`artifacts/v020/logs/01_pytest_full.log`）。v0.1.3 基线是 568 passed，本轮**新增 49 个**测试，未删除、未跳过任何既有用例；原有单网跨墙与旧反例回归全部保留并通过。
9. **整板完整布线图已交付。** 11 张 `artifacts/v020/figures/board_layout_*.png`，其中 `board_layout_all16.png` 是 16 通道整板图（三维整板、俯视、侧视、逐网间距认证柱状图）；绘图脚本只读 case JSON，不 import 搜索引擎。
10. **仍未解决的范围（不夸大、不隐藏）**：搜索状态没有路径历史，因此**不声称在自间距约束下搜索完备或全局最优**；顺序基线没有 net ordering 优化、没有 rip-up & reroute，所以交叉/置换类案例只能部分成功；障碍仍是 AABB；有限格点仍不能复现任意连续路线；warm 图复用分支、`constraints.checks.path_self_spacing()`（旧 API）以及极端输入审计仍未验证。
11. **本轮停在 16 通道。** 未扩展 64/256/1024，未加入任何光学损耗或串扰 dB 数字（全流程无 dB、无插入损耗、无串扰）。

### 0.1 新增/修改文件一览

| 文件 | 关键内容 |
|---|---|
| `src/native3d/multinet/models.py` | 新增。`NetSpec`/`MultiNetCase`/`NetOutcome`/`MultiNetResult`、`SelfSpacingPolicy`、`default_self_spacing_policy`（`ell_local = 2*dmin`）、v0.2 状态词汇与三类计数的定义 |
| `src/native3d/multinet/occupancy.py` | 新增。`Occupancy`/`OccupancyEntry`：发布**完整真实曲线**（net_id + segment_id + occupancy_revision + 几何 + 内容指纹）；只允许已验证路径发布、一个 net 只能发布一次、发布后不可改；`scene_for_net()` 组装“静态参考线 + 已发布曲线”的新 Scene（深拷贝） |
| `src/native3d/multinet/self_spacing.py` | 新增。`path_self_spacing_global()`：全局弧长自间距，域 `|s-t| >= ell_local`；弦元 + 凸二次在凸多边形上的精确极小（含内部驻点与边界闭式解）；witness 必须在检查域内；`SELF_SPACING_FP_MARGIN_UM = 1e-9` |
| `src/native3d/multinet/inter_net.py` | 新增。`pair_spacing_certified()`/`inter_net_spacing_check()`：网间**没有**自间距局部豁免；分别输出采样估计、保守下界、违规 witness、认证状态；按 line-line/line-arc/arc-line/arc-arc 分类计数 |
| `src/native3d/multinet/sequential.py` | 新增。`route_multinet()` 顺序调度器（每网新 Scene/新图/ZeroCongestion/硬约束占用/三类验收/发布规则/部分成功），`joint_validation_payload()` 生成独立验收器的输入 |
| `src/native3d/multinet/cases_v020.py` | 新增。5 个两通道最小验收案例、2/4/8/16 跨墙系列、4/8 通道交叉置换压力案例、控制跑法 |
| `src/native3d/validation/joint_validate.py` | 新增。**独立**联合验收器：只读 JSON，只 import 标准库；自带几何第二实现（直线/圆弧求值、点到平面圆弧距离、精确有理线段距离、弦细分保守界、自己的全局弧长自间距算法——用“矩形边 + 带边界线”而非多边形裁剪） |
| `examples/run_multichannel_baseline.py` | 新增。可直接运行的公共入口，写出 `cases/`、`logs/`、`metrics/`，并把**写入磁盘后读回**的 JSON 交给独立验收器 |
| `examples/plot_board_layout.py` | 新增。整板完整布线图（三维/俯视/侧视/通道地图/间距认证面板），只读 case JSON |
| `tests/test_v020_*.py`（4 个文件）+ `tests/v020_helpers.py` + `tests/conftest.py` | 新增 49 个回归 |
| `README.md` | 增补 v0.2 章节（模块职责、命令、范围与限制） |

---

## 1. 运行方式（公共入口）

```powershell
cd "C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D"

# 全部 11 个案例：5 个两通道最小验收 + 2/4/8/16 跨墙 + 4/8 通道压力 + 声明好的控制跑法
.venv\Scripts\python.exe examples\run_multichannel_baseline.py

# 只跑某几个案例 / 关掉控制跑法
.venv\Scripts\python.exe examples\run_multichannel_baseline.py --only crosswall
.venv\Scripts\python.exe examples\run_multichannel_baseline.py --no-controls

# 整板完整布线图（只读 case JSON）
.venv\Scripts\python.exe examples\plot_board_layout.py

# 独立联合验收器（只吃一个 JSON 文件，不 import 引擎）
$env:PYTHONPATH = "src"
.venv\Scripts\python.exe -m native3d.validation.joint_validate artifacts\v020\cases\crosswall_16_channels.json

# 对抗式最终复核（对每个 artifact 逐网 brute-force 重算）
.venv\Scripts\python.exe artifacts\v020\probe\v020_final_review.py

# 全量测试
.venv\Scripts\python.exe -m pytest
```

---

## 2. 数据流：顺序固定占用与发布规则

```
MultiNetCase (chip, 静态障碍, 静态参考中心线, RouterConfig, 有序 net 列表)
        │
        ├─ net i:  scene_for_net(case, net_i, occupancy)
        │             = deepcopy(case.chip / obstacles / config)
        │             + [静态参考中心线] + [occupancy 已发布曲线]     ← 顺序耦合的唯一通道
        │           route_finite_native3d(scene, mode="native3d", congestion=None, graph=None)
        │             → 内部新建 FiniteGraph（graph= 从不传）
        │             → ZeroCongestion：cost = 真实路径长度，固定占用是硬约束
        │
        ├─ 候选验收（全部通过才算 verified）
        │     1) 生产验收：装配曲线硬约束 + 真实端口/C1 端点核对（router 内部已判，artifact 记录）
        │     2) 后验全局弧长自间距（path_self_spacing_global）
        │     3) 发布前网间间距（inter_net_spacing_check，无局部豁免）
        │
        ├─ 通过 → occupancy.publish(net_id, 全部真实直线/圆弧) → revision +1
        │         每条曲线记录：origin / net_id / segment_id / occupancy_revision /
        │                       kind / length_um / fingerprint / 完整几何
        │         发布后 verify_immutability() 复核内容指纹
        │
        └─ 不通过 → 不发布。保留原始候选几何（candidate_segments）与失败类别；
                    默认继续尝试下一个 net（`continue_after_failure=True`），
                    结果记为部分成功，而不是 all_success
```

**发布规则（代码强制，不是口头约定）**

- `Occupancy.publish(..., verified=False)` 直接 `OccurancyError`：失败/未认证路径绝不可能进入占用。
- 同一个 `net_id` 第二次发布直接 `OccurancyError`：无隐式 rip-up、无偷偷改线。
- 空路径拒绝发布。
- 发布存的是**真实 `LineSegment3D`/`CircularArc3D` 对象**（深拷贝 + 内容指纹），**不是**节点、控制点、折线弦或 AABB。
- 静态参考中心线以 `origin = "static_reference_centerline"`、`net_id = None` 记录：它们对每条网都是占用几何，但**永远不属于本轮任何一条网**，不会被误当成某网的路径。
- `scene_for_net()` 每次重新深拷贝，所以后续 net 既改不到 case，也改不到已发布曲线；`verify_immutability()` 用指纹复核，结果写进 artifact（11 个案例全部 `ok = True`）。

**每次调用的隔离方式（对应 prompt 2.1）**

| 风险 | v0.2 的处理 | 证据字段 |
|---|---|---|
| 图持有可变 Scene/config/成本场引用 | 每条网新建 `FiniteGraph`，`graph=` 从不传 | `graph_branch = "fresh_cold_graph"`、`graph_reuse.accepted = True`、`caller_supplied_graph = False`、`graph_cache_state = "cold"` |
| 跨网复用边缓存 | 不跨网复用任何图或边缓存 | 同上 + 每网 `occupancy_revision_before/after` |
| 可变自定义成本场 | 不接入；`ZeroCongestion` | `congestion_field = "ZeroCongestion (cost = real path length)"`，每网 `cost == length` |
| 共用可变输入对象 | Scene/chip/obstacles/config/poses 全部深拷贝 | `scene_for_net` 的 metadata + 测试 `test_scene_for_net_is_a_fresh_copy_carrying_static_plus_published` |

---

## 3. 自间距语义、保证范围与失败语义（对应 prompt 4.1）

### 3.1 语义

整条路径用累计弧长参数化：`P(s)`，`s ∈ [0, L]`。检查域

```
D = { (s, t) ∈ [0, L]² : |s − t| ≥ ell_local }
问题：min_{D} |P(s) − P(t)| ≥ dmin ?
```

- 只豁免**全局弧长近邻带** `|s − t| < ell_local`，不是“同段/相邻段的整个组合”。
- `ell_local` 是纯几何长度（μm），与采样步长、弦细分层数、段索引**无关**。
- 本轮基线显式取 `ell_local = 2 * dmin = 10 μm`，记为 `kind = "two_times_dmin"`。**这是本轮的局部几何豁免政策，不是论文参数，也不是光学安全距离**；适用范围就是本报告的这批合成案例与这套 `RouterConfig`。
- 覆盖：同一段与自身比较（含单段内部相邻元对）、多圈回返、相邻段远端、非相邻段。
- witness 的两个全局弧长必须满足 `|s − t| ≥ ell_local`（字段 `in_check_domain`），豁免带内的最近点永不作为违规。

### 3.2 判定方法与两个保守保护

每条段先做域安全（|sweep| ≤ π）的弦元分解，弦按自身弧长以 `|chord|/|arc|` 速度参数化。对一元对，检查域在该弧长矩形内等于矩形减对角带，即两个**凸多边形**：

```
P_left  = 矩形 ∩ { t ≤ s − ell_local }
P_right = 矩形 ∩ { t ≥ s + ell_local }
```

弦点间平方距离是 `(s,t)` 的**凸二次**，凸函数在凸多边形上的极小值要么在内部驻点、要么在边界；边界逐边闭式求解。于是

```
min_D |P − P| ≥ (两个多边形上的弦距离极小) − δ₁ − δ₂      ← 认证下界
min_D |P − P| ≤ (两个多边形上的弦距离极小) + δ₁ + δ₂      ← 认证上界
```

符号不是装饰：元上任意点离弦最多 δ，所以曲线可能比弦**更近**（下界要减），也可能**更远**（上界要加）。

两个保守保护，都写进结果：

- `SELF_SPACING_FP_MARGIN_UM = 1e-9 μm` 从每个认证下界里扣除，裁剪切片的浮点误差不可能把违规变成 safe；
- 每个元对的认证下界不高于**整个矩形**的认证下界（检查域是矩形的子集，矩形极小值不可能超过受限极小值）。

违规只能用**显式验证过落在矩形内且 `|s−t| ≥ ell_local`** 的点对证明；安全只能由认证下界达到 dmin 证明；细化上限内两者都决定不了就是 `undecided`，**永远不算 pass**。

### 3.3 为什么不能直接用旧的 `path_self_spacing(exempt_adjacent=True)`

那个函数按**段索引**豁免 `j − i == 1`：它既看不见同一段内部的自交（多圈圆弧、被拼成两段的同一条直线），也把相邻段**远端**一起豁免掉——豁免与物理长度无关。测试 `test_the_old_segment_index_exemption_could_not_see_this` 把两种语义并排固定下来。

### 3.4 失败语义（不伪装成别的状态）

| 情况 | 状态 | 说明 |
|---|---|---|
| 候选存在但自间距违规 | `self_spacing_violation` | 保留原始候选几何 + witness（段对、两个全局弧长、弧长间隔、`in_check_domain`、采样距离、上界、细化层数） |
| 候选存在但细化上限内无法认证 | `self_spacing_uncertified` | 记录认证下界 < dmin，**不是** `search_exhausted` |
| 候选存在但与已发布占用间距违规 | `inter_net_spacing_violation` | 同上保留候选 |
| 候选存在但网间距无法认证 | `inter_net_spacing_uncertified` | 同上 |
| 搜索本身没找到候选 | `search_exhausted` / `budget_exhausted` / `port_attachment_failed` 等 | 直接沿用 router 的状态词汇，**没有**把后验失败改写成它们 |

反过来的保证也写死在测试里：`test_a_post_check_failure_is_never_relabelled_as_a_search_failure` 断言凡是 `routed_candidate=True` 却未被接受的网，其 `search_status` 仍是 `success`、`status` 不是 `search_exhausted`、且候选段非空。

---

## 4. 网间验收（对应 prompt 4.2）

跨网**没有**自间距局部豁免：不同网不会因为段号相同、端口靠近或 `net_id` 规则而忽略冲突。每一对连续曲线都判定 `line-line / line-arc / arc-line / arc-arc`，并且当任一侧是曲线时，保守下界同时扣掉**两侧**的几何逼近误差。三类数字分开输出：`sampled_estimate_um`（有限点采样，只能偏大，永不作为证书）、`certified_lower_bound_um`（永不大于真实最小距离）、`violation witness`（真实落在两条曲线上的点对）。`undecided` 不算 pass。

联合验收器还额外检查一个不变量：**已发布集合内部两两 ≥ dmin**（`n_committed_pairs_checked` / `n_inter_net_violations`）。被拒绝的候选不在这个不变量里（它从未发布），它与已提交集合的间距作为**证据**单列在 `rejected_candidate_spacing_evidence`，不构成整轮失败。

---

## 5. 真实结果

### 5.1 两通道最小验收 5 案例（prompt 5）

| # | 案例 | 有序 net | routed/verified/committed | 结果要点 |
|---|---|---|---|---|
| 1 | `two_channel_01_parallel_straights` | netA→netB | 2/2/2 | 两条互不干扰直线，各 450.000 μm，z 全程 0；联合验收 pass |
| 2 | `two_channel_02_barrier_3d_avoidance` | netA→netB | 2/2/2 | netA 直线 296.000 μm（z=0）；netB **594.159265 μm、z 从 0 升到 100**，纯三维搜索绕行；控制跑法（去掉 netA）netB = 480.000 μm 且 z 不变 |
| 3 | `two_channel_03_arc_occupancy` | netA→netB | 2/2/2 | netA 378.540 μm，段型 `LLA`（**含真实 90° 圆弧**）；netB 直线原本 300.000 μm 且会穿过该圆弧，发布圆弧占用后变成 414.159 μm 绕行；联合验收对该圆弧给出认证间距 |
| 4 | `two_channel_04_no_solution_under_occupancy` | netA→netB | 1/1/1 | 薄片芯片（z ∈ [−30,30]，晶格只有 z=0 层）里 netA 跨越整个内侧 y 范围：netB `search_exhausted`，**netA 保持原样**；控制跑法证明 netB 单独可布（committed=1），即占用是原因 |
| 5 | `two_channel_05_reversed_order` | netB→netA | 2/2/2 | 同一几何交换顺序：netB 先走成 480.000 μm 直线，netA 变成 410.159265 μm、z 升到 100 —— 与案例 2 的 296.000 μm 明确不同，顺序真的影响结果 |

### 5.2 16 通道必达基线（prompt 6.1）

同一有序 16 网列表，2/4/8/16 都取**前 N 条**，主流程完全相同。

| 案例 | N | routed | verified | committed | 联合验收 | 总长度 (μm) | wall (s) | 每网最大 wall (s) | 占用 revision | 已发布曲线 | 总扩展 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `crosswall_02_channels` | 2 | 2 | 2 | 2 | pass | 1188.318531 | 0.15 | 0.086 | 2 | 14 | 130 |
| `crosswall_04_channels` | 4 | 4 | 4 | 4 | pass | 2376.637061 | 0.37 | 0.119 | 4 | 28 | 256 |
| `crosswall_08_channels` | 8 | 8 | 8 | 8 | pass | 4753.274123 | 1.25 | 0.256 | 8 | 56 | 517 |
| `crosswall_16_channels` | **16** | **16** | **16** | **16** | **pass** | **9506.548246** | **3.61** | **0.341** | 16 | 112 | 1017 |

- 16 通道**每条网长度都恰好是 594.159265359 μm**，段型全为 `LAALAAL`（3 直线 + 4 个 90° 圆弧），**z 全部从 0 升到 100**：墙顶 z=40、要求 5 μm 避让，`z = 100` 是格点层里合法的高度，**没有任何手写层高、中点或换层位置**。
- 墙阻断端口平面：16 条路线的 z 变化全部来自三维搜索动作（`trace` 里逐节点记录 action/x/y/z/direction/azimuth/elevation/g）。
- 16 条路的 x-z 剖面完全重合（每条在自己的 y 泳道里做同一个翻越），**差异只在 y**；整板图 `board_layout_all16.png` 的面板 (c) 明确标注了这一点。
- 逐网认证间距（独立验收器复算）：对已发布占用的认证下界 = **10.000000 μm**（要求 5 μm），自间距认证下界 = **5.854793 μm**（要求 5 μm），`undecided = 0`。
- 对抗式 brute-force 复核（§7.2）给出更紧的实际值：16 条网自间距实际最小 **10.914139 μm**，已提交网两两实际最小 **10.000000 μm**（泳道间距 10 μm，要求 5 μm），两者都远高于 dmin，且两个实现的认证下界都没有超过 brute-force 最小值。
- 完整证据：`artifacts/v020/cases/crosswall_16_channels.json`（输入 + 每网状态/长度/扩展数/占用 revision + 全部曲线）、`crosswall_16_channels.validated.json`（+ 独立验收全文）、`artifacts/v020/logs/crosswall_16_channels.log`、`artifacts/v020/metrics/crosswall_16_channels_metrics.json`。

### 5.3 压力案例（prompt 6.2）：真实成功率与失败原因

| 案例 | N | routed | verified | committed | 失败类别 | wall (s) | 每网最大 wall (s) | 联合验收 |
|---|---|---|---|---|---|---|---|---|
| `stress_swap_04_channels` | 4 | 4 | **0** | 0 | `self_spacing_violation` ×4 | 2.71 | 0.694 | pass（4 个 `rejected_ok`） |
| `stress_swap_08_channels` | 8 | 6 | **5** | 5 | `self_spacing_violation` ×1、`search_exhausted` ×1、`port_attachment_failed` ×1 | 136.77 | 54.086 | pass（5 pass + 1 rejected_ok + 2 no_path） |

压力案例是确定性端口置换：net i 从泳道 i 出发，必须到达泳道 `N−1−i`，因此各网必须交叉并同时翻墙。已发布占用**确实改变了后续搜索**：不是一组互不相干的平行直线。

**第一失败原因不是拥塞，而是自间距。** 4 通道案例里 4 条候选路径都被后验全局弧长自间距判违规，独立复核给出：

| net | witness 段对 | 弧长间隔 (μm) | 独立采样距离 (μm) | brute-force 复核最小距离 (μm) | dmin (μm) |
|---|---|---|---|---|---|
| net00 | [6, 8] | 314.181 | 0.0220 | **0.020944** | 5 |
| net01 | [6, 8] | 314.177 | 0.0177 | **0.043967** | 5 |
| net02 | [6, 7] | 313.883 | 2.566 | **0.596775** | 5 |
| net03 | [5, 7] | 316.350 | 3.966 | **3.861167** | 5 |

这四条路径都出现了**沿弧长走出去 314 μm 后又回到自己附近**（近闭合回路）的结构：搜索状态没有路径历史，所以搜索本身不会阻止这种事。v0.2 的做法是**如实拒绝**并把候选与 witness 留在证据里，而不是发布一条自接近的路径，也不是伪称“找到了另一条合法路线”。这正是 prompt 4.1 要求的语义。

8 通道案例的三类失败：

- `self_spacing_violation`（net00）：候选自接近，后验拒绝；独立验收同样判违规（`rejected_ok`）。
- `search_exhausted`（net03）：在已发布占用下该网格耗尽（**这是“这个被构造的有限图被穷尽”，不是“物理不可达”**）。
- `port_attachment_failed`（net07）：其端口邻域内已没有一条合法附着曲线能通过 `_accept` 的占用检查（**这是“支持的动作集合没找到”，不是物理不可达**）。

**下一步是否需要 net ordering / rip-up & reroute？** 需要，但理由要写清楚：

1. 顺序固定时，先布者占优，交叉/置换案例的成功率由顺序决定；引入 net ordering（按拥塞/关键度排序、或多次顺序尝试并择优）是最小成本的提升。
2. 自间距失败**不是**靠 rip-up 就能解决的，它需要搜索侧带路径历史（或发布前对候选做局部修正）；这是真正的下一阶段工作，本轮明确不做。
3. 端口的 `port_attachment_failed` 说明“端口邻域 + 原点附着曲线”这一族在拥堵下会先失效，值得在下一轮与 ordering 一起处理。

---

## 6. 性能与预算（prompt 7）

- 每网仍用有限图 A*，预算 = **60 s / 200 000 扩展**（`finite_max_runtime_s` / `finite_max_expansions`），预算**不跨网共享、不追加**。
- 16 通道全部在 0.31 s/网以内完成，没有任何一次 budget 停止。
- 8 通道压力案例最慢单网 52.50 s（仍在 60 s 内），整案例 136.77 s；这是**真实耗时**，没有被写成“不可达”。
- 超预算的语义不变：`budget_exhausted` 与 `search_exhausted` 是两个不同状态，前者绝不写成“不可达”。

| 案例 | 总 wall (s) | 总和 search (s) | 每网最大 search (s) | 总扩展 |
|---|---|---|---|---|
| crosswall_16_channels | 3.61 | 0.92 | 0.11 | 1017 |
| stress_swap_08_channels | 136.77 | 128.11 | 52.50 | 6645 |
| two_channel_03_arc_occupancy | 3.82 | 3.79 | 3.78 | 57 |

（`two_channel_03` 偏慢的原因是对含圆弧的参考曲线做弦细分保守判定：44 次扩展却花了 3.78 s，属于已知的“保守下界算得贵”，不是搜索爆炸。）

---

## 7. 证据与验证

### 7.1 独立 JSON 联合验收器

`src/native3d/validation/joint_validate.py`：

- **只读一个 JSON 文件**，`python -m native3d.validation.joint_validate <file>` 可直接跑。
- **只 import 标准库**（`json, math, sys, fractions, typing, __future__`），测试 `test_the_validator_is_standard_library_only` 用 AST 断言源码里**没有** `native3d` 导入。
- **不导入调度器任何验收结果**：它自己重算端口（源用源容差、终点用接受球）、C1、Rmin、边界、障碍、静态参考线间距、**顺序窗口内**的已发布占用间距、全局弧长自间距、以及已提交集合的两两网间间距。
- 自带几何第二实现，其中全局弧长自间距用的是**另一种算法**：不做多边形裁剪，而是在“矩形边 + 带边界线”上做一维闭式极小（生产侧是裁剪 + 逐边极小）。这是刻意的第二实现，不是复制粘贴。
- 数字分三类报告：`sampled_estimate` / `certified_lower_bound` / `violation_witness`，`undecided` 永不算 pass。
- 契约违规直接 `ContractError`（单位不是 `um`、未知 primitive kind、零方向切向、零长度线段、非正交圆弧基、未知占用 origin），**不会**被静默跳过，也不会被降级成“某网几何不好”。

**本轮位置与结论**：11 个案例 `overall_verdict = pass`，`n_disagreements = 0`。声明 `committed` 的网全部 `verdict = pass`（含占用身份校验：发布曲线必须逐字节等于该网返回路径的某一条 primitive）；声明被拒的网全部 `verdict = rejected_ok`（几何确有失败）；无路径的网记 `no_path`。

### 7.2 对抗式最终复核（brute force）

`artifacts/v020/probe/v020_final_review.py` 只读 artifact，独立地：

1. 重跑联合验收器；
2. 对**每条网**按 ~1 μm 采样 brute-force 重算 `min_{|s−t| ≥ ell} |P(s) − P(t)|`（用分块剪枝保证每一个被比较的点对真的在检查域内），并断言**两个实现的认证下界都不超过 brute-force 最小值**（下界超过真实最小值在数学上不可能，出现即为伪造）；已提交的网必须 `brute ≥ dmin`；
3. 对**每一对已提交网**做无豁免 brute-force 最小距离；
4. 检查“committed 集合互相避让”“未提交的网不出现在占用里”“occupancy 不可变”；
5. 核对 artifact 里引用的三个计数与逐网记录一致。

结果：`artifacts/v020/probe/v020_final_review_result.json`，`artifacts/v020/logs/02_final_review.log`。

```text
adversarial final review verdict: PASS (0 problem(s))     # 11 个案例全部 problems = 0
```

逐案例复核结果（`brute self-spacing min` = 该案例所有网路径在检查域内实际最近距离的最小值；`committed-pair min` = 已提交网两两实际最近距离的最小值）：

| 案例 | 联合裁决 | 有路径的网 | brute 自间距最小 (μm) | 已提交网对最小 (μm) | problems |
|---|---|---|---|---|---|
| crosswall_02_channels | pass | 2 | 10.914139 | 10.000000 | 0 |
| crosswall_04_channels | pass | 4 | 10.914139 | 10.000000 | 0 |
| crosswall_08_channels | pass | 8 | 10.914139 | 10.000000 | 0 |
| crosswall_16_channels | pass | 16 | 10.914139 | 10.000000 | 0 |
| stress_swap_04_channels | pass | 4 | 0.170745 | -（无已提交网） | 0 |
| stress_swap_08_channels | pass | 6 | 0.487277 | 6.695667 | 0 |
| two_channel_01_parallel_straights | pass | 2 | 11.000000 | 180.000000 | 0 |
| two_channel_02_barrier_3d_avoidance | pass | 2 | 10.914139 | 98.661631 | 0 |
| two_channel_03_arc_occupancy | pass | 2 | 10.914139 | 30.000000 | 0 |
| two_channel_04_no_solution_under_occupancy | pass | 1 | 11.000000 | -（只有一条已提交网） | 0 |
| two_channel_05_reversed_order | pass | 2 | 10.914139 | 22.146361 | 0 |

注意 `stress_swap_04_channels` 的 brute 自间距最小值 0.170745 μm 与 `stress_swap_08_channels` 的 0.487277 μm 都来自**被拒绝**的候选路径，正是它们被拒绝的原因；而已提交的部分（stress_08）两两 6.695667 μm ≥ dmin，说明“已提交的就必须合法”这条不变量真的成立。

### 7.3 全量测试真实数量与日志

```text
权威运行：.venv\Scripts\python.exe -m pytest -q
collected 617, passed 617, failed 0, skipped 0, xfail 0, error 0
退出码 0，墙钟 185.96 s
日志：artifacts/v020/logs/01_pytest_full.log
```

- v0.1.3 基线：**568 passed / 210.09 s**（本轮开工前实测，命令相同）。
- 本轮新增 **49** 个测试（`tests/test_v020_*.py`），**未删除、未跳过**任何既有用例；原有单网跨墙、旧反例、Dijkstra 交叉校验、v0.1.1/0.1.2/0.1.3 回归全部保留通过。

新增 49 个测试分布：

| 文件 | 数量 | 覆盖 |
|---|---|---|
| `tests/test_v020_self_spacing.py` | 13 | ell_local 是长度不是段索引；C1 直线拆分前后一致（LB 都是 9.999999999）；正常空间弯曲通过；单段整圈 / 拆成 4 段 / 两圈都检测到非局部回返且 witness 在域内；真实自接近（2 μm、4 μm）违规、6 μm 合法、40 μm 合法；原跨墙 594.159265 μm 路径保持合法；4π 弧不被 sagitta 周期性伪造成安全 |
| `tests/test_v020_occupancy.py` | 9 | 静态参考线不归属任何网；发布保存完整曲线与 provenance；未验证/空/二次发布被拒；发布单调且不可变（含篡改检测）；`scene_for_net` 深拷贝且带“静态 + 已发布”；router 真的绕开已发布的直线棒（z > 40）且 `congestion_cost == 0` |
| `tests/test_v020_sequential.py` | 14 | 5 个最小验收案例的真实数字；顺序交换确实改变结果；每网 `fresh_cold_graph` / 从不传 `graph=`；拥塞恒 0、占用是硬约束；后验失败不被改写成 `search_exhausted`；未接受的网不发布任何东西；`publish_on_verify=False` 只验证不提交；`continue_after_failure` 两种行为；joint payload 契约 |
| `tests/test_v020_joint_validate.py` | 13 | 只依赖标准库（AST）；atan2 vs acos 精度回归；圆弧 AABB 在 wrap 相位上的回归 + 120 条随机多圈圆弧与密集采样一致；精确线段-盒距离与密集采样一致；拒绝被几何证实 = `rejected_ok`、不被证实 = `fail`；已提交集合两两不变量（顺序重算 + 成对检查两条路径）；被拒候选只作证据；多圈回返；与生产侧数值一致；契约违规全部 raise |

### 7.4 新增证据目录

```text
artifacts/v020/cases/     <case>.json（输入 + 每网结果 + 占用）、<case>.validated.json（+ 独立验收全文）
artifacts/v020/logs/      <case>.log（逐网表格）、01_pytest_full.log、02_final_review.log
artifacts/v020/metrics/   <case>_metrics.json、summary.json
artifacts/v020/figures/   board_layout_<case>.png、board_layout_all16.png
artifacts/v020/probe/     v020_geometry_spike.py、v020_cases_spike.py、v020_self_spacing_probe.py（开工前的可行性格点预研）、v020_final_review.py + v020_final_review_result.json
```

`artifacts/cases/`、`artifacts/v01_preserved/`、`artifacts/v011/`、`artifacts/v011_preserved/` 本轮**零写入**（实测：今天 12:10 之后写入文件数均为 0）。

**关于 v012 / v013 的精确说明（避免夸大"零写入"）**：v0.2 自己的代码路径只写 `artifacts/v020/`；但 prompt 要求的"运行全部既有和新增测试"会触发**既有测试套件**自身的行为：

| 目录 | 文件 | 谁写的 | 性质 |
|---|---|---|---|
| `artifacts/v013/cases/` | 7 个 `core_case_*.json` | `tests/conftest.py` → `io/artifacts.py`（`EVIDENCE_VERSION` 默认 `v013`） | v0.1.3 层的当前证据目录，v0.1.x 测试套件**每次跑都重生成**（v0.1.3 起既有行为） |
| `artifacts/v012/cases/` | 7 个 `core_case_*.json` | `artifacts/v013/prefix_src/io/artifacts.py`（**冻结的 v0.1.2 源码快照**，其默认版本是 `v012`） | v0.1.3 的"修复前红-绿复现"测试按设计重建它自己的 v0.1.2 证据 |

这两个目录内其它文件（`v013/logs/`、`v013/probe/`、`v013/prefix_src/`、`v012/logs/`、`v012/probe/`）本轮均未被改写（实测分别有 80 个 / 若干个文件 mtime 早于本轮）。也就是说：**没有任何 v0.2 代码写历史证据**；变化的只有既有测试套件本来就负责重建的那 14 个 JSON，且它们的内容被 v0.1.3 的逐字段断言（与原跨墙 594.1592653589794 μm / 75 次扩展对比冻结 artifact）在 617 passed 里一并校验过。

---

## 8. 整板完整布线图

`examples/plot_board_layout.py` 只读 `artifacts/v020/cases/*.json`（优先 `.validated.json` 以取得独立裁决），不 import 搜索引擎，也不重新布线；画出来的每一条曲线就是某条已提交网的某一条真实 primitive。

- `board_layout_all16.png`：**16 通道整板图**——(a) 三维整板（芯片盒 + 墙 + 16 条彩色曲线，黑色细线为段边界）、(b) 俯视 x-y（端口 / 泳道 / 墙）、(c) 侧视 x-z（翻墙的 z 变化，并注明 16 条剖面完全重合）、(e) 逐网间距认证柱状图（对已发布占用的认证下界 = 10 μm、自间距认证下界 = 5.85 μm，红色虚线为 dmin = 5 μm）。
- `board_layout_<case>.png`：每个案例一页，(a) 三维 + (b) 俯视 + (c) 侧视 + (d) 端口/泳道地图 + (e) 间距认证 + (f) 案例摘要表（通道数、顺序、routed/verified/committed、失败类别、总长度、占用 revision、独立裁决）。有障碍的案例 (e) 画墙平面 y-z 剖面。

---

## 9. 已知缺陷（本轮开发中真实抓到并修掉的）

这些是**在实现和验证过程中实际出现**的错误，写进来是因为它们决定了现在哪些数字可以信：

| # | 位置 | 症状 | 原因 | 修法 |
|---|---|---|---|---|
| 1 | 自间距凸二次极小 | 认证下界出现 0（本该 10） | `Q1−Q2` 的常数项 `W` 符号写反 | 改为 `A.start − B.start − s₀v₁ + t₀v₂` |
| 2 | 自间距上界 | 认证上界方向错误 | 弦比曲线远 δ，上界应 `+δ` 而非 `−δ` | 改为 `chord_min + δ`；下界仍 `−δ` |
| 3 | 自间距元对极小 | 凸多边形只逐边求解会漏掉内部驻点 | 平行弦的极小值可能落在内部平坦段之外的驻点 | 加回内部驻点候选（宽松判定，宁可保守） |
| 4 | witness 取点 | 直线路径的 witness 距离报 100 而真实是 2 | 全局弧长落在分段边界时 `_point_at_global` 取了前一段 | 分段改为半开区间 `[a, b)`，junction 归下一段 |
| 5 | 独立侧矩形边参数化 | 部分元对总判 `undecided` | 半平面写成了检查域的**补集**，且边的参数区间与基点不一致 | 改为 `g = ell − sign·(t − s) ≤ 0`，边用“角点 + 方向 + 参数长度” |
| 6 | 独立侧 C1 | 生产已接受的 junction 报 1.207e-6°“折角” | 用 `acos(dot)`，在 θ≈0 附近分辨率只有 ~1e-6° | 改用 `atan2(|u×v|, u·v)`（与生产侧一致） |
| 7 | 独立侧圆弧 AABB | 合法路径报越界 12–32 μm | 缺 `amp = R·hypot(uᵢ,vᵢ)`；且在 wrap 相位 `φ*+π = 2π` 上按“圆心+半径”假设符号 | 逐轴用真实 `amp`，候选相位**求值**而不是假设符号 |
| 8 | 独立侧障碍避让 | 合法网报 `obstacle_clearance` 违规 | 弦细分后固定扣一个 sagitta，得到的是保守下界却被当成精确值 | 改成认证判定：witness 优先、LB/UB 细化到能判定或到上限才 `undecided` |
| 9 | 独立侧精确线段-盒距离 | 精确值 10.76 < 密集采样 16.08（不可能是精确最小） | 在区间端点采样活跃项集合，端点恰好是断点时该项被丢弃 | 用**区间中点**确定活跃项集合 |
| 10 | 独立侧裁决语义 | 正确的“拒绝”被算成整轮 `fail` | 把“该网被拒”与“整轮不合法”混为一谈 | 引入 `rejected_ok` / `no_path` / `fail` 四值，整轮只在矛盾或已提交集合违规时失败 |
| 11 | 独立侧占用窗口 | 后续网的曲线被拿来判前面网的违规 | 用“全部已发布曲线”而不是“该网发布之前已发布的” | 按 `occupancy_revision` 顺序窗口比较；后续曲线交给已提交集合成对检查 |
| 12 | 独立侧契约违规 | 坏 primitive 只被记成某网 `fail` | 逐网 `try/except` 吞掉了 `ContractError` | 让 `ContractError` 直接冒泡，artifact 被整体拒收 |

---

## 10. 未解决的范围与边界（本轮明确不声称）

1. **自间距搜索完备性**：搜索状态没有路径历史，本轮**不声称**在自间距约束下搜索完备或全局最优。后验检查只能“拒绝”，不能“改出一条更好的路”。
2. **顺序基线**：没有 net ordering 优化、没有 rip-up & reroute、没有 history congestion；交叉/置换案例部分失败是**预期**结果。
3. **warm / prebuilt 图分支**：`graph=` 从不传入，所以“预建图 + 缓存复用”这条公共分支本轮**没有被执行、也没有被认证**；已知的可变别名风险是靠“每网新建图 + 深拷贝输入”隔离的，而不是修好了那个分支。
4. **旧 API `constraints.checks.path_self_spacing()`**：仍然是按段索引豁免的旧语义，v0.2 主流程与两个验收器都不再用它得出裁决（测试只把它作为“旧语义看不见同段自交”的对照）。
5. **几何能力**：障碍仍只支持 AABB；端口附着仍只有六轴方向、m·h 圆弧与共面 Dubins 族（无 LRL/RLR）；有限格点（h=50、m ∈ {1,2}）不能复现任意连续路线，因此比连续最优解更长是**已披露**的离散动作限制。
6. **认证下界 vs 精确距离**：生产侧线段/圆弧间距返回的是**保守下界**，不是精确距离；报告里的“认证下界”一律按下界理解。精确距离由独立实现用有理算术在 line-line 上给出。
7. **数值范围**：`SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9 μm` 量级以下的比较不可信；联合验收器的自有裕度 `VALIDATOR_MARGIN_UM = 1e-9 μm`。极端尺度/病态输入本轮未审计。
8. **无光学模型**：没有损耗、没有串扰、没有 dB。所有数字都是合成几何参数，不是任何论文的材料参数。
9. **停在 16 通道**：64/256/1024 未做。
10. **`tracemalloc` 峰值**（若在别处被引用）是 Python 追踪分配峰值，不是进程 RSS。

---

## 11. 文件与命令索引

| 目的 | 命令 |
|---|---|
| 跑全部多通道案例 | `.venv\Scripts\python.exe examples\run_multichannel_baseline.py` |
| 只跑 16 通道跨墙 | `.venv\Scripts\python.exe examples\run_multichannel_baseline.py --only crosswall_16` |
| 生成整板图 | `.venv\Scripts\python.exe examples\plot_board_layout.py` |
| 独立验收一个 artifact | `$env:PYTHONPATH="src"; .venv\Scripts\python.exe -m native3d.validation.joint_validate artifacts\v020\cases\crosswall_16_channels.json` |
| 对抗式复核 | `.venv\Scripts\python.exe artifacts\v020\probe\v020_final_review.py` |
| 全量测试 | `.venv\Scripts\python.exe -m pytest` |
| 自间距语义自检 | `.venv\Scripts\python.exe artifacts\v020\probe\v020_self_spacing_probe.py` |
| 开工前可行性预研 | `.venv\Scripts\python.exe artifacts\v020\probe\v020_cases_spike.py`（5 个两通道案例）、`v020_geometry_spike.py`（跨墙系列与置换案例） |
