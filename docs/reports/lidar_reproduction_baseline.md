# LiDAR 官方最小 benchmark 复现审计报告

> **报告标题的版本限定（重要）**：本报告复现并审计的对象是 **LiDAR ISPD 2025 (v1.0)** 的公开官方实现。
> 本课题持有的论文 `2505.17239v2.pdf` 是 **LiDAR 2.0: Hierarchical Curvy Waveguide Detailed Routing**。
> **在官方公开仓库中不存在与 LiDAR 2.0 对应的 commit / tag / release**，因此本报告**不是** LiDAR 2.0 的复现。
> 凡涉及 2.0 特性的条目，均按"固定版本中缺失/未确认"处理，**不得**表述为"2.0 已复现"。

- 执行日期：2026-10-06 ~ 2026-10-07（本机时间）
- 工作目录：`C:\Users\lihao\Desktop\Graduation Project`
- 隔离布局：`work/lidar_baseline/{repo,env,runs,provenance}`
- 源固定记录（完整证据）：[`work/lidar_baseline/provenance/00_source_pinning.md`](../../work/lidar_baseline/provenance/00_source_pinning.md)

---

## PART A1 — 获取官方代码并固定版本

### A1.1 固定结果

| 项 | 实测值 |
|---|---|
| 官方入口 | https://github.com/ScopeX-ASU/LiDAR |
| remote origin | `https://github.com/ScopeX-ASU/LiDAR.git` |
| 本机路径 | `work/lidar_baseline/repo` |
| 克隆时间 | 2026-10-06T23:57:45+08:00（后台 job `pwsh-2373`，约 3.4 s） |
| clone 退出码 | 0 |
| **固定 commit SHA（40 位）** | **`4e7004d32b0b80d959553797fb30f0737513de0c`** |
| commit 时间 | 2026-01-13 00:46:15 -0700 |
| commit 作者 / 标题 | Jiaqi Gu / `Update README with ISPD 2025 reference` |
| 分支 | `main`（唯一分支；`remotes/origin/HEAD -> origin/main`） |
| **tags** | **0 个** |
| **releases** | **0 个** |
| 提交总数 | 18 |
| clone 后工作区状态 | `git status --porcelain` 输出为空 → 干净，即算法源码运行前基线 |
| 在线可用性 | 执行时 `GET https://api.github.com/repos/ScopeX-ASU/LiDAR` 返回 HTTP 200 |

**版本固定方式说明**：该仓库**没有 tag、没有 release**，因此不存在"用 tag 固定 2.0"的可能。本轮固定方式为 checkout 明确的 40 位 commit SHA，不记录可变的 `main`。

### A1.2 版本判定：公开仓库 = LiDAR ISPD 2025 (v1.0)，不是 2.0

证据全部来自固定 SHA `4e7004d3` 的实际文件内容：

| # | 证据 | 文件位置 |
|---|---|---|
| 1 | README 标题为 "LiDAR: Automated Curvy Waveguide Detailed Routing for Large-Scale Photonic Integrated Circuits"，并写明 "This repo is the official implementation of \"LiDAR:...\" (https://arxiv.org/abs/2410.01260), ISPD 2025" | `README.md` 第 1、9 行 |
| 2 | README 全文**未出现** "2.0"，**未引用** arXiv 2505.17239，**未提及** hierarchical routing | `README.md` 全文 |
| 3 | `__version__ = "0.0.1"`，文件头日期 2024-06-04 | `src/picroute/version.py:7` |
| 4 | 源码目录无任何 hierarchy 模块（grep 见 A2 第 10 项） | `src/picroute/` |
| 5 | 唯一指向的论文是 2410.01260（LiDAR 1.0）与 ISPD 2025 | `README.md:9` |

**结论**：本固定版本对应的是 **LiDAR 1.0 / ISPD 2025**。论文 2.0 的 hierarchical 特性在本 SHA 中缺失。参见 A2 第 10 项。

### A1.3 README 自身的入口不一致（如实记录，未修改官方文件）

README "Get the LiDAR Source" 一节给出的命令是
`git clone https://github.com/ScopeX-ASU/PICRoute.git`，
与实际仓库名 `ScopeX-ASU/LiDAR` 不一致。这是官方 README 的文档缺陷，已记录，未做任何修改。

### A1.4 license

仓库含 `LICENSE` 文件（`git ls-files` 确认在版本控制内）。README 顶部徽章链接指向该文件。**官方算法源码未做任何修改**：clone 后 `git status --porcelain` 为空，A3 运行前后均以该命令核验。

---

## PART A2 — 源码与论文逐项对照

对照方法：论文侧以本课题持有的 `2505.17239v2.pdf`（LiDAR 2.0）为参照；源码侧全部定位到固定 SHA `4e7004d32b0b80d959553797fb30f0737513de0c` 的文件、函数与行号。
由于固定源码是 1.0，**"论文(2.0) → 源码(1.0)"的差异本身就是本表的结论之一**。

### A2.1 逐项对照总表

| # | 项目 | 源码定位（固定 SHA 4e7004d3） | 实际行为 | 与论文(2.0)一致性 |
|---|---|---|---|---|
| 1 | commit 与版本对应 | 全仓库；`src/picroute/version.py:7` | 仓库为 ISPD 2025 v1.0；版本号 0.0.1；无 tag/release | **不一致/缺口**：无 2.0 对应版本 |
| 2 | dependencies | `requirements.txt`；各文件 import | 声明 7 项；实际 import 见 A2.2 | **声明不完整** |
| 3 | benchmark | `src/picroute/benchmarks/`；`toy_example/toy_example.gp.yml` | YAML 网表；自带最小案例 toy_example（2 nets / 6 instances） | 部分一致 |
| 4 | A* | `routing/astarsearch.py` `AstarSearch.route` (683) | heapdict 开集 + visited 闭集；状态键 (x,y,ori) | 见 A2.3 |
| 5 | orientation state | `astarsearch.py:38,51,147` | **(x, y, orientation)**，orientation ∈ 8 个离散方向 | 一致（且确认**非空间三维**） |
| 6 | bend-aware neighbors | `astarsearch.py:271-340`；`drcmanager.py:361` | 参数化曲线邻居 + 逐候选 DRC | 一致（2D） |
| 7 | congestion | `drgridroute.py:22-24,598-611`；`astarsearch.py:747-786` | group penalty + history penalty | 一致（2D） |
| 8 | net ordering | `drgridroute.py:73-91`；`schematic.py:261-271` | topo/naive 排序 + 组内序 | 一致 |
| 9 | rip-up & reroute | `drgridroute.py:107-165,350-397` | 全局 + 局部 rip-up，history 清零 | 一致；**`ripup_times` 配置实际未生效** |
| 10 | hierarchy | 全 src grep | **完全不存在** | **缺失（2.0 特性）** |
| 11 | loss evaluation | `astarsearch.py:249-254,386-408`；`drgridroute.py:844-932` | 搜索代理 cost 与报告 IL 分开计算 | 见 A2.5 |

### A2.2 第 2 项 — dependencies（声明 vs 实际）

`requirements.txt` 声明（固定 SHA）：

```
gdsfactory==8.26.1
numpy==1.26.4
ryaml==0.4.0
numba==0.60.0
multimethod==1.12
pydantic == 2.10.6
pydantic-core == 2.27.2
```

对 `src/picroute` 全目录 import 做静态汇总，实际被 import 的第三方模块为：

```
bokeh, gdsfactory, matplotlib, multimethod, networkx, numba,
numpy, pydantic, rtree, ryaml, shapely, yaml
```

**未被 requirements.txt 声明但确实被 import 的模块**：`networkx`、`rtree`、`bokeh`、`matplotlib`、`shapely`、`yaml`。
其中 `networkx` 出现在 `drgridroute.py:845`（`evaluation()` 内的函数级 import），而 `evaluation()` 在 `solve()` 中被**无条件调用**（`drgridroute.py:66` 与 `:69`），因此 requirements 不完整会直接导致运行期 `ModuleNotFoundError`。`shapely`/`rtree`/`matplotlib`/`bokeh`/`yaml` 通常随 gdsfactory 传递依赖装入，`networkx` 是否传递装入需以实际环境为准（见 A3）。

另注：`utils/general.py:80` 的 `set_torch_deterministic()` 名字带 torch，**实际只做 `random.seed()`，不依赖 torch**（`main/picroute.py:40` 调用），因此 torch 不是本项目的真实依赖。

README 声明 Prerequisites 为 **Python >= 3.11**；本机实测仅 **Python 3.10.11**（`py -0p` 仅列出 3.10-64，无 conda）。该差异对 A3 的影响见 A3 环境记录。

### A2.3 第 4 项 — A* 细节

| 子项 | 实际实现 | 位置 |
|---|---|---|
| 开集 | `heapdict`，`self.nodepq[node] = node.costF` | `astarsearch.py:237, 703, 796` |
| 取最小 | `current_node, _ = self.nodepq.popitem()` | `:706` |
| 闭集 | 节点布尔标记 `visited`（非独立集合） | `:707`（置位）、`:725`（跳过） |
| 状态键 | `__hash__ = hash(self._pos)`，`_pos` 为完整 `(x, y, ori)` | `:147`、`:38, 51` |
| 节点复用表 | `vAllNodesMap[neighbor_position]`，键含方向 | `:828-838` |
| 累计 cost | `costG = current_node.costG + self.stepG[nb_type]` | `:732` |
| 更新判据 | `bNeedUpdate`：`neighbor.costG > costG` 才更新 | `:1068-1071` |
| f 值 | `f = costG + penalty["propagation_loss"] * costH(neighbor.pos, self.endNode.pos)`，严格模式下再加 history | `:779-786` |
| heuristic 选择 | 由配置决定：无 45° 邻居（或 net 指定 customH 为假）用 `scaledMDist`；启用 45° 用 `customH` | `:239-247` |
| heuristic 实现 | `scaledMDist`：`x+y+resolution`（x,y 均>0）否则 `x+y`；`customH`：octile + `bending_loss/2` | `:865-871`；`routing/utils.py:13-21` |
| 目标判定 | `check_connection`：位置在 access_points **且** `abs(ori1-ori2)==180`；随后 `hard_connection` 物理校验，失败则撤销 visited 继续搜索 | `:709-718`、`:873-879` |
| 失败条件 | 开集耗尽 `while not self.nodepq.empty()` 结束 → `return False`；入口 `findAcsPts()` 失败亦返回 False | `:798`、`:699-700` |
| 搜索预算 | **无 max-expansions 实现**（`:705` 注释 `Need to check the _param.maxExplore` 即为未实现证据） | `:705` |

**目标判定的切向语义**：`check_connection` 要求 `|ori1-ori2| == 180`，即**到达方向与端口朝向相反**，说明源码把端口 orientation 当作**端口外法向**，进入方向需与其反向对齐。这一点对 PART B 的 target tangent 约定有直接参考价值。

**heuristic 与论文的差异（如实记录）**：`customH` 在 `dx≠0 且 dy≠0` 时附加 `bending_loss/2`（默认 50/2=25，且随后又被 `propagation_loss`=1.5 相乘），该附加项并非对所有节点均匀，因而 `customH` **不构成严格可采（admissible）下界**；而 `scaledMDist` 分支（未启用 45° 邻居时）不含 bend 项。论文侧对 45° bend term 的定位见 A2.6 说明。本报告不把二者等价。

### A2.4 第 6 项 — bend-aware neighbors 实际生成规则

邻居表 `self.nextSteps` 以**方向**为键（`astarsearch.py:271-320`），步长由半径与网格推导（`:262-267`）：

```
grid_radius = ceil(bend_radius / grid_resolution)
bend45_1    = ceil(bend_radius * 0.415 / grid_resolution)
bend45_2    = ceil(bend_radius * 0.300 / grid_resolution)
```

| 当前方向 | 候选动作（步长以网格为单位） |
|---|---|
| 0 / 90 / 180 / 270 | `bend_90`（±90°）、`straight_0`；若 `net.enable_45 and config.dr.enable_45_neighbor` 再追加 2 个 `bend_45_1` → 共 **5 邻居** |
| 45 / 135 / 225 / 315 | `straight_45`、2 个 `bend_45_2` → **3 邻居** |

要点与差异（均以源码为准）：

1. **只有轴对齐 4 个方向**在开关打开时获得额外 45° 邻居；对角方向不额外扩到 5 邻居（`:322-340` 仅对键 0/90/180/270 `append`）。
2. 90° 弯的邻居终点偏移为 `(±grid_radius, ±grid_radius)`，与半径 R 的 90° 圆弧弦终点几何自洽（圆心在其中一侧 R 处）。
3. 45° 弯的步长常数已在论文中定位到出处，但**论文公式与代码端点不一致**：
   - 论文 Eq.(4)（PDF **p5**）：`step90,x = step90,y = ⌈r/s⌉`，`step45,x = ⌈(√2−1)·r/s⌉`，`step45,y = ⌈(1−√2/2)·r/s⌉`。
   - 代码：`bend45_1 = ⌈0.415·r/s⌉`、`bend45_2 = ⌈0.3·r/s⌉`。
   - 数值对照：(√2−1)=0.41421 ↔ 0.415（吻合）；(1−√2/2)=0.29289 ↔ 0.3（吻合）。**故这两个"魔法常数"实为 Eq.(4) 的向上取整写法**，并非无出处。
   - **但端点的 x 分量不一致**：代码把 45° 邻居终点取为 `(bend45_1 + bend45_2, bend45_2)`（`:323-324`；`:372-378` 的 `smooth` 控制点同构），即 x ≈ (0.4142+0.2929)·r/s = **0.7071·r/s**；而论文 Eq.(4) 字面给出 `step45,x` ≈ **0.4142·r/s**。代码的 0.7071·r/s 恰为半径 r 的 45° 圆弧真实终点 x 分量（1/√2），其 y 分量与论文一致（0.2929·r/s）。
   - 结论：**代码实现的 45° 端点几何自洽（等于真实 45° 圆弧终点），而论文 Eq.(4) 的 `step45,x` 字面值与代码不符**。此处按"论文与源码存在差异"记录，不替任何一方做解释性补全。
4. 步长代价并非解析式，而是把折线点交给 `smooth(points, radius-1e-3, bend=gf.path.arc)` **实际生成曲线后取 `P.length()`**，再乘 `propagation_loss` 并按类型加 `bending_loss/2` 或 `bending_loss`（`:379-408`）。
5. **候选 DRC 前置**：`findNeighbors` 对每个候选调用 `self._drcmgr.bViolateDRC(...)`，只有不违规（或可插 crossing / 放宽 DRC）才 `addNeighbors`（`:819-863`）；同时检查 `routing_bound`（`:811-818`）。
6. **代码质量观察（如实记录）**：`drcmanager.bViolateDRC` 中 `checkcrossing` / `checkBitmap` 的三个分支（`if self.resolution > 1` / `elif enable_prediction` / `else`）**全部赋值为同一个 `self.checkSingleNode`**（`drcmanager.py:374-380`），即该三路分支是空操作，与"按分辨率或预测开关选择不同检查器"的字面意图不符。

### A2.5 第 11 项 — loss evaluation：搜索代理 cost 与报告 IL 是两套

**（a）搜索期代理 cost（A* 用）**

```
stepG["straight_0"]  = grid_resolution * loss_propagation
stepG["straight_45"] = sqrt(2) * grid_resolution * loss_propagation
stepG["bend_90"]     = ArcLength(radius-1e-3) * loss_propagation + loss_bending
stepG["bend_45_1"]   = ArcLength(...)        * loss_propagation + loss_bending / 2
stepG["bend_45_2"]   = stepG["bend_45_1"]
```
位置：`astarsearch.py:249-254`、`:386-391`、`:406-408`。

叠加项：crossing 候选直接把 `loss_crossing` 加到 g（`:728-730`）；非严格 DRC 下按冲突网数量加 crossing 相关罚（`:734-743`）；组拥塞按 `个数 × loss_congestion`（`:747-776`）；history 仅在严格模式加到 **f**（`:783-786`）。

单位口径：`grid_resolution` 为 µm，`loss_propagation` 无量纲倍率，`loss_bending`/`loss_crossing`/`loss_congestion` 与长度量相加，属**等效长度混合量纲**，源码未声明其物理单位。

**（b）报告期物理 IL 估计（evaluation 用）**

```
propagation_loss = loss_propagation * 1e-4      # 1.5e-4 /um  = 1.5 dB/cm
bending_loss     = 0.01 / 90                    # dB per degree（即 0.01 dB / 90°）
crossing_loss    = il_cross                     # comp_LiDAR.yml = 0.5
IL = wirelength * propagation_loss
   + bending * bending_loss
   + crossing_num * crossing_loss
   (+ device_loss, eval=="comp" 时)
```
位置：`drgridroute.py:847-849`、`:863-867`、`:906-910`；另有简化版 `eval_net`（`:360-364`，只用 prop 与 cross，**不含 bending 项**）。

**关键结论**：搜索代理 cost 与实际报告 IL **不是同一个函数**（前者用 length×1.5 与 50/5000 级常数，后者用 1.5e-4/µm 与 0.01dB/90° 与 il_cross），两者只是"同源参数的不同标度"。`loss_bending`/`loss_crossing`（50 / 5000）**不是 dB**。

**配置完整性缺陷**：`DrGridRoute.__init__` 第 29 行读取 `self._config.il_cross`，但 `default_config.yml` **没有** `il_cross` 键（仅 `comp_LiDAR.yml:28` 有 `il_cross: 0.5`）。因此用 `default_config.yml` 运行会在构造 router 时抛 `AttributeError`。这是 A3 选择 `comp_LiDAR.yml` 的客观理由之一。

### A2.6 第 7/8/9 项 — congestion、net ordering、rip-up & reroute

**congestion（第 7 项）**

| 机制 | 实现 | 位置 |
|---|---|---|
| 占用结构 | `historyMap = np.zeros((bitMap_width, bitMap_height, 8), dtype=int32)`，第 3 维按 8 个方向分桶 | `drgridroute.py:22-24` |
| 方向索引 | `his_index = {0:0, 45:1, ..., 315:7}` | `astarsearch.py:342` |
| history 累加 | 路径每点的 `historyMap[x][y][ori_index] += historyCost`，**跳过端口网格** | `drgridroute.py:598-611` |
| history 计入 cost | 仅 `_bStrictDRC` 时加到 **f** | `astarsearch.py:783-786` |
| history 清零 | `clearHistoryMap()` 全零重置，在 `_ripup_times == 2` 时触发 | `drgridroute.py:588-591`、`:394-395` |
| group penalty | 仅当 `self.groups` 非空；`checkSpacing(neighbor, unrouted, groups)` 返回计数，`costG += count * loss_congestion` | `astarsearch.py:747-776` |
| 邻域口径 | `checkSpacing` 只沿**垂直于当前方向**的两侧逐格探查 `check_region` 步 | `drcmanager.py:1155-1199+` |
| 只在直行加 | 只有 `straight_0` / `straight_45` 计算拥塞，弯段不计 | `astarsearch.py:758-774` |
| repel 模式 | `grid_resolution < 20` 时 `repel=True`；此时探查长度**硬编码为 5/4**，且要求 `(ori + end_port.orientation) % 180 != 0` 且 `failed_count > 1` | `astarsearch.py:176-177`、`:752-763` |
| 探查长度 | `unrouted = max(2, min(10, Dr.unrouted))` | `astarsearch.py:179` |

**net ordering（第 8 项）**

- `solve()` 首先调用 `process_net_order()`（`drgridroute.py:34`）。
- `net_order == "topo"` 时：按 `self._cirdb.topology_orders` 的层遍历，令 `net.comp_dist = level`，并用 `LineString` 统计网间拓扑交叉数（`:73-91`）。
- 网间优先级由 `Nets.__lt__` 决定：`(comp_dist, routing_order)` 字典序（`schematic.py:267-271`）；`__eq__` 同（`:261-265`）。网被放入 `AdvancedPriorityQueue`（`drgridroute.py:43,49`）。
- **组内排序**：`config.dr.group` 为真时，对当前网所属每个 group 建立 `local_PQ`，按 group 内 `nets` 的出现次序赋 `net.routing_order = i`，并在组内先路由完再返回外层（`:119-149`）。
- **端口访问区**：`config.dr.group` 为真时调用 `drcmgr.initPorts()`，否则 `bitMap.initPorts(dbNets)`（`:37-40`）；端口扩展/预留实现于 `drcmanager.py:73 (initPorts)`、`:87 (spread_ports)`、`:140 (spread)`。访问点由 `AstarSearch.findAcsPts` 计算（`astarsearch.py:438+`），并用于目标判定（`:876`）。

**rip-up & reroute（第 9 项）**

| 子项 | 实现 | 位置 |
|---|---|---|
| 触发条件 | 单网严格 DRC 失败 → 放宽 DRC 重试；**两次都失败**则从 `dbNets` 弹出、移入 `abNets` | `drgridroute.py:151-158`（非组模式）、`:140-149`（组模式） |
| 冲突登记 | `mark_conflict_nets(violatedNets, cur_net)` 把当前网与所有违反网加入 `global_failedNet` | `:593-596` |
| 全局 rip-up | `ripupfailedNets()`：删除失败网在 bitmap 的占用、`net.clearNet()`、`_ripup_times += 1`、清空 `global_failedNet` | `:378-397` |
| 局部 rip-up | `ripuplocalnets(cur_net_crossing)`：只撤销 crossing 冲突网 | `:350-358`；调用点 `:230`、`:298` |
| 轮数上限 | `runNRR(net_pq, maxIteration)` 外层 `for iter in range(maxIteration)`，即由 `config.dr.maxIteration` 决定 | `:107-165` |
| 停止条件 | 全部网路由完成 `net_pq.empty()` → True；最后一轮仍有 `global_failedNet` → False | `:159-164` |
| history 更新 | 成功注册时 `registar_net` → `updateHistoryMap`；并 `drcmgr.updateBitmap` | `:366-377` |
| **配置失效（重要）** | `self._ripup_times = 0  # config.dr.ripup_times` —— 该行把配置读取**注释掉了**；全 src 中 `ripup_times` 除两个 YAML 外仅出现在此注释。rip-up 次数实际由 `_ripup_times == 2` 的硬编码魔法数与 `maxIteration` 共同决定 | `:26`、`:390-395` |

### A2.7 第 10 项 — hierarchy：**本固定版本中完全不存在**

对 `src/` 全目录 grep `hierarch|Hierarch` 仅得 **4 处命中，且全部与路由层次无关**：

| 位置 | 内容 | 是否层次路由证据 |
|---|---|---|
| `config/config.py:30` | `## try hierarchical access`（dict 点号取值注释） | 否 |
| `drc/write_drc.py:234` | "mode: tiled, default or deep (hierarchical)" KLayout API 文档 | 否 |
| `drc/write_drc.py:256-257` | KLayout hierarchical mode 文档 | 否 |

目录结构（`benchmarks/ config/ database/ drc/ main/ queue/ routing/ scripts/ utils/`）中**没有**模块树、bottom-up 路由、模块复用或复用失败恢复的任何实现。按任务的判定要求（"不可仅凭 YAML 支持嵌套认定已实现"），结论为：

> **论文 2.0 的 hierarchy / bottom-up routing / reuse 特性在本固定 SHA 中缺失（未实现），不是"已复现"。**

### A2.8 第 3 项 — benchmark

| 子项 | 实测 |
|---|---|
| 输入格式 | 单个 YAML 文件（"gp" 格式）：`instances`（器件 + placement）、`schematic_placements`、`nets`（每网两个端点 `'instance,port'`）、`library`（宏尺寸与 pin 偏移/朝向）、`settings.die_area`、`constraints` |
| 生成脚本 | `benchmarks/clements.py`、`benchmarks/MMIports.py`、`benchmarks/mrr_weight_bank.py`、`benchmarks/func.py`、`benchmarks/schematic.py` |
| 自带案例 | 7 组 YAML：`clements_8x8/16x16`、`multiportmmi_8x8/16x16/32x32`、`mrr_weight_bank_4x4/8x8/16x16`，以及 **`toy_example/toy_example.gp.yml`** |
| 入口 | `main/picroute.py --benchmark <yml> --config <yml>`；批处理 `scripts/lidar/run_route.py`（其 `tasks` 列表中**未被注释的只有 `multiportmmi_8x8` + `comp_LiDAR.yml`**，其余全部被注释） |
| 最小案例 | `toy_example`：`num_instances: 6`、`num_nets: 2`、`num_ports: 0`、`die_area: [[0,0],[200,200]]`、`wg_radius: 5`，含 2 个矩形障碍（`obstacle1` 10×60、`obstacle2` 10×10） |

**最小案例选择理由**：仓库**自带**名为 `toy_example` 的案例，且其 net 数（2）与器件数（6）均为全部自带案例中最小，因此无需"从大案例中自行挑最小者"，直接用官方自带的 toy_example 即满足"仓库自带的最小有效 benchmark"。

### A2.9 A3 之前的源码状态核验

```
git -C work/lidar_baseline/repo status --porcelain   ->   （空）
git -C work/lidar_baseline/repo rev-parse HEAD       ->   4e7004d32b0b80d959553797fb30f0737513de0c
```

即截至 A3 运行前，官方源码**零修改**。

---

## PART A3 — 运行官方最小 benchmark

### A3.1 环境（真实构建过程与偏差记录）

本机 **只有 Python 3.10.11**（`py -0p` 仅列出 3.10-64；无 conda；无 3.11/3.12），低于 README 声明的 "Python >=3.11"。
已核实 PyPI 元数据：`gdsfactory 8.26.1` 的 `requires_python = <3.13,>=3.10`，**故 3.10 实际可用**；README 的 >=3.11 比包本身要求更严。

隔离环境：`work/lidar_baseline/env`（独立 venv；**未改动** `OpticalWaveguideRouter2D/3D` 的环境）。

构建过程与真实失败（如实记录，未隐藏）：

| 步骤 | 结果 |
|---|---|
| `python -m venv env` | 成功 |
| `pip install --upgrade pip` | **挂死**：进程持续占满约 1 核（累计 578 s CPU）且无输出，超时终止 |
| `pip install -r requirements.txt`（默认解析器） | **挂死**：同样约 1 核空转约 20 min，日志停在 `Downloading gdsfactory-8.26.1...` |
| 改 `--use-deprecated=legacy-resolver` | **仍挂死**，同一位置 |
| 改用 `uv pip install` | **失败**：`rectangle-packer` 需现场编译，本机缺 MSVC C++ Build Tools |
| 定位：PyPI 上 `rectangle-packer` **2.1.0 无 cp310 Windows 轮子**，而 **2.0.5 有** `cp310-cp310-win_amd64` | — |
| 最终 `uv pip install "rectangle-packer==2.0.5" -r requirements.txt` | **成功（exit 0）** |

**受控偏差（不涉及算法）**：解析器默认选中的 `rectangle-packer==2.1.0` 在 Windows/py3.10 无预编译轮子，显式固定为 **`rectangle-packer==2.0.5`**（该版本提供 cp310 win_amd64 轮子）。它是 `gdsfactory`→`kfactory` 的几何打包依赖，**不参与 LiDAR 路由算法**。官方算法源码未做任何改动。

实际安装版本（`runs/pip_freeze.log` 摘录）：

    gdsfactory 8.26.1   numpy 1.26.4        numba 0.60.0      multimethod 1.12
    pydantic 2.10.6     pydantic-core 2.27.2  ryaml 0.4.0     pyyaml 6.0.3
    networkx 3.4.2      shapely 2.1.2       scipy 1.15.3      matplotlib 3.10.9
    kfactory 0.21.11    klayout 0.30.12     rectangle-packer 2.0.5

与声明版本对照：`requirements.txt` 的 7 项**全部按声明版本装入**（gdsfactory / numpy / numba / multimethod / pydantic / pydantic-core / ryaml），一致。

**声明外依赖的实际影响（A2.2 的落地验证）**：

| 模块 | 是否声明 | 是否装入 | 本次运行是否需要 |
|---|---|---|---|
| `networkx` | 否 | 3.4.2（随 kfactory 传递装入） | **需要**（`evaluation()` 的 `import networkx`）；能跑通属传递依赖"侥幸"装入 |
| `shapely` / `pyyaml` / `matplotlib` | 否 | 已装入 | 是 |
| `rtree` | 否 | **未装入** | **本次未触发** |
| `bokeh` | 否 | **未装入** | **本次未触发** |
| `gdstk` | 否 | **未装入** | **本次未触发** |

即：`requirements.txt` 不完整这一点在本最小案例下**未导致失败**，但 `networkx` 未被声明却为必需，属实质性依赖声明缺陷。

机器：CPU=13th Gen Intel(R) Core(TM) i9-13900HX; CORES=24 LOGICAL=32; RAM=15.7 GB; OS=Microsoft Windows 11 家庭版 build 26200。**未对照论文硬件宣称任何数值等价**。

### A3.2 最小案例选择

仓库**自带**名为 `toy_example` 的官方案例 `src/picroute/benchmarks/toy_example/toy_example.gp.yml`：
`num_instances=6`、`num_nets=2`、`num_ports=0`、`die_area=[[0,0],[200,200]]`、`wg_radius=5`，含 2 个矩形障碍（10×60 与 10×10）。
其 net 数与器件数均为全部自带案例中最小，故直接采用；**未编写任何替代 router**。

配置选 `config/comp_LiDAR.yml`，理由：它是官方批处理脚本 `scripts/lidar/run_route.py` 中**唯一未被注释**的组合（`:68`，`multiportmmi_8x8` + `comp_LiDAR.yml`）所用配置；且 `default_config.yml` 缺 `il_cross` 键会被 `drgridroute.py:29` 读取而报错（见 A2.5）。

### A3.3 逐字命令与运行记录

工作目录：`work/lidar_baseline/repo/src/picroute`

    ..\..\..\env\Scripts\python.exe main\picroute.py ^
      --benchmark benchmarks/toy_example/toy_example.gp.yml ^
      --config config/comp_LiDAR.yml ^
      "--run.output_layout_gds_path=<ABS>\work\lidar_baseline\runs\toy_comp\toy_example_comp.gds"

| 项 | 运行 1（`runs/toy_comp`） | 复跑（`runs/toy_comp_rerun`） |
|---|---|---|
| 退出码 | **0** | **0** |
| 墙钟（解释器启动→GDS 导出） | **3.858 s** | **3.252 s** |
| benchmark 读取 | 1.06 s | 0.89 s |
| bitmap 初始化 | 0.00 s | 0.00 s |
| **detailed routing** | **0.42 s** | **0.29 s** |
| 迭代 | Iteration 1/10 → `DrGridRoute succeed` | 同 |
| 输出 GDS | 34,476 B | 34,476 B |
| **GDS SHA256** | `7612BE0927FF930DEA3E07950BBBD76422F06E26103F38CDF9CE9236C93E6689` | **完全相同** |
| 完整 stdout | `runs/toy_comp/stdout.log`（53 行） | `runs/toy_comp_rerun/stdout.log` |

计时边界：`Stopwatch` 包住整个 `python` 进程，**包含**解释器启动、benchmark 解析、bitmap 初始化、布局生成与 GDS 导出；故 3.8 s / 3.3 s 是端到端值，其中算法搜索本身仅占 0.42 s / 0.29 s。

stderr 的 `kfactory.kcell:show - Could not connect to klive server` 是 klive 可视化服务缺失的**预期告警**（README 标注 klayout/klive 为 Optional），不影响路由。

### A3.4 成功/违规口径核查（不把退出码 0 当作成功）

| 判据 | 实测 |
|---|---|
| 进程退出码 | 0 |
| `DrGridRoute` 是否报成功 | `INFO: DrGridRoute succeed` |
| 两条网是否真的被路由 | 是：`route net: n_1`、`route net: n_0` 均 **strictDRC: True** |
| **是否使用 DRC 放宽回退** | **否**（关键：若走 `strictDRC=False` 分支，即使"成功"也含违规路线） |
| DRC 违规数 | **0**（两网均 `DRV: 0`，`DRV_path: 0`） |
| 交叉数 | 0（两网 crossing 均为 0） |
| 异常网 `abNets` | 无 |
| 拓扑交叉 CR | 0 |

结论：本最小案例**路由成功且无 DRC 违规**，且**未触发 DRC 放宽**——这使本次结果可以作为忠实 baseline 的证据，而非放宽约束后的假成功。

两网实测指标（evaluation 的物理 IL 估计，**不是**搜索代理 cost）：

| net | 线长 WL (µm) | 累计转角 (°) | 交叉数 | IL 估计 |
|---|---|---|---|---|
| n_0 | 129.119 | 270.0 | 0 | 0.24936785 |
| n_1 | 46.667 | 90.0 | 0 | 0.21700005 |

`criticalPath: ['n_0'], WL: 129.12, crossing: 0, Max Insertion loss: 0.249, DRV: 0`

### A3.5 源码零修改与产物分离

- `git -C work/lidar_baseline/repo status --porcelain` → **无任何被跟踪文件被修改**（无 ` M ` 行）。
- `git rev-parse HEAD` → 仍为 `4e7004d32b0b80d959553797fb30f0737513de0c`。
- 唯一新增项是一个**未跟踪的官方程序生成物**：`src/picroute/benchmarks/toy_example/toy_example.gp.layout.yml`（1,408 B，运行时生成）。
  这是官方程序自身写出的布局副本，**属程序输出而非源码修改**，按任务要求与源码状态分开记录。

**未运行验证的部分**：本轮只跑了 1 个最小案例（2 nets）。A2 表中 crossing 插入、history 代价生效、rip-up & reroute、group 拥塞罚、port access 预留等机制，在本次案例中**未被动态触发**（crossing=0、CR=0、无 rip-up、1 次迭代即收敛），因此**只能记为"源码已审计但未动态验证"**，不能因为程序跑通就声称这些机制已复现。

---

## PART A4 — 官方算法行为记录

| 字段 | 必须说明 | 实测/源码结论 |
|---|---|---|
| **input** | 真实文件、网表/器件/端口/障碍与配置 | `benchmarks/toy_example/toy_example.gp.yml`（YAML 网表，version 1.0）：6 instances、2 nets（n_0 = mrr_array_0_mrr_0.o2 → fanout_yb_0_0.o1；n_1 = fanout_yb_0_1.o2 → fanout_yb_0_2.o1）、0 ports、die 200×200 µm、wg_radius 5 µm、2 个矩形障碍。配置 `config/comp_LiDAR.yml`：maxIteration 10、net_order topo、group True、enable_45_neighbor True、historyCost 1000、ripup_times 10（**未生效**）、grid_resolution 2、bend_radius 5、loss 1.5/50/0/500、il_cross 0.5、eval comp |
| **state** | 实际状态字段、方向集合、状态键与合并规则 | `(x, y, orientation)`，orientation ∈ {0,45,90,135,180,225,270,315} 共 8 个离散方向（`astarsearch.py:271-320`、`drgridroute.py:25`）。状态键即完整三元组：`__hash__ = hash(self._pos)`（`:147`），节点经 `vAllNodesMap[(x,y,ori)]` 合并（`:828-838`）。**不存在任何 z / 层 / 高度字段** |
| **action** | 候选动作、曲线几何、半径与网格关系 | `straight_0`、`straight_45`、`bend_90`、`bend_45_1`、`bend_45_2`（另有 crossing_0/45 分支）。轴对齐方向 5 邻居（开启 45° 时）、对角方向 3 邻居。步长 `grid_radius=⌈r/s⌉`、`bend45_1=⌈0.415r/s⌉`、`bend45_2=⌈0.3r/s⌉`（`:262-267`）。曲线几何由 `smooth(points, radius−1e-3, bend=gf.path.arc)` 实际生成后取弧长（`:379-408`）。**全部动作位于 z=0 平面内** |
| **cost** | 累计项、拥塞/history 项、单位与权重 | 边代价 `stepG`：直行 = 网格长×1.5；`bend_90` = 弧长×1.5 + 50；`bend_45_*` = 弧长×1.5 + 25（`:249-254, 386-408`）。叠加：crossing 候选直接加 5000（`:728-730`）；组拥塞 `count × 500`（`:747-776`）；history 计入 **f**（`:783-786`）。量纲为"长度×倍率 + 等效长度常数"的混合量，**不是 dB** |
| **heuristic** | 实际函数与源码位置；与论文是否一致 | `scaledMDist`（`:865-871`，默认 / 无 45° 时）或 `customH`（`routing/utils.py:13-21`，开启 45° 时）。**`customH` 与论文 Eq.(6)（PDF p6）形式一致**：h = dmax − dmin + √2·dmin + α·IL_bd,45，代码中 α·IL_bd,45 即 `bending_loss/2`。但该 bend 附加项对节点不均匀，**不构成严格 admissible 下界**（见 A2.3） |
| **constraints** | 端口、半径、spacing、crossing、器件障碍检查 | 候选在 `findNeighbors` 中经 `drcmgr.bViolateDRC` 检查后才入 open 集（`:819-825`）；含 `routing_bound` 越界剔除（`:811-818`）、障碍/端口命中、90° 交叉插入（受 crossing_budget 限制，`drcmanager.py:397-400`）、`checkSpacing` 侧向拥塞探查（`drcmanager.py:1155+`）、`crossing_check`（同宽/同波长/同材料且直线长度 > half_size，`schematic.py:273-281`）、终点 `hard_connection` 物理校验（`astarsearch.py:712`）。**半径**靠 `⌈r/s⌉` 的格点步长保证，不是连续半径求解 |
| **output** | 真实路线/布局/统计文件及成功/违规口径 | `runs/toy_comp/toy_example_comp.gds`（34,476 B，GDSII 头 `0,6,0,2`，SHA256 `7612BE09…E6689`）；stdout 全文 `runs/toy_comp/stdout.log`；另有程序生成的 `toy_example.gp.layout.yml`。成功口径 = 退出码 0 **且** 两网 strictDRC 路由 **且** DRV=0，本次三者全部满足 |
| **runtime** | 真实测量值、计时边界、机器与环境 | 端到端 3.858 s（复跑 3.252 s），其中 detailed routing 0.42 s / 0.29 s；计时含解释器启动、benchmark 解析、布局生成与 GDS 导出。环境：Windows 11 + Python 3.10.11 + 上述 venv。**未与论文机器/数值做任何等价宣称** |

---

## PART A 结论清单

### 一、已忠实复现（官方源码已实际运行）

- 固定 commit `4e7004d3` 的**最小自带案例 toy_example 端到端跑通**：退出码 0、两网 strictDRC 路由成功、DRV=0、未使用 DRC 放宽、输出 GDS 两次运行 SHA256 完全一致。
- 输入 YAML 网表解析、bitmap 构建、topo 网序、A* 主循环、evaluation 物理 IL 估计这条主链路**在本次运行中真实执行**。

### 二、源码已审计但本轮未动态触发

crossing 插入与 crossing_budget、history 代价实际生效、group 拥塞罚、rip-up & reroute（全局/局部）、组内排序、port access region 预留、45° 邻居的额外两邻居路径、DRC 放宽回退分支。
本次 2-net 案例 crossing=0、CR=0、无 rip-up、1 次迭代即收敛，**不能据此声称这些机制已被动态验证**。

### 三、论文 2.0 特性在固定版本中缺失/未确认

| 论文 2.0 特性（PDF 位置） | 固定版本状态 |
|---|---|
| Hierarchical netlist tree + bottom-up 路由复用（p7 Fig.11、p8、p13） | **缺失**：src 全目录无层次实现（A2.7） |
| Offset neighbor（p8 "Offset Neighbor"） | **代码中被禁用**：`astarsearch.py:344-370` 整段被 `""" disable rightnow """` 注释掉 |
| Crossing-space penalty g_cr(n)=λ_c·#grids(w_cr) Eq.(7)（p7、p8） | **未在固定源码中定位到对应的独立 cost 项**；代码有"预测式交叉插入"（`drcmanager.py:566-570, 619-623, 683-687`），但其语义与 Eq.(7) 的代价项不同 |
| 论文 p13 自述 2.0 四项新增：hierarchical reuse / intra-group ordering / offset-neighbor / crossing-space preservation | 仅 intra-group ordering 存在于固定版本；其余三项分别为缺失、被注释、未定位 |

### 四、与论文存在差异（数值/实现）

| 项 | 论文 | 固定源码 | 判定 |
|---|---|---|---|
| 90° 弯损耗（物理 IL） | Table V（**p11**）：**0.005 dB** | `bending_loss = 0.01/90` → **0.01 dB / 90°** | **相差 2 倍** |
| 交叉损耗（物理 IL） | Table V：**0.52 dB** | `comp_LiDAR.yml: il_cross: 0.5` | 不一致（0.5 vs 0.52） |
| 传播损耗 | Table V：1.5 dB/cm | prop=1.5，×1e-4/µm → 1.5 dB/cm | **一致** |
| 搜索权重基准 | p10：base propagation/bending/crossing = **1.5 / 50 / 5000** | `default_config.yml` 正是 1.5/50/5000（一致）；但**实际使用的 `comp_LiDAR.yml` 是 1.5/50/`0`** | comp 配置的 crossing=0 既非论文基准 5000，也非 p10 所述 PTC 配置(= bending 50) |
| GCP 系数 | p10：**500**，对应约 200 µm 预留间距 | `loss_congestion: 500` | **一致** |
| 45° 邻居端点 x | Eq.(4) p5：⌈(√2−1)·r/s⌉ ≈ 0.4142r/s | 代码取 `bend45_1+bend45_2` ≈ **0.7071r/s** | **不一致**（代码值等于真实 45° 圆弧终点 x） |
| 45° 邻居端点 y | Eq.(4) p5：⌈(1−√2/2)·r/s⌉ ≈ 0.2929r/s | `bend45_2` ≈ 0.3r/s | 一致 |
| `ripup_times` 配置 | 语义为最大 rip-up 次数 | 读取被注释掉（`drgridroute.py:26`），实际由硬编码 `==2` 与 maxIteration 决定 | **配置未生效** |
| `customH` 可采性 | Eq.(6) 为启发式 | 含非均匀 bend 附加项，非严格下界 | 差异 / 待论证 |
| `default_config.yml` 完整性 | — | 缺 `il_cross` 键，单独使用会在 `drgridroute.py:29` 报 AttributeError | **配置缺陷** |

### 五、未运行验证 / 阻塞情况

- **无阻塞**：A1–A4 全部完成，官方最小 benchmark 已真实跑通。
- **未运行验证**：仅 1 个最小案例（2 nets），远小于论文 p9–p11 的 benchmark 规模（Benes_32×32、TeMPO_16×16、GWOR_32×32、Clements_16×16_C、ADEPT_16×16_C 等）；**论文 Table V 与 p10–p11 的任何数值结果均未复现，也未做对比**。
- 依赖声明缺陷（`networkx` 等未声明）见 A3.1；`rtree`/`bokeh`/`gdstk` 未装入但本次未触发。

