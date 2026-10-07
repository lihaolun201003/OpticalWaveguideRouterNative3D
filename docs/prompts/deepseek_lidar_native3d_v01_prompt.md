# DeepSeek 执行提示词：LiDAR 原算法复现审计与 Native3D v0.1

请直接执行下面的科研复现与开发任务。不要只给计划、摘要、伪代码或声称“可以实现”；必须交付真实文件、运行记录和测试结果。若你的环境不能访问本机目录、网络或执行命令，先如实说明具体能力限制，不得编造下载、commit、运行时间或测试通过结果。

## 0. 研究目标与任务边界

最终研究目标已经固定：

**1024-channel native 3D optical waveguide router**

这里的 native 3D 指：新路径从搜索开始就在 `(x, y, z)` 空间产生，联合选择三维位置、三维方向、弯曲与避让。

**绝对禁止：先生成二维路线，再给二维路线分配层高或进行 z 方向抬升/修复。**

本轮只做两件事：

1. PART A：固定并运行官方 LiDAR 最小基准，审计它实际实现的方法。
2. PART B：在独立新项目中实现 Native3D v0.1，证明单条波导可直接在三维空间搜索合法路径。

本轮不做1024通道、不做16通道联合路由、不加入ML、BPM、未经验证的光学损耗或串扰模型，不追求GUI/GDS。PART A为运行官方程序而产生的官方GDS输出可以保留；不得因此为PART B开发GDS或GUI。

## 1. 工作环境、依据与保护范围

工作根目录：

```text
C:\Users\lihao\Desktop\Graduation Project
```

必须先阅读的已有审计报告：

```text
C:\Users\lihao\Desktop\Graduation Project\docs\reports\3d_waveguide_literature_reproduction_audit.md
```

LiDAR论文原文：

```text
C:\Users\lihao\Desktop\Graduation Project\上海交通大学\文献\论文\2505.17239v2.pdf
```

新项目必须建立在：

```text
C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D
```

**不要修改旧 `OpticalWaveguideRouter3D`，不要修改旧 `OpticalWaveguideRouter2D`，不要调用旧二维Router或旧3D elevation repair作为新算法的前置步骤/核心。** 不改写已有文献审计报告。

先检查工作区适用的 `AGENTS.md` 等约束、已有文件与Git状态。若目标新目录已经存在，先检查并保留用户已有内容，不删除、不强制覆盖、不重置。依赖环境与官方代码放在独立目录，不改变旧项目环境。

报告内页码采用PDF物理页，从1开始；论文结论注明页码/公式/图表，源码结论注明固定commit下的文件、函数及行号。自行设计的三维方法标为“本课题扩展”，不得写成LiDAR论文原算法。

## PART A — LiDAR 原算法复现审计

### A1. 获取官方代码并固定版本

已有审计报告核实的作者官方仓库入口：

```text
https://github.com/ScopeX-ASU/LiDAR
```

此URL是核查起点，不是已固定的版本。你必须在执行时验证官方来源、重定向、branches/tags/releases和仓库当前内容，不能虚构commit。

建议隔离布局：

```text
work/lidar_baseline/repo/
work/lidar_baseline/env/
work/lidar_baseline/runs/
work/lidar_baseline/provenance/
```

要求：

- 下载官方仓库，记录remote URL、下载时间、branch/tag和完整40位commit SHA；checkout到明确commit，禁止仅记录可变的`main`。
- 保存官方README、依赖声明、实际环境版本和执行命令；记录license。
- 检查实际代码属于LiDAR ISPD 2025原版，还是包含论文2.0扩展；逐项给证据。
- 已有审计指出：公开README曾标原版ISPD实现，完整2.0特性与benchmark尚未对应到明确commit。必须重新核实，**不能把“仓库存在”写成“2.0已全部复现”**。
- 若找不到2.0对应版本，仍可运行确认过的官方版本，但报告标题和结论必须说明版本缺口。
- 不修改官方算法源码，不删除DRC，不替换heuristic，不为跑通而关闭约束。需要环境修复时仅做有记录的依赖/运行配置调整；任何改变算法的方案只能记录建议，不能伪装成忠实baseline。
- 记录运行前后的源码差异/状态，证明没有算法修改。官方生成的输出、缓存或日志与源码修改分开记录。

### A2. 源码与论文逐项对照

必须核查下面项目，报告用表格列出“论文定位→固定commit源码定位→实际行为→是否一致/缺失”：

1. commit与版本对应关系。
2. dependencies：Python、GDSFactory、KLayout及其它实际需要的库；声明版本与实际安装版本。
3. benchmark：输入格式、官方最小可运行案例、生成脚本、配置、真实nets/components数量。
4. A*：open/closed集合、状态键、累计cost、heuristic、目标判定与失败条件。
5. orientation state：确认是`(x,y,orientation)`，不得把它的“三变量状态”称为空间三维。
6. bend-aware neighbors：直行、45°/90°候选、半径和网格如何决定终点、候选DRC。
7. congestion：group penalty、history penalty、占用结构、权重及累计方式。
8. net ordering：组间/组内排序、端口访问区和失败后的次序更新。
9. rip-up & reroute：触发条件、撤销范围、history更新、轮数上限与停止条件。
10. hierarchy：是否真的有模块树、bottom-up路由、复用和复用失败恢复；不可仅凭YAML支持嵌套认定已实现。
11. loss evaluation：搜索代理cost与报告中的物理IL估计分别如何计算；长度、转角、crossing与器件项及其单位。

论文对照重点：P2 p3 Eq.(1)–(2)、p5 Fig.5/Eq.(4)、p6 Eq.(5)–(6)及DRC、p7重布线、p7–9层次/排序、p9–10 benchmark配置、p11 Table V。定位须再次以原文和固定源码核实，不盲目照抄审计结论。

### A3. 运行官方最小benchmark

- 先依据官方README/脚本确定真实可运行入口，不编造命令。
- 优先运行仓库自带的最小有效benchmark；若没有单独命名的minimal case，从官方案例中选最小者并解释选择。
- 不以自行写的toy router代替官方运行；用官方参数生成的小案例时，明确生成命令及其来源。
- 记录完整命令、工作目录、环境、输入文件/配置、标准输出、错误、退出码、输出文件、runtime和可获得的搜索/DRC统计。
- runtime使用真实计时方法，说明是否包括启动、布局生成和文件导出；不要对照论文硬件直接宣称数值相同。
- 核查输出连通与约束结果；程序退出码0不自动等于路由成功。
- 如果失败，保留失败日志与原因。不得删去失败、修改算法后写“官方原算法重现成功”。在不改算法的前提下尝试合理的环境修复。
- Windows不可运行而已有WSL/兼容环境可用时，可以尝试并说明真实执行环境；不得声称运行在未实际使用的平台。
- PART A遇到不可解决的访问/平台/依赖问题时，如实记录“阻塞/未重现”，继续独立完成PART B，不编造成功。

### A4. 官方算法行为记录

至少记录这些字段：

| 字段 | 必须说明 |
|---|---|
| input | 真实文件、网表/器件/端口/障碍与配置 |
| state | 实际状态字段、方向集合、状态键与合并规则 |
| action | 候选动作、曲线几何、半径与网格关系 |
| cost | 累计项、拥塞/history项、单位与权重 |
| heuristic | 实际函数与源码位置；与论文是否一致 |
| constraints | 端口、半径、spacing、crossing、器件障碍检查 |
| output | 真实路线/布局/统计文件及成功/违规口径 |
| runtime | 真实测量值、计时边界、机器与环境 |

最终输出：

```text
docs/reports/lidar_reproduction_baseline.md
```

报告必须包含固定SHA、版本对应证据、环境、复跑命令、最小benchmark输入输出、成功或失败日志索引、源码映射表，以及“已忠实复现/存在差异/未运行验证”的清单。

## PART B — Native3D v0.1

### B0. 最小实现原则

仅实现单条net的最小可验证系统。可以采用Python和pytest及必要的数值库；语言/目录细节可以合理选择，但保持模块简单、接口明确、命令可复跑。

几何统一单位（建议µm），数值容差、direction表示、边界接触规则和spacing口径写入配置。测试数值属于合成几何参数，不冒充论文材料参数。

输入至少包含：source pose、target pose、三维chip volume、三维障碍、Rmin、dmin、必要的固定已有波导中心线、运动参数和搜索预算。

输出至少包含：原始搜索得到的连续`Path3D`、分段参数、搜索轨迹/父节点、成功或失败状态、实际长度、累计拥塞cost、约束验证结果及runtime。

建议模块：

```text
OpticalWaveguideRouterNative3D/
  README.md
  pyproject.toml
  src/native3d/
    geometry/
    search/
    constraints/
    validation/
    io/
  tests/
  examples/
  artifacts/
```

这是建议结构；不得为了目录完整创建大量无用模块。

### Phase 1 — 3D geometry

实现：

- `Pose3D`：三维位置与真正三维单位切向；若动作需要局部坐标系/roll，须显式存储或用明确的确定规则构造。
- `LineSegment3D`：任意空间直线段、解析长度、位置和切向求值。
- `CircularArc3D`：任意空间平面中的圆弧，保存圆心/局部正交基/半径/转角等足够重建的参数。
- `Path3D`：有序分段集合，累计长度，连接位置与切向检查。

不能只实现XY平面圆弧然后附加layer标签。空间圆弧使用三维向量定义，长度为`R × |theta|`，theta使用rad。

检查几何连接至少C1连续：位置连续、切向连续。曲率突变与非C1尖角分别记录；不得把折线拐点当成合法有限半径波导。退化/零向量/非法半径输入有明确行为。

### Phase 2 — native 3D state

状态必须包含：

```text
(x, y, z, direction)
```

`direction`必须能表示真正三维方向，例如归一化三维向量或方位角+俯仰角；不允许只存二维orientation+layer。

如果合法动作依赖局部frame、上一primitive或其它历史，必须把这些影响未来可行性的量纳入状态或说明其等价规则。

若状态离散/量化：明确位置与方向分辨率、去重键、同格不同真实pose处理方法。不能只按`(x,y,z)`关闭节点并丢掉方向；不得把圆弧终点任意snap到网格导致半径和切向失真。有限动作/量化图的最优性不等于连续空间全局最优或完备。

### Phase 3 — 3D motion primitives

至少实现：

- straight；
- yaw left/right；
- pitch up/down；
- 为验收案例所需的空间斜向动作，例如倾斜弯曲平面的圆弧或三维方向直行。

定义yaw/pitch相对全局坐标还是局部frame，说明旋转轴和正负号；从真实三维pose生成下一pose及完整连续primitive。

**动作必须在搜索扩展阶段改变三维位置和三维方向。** 不允许先固定二维节点序列，再给节点补z。

每次候选生成后，在加入open集合前验证完整primitive的半径、边界、障碍与spacing；合法终端连接也必须使用相同规则。禁止先接受违规路径再调用elevation repair修复。

### Phase 4 — A* 3D

实现最小native 3D A*，第一版：

```text
edge_cost = primitive_length + congestion_penalty
g = 所有实际边cost之和
f = g + h
```

要求：

- `congestion_penalty >= 0`，与长度使用兼容单位或明确的等效长度权重。
- 单net无拥塞场时penalty可以为0，但接口必须明确；至少有一个非零拥塞案例验证影响cost，hard spacing不得用soft penalty代替。
- 拥塞场在单次搜索期间固定；若动态修改，要重新说明cost与搜索正确性。
- 禁止加入未验证bend loss/crosstalk dB；合法弯曲的弧长已计入length。
- heuristic采用目标位置的三维欧氏距离下界，不附加未经证明的方向/弯曲罚项。
- 如果允许目标位置容差epsilon，heuristic须对目标集合仍是下界，例如`max(0, distance_to_target - epsilon)`；不能照抄点目标距离后宣称始终admissible。
- 解释admissibility：任何合法连续路径至少需要相应的空间距离，拥塞项非负；方向/半径/障碍只会限制可行路径。
- 在有限小状态图上以`h=0`的Dijkstra/统一代价搜索作为独立对照，检查找到的cost一致；不宣称连续全局最优。
- goal判定同时检查目标位置和目标切向；不同切向到达同一位置不能直接判成功。
- source/target pose无效、预算耗尽、有限图搜索耗尽分别返回明确状态。

不要把timeout或max-expansions耗尽写成“证明不可达”。

### Phase 5 — hard constraints

必须实现：

1. **minimum bend radius**：所有弯段`R >= Rmin`；直线半径为无穷大；连接点不存在非法尖角。
2. **3D obstacle collision**：完整连续primitive与三维障碍不相交。
3. **minimum centerline spacing**：对固定已有波导中心线，连续路径最近距离`>= dmin`；即使本轮只搜索一条新net，也要用固定参考波导测试此功能。
4. **chip volume boundary**：整个primitive位于三维可行域内，不只检查首尾。
5. **source/target pose**：位置和方向符合明确容差。
6. **target tangent**：终点真实切向与目标方向一致；说明目标方向是期望传播切向，还是端口外法向及其符号转换。

明确区分：

- 波导中心线之间的`dmin`；
- 中心线到障碍的keep-out/clearance；
- 波导实体半径或工艺占用包络。

可以用同一个dmin作为合成案例中的障碍clearance，但必须显式配置，不能暗中混用。报告中的`clearance >= dmin`要说明它指哪种距离。

**禁止只检查采样点就宣称连续无碰撞。** 采用解析检查或有误差界的保守细分，例如用圆弧弦高界对距离下界扣除逼近误差；无法判定时继续细分或拒绝。粗筛包围盒不相交可用于排除，但包围盒重叠不自动等于碰撞。

输出连续几何检查的方法、容差和实际距离下界。不要只比较两条曲线相同参数/相同传播坐标的点；spacing须考虑任意参数对的最近接近。

另外说明是否检查路径的非相邻自接近；若未实现，明确列为限制。相邻连接段共有端点不应误判为spacing违规；若设置自接近检查豁免，给出其范围，不能用豁免掩盖路径回绕后的真实近接。

搜索后由独立验证入口重新核验最终原始路径。若有后处理，必须单独标记；native3D核心验收以原始搜索路径为准。

### Phase 6 — 最小验收案例

#### 核心案例：二维不可达，三维可达

构造完全相同输入，起点和终点均在`z=0`，目标切向明确。

三维chip volume内放置一堵障碍墙：

- 墙在`z=0`切片中横贯整个可行横向宽度，且与边界结合后不留下侧绕缝隙；
- 墙上下的范围经过明确设计，使`z=0`平面内源/目标位于不同连通区域；
- 仅墙上方保留足够高度与长度空间，使给定Rmin和clearance下存在三维可行路径；
- 不允许从chip volume外绕墙。

使用同一新引擎与相同几何输入做两个对照：

```text
2D mode：限制z=0，并禁用所有离开该平面的动作 → 不可达
Native3D mode：开放真正三维动作 → 可达
```

2D mode不能调用旧二维Router。以墙在平面内形成隔断的几何证据说明平面不可达；若搜索只因预算超限停止，不算该验收通过。

Native3D必须验证：

- 搜索状态/父节点轨迹中z真实变化；
- 初始状态的候选动作中已存在改变z/俯仰的可行动作，或明确证明初始状态空间与生成器已开放z自由度；**不要求最终路线第一段必须升高**；
- 最终路径的非零z段来自search的primitive，非后置抬升；
- 全程`R >= Rmin`；
- 连续障碍clearance达到案例配置的dmin，另行验证参考波导spacing；
- 无碰撞、不越界；
- 起终点正确、起终切向正确、路径C1连续。

保存案例输入、原始搜索轨迹、生成动作类型、路径几何和独立验证结果。图像可辅助展示，但不能以一张3D图替代上述证据。

#### 其它必须增加的案例

1. **纯直线**：已知三维端点，长度等于欧氏距离，首尾方向一致。
2. **空间弯曲**：至少一条路径包含非XY平面圆弧；增加倾斜平面圆弧的几何测试，不只是轴对齐高度变化。
3. **封闭障碍不可达**：起点与目标各自合法，但目标所在可行空间被闭合障碍隔离；区分搜索耗尽与预算耗尽。
4. **半径边界**：`Rmin-epsilon`违规、`Rmin`合法、`Rmin+epsilon`合法；epsilon须大于判定容差，容差内行为明确。
5. **spacing边界**：两中心线最近距离在dmin上下；包含最近点位于采样节点之间的案例。
6. **曲线中段越界/碰撞**：端点合法但圆弧中间穿墙或越出chip volume，必须被拒绝。
7. **终端切向**：同一位置但错误目标切向不得成功；合法终端connector完整接受约束验证。
8. **admissibility/cost**：有限小图A*与h=0对照；非零且非负拥塞项；未知光学参数不会生成dB结果。

这些是本轮最小实现的正确性测试，不扩展成多网联合routing。

## 2. 测试、真实证据与报告要求

完成前运行全部新增测试。若失败，继续修复并重跑相关测试，最后给出一次完整测试执行结果；不要仅展示部分成功案例。

必须报告：

- 精确命令、工作目录、环境和退出码；
- 实际collected/passed/failed/skipped/xfail数量（不适用项写0或说明）；
- 总耗时；参数化测试以测试工具实际统计为准；
- 完整测试日志及各验收案例产物路径；
- 路线成功率、终端位置/切向误差、最小半径、距离下界、collision/boundary违规数；
- 若测试未执行或有失败，明确写未完成，不得用“预计通过”代替结果。

不得预先承诺或填入虚构的测试数量与运行时间。PART A官方benchmark运行与PART B新增测试分别统计。

核心验收测试不得通过skip/xfail掩盖失败；执行了多少、通过了多少、哪些机制尚未动态验证，必须分别说明。

PART B最终输出：

```text
docs/reports/native3d_v01_report.md
```

报告至少包含：架构/文件职责、输入输出契约、状态与动作定义、几何公式与单位、cost/heuristic证明范围、连续约束检查方法、全部案例和真实结果、复跑命令、已知限制。

## 3. 最终必须回答的四个问题

1. **LiDAR哪些设计被忠实复现？** 分开写“官方源码已运行”“源码已审计但未触发”“论文2.0特性在固定版本中缺失/未确认”。一个最小benchmark不能证明所有机制都已动态验证。
2. **哪些是我们自己的native3D扩展？** 三维pose、方向、空间圆弧、yaw/pitch/斜向动作、三维搜索、连续collision/spacing与三维终端约束逐项列出，不冒充论文贡献。
3. **是否能证明搜索从第一步就具有z自由度？** 给状态定义、初始邻居、搜索trace、二维对照及原始路径证据；说明这是针对实现与验收案例的证据，不是对任意输入的连续完备性证明。
4. **当前还缺什么才能进入16-channel？** 至少讨论多网调度、3D占用更新、冲突检测、局部rip-up & reroute、history congestion、顺序依赖、整体独立验证、失败预算/恢复、空间索引性能及制造约束；只列下一阶段计划，本轮不要直接实现16通道。

## 4. 最终交付清单与执行纪律

交付：

- 固定版本的官方LiDAR副本、版本来源与依赖记录。
- 官方最小benchmark的真实输入/配置、运行日志与输出；若未成功则保留失败证据。
- `OpticalWaveguideRouterNative3D`独立项目、可运行入口与全部新增测试。
- 核心“2D不可达/Native3D可达”案例的输入、原始搜索trace和验证证据。
- `docs/reports/lidar_reproduction_baseline.md`。
- `docs/reports/native3d_v01_report.md`。
- 最终回复中的四问答案、真实测试数量/结果和可复跑命令。

按“阅读依据→固定官方版本→审计/最小运行→实现单导native3D→运行测试→完成报告”推进。合理的小型实现选择自主决定；遇到未知论文参数或真正阻塞如实记录，不脑补，不把未完成工作写成完成。

再次确认：**原生三维路径必须由三维搜索本身产生；本轮不做1024、不做ML、不做BPM、不造光学loss、不使用旧二维路由后升高。**
