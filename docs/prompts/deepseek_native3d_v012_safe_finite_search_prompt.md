# DeepSeek 执行任务：Native3D v0.1.2 残余正确性修复与有限多跳三维搜索

你是本项目的科研复现与算法工程执行者。请实际完成下列任务，提供源码、反例回归、运行证据和报告，不要只给建议。

最终研究目标保持不变：

**1024-channel native 3D optical waveguide router**

本轮仍然只处理单条波导。先关闭 v0.1.1 的残余正确性问题，再建立可以在本机完成跨墙案例的有限、多跳、原生三维搜索。不要进入 16 通道或 1024 通道。

## 0. 工作区、保护范围与阅读材料

工作区：

```text
C:\Users\lihao\Desktop\Graduation Project
```

可修改独立项目：

```text
OpticalWaveguideRouterNative3D/
```

先读：

```text
docs/prompts/deepseek_lidar_native3d_v01_prompt.md
docs/prompts/deepseek_native3d_v011_correctness_repair_prompt.md
docs/reports/native3d_v011_correctness_report.md
OpticalWaveguideRouterNative3D/README.md
```

保护范围：

- 不修改旧 `OpticalWaveguideRouter2D`、`OpticalWaveguideRouter3D`。
- 不修改固定官方 LiDAR 源码，不重跑 PART A，不改写 LiDAR baseline。
- 不改写文献审计报告。
- 保留 v0.1 与 v0.1.1 原始证据，新增输出使用 `artifacts/v012/`。
- 不加入 ML、BPM、GUI、GDS、光学损耗或串扰模型。
- 不通过修改既有墙体、Rmin、dmin、芯片体积或数值容差，让旧验收案例变容易。
- 不调用旧二维 Router，不做二维路线后抬升或 elevation repair。

v0.1.1 日志确实记录了 `127 passed in 142.10s`。这仅是已有测试结果，不能证明全部受支持输入都满足当前正确性声明。下面的残余缺陷已经通过只读反例重现。

## PART A — 先关闭残余正确性问题

先建立失败回归，再修复。PART A 未通过，不开展 PART B；明确报告未完成项，不用文档修改代替实现修复。

### A1. `key_exact()` 仍然通过舍入合并不同实际 pose

定位：

```text
src/native3d/search/state.py:102–112
src/native3d/search/router.py:99–107
src/native3d/search/astar.py:268–271
```

当前 `round(..., 9)` 仍然属于近似状态合并。容差远小于 Rmin，不能证明同桶 pose 的后继等价。

已实际运行的反例：

```text
S = (0,0,0)
A = (10,0,0)
B = (10.0000000002,0,0)
G = (20,0,0)
方向全部为 +x

S -> A：cost = 10，A 无后继
S -> B：cost = 10.0000000002
只有 B -> G：cost = 9.9999999998
```

实际 `astar_search`、欧氏启发式下：

| key | 结果 | 代价 | 扩展数 |
|---|---|---:|---:|
| 当前 `key_exact()` | search_exhausted | infinity | 2 |
| 实际 position + direction 浮点元组 | success | 20 | 3 |

修复要求：

- 连续 pose 调试搜索如继续保留，使用不经舍入的实际存储 pose 身份，不凭近似容差剪掉不同 pose。
- 新有限搜索使用真正的规范节点 ID，每个 ID 唯一对应一个规范 pose，后继从该规范 pose 计算。
- 舍入只能用于近似索引或明确标注的近似模式，不能继续称为无损 exact。
- 更新 `SAFE`、`Every distinct real pose`、绝不合并后继不同 pose 等错误声明。
- 加入上述微小差值反例；原来的 10 与 10.4 反例也必须保留。
- 说明 g 比较容差对数值最优性的影响。若使用 1e-12 等比较阈值，必须写出数值误差口径，不能宣称任意实数成本下的严格最优。
- 不声称无舍入浮点身份等于连续空间完备搜索。

### A2. 独立验证器漏掉 source pose 违规

定位：

```text
src/native3d/validation/validate.py:585–586, 623–642
```

已实际运行的输入：

```text
path：LineSegment3D((10,0,0), (100,0,0))
source pose：(0,0,0)，切向 (0,1,0)
target pose：(100,0,0)，切向 (1,0,0)
chip：[-2000,2000]^3
无障碍、无参考中心线，默认 RouterConfig
```

旧结果：`verdict=pass`，同时输出起点误差 10 μm、起始切向误差 90°。

修复要求：

- source position、source tangent、target position、target tangent 都必须进入最终判定。
- 明确起点与终点使用的容差；起点应满足源 pose 的约束，不默认套用宽松的终点接受球。
- 增加起点位置错误、切向错误、各自容差边界、合法起终点测试。
- 保留 C1、最小半径与长度检查，不能用端点正确替代全路径正确。

### A3. 独立验证器忽略参考圆弧

定位：

```text
src/native3d/validation/validate.py:383
src/native3d/io/caseio.py:48–50
```

Scene 已支持圆弧参考中心线，但验证器仅筛选 `kind == "line"`。

已实际运行的反例：

```python
path = LineSegment3D((10,0,0), (100,0,0))
reference_arc = CircularArc3D(
    (50,50,0), (0,-1,0), (1,0,0), 50, math.pi
)
far_reference_line = LineSegment3D((0,100,0), (100,100,0))
```

使用路径自身的起终点 pose、chip `[-2000,2000]^3`、默认配置、无障碍。

路径与参考圆弧在 `(50,0,0)` 相交。生产 spacing 检查为 `violation`，独立验证器却为 `pass`、间距 100 μm。
若只保留参考圆弧，还会因 None 与 float 比较而出现 TypeError。

修复要求：

- 独立验证覆盖 line-line、line-arc、arc-line、arc-arc，不能静默丢弃已支持的类型。
- 两侧均为曲线时，认证下界计入双方的几何近似误差。
- 纯圆弧参考集合、混合集合、多个参考曲线分别测试。
- 未支持或畸形输入必须明确拒绝或返回不能认证，不能忽略后给出 pass。
- 保持验证器独立：从 JSON 自行重建和检查几何，不导入生产 geometry/search/constraints 来充当第二实现。

### A4. 大扫角圆弧使弦误差界失效，生产和独立验证均可能假通过

定位：

```text
src/native3d/geometry/primitives.py:175
src/native3d/constraints/checks.py:246–249, 313–338
src/native3d/validation/validate.py:299–305, 349–352, 402–420
```

已实际运行的参数：

```python
cfg = RouterConfig()
arc = CircularArc3D(
    (0,0,0), (1,0,0), (0,1,0), 50, 8*math.pi
)
reference_line = LineSegment3D((-51,-1,0), (-49,1,0))
obstacle = Box3D((-51,-1,-1), (-49,1,1))
```

圆弧经过 `(-50,0,0)`，真实与参考线相交，也穿过障碍。固定 n=1 或 n=2 会产生多圈子弧，使周期性余弦错误给出 sagitta=0。

已实际观察到：

```text
生产 spacing：safe，所谓下界 99.005050376 μm
生产 obstacle clearance：safe，所谓下界 99 μm

独立验证（无障碍，仅上述参考线）：
chip = [-2000,2000]^3
source/target 使用该圆弧自身的 pose
sampled spacing = 0.022088195608 μm < dmin=5 μm
claimed lower bound = 99.005050376 μm
lower-bound consistency flag = False
verdict = pass
```

当前搜索通常不生成 8π 圆弧，但公开几何与约束接口接受它，不能据此忽略反例。

修复要求：

- 每个参与弦近似的子弧必须满足误差界的已证明适用域，例如初始细分确保 `abs(delta_theta) <= pi`，再继续加密。
- 或者统一、明确拒绝不支持的扫角；构造器、JSON 输入、生产检查和独立验证的支持范围必须一致。
- 不得用多圈弧端点重合来推断弧段偏差为零。
- 加入正负扫角、跨 2π、接近适用域边界、多个绕圈的测试。
- 任意可靠违规见证必须优先判失败，不能让所谓下界的 safe 分支盖过它。
- `lower_bound > sampled_minimum + documented_tolerance` 时不得 pass；认证自检失败必须改变 verdict。
- 下界小于采样值仅是必要一致性检查，不能替代误差界的证明。

### A5. 近乎平行线段的浮点消减不能被称为无误差认证

定位：

```text
src/native3d/constraints/checks.py:140–141
src/native3d/validation/validate.py:174–175
```

已只读运行的反例：

```text
A：(0,0,0) -> (1000,0,0)
B：(0,-5e-6,0) -> (1000,5e-6,0)
真实距离 = 0，交点为 (500,0,0)
dmin_centerline_um = 5e-7
```

当前 `a*e-b*b` 存在浮点消减，计算出约 9.04e-7 μm，生产可判 safe、独立验证可判 pass，采样却命中距离 0。
这个案例使用比默认 5 μm 更小的 spacing；不要声称它直接证明默认案例违规。但它确实推翻当前“数值结果永远是认证下界”的声明。

修复要求：

- 对近乎平行、尺度变化、退化情况采用稳定计算和明确的数值误差处理。
- 必要时使用独立高精度参考或保守区间判定；无法可靠认证时返回 undecided/uncertified 或拒绝。
- 数值容差与长度单位必须明确，不能把过大的近似距离当成严格下界。
- 本例及其放大、缩小、平移、旋转版本进入回归；明确支持的数值尺度。
- 生产检查和独立验证都要修复，不能仅让独立验证器在看到矛盾时兜底。

### A6. 修正文档中的保证范围

保留 v0.1.1 历史报告，只增加醒目勘误链接。新报告必须纠正：

- 九位小数舍入并非无损状态身份。
- 127 passed 不能证明全部受支持输入均已认证安全。
- 参考图共面性实际走浮点连接器判断，当前没有报告所称的精确整数共面测试。
- 1200 秒预算停止只能说明该实现和预算下未完成，不能推出“不可有限完成”“超指数增长”等理论结论。
- 自间距若被写为“已有后验检查”，核对是否真的存在；没有实现就明确记为未实现。本轮不扩展路径历史搜索。

PART A 验收必须同时通过生产几何/约束测试与独立 JSON 验证。只修报告、只缩小桶、只增加采样点或只检查一个字段均不算完成。

## PART B — 建立有限、多跳、原生三维单网搜索

只有 PART A 完成后开始此部分。

### B1. 当前参考图的真实范围

`reference_graph.py` 的中间节点是自动生成的三维格点，不是手工指定获胜中点，这一点应如实保留。
但当前后继严格限定为：

```text
source -> target
source -> mid -> target
max_hops = 2
```

没有 `mid -> mid`。这证明有限两跳连接器候选图可搜，不代表一般多跳生产 Router 已通过跨墙验收。

保留该参考实现用于对照。不要把它改名后继续作为新生产 Router，也不要通过枚举固定数量的 waypoint 模板代替多跳搜索。

### B2. 最小可行的有限状态图

实现新的公共入口，例如：

```text
route_finite_native3d(scene, ...)
```

或为 `route()` 增加明确的 strategy。报告、CLI、示例和 artifacts 必须区分：

- 连续 pose 调试搜索。
- 原两跳有限参考图。
- 本轮新的有限多跳原生三维 Router。

有限图必须满足：

- 状态使用 `(i, j, k, direction_id)` 等整数 ID。
- 每个 ID 唯一对应规范 `(x,y,z)` 和真正三维单位切向。
- 方向集合包含三维方向及上下俯仰转移，不能只存二维 orientation 加层号。
- 芯片体积、格距、方向数和有限动作集合共同给出有限节点数上界。
- 同一规范节点始终有同一组确定后继；每次从规范 pose 生成连续几何。
- 相同节点按 g 去重的依据是规范状态等价，不能是实际 pose 舍入后近似相同。
- 所有中间节点均能继续生成局部后继，允许任意多跳；不能限制为 source/mid/target 三类跳转。

可采用如下最小方案，也可提出等价、可证明的方案：

```text
位置：有界三维规则格
方向：六个轴向切向 ±x、±y、±z
直线：一个或有限多个整数格距
转弯：在空间坐标平面内的 ±90° 圆弧
半径：格距的整数倍，且 >= Rmin
```

这是本轮自主设计的离散搜索方案，不要写成 LiDAR 官方算法。

六方向可作为 v0.1.2 的最小支持范围；不要求立即扩展 26 方向。现有任意空间平面圆弧几何能力应保留。

### B3. 动作与几何一致性

- 直线、yaw、pitch 从三维规范 pose 直接生成，搜索第一步即有 z 自由度。
- 圆弧起点、终点与切向应由所选规范转移的几何公式给出。
- 可以使用明确的浮点构造容差，但不能先生成错误端点，再吸附到格点、扭曲曲线或做 elevation repair。
- 每条边进入 OPEN 前检查完整连续曲线：Rmin、芯片边界、障碍 keep-out、已有中心线 spacing。
- 最终组装结果再次检查端点、目标切向和 C1，并由独立验证器认证。
- 将曲线终点与规范目标节点的误差记录下来，不能只检查节点 ID。
- 选择动作半径可以大于 Rmin，但不能修改输入的 Rmin。

### B4. 端口接入与终端连接

源和目标可作为虚拟节点，使用已验证的局部接入曲线连接规范图。原核心案例的端口与格点相位不能改变。

- 接入/终端边必须满足同一连续几何约束和切向约束。
- 接入邻域由通用配置决定，不能为跨墙案例指定中点、层高或路线。
- 支持范围不足时返回明确的 unsupported_pose 或 port_attachment_failed，不把它写成物理不可达。
- 可以保留当前共面连接器范围；不要求本轮发明完整非共面连接器。
- 必须有至少一个测试，其正常中间节点到中间节点转移构成路径，证明不是全靠 source->mid->target。

### B5. A*、成本与状态语义

继续使用：

```text
cost = real_path_length + nonnegative_congestion_penalty
h = max(0, Euclidean_distance_to_target - eps_pos)
```

- 正确的目标出堆或有证明的 incumbent 下界终止。
- 不把贪心提前返回重新引入来解决性能问题。
- 结果保证限于所构造的有限图及明确的数值容差；不声称连续空间全局最优或完备。
- `search_exhausted` 只表示该有限图无解。
- `budget_exhausted`、端口接入失败、输入 pose 非法和数值未认证要分开。
- 有限节点/边数量上界应从构造计算，而不是凭运行没有继续扩展推断。
- 小图由独立 Dijkstra 对照，不仅把同一 A* 的 h 改成零。
- 可以缓存静态边与碰撞检查，但缓存键必须包含影响几何、约束和代价的配置；不得跨场景复用错误结果。
- 本轮拥塞场是固定、非负的单网代价，不增加多网调度。

## PART C — 实际验收与性能记录

### C1. 原始跨墙案例必须由新公共 Router 跑通

保持原输入：

```text
chip：(0,0,-60) -> (600,300,120)
wall：(290,-50,-70) -> (310,350,40)
source：(60,150,0)，tangent=+x
target：(540,150,0)，tangent=+x
Rmin=50 μm
obstacle clearance=5 μm
dmin=5 μm
boundary margin=2 μm
原参考中心线保持不变
```

要求：

- 新有限多跳 Router 的 native3d 模式真实 success。
- 同一图构造、几何和约束下，2D 模式禁用离开 z=0 平面的边，真正 OPEN 耗尽；没有超时或预算停止。
- 无需复现旧长度 495.301314 μm；新图可以得到更长的合法路线，应解释离散动作带来的限制。
- 路径的 z 变化来自搜索边；提供初始候选、展开 trace、逐段 provenance。
- 验证连续曲线的 R、clearance、spacing、边界、源/目标 pose、目标切向和 C1。
- 使用新公共入口产生证据，禁止在失败后悄悄回退到两跳参考图。

性能验收预先固定：该核心案例单次搜索预算 60 秒、最多 200000 次扩展。记录建图/预计算时间与搜索时间，核心端到端也应在 60 秒内完成；两者都要披露，不能把预计算藏在计时之外。
这里的端到端指冷缓存下公共 Router 接收 Scene 到返回结果，包含本次建图、预计算、搜索与生产约束检查。独立验证和证据写盘另行计时，同时报告整个验收流程总耗时；不能用预热缓存代替冷运行结果。
如果未达到，就报告该项未完成，给出真实瓶颈；不能修改验收场景或用旧参考图结果代替。

### C2. 增加一般性与多跳测试

至少包括：

1. 纯直线。
2. 真实三维弯曲。
3. 跨墙核心案例。
4. 多个中间节点、三个以上动作的绕障案例；节点之间必须继续搜索，不能依赖固定两跳模板。
5. 多墙或连续绕障案例，证明一般局部后继能组成更长路径。
6. 封闭障碍、端口本身合法但目标不可达，有限图真实耗尽。
7. 原场景改变墙位置、高度、源/目标位置与格点相位的确定性变体。
8. 起点/终点切向错误、Rmin 和 spacing 刚好合法/违规。
9. PART A 中所有已确认反例。

不得向 Router 提供人工 waypoint、指定最高 z、规定换层位置或预制路径。
固定场景允许人工设计用于测试，但算法实现不得识别案例名称或硬编码坐标。

### C3. 全量测试、证据与计时

运行全部原有与新增测试。不通过删测试、skip、xfail 或改成只检查 budget_exhausted 来让验收通过。
旧连续搜索与参考图的测试可保留并显式选择对应策略；新生产可达性测试必须检查 success 与合法路径。

新增证据：

```text
OpticalWaveguideRouterNative3D/artifacts/v012/logs/
OpticalWaveguideRouterNative3D/artifacts/v012/cases/
OpticalWaveguideRouterNative3D/artifacts/v012/probe/
docs/reports/native3d_v012_safe_finite_search_report.md
```

- 先真实运行失败回归，保存日志；不要事后编造修复前失败。
- 记录 Python/依赖版本、命令、退出码、collected/passed/failed/skipped/xfail/error。
- 不能把旧的 127 passed 当成本轮结果，也不预设必须得到某个测试数量。
- 给出图节点/边上界、实际访问节点、扩展数、生成/拒绝候选数、缓存命中、峰值内存及计时方式。
- 区分建图、几何检查、搜索、独立验证、pytest 墙钟。
- 每份核心 JSON 记录 strategy、状态身份、数值容差、输入配置及约束认证。
- 保留历史证据，避免新测试覆盖 v0.1/v0.1.1。
- README 与示例明确如何复跑新公共 Router，及其方向、格距、半径和端口支持范围。

## 最终报告与回复必须回答

1. 残余状态合并、source pose 漏检、参考圆弧漏检、大扫角和数值消减分别如何修复？原反例现在是什么结果？
2. 生产约束与独立验证是否仍可能对已证实违规输入输出 safe/pass？遇到不能认证时如何处理？
3. 新有限状态的唯一规范 pose 是什么？为何按节点 ID 去重成立？
4. 新图如何自动生成 mid->mid 后继？哪项测试证明它不再局限两跳？
5. 原跨墙案例是否由新公共 Router 跑通，是否存在任何参考图回退？2D 是否真实耗尽？
6. 搜索从第一步是否具有 z 自由度，路径是否逐段对应三维动作？
7. 本轮真实测试数量、结果、耗时、核心搜索性能与峰值内存是多少？
8. 当前能保证什么、不能保证什么？进入 16 通道前仍缺哪些模块与验证？

完成后停止。本轮不进入多通道，不加入光学损耗，也不把有限图保证外推到连续空间。
