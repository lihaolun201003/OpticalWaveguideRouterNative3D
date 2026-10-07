# DeepSeek 执行任务：Native3D v0.1.3 公共接口一致性与三维几何验证修复

你是本项目的科研复现与算法工程执行者。请实际修复下列已重现的问题，建立回归并重新验收，不要只修改报告措辞。

最终目标保持：

**1024-channel native 3D optical waveguide router**

本轮保留已经跑通的有限多跳原生三维架构，只补齐公共接口和三维几何验证契约。仍然只处理单网及固定参考中心线，不开始 16 通道，不加入 ML、BPM、GUI、GDS 或光学损耗模型。

## 0. 已确认有效的结果与本轮范围

v0.1.2 的以下结果有源码与日志支持，应保留：

- 整数规范节点 `(i,j,k,direction_id)`、六个真正三维轴向方向、统一局部后继和任意多跳。
- 原跨墙案例由 `route_finite_native3d` 跑通，没有两跳参考图回退。
- native3d 路径长 594.159265 μm、75 次扩展；2D 真正耗尽，81 次扩展。
- z 变化来自空间转弯搜索动作，不是二维路径后抬升。
- `20_final_full_pytest.log` 为 308 passed / 213.12 s；`21_verification_final.log` 为 308 passed / 213.71 s。
- 默认新建图分支的冷缓存计时包括建图、搜索和组装检查。探针 0.165 s 与另一次约 0.184 s 属于不同运行。

下面的问题没有推翻这条核心路径，但公共 API 仍可报告错误成功，独立验证也存在漏检和错误的三维距离计算。

工作区：

```text
C:\Users\lihao\Desktop\Graduation Project
```

先读：

```text
docs/prompts/deepseek_native3d_v012_safe_finite_search_prompt.md
docs/reports/native3d_v012_safe_finite_search_report.md
OpticalWaveguideRouterNative3D/README.md
```

可修改：独立项目 `OpticalWaveguideRouterNative3D/` 及本轮报告。保护旧 2D/3D 项目、官方 LiDAR 源码、文献审计及历史证据。v0.1.2 报告只增加勘误链接，保留正文。

新增证据使用：

```text
OpticalWaveguideRouterNative3D/artifacts/v013/
docs/reports/native3d_v013_contract_geometry_repair_report.md
```

以下行号用于定位，以实际源码为准。先真实运行失败回归，再修复。

## PART A — 公共 Router 的输入、图与结果必须一致

### A1. 不匹配的预建 graph 可返回错误终点的 success

定位：

```text
src/native3d/search/finite_graph.py:566–587
src/native3d/search/finite_graph.py:631, 660–664, 688–696
src/native3d/search/finite_graph.py:531–558
```

已运行的反例：相同芯片、默认配置、无障碍/参考线，source `(0,0,0)`、切向 +x。

```python
# scene A target = (100,0,0), tangent = +x
# scene B target = (50,0,0), tangent = +x
graph = FiniteGraph(scene_A)
result = route_finite_native3d(scene_B, graph=graph)
```

旧实际结果：

```text
status = success
actual endpoint = (100,0,0)
goal_pos_error_um = 50
cost = 100
```

fresh scene B 则正确返回终点 `(50,0,0)`、cost=50。

根因：传入 graph 没有一致性检查；目标键来自旧图；成功只看旧图的键；组装复验没有核对真正输入的 source/target pose。

修复方案可以选择：

1. 明确删除或限制公共入口的 `graph=` 注入，只允许内部构造新图；或
2. 接受预建图，但验证完整不可变快照一致性，不匹配时在搜索前明确拒绝。

如果保留复用，检查范围至少包含：

- source/target 位置及三维切向。
- chip、障碍物几何、所有参考中心线的完整几何内容，而不只是数量。
- mode，特别是 2D 与 native3d。
- 动作、半径、格距、边界、clearance、spacing、端口接入和所有影响边可行性的配置。
- 拥塞场或成本场的身份/版本/内容契约。无法证明不可变或提供稳定版本时，禁用该类缓存复用。
- Scene、config、参考曲线或拥塞场在构图后发生变化的情况。

不能只是存一个名为 fingerprint 的字典却从不检查。不能只比较对象地址，因为 Scene 可变。不能仅比较参考曲线数量。

回归至少包括：

- 上述换 target 反例。
- 换 source 或 source/target tangent。
- 2D/native3d 混用预建图。
- 相同数量、不同几何内容的参考线/圆弧。
- 构图后增加障碍或修改 occupancy。
- 改成本场或连接器/动作配置。
- 若保留图复用接口，真正匹配的图可合法使用，并明确是冷图还是热缓存；若删除公开注入，测试明确拒绝该调用方式。

输入与图不匹配时，可以抛明确 ValueError 或返回明确状态，不能输出 success，也不能标成 search_exhausted。

### A2. success 必须验证实际返回曲线的端口

- 同时核对搜索目标 ID 和真正输入目标的 pose，不能只相信旧图目标键。
- 返回 success 前检查组装路径的实际 source position/tangent、target position/tangent 与 C1。
- 原则上每条边的实际曲线终点和规范目标 pose 误差超限就拒绝，而非仅记录误差后仍放行。
- 明确源容差与目标接受球容差，保持 v0.1.2 的配置语义。
- 如果组装曲线检查失败，返回 `constraint_failed`、`uncertified` 等明确状态；不能用 search_exhausted，除非 OPEN 真正耗尽。
- cost、length、path、endpoint_errors、status 之间必须一致。失败后不能留下可被误读为有效成功的路径或保证字段。

增加结果不变式：所有 success 要么是明确定义的零长度解，要么含实际合法路径，且实际端口误差在允许范围内。

### A3. 起终点相同的合法输入不能崩溃

已运行的输入：

```text
source = target = Pose3D((0,0,0), (1,0,0))
chip = [-60,60]^3
默认配置，无障碍、无参考线
```

当前公共入口抛出：

```text
AttributeError: 'NoneType' object has no attribute 'segments'
```

原因：A* 的零长度成功返回 `path=None`，有限 Router 无条件读取 `res.path.segments`。

修复要求：

- 定义零长度解的统一返回契约：cost=0、length=0、源/目标误差可核对，path 表示有明确含义。
- Router、JSON 序列化、独立验证和 CLI 对此契约一致，不靠虚构微小线段回避。
- 相同位置但不满足目标切向要求时，不能自动当成零长度成功。
- 源 pose 本身非法、在障碍内或违反固定中心线 spacing 时，不能因为 source==target 绕过输入验证。
- 测试有限、连续及参考策略适用的边界行为；不强迫每个策略采用完全相同内部表示，但公开语义必须明确。

## PART B — 三维点到圆弧距离与 JSON 几何契约

### B1. 点到空间圆弧的公式漏掉法向高度

定位：

```text
src/native3d/validation/validate.py:569–603
```

当前用 `abs(norm(p-C)-R)`，这是点到球面的径向差，不是一般三维点到平面圆弧的最近距离。

距离函数级反例：

```text
参考圆：center=(0,0,0), u=(1,0,0), v=(0,1,0), R=50, theta=2π
p=(0,0,50)
当前距离=0
真实距离=sqrt(50²+50²)=70.710678... μm
```

完整合法路径反例：

```text
path：(-10,0,50) -> (10,0,50)
reference：上述 XY 平面整圆
chip：[-2000,2000]^3
source/target：路径自身 pose
默认配置，无障碍
```

真实最小距离 `sqrt(40²+50²)=64.031242... μm`，生产 spacing 判 safe，独立验证却判 fail、所谓采样最小值=0。

这是错误拒绝，不是发现核心路径碰撞；但当前值被标成“达到的距离”“可靠违规见证”，也会误拒合法三维避让。

修复要求：

- 对真实正交单位 basis，令 `q=p-C`、`a=q·u`、`b=q·v`、`h=q·(u×v)`、`rho=sqrt(a²+b²)`。
- 若最近相位在扫角区间内，完整圆距离为 `sqrt((rho-R)²+h²)`；否则检查弧端点。
- rho=0 时单独处理，不能把位于圆平面法向上的点当成落在圆上。
- 覆盖正负扫角、跨周期、多圈、任意空间平面、弧端点最近与位于圆心轴上的点。
- 返回的 witness 必须对应实际曲线点/参数；核对 `reported_distance` 与两实际点的距离一致。
- 保持采样估计/达到的上界、认证下界和未认证状态的区别。
- 不通过删掉参考圆弧、仅支持二维点或把所有失败改成 pass 来解决。

验收：上述函数级和完整路径反例给出正确值，合法路径独立验证可通过；真实相交/spacing 违规的既有案例仍必须失败。

### B2. JSON source/target 切向未校验或归一化，可假通过

定位：

```text
src/native3d/validation/validate.py:960–975
src/native3d/geometry/primitives.py:Pose3D.from_dict
```

已运行的反例：

```text
path：(0,0,0) -> (100,0,0)
source tangent=[1,0,0]
target tangent=[1,1,0]
默认配置、chip=[-2000,2000]^3，无障碍/参考线
```

独立验证原始点积被 clamp 后得到 angle=0°、verdict=pass。
生产 `Pose3D.from_dict` 正确归一化后，目标方向与路径切向相差 45°，不满足默认 3° 目标容差。

修复要求：

- 明确 JSON pose 的公开语义：与生产解析保持一致，接受合法非零三维方向时独立归一化；或者严格拒绝不符合契约的原始输入。
- 不允许未经单位化的点积进入 acos。
- source 与 target 都校验三维有限位置、非零有限方向、正确维度。
- 零方向、NaN、Infinity、缺字段、错误维度明确拒绝，不能产生 pass。
- 增加 scaled tangent、上述非单位斜向切向及方向反向测试。
- 独立验证器自行实现检查，不通过导入生产 Pose3D 来伪装独立实现。

### B3. 接受近单位 basis，却把实际椭圆当成圆弧认证

定位：

```text
src/native3d/geometry/primitives.py:177–184
src/native3d/constraints/checks.py:654
src/native3d/validation/validate.py:圆弧输入与半径检查
```

已运行的反例：

```text
R=2000 μm
u=(1+8e-10,0,0)
v=(0,1,0)
theta=π/2
rmin=2000 μm
turn/connector radii=(2000,)
radius tolerance=1e-6 μm
chip=[-5000,5000]^3，无障碍/参考线
source/target=该 primitive 自身 pose
```

构造器的 1e-9 范数容差允许这组输入，但保留原 basis，实际曲线为椭圆。
生产 radius_check=safe、独立 validator=pass；真实最小曲率半径 `2000/(1+8e-10)=1999.9999984 μm`，低于允许下限 1999.999999 μm。

这个差异很小，不要说它证明原核心路线违规。它证明当前圆弧与曲率契约未闭合。

修复要求：

- 确保实际存储和计算的 primitive 为圆，而非仅在注释中称为圆。
- 构造时可规范化/正交化 basis，或明确拒绝不支持的偏差；说明处理对实际端点与输入语义的影响。
- JSON 独立验证不能静默改变几何后仍对原始几何宣称认证。规范化语义若被采用，必须在序列化契约中明确并与生产一致；否则显式拒绝不合规几何。
- 长度、切向、AABB、曲率和距离界必须使用同一实际几何。
- 任意空间平面正常输入仍受支持；测试近单位、近正交、符号反向、畸形 basis 和正常旋转输入。
- 明确浮点容差与认证余量，不能把一个字段 R 当成所有近似曲线的真实最小曲率半径。

## PART C — 边界、统计和报告口径

以下代码审计点一并修复，不把它们夸大为原核心案例失败：

1. 当前边上界是 `N*(straight_count+4*radius_count)+4096`。端口候选与连接器半径可配置，硬编码 4096 不是一般构造证明。由实际节点与动作/候选配置推导有效上界；说明节点上界是否包含虚拟端口。
2. `edge_check_s` 累计包含构图期间端口附件检查，不能说全部是 search_s 的子集。拆分统计或准确改为建图与搜索期间的累计边检查时间。
3. 预建图/热缓存分支计时与默认冷图分支分别标记。默认冷缓存的已有结果应保留，不把该保证外推到所有调用分支。
4. `tracemalloc` 峰值是 Python 跟踪分配量，不是进程总内存。
5. search_exhausted、budget_exhausted、端口接入失败、缓存不匹配、输入非法、constraint_failed、uncertified 明确分开。

本轮不重写有限图，不增加方向集或大规模优化。保持六轴向、空间 90° 动作、长度+非负拥塞代价及正确 A* 终止。

## PART D — 回归与重新验收

先把上述反例写入测试，在修复前真实跑出失败。保存原始退出码和日志，不能编造红色基线。

修复后：

- 运行全部既有和新增测试，不删测试、不 skip/xfail 关键正确性用例。
- 原跨墙场景完全不变：新有限生产 Router native3d success，2D 真正耗尽，无参考图回退。
- 保留原 60 秒冷调用端到端预算，披露建图、搜索、组装检查和独立验证各自耗时。
- 原 PART A source、参考圆弧、8π、近平行、状态身份反例继续正确。
- 新增合法三维路径高于参考圆弧的完整验收，不能只有点距离单元测试。
- 新增缓存错配、输入变更、零长度及实际端口不变式测试。
- 输出策略、状态 ID、动作 trace、实际曲线端口误差、连续约束与独立验证结果。
- 拥塞、length、cost、mode、scene 与 graph 快照一致；不能把旧图成本当成新调用成本。
- 每次运行的新证据写入 `artifacts/v013/`，保留 v0.1/v0.1.1/v0.1.2 历史文件。
- 给出真实 collected/passed/failed/skipped/xfail/error、命令、退出码与耗时，不沿用旧 308 passed。

报告至少包含：

1. 每项反例的真实修复前/后结果、对应文件和函数。
2. 公共 graph 复用契约及一致性检查范围。
3. 零长度解、实际端口与错误状态的返回契约。
4. 三维点-弧距离推导、witness 与上下界的物理/几何含义及单位。
5. JSON pose/basis 解析契约及数值误差范围。
6. 原跨墙与新增合法三维场景的验收结果。
7. 全量测试、冷/热计时与内存口径。
8. 尚未解决的范围：自间距未接入、AABB 障碍、端口连接覆盖有限、离散图保证、不含光学模型。

最后给出下一阶段的最小设计建议即可，不执行：

- 首先接入真实自间距验收，明确局部接缝豁免的几何定义；不能简单跳过所有相邻段或只比较同参数点。
- 然后两条路径顺序固定占用，第一条的全部直线与圆弧都成为第二条的参考中心线，缓存必须正确失效。
- 这两项通过后再进入 16 通道确定性顺序基线，暂不加入复杂损耗或 ML。

## 最终回复必须回答

1. 不匹配图是否在搜索前拒绝？实际端口是否独立决定 success？
2. 零长度调用是否返回明确定义的合法结果而不崩溃？
3. 合法三维点/路径到参考圆弧的距离是否正确，真实违规见证是否来自实际曲线点？
4. 非单位 pose 与近单位 basis 是否存在可假通过通道？支持范围与解析契约是什么？
5. 原跨墙结果是否保持，全部测试真实数量与结果是多少？
6. 哪些保证仅适用于构造的离散图和数值容差？进入两通道固定占用与 16 通道还缺什么？

完成本轮修复与验收后停止，不开始多通道开发。
