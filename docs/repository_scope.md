# 独立仓库范围与版本说明

此仓库从毕业设计总工作区的 `OpticalWaveguideRouterNative3D/` 子工程建立，当前开发基线为 v0.2。

包括源码、测试、示例，早期修复所需冻结源码、JSON、日志，多通道案例与图表，以及相关报告和任务 prompt。报告和 prompt 是从总工作区复制的版本，原件保留在原位置。历史文档仍可能记录原机器路径、旧命令和旧版本结论；实际入口以根 README 为准。

不包括二维/旧三维工程、官方 LiDAR 仓库、论文 PDF、输入工作簿、PPT/视频、虚拟环境、安装缓存、凭据或毕业设计总目录的其他资料。LiDAR 报告只提供背景和源码审计记录，不能当作本仓库包含了官方算法。

`pyproject.toml` 的发行版本为 `0.2.0`。`native3d.__version__ == "0.1.3"` 表示单网几何/搜索内核版本；多网调度层为 v0.2，两者含义不同，未改写历史证据中的引擎版本。

`.gitattributes` 禁止自动换行转换，保留冻结证据的字节内容。核心 Router 无第三方运行依赖；pytest 用于测试，matplotlib/numpy 仅用于绘图。`requirements-reproduction.txt` 记录实际环境的顶层版本，不是完整环境锁定。

pytest 会生成验收 JSON，基准脚本会重写 v020 的案例/指标/日志。需要保持历史证据原样时，在独立 clone 中复跑；支持的单网输出可通过 `NATIVE3D_ARTIFACT_DIR` 改到其他目录。不要把复跑耗时与历史记录混写。

旧完整 README 保存在 [technical_overview_v020.md](design/technical_overview_v020.md)。v0.3 的任务 prompt 是下一阶段计划，当前没有实施路径历史搜索、net ordering 或 rip-up & reroute。
