# BS v11-r3.2 开发接手

最新方向：目标元素前置结构与目标锁定补全。`goal_finish.hpp`只允许至多一个父状态known-pair辅助操作，然后真正画可生成的目标元素；missing-count不能当硬下界。`diameter_probe.hpp`/`mirror_probe.hpp`/`homothety_probe.hpp`/`tangent_probe.hpp`均保留paid parent，不免费给中点/镜像/切点。统一`probe_validation.hpp`验birth/finite；ApplyFiniteProbe捕获新交点溢出即rollback。镜像模块为可测能力，当前默认根规划主要diameter/homothety/tangent。新目标图在 `benchmarks/r32`，旧r3.1冻结 `build/revision_r31`。

重要数学边界：external tangent旧seed1合法EPS近切（切点差~5.8e-6）不等同精确相切；后续seed2找到真正泛型8E结构，已落tangent_probe四笔收尾（O,H圆→P,H圆→W,V圆→HX），其中HW直径、WV=2r、中位线距离=r给解析证明。最终新版两平台三个seed均高精度真相切8E。报告不得仍说“只有9E精确”；早期9E仅控制。Orthic泛型8E由diameter+reflection+三目标线给出。截图非游戏精确坐标，scope公开。

本轮仍未解决13.5/14.5原60秒预算，失败日志保留；新题很多旧版耗时不足10秒，不能据此宣传普遍长题提速。Windows/Linux严格FP，用户明确不测T6。

本轮在r3上继续优化，不是v12。冻结旧r3在 `build/revision_r3`（两平台），最终比较在 `benchmarks/r31`。用户最新新题测试暴露15.4三等线段旧r3超时；新增 `equal_radius_probe.hpp` 从真实C–C–L–C生成paid prefix，再真实补前置点并在尾助手开启时收尾，成功单独radiusProbe记录。`novelty_pool.hpp`在family入口保留随机家族，不让同族排列买更多抽样票，完整key而非hash判等；单测792组oracle。

并行coverage在 `RunHeuristic` 内使用原有有界PrefixTaskQueue，覆盖producer/consumer+beam合计严格threads。默认E<8分约一半slot，E>=8分约四分之一，至少2，保留至少1beam；允许 `--coverage-threads=0|auto|N`。只有adaptive且tail开启、restarts=0、threads>=4、E>=4可启动。成功Coverage单独计，不能冒称beam提速。队列等候每20ms复核stop，quota或外部纯atomic取消必须及时join。

新文件测试：coverage_tests（取消/预算/多解/源归因）、equal_radius_tests（paid witness/7E/变换/有限域）、novelty_tests。scope设置explicit关闭tail/有限重启必须尊重；EqualRadius callback已禁止在tailCandidates0/tailSeconds0时偷偷使用Solver。当前结构未解13.5和14.5，不得抹掉超时记录。

r3新增 `point_join.hpp`（父图一笔可达点带witness的双点配对，最多3E）、`chain_join.hpp`（真实依赖H→M→Q及可选目标线，3/4E）、`foundation.hpp`（点对连线和两互心圆，均普通E，整体预检失败无mutation）。结构helper处理已付费parent，必须保留root initialElementCount/birth，不可重新Seal；root API仍要求sealed。调度在heuristic.hpp，成功归因新增chain/point。单个beam结构调用≤0.6秒，每worker累计≤6秒；root scaffold另≤3秒；局部超时仅Unknown。冻结r2对照存 `build/revision_r2`，Linux同路径。

新主长测在 `benchmarks/r3`，报告 `docs/R3_PERFORMANCE.md`；优先r2旧10秒以上/90秒超时任务，另含未参与诊断的T7/T8留出任务。必须保留T7准线长例未加速的事实，不宣传普遍数量级提速。

用户当前要求：继续改善启发式，性能优先采用旧版10秒以上的困难例；**不测试T6**。r1二进制和源码已存 `build/heuristic_r1`，Linux `build/r1`。新成果需沿用Windows/Linux x64支持、独立高精度回放和无Git发行包。

r2新增 `rendezvous.hpp`（可重放见证的四笔反向会合）、`family_pool.hpp`（启发式家族配额，不声称等价）、`landmarks.hpp`（有界纯评分）。默认只给目标元素任务启用新前置预览/家族/部分地标，避免纯目标点任务多算而不改善路线；显式 `--landmarks` 可强制。多线程、助手启用时worker0在首beam后转持续DFS，用真正共享collector正确枚举多解，并发布累计进度。成功来源分beam/helper/rendezvous记录。

会合方向索引上限32MiB仅payload，不是进程总内存限制。所有目标虚拟坐标只用于检索建议；落地必须从真实known pair replay，`GoalsMet`后提交。API要求初图 `initialElementCount==elements.size()`，防已画前缀漏计E。

## 版本边界

- `../bs_optimized` 保留原优化版（v10）和9950X专用native构建（v10.1）。本项目是独立 v11；原几何题结论与历史源码不改。
- 用户已明确授权 v11 大改搜索逻辑为启发式，以较快找到构造为目标。因此允许显式 beam/branch 截断和随机多样性，但必须统计并声明不完备，绝不能把启发式失败标成穷尽无解。
- 默认 `--search=heuristic`；`--search=exhaustive` 是保留v10旧路径，用于需要旧枚举行为的场景。
- Windows/Linux x64，C++20。禁止fast-math和浮点收缩；默认portable x64，可选native/LTO。

## 文件与职责

- `src/heuristic.hpp`：v11多启动、多线程启发式beam搜索；精英候选+随机保留、目标结构评分、真实Apply后的状态评分、限时尾搜助手。
- `src/solver.hpp`：保留v10 DFS，及可快速找证书的旧尾搜助手。旧下界/逆向过滤仍有EPS边界风险，失败不作为严格数学证明。
- `src/geometry.hpp`/`core.hpp`：沿用几何语义和回滚顺序，不因启发式而免费添加目标点、圆心或长度。
- `src/progress.hpp`：只从原子快照读进度；启发式禁止显示搜索覆盖百分比或搜完ETA。
- `src/main.cpp`：旧stdin兼容、参数验证、选择引擎、输出状态。
- `src/reporting.cpp`：搜索结束后输出实际作图步骤。

## 必须保持的正确性与诚实边界

1. 每一步只能用当时已知的两点画普通圆/直线，交点按原规则加入。目标坐标仅用于打分，不是免费点。返回解必须实际满足全部点/元素目标及E预算。
2. 启发式失败输出 `HEURISTIC_STOPPED`（退出4）或 `TIMEOUT_PARTIAL`（退出3）；达解配额 `QUOTA_REACHED`（退出0）。不能输出 `EXHAUSTED`。
3. 搜索宽度/候选上限/随机保留影响成功率；比较性能必须报在同预算下的成功率、找到解用时、返回E。不以丢树后更早停止算“穷尽提速”。
4. EPS近似不传递。主beam不继承legacy目标数量或支撑数下界作为硬剪枝，不用无序浮点状态hash宣称等价。
5. 尾搜助手用私有超时控制，局部超时不能误取消全部搜索；成功经GoalsMet再提交全局收集器，失败仅Unknown。
6. Graph/候选/beam每worker私有，只有collector和取消标志共享。worker异常必须join并上抛，不能变成无解。
7. 固定seed决定随机序列；单线程、有限重启、禁用限时尾搜且不触及总时限时可严格复现。墙钟截断、多线程调度和解输出顺序不保证相同。
8. `--restarts=N`按每worker启动次数计算（包括首轮），0表示直至时间预算；`beam-width`/`branch-limit`也是每worker。
9. `--search=exhaustive`保留旧数值边界，EXHAUSTED仍不是精确代数证明。

## 验证

```bash
python tools/build.py --run-tests
python tools/heuristic_validate.py --help
```

Linux用python3和不带.exe的程序。完整构建支持CMake及CTest。

旧 `tools/validate.py`、`tests/cli_tests.py` 是v10格式回归，默认启发式下无解状态刻意不同，不能直接把状态不一致认作bug；用v11专用验证或显式exhaustive模式。
