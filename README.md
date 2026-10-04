# Euclid Search — Heuristic

Portable C++20 geometric construction search with beam, structural witness planning and bounded coverage workers. Current release: **v11-r3.2**. Heuristic failure is not proof of impossibility.

# v11-r3.2 启发式几何搜索

这是r3的继续改进版，版本输出为 `brute_search v11-r3.2 heuristic`。本轮方向是**目标元素的前置结构规划**：普通付费直径圆、镜像/位似/切线结构，加“至多一笔辅助操作＋实际可画目标元素”的锁定补全。目的不是继续盲目加宽beam，而是补齐目标依赖链。

新图模型、真实结果和近切数值风险见 **[R32_PERFORMANCE.md](docs/R32_PERFORMANCE.md)** 与 `benchmarks/r32/MODEL_SCOPE.md`。所有历史版本报告保留；输入/计费规则、严格浮点、两平台支持不变。

版本划分：原优化版为 **v10**；9950X native + LTO 构建为 **v10.1**；本目录是独立开发的 **v11**。

v11不是单纯重编译。默认搜索从DFS改成有界、多样性、多启动的目标引导beam搜索，优先寻找满足几何目标的构造。**它允许舍弃低优先级路径来换取找解机会，因此不保证搜全、不保证最短E，也不能以未找到解证明无解。**

## 运行

Windows发行程序在 `dist/windows-x64/bs_v11.exe`（静态链接MinGW运行库，无需额外DLL）。Linux完整项目位于 `./bs_v11`，通用x64程序在 `dist/linux-x64/bs_v11`。

Windows：

```bash
bs_v11.exe --threads=8 --solutions=1 --time-limit=120 < input.in > result.out
```

Linux 9950X：

```bash
./bs_v11 --threads=32 --solutions=1 --time-limit=180 --seed=1 < input.in > result.out
```

输入格式仍兼容v9/v10。初始有限线段不会免费变成无限直线，给定无心圆的圆心不会自动成为已知点。

## 搜索参数

| 参数 | 含义 |
|---|---|
| `--search=heuristic` | 默认v11启发式引擎 |
| `--search=exhaustive` | 切回保留的v10 DFS逻辑 |
| `--coverage-threads=auto` | 在总线程数内分配共享前缀DFS覆盖组（包含producer）；默认E<8用约一半，E≥8用约四分之一，至少2个slot，其余仍是启发式 |
| `--coverage-threads=0` | 禁用共享前缀覆盖组；显式N需为2..threads-1。有限restarts、关闭adaptive/尾助手、少于4线程或E<4时不启用 |
| `--beam-width=64` | 每个启发式worker每层保留状态数 |
| `--branch-limit=96` | 每父状态保留并实际评估的候选上限 |
| `--restarts=0` | 每worker重启次数，含首轮；0表示在时间预算内继续 |
| `--seed=1` | 随机序列种子；墙钟截断/多线程结果不保证完全复现 |
| `--tail-seconds=0.02` | 短尾搜助手的每次预算；0禁用助手 |
| `--tail-candidates=4` | 每层使用旧尾搜助手的前沿数；0禁用旧尾搜及持续DFS线程，但不禁用r2会合策略 |
| `--prerequisites / --no-prerequisites` | 默认开启付费前置结构与目标锁定补全；失败只是Unknown，关闭可作r3.1逻辑消融 |
| `--structural / --no-structural` | 开关全部结构探测（含本轮前置规划）；每笔普通操作正常计E |
| `--adaptive / --no-adaptive` | 默认启用结构会合、部分家族/地标策略和混合调度；可关闭用于消融 |
| `--landmarks` | 强制每个worker使用反向地标评分（一般无需强制）；`--no-adaptive --landmarks --tail-candidates=0`是纯beam地标实验 |
| `--certificate=FILE` | 输出17位JSON构造证书，仅允许新文件，不覆盖已有文件 |
| `--progress-interval=2` | stderr进度更新间隔，stdout保留结果 |

宽度大不一定快：它增加分支覆盖，也增加评分和求交成本。深问题可以尝试64/128/256宽度及不同seed，但不存在通用最优参数。

**推荐工作流**：默认r3.2在指定总线程内混合前置规划、结构探测、beam与共享前缀覆盖组，不增加额外搜索worker。仍可能有适合纯v10枚举的输入，可用 `--search=exhaustive`。结果和数值/精确构造区别见 `docs/R32_PERFORMANCE.md`，不把短时微基准当普遍性能保证。

## 与v10的区别

1. 候选先按目标命中、入射支持、构造前置条件、方向等评分，不仅按坐标距离。
2. 实际应用候选并生成合法交点后，对整个状态评分。
3. 每层保留高分状态，同时保留随机多样性，避免所有路径集中在同一个局部选择。
4. 多worker采用不同随机流和权重组合；每worker只持有一份可回滚Graph及紧凑有序前缀，而不是复制大量完整图。
5. 部分前沿使用限时旧尾搜助手快速找证书；助手失败不作为无解证明。总预算≤3E时，worker0额外做一次不超过剩余预算25%且最多2秒的旧DFS浅层尝试；其他worker仍跑beam。`--tail-candidates=0`关闭全部旧助手。
6. 每个结果仍须满足原始全部目标。可导出证书，用独立高精度脚本回放检查。

部分portfolio使用受限两步几何桥接评分，所有虚拟目标方向仅作评分。新同半径探测按普通known-pair构造 C–C–L–C 前缀，再实际补出目标前置点，并在启用尾助手时完成剩余两步。它不直接套入题目坐标，也不免费转移长度。

默认是混合搜索，不是纯beam；输出分别记录 `Beam`、`Helper`、`Rendezvous`、`Chain join`、`Point join`、`Coverage`、`Equal-radius probe`、`Prerequisite successful state visits`。覆盖组共享有界前缀队列，每个任务只交一个消费者，不是多个线程从根重复跑DFS。成功来源不混为“启发式评分提速”。

仅测试纯beam可用 `--no-adaptive --tail-candidates=0`，或加 `--landmarks` 使用新地标评分。r2的构造家族配额只属于启发式资源分配，同元素不同顺序的浮点状态不被宣称等价；所有实际操作保留原顺序。

## 结果状态

- `QUOTA_REACHED`，退出0：达到所需解数。
- `TIMEOUT_PARTIAL`，退出3：预算结束，可能有部分解。
- `HEURISTIC_STOPPED`，退出4：有限重启等启发式运行结束，但没有达到解配额，**不是EXHAUSTED**。
- `EXHAUSTED`，退出2：仅旧exhaustive路径可给出，仍有原浮点边界，不是精确数学证明。
- 退出1：输入、内存、线程等执行错误。

启发式无法诚实估计“搜完整棵树”的时间，因此进度显示 `coverage=unknown`、`exhaustion ETA=unavailable`，而不是v10中可能严重乐观的root百分比。

## 构建

```bash
python tools/build.py --run-tests
```

输出Windows `build/bs_v11.exe`、Linux `build/bs_v11`；Linux使用 `python3`。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

默认portable x64，严格浮点。可选native/LTO仅适合当前CPU，不要当通用程序分发：

```bash
python3 tools/build.py --native --lto --build-dir build/native --run-tests
```

## 验证与性能

性能评价必须看相同预算下的成功率、找到解用时、返回E，而不是把舍弃搜索树后更早停止当作加速。运行记录和局限见 `docs/PERFORMANCE.md`；算法设计见 `docs/DESIGN.md`。

```bash
python tools/heuristic_validate.py --exe build/bs_v11.exe --checker build/heuristic_tests.exe
python tests/v11_cli_tests.py --exe build/bs_v11.exe
```

证书独立高精度回放（需要mpmath）：

```bash
python tools/verify_certificate.py --input input.in --input-layout=compact --certificate solution.json --precision 100 --require-original-eps
```

本版本是“更积极地找解”的工具，不替代数学证明。重要新构造仍应跨非相似参数回放，不能只凭一个double数值实例宣布普遍成立或最少步数。

## License / 许可证

Copyright (c) 2026 CarefreeSongs712 and contributors.

This project is licensed under the GNU General Public License, version 3 only (**GPL-3.0-only**). See [LICENSE](LICENSE) for the complete terms.

本项目采用 GNU GPL 第3版（GPL-3.0-only）许可证，完整条款见 [LICENSE](LICENSE)。软件按原样提供，不附带任何担保。
