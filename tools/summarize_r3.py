#!/usr/bin/env python3
"""Generate r3 performance report from retained raw comparisons."""
from pathlib import Path
import json

P=Path(__file__).resolve().parents[1]
text='''# v11-r3：结构化补全与长任务对照

## 口径

本轮保留冻结的r2源程序与二进制作为对照，v10也可参与相同输入测试。Windows为i9-13900HK，Linux为9950X；相同EPS、同线程、同预算，seed1/2并交错执行顺序。优先选择旧程序十秒以上或到时仍未命中的实例，新算法若快速找到不人为拖慢。短回归及微秒实例不用于宣称普遍提速；本轮**未测试T6**。

r3仍是不完备启发式，只比较同预算成功率与实际找到构造的时间，不把舍弃搜索空间后的快速停止叫穷尽加速。超时不等于无解，返回构造须完整算E、每个定义点先已知、目标全满足，再独立高精度回放。

## 本轮新增

1. **依赖链反向会合**（`chain_join.hpp`）：目标Q沿已知锚点反查虚拟M，再查另一锚点方向上的一笔可达H；真实依次作出H的见证、A2H、A3M以及必要的目标线。虚拟坐标仅检索，M/H必须在合法Apply后的已知点中取得。
2. **目标点双见证会合**（`point_join.hpp`）：将不同合法一笔操作产生的点按相对目标的无向角配对，再付费重放两见证与最后连线。支持已知点零成本、同见证只付一次、剩余1/2/3E预算。
3. **显式付费基础三笔**（`foundation.hpp`）：已知点对的连线和互为圆心的两圆，全部按普通1E操作计数；整体预检，失败不修改图。仅在小初图少量点对上提议，随后使用点会合验证完整解，不是免费宏工具。
4. **受限调度**：beam中优先的少量前缀调用结构补全；每次≤0.6秒、每worker累计≤6秒，根部基础三笔另有≤3秒预算。失败返回未知，不剪除父状态；`--no-structural`可禁用r3新增模块。

成功归因分别输出Beam/Helper/Rendezvous/Chain join/Point join，避免把后备DFS找到的解当作新启发式收益。

## 主要长任务
'''
labels={'t1_conditional_E5':'T1条件模型5E','t5_aux_J_E6':'T5辅助点J/6E','t2_OX_E4_control':'T2目标线/4E（防退化）',
        't7_directrix_E6':'T7给定准线后求D/6E','t7_aux_BV_E5':'T7辅助BV/5E','t8_early_centers_E7':'T8较早前缀双圆心/7E'}
for folder,title in [('windows_long_final','Windows单线程/90秒预算'),('linux_long_final','Linux单线程/90秒预算'),
                     ('linux_holdout_pilot','Linux独立留出任务/8线程/90秒预算')]:
    f=P/'benchmarks/r3'/folder/'results.json'
    if not f.exists():continue
    rows=json.loads(f.read_text())['runs']
    text+=f'\n### {title}\n\n| 任务 | 版本 | 成功/次数 | 各轮Search Time（秒） | r3命中来源 |\n|---|---|---:|---|---|\n'
    for name in dict.fromkeys(r['name'] for r in rows):
        for label in ['v10','v11-r2','v11-r3']:
            rr=[r for r in rows if r['name']==name and r['label']==label]
            if not rr:continue
            hits=[r for r in rr if r['status']=='QUOTA_REACHED']
            times=', '.join(f"{r['seconds']:.6f}"+(f" {r['status']}" if r['status']!='QUOTA_REACHED' else '') for r in rr)
            origin=', '.join('/'.join(k for k in ['beam','helper','rendezvous','chain_join','point_join'] if r.get(k)) for r in hits) if label=='v11-r3' else '—'
            text+=f"| {labels.get(name,name)} | {label} | {len(hits)}/{len(rr)} | {times} | {origin} |\n"
text+='''
## 必须保留的限制

- 结构模板由T1/T5的失败路径诊断启发，但没有题号、坐标或答案分支。它们是通用有界搜索模式，不是适用于所有几何构造的完备算法。
- T1/T2输入为历史条件模型，T5是辅助点，T7/T8是条件尾部；预算不能冒充原题整题的新E。
- 留出T7准线问题仍需约一分钟，r3没有显著加速；它提供反例，不能只报T1/T5突破。
- 结果用时低于采样周期时，RSS采样峰值不可靠；完整数值见JSON。
- 最大方向索引payload与整体RSS不同；预览/回放次数受cap和局部时限限制。局部超时不设置全局无解结论。

## 验证与复现

```bash
python tools/build.py --run-tests
python tools/compare_r3.py --r2 OLD_EXE --r3 NEW_EXE --manifest benchmarks/r3/long_final.json --out NEW_RESULTS --repeat 2 --seeds 1 2
python tools/verify_r3_runs.py --directory NEW_RESULTS --output verified.json
```

- Windows MinGW11.5和Linux GCC14.2实际构建通过；Linux ASan/UBSan下7组CTest全部通过。
- 新增r3测试12组连续通过，覆盖链补全、基础构造、点会合、预算/配额/取消/浮点负例；短单测耗时不用于性能宣传。
- `tests/r3_tests.cpp`：实际5E/6E完整证书、known-pair bitwise回放、闭网格、付费prefix/birth、预算、配额、取消、非有限/零圆等负例。
- `benchmarks/r3/*certificates.json`：独立100位回放，要求原EPS。
- 保留全部未命中、既有r2和试验版本结果，不能把short pilot代替最终长任务对照。
'''
(P/'docs/R3_PERFORMANCE.md').write_text(text,encoding='utf-8')
print('Wrote docs/R3_PERFORMANCE.md')
