#!/usr/bin/env python3
"""Report revision comparison with explicit success/censoring and origin."""
from pathlib import Path
import json
import statistics

P=Path(__file__).resolve().parents[1]
text='''# v11-r2：困难长任务验证

## 版本与测试口径

r1指上一次交付的v11启发式程序；r2是本次改进。保留r1二进制作为真实对照，没有把旧版重新编译成较慢配置。

按用户要求，主要证据选择旧版运行超过10秒或60秒仍未命中的困难任务；不以毫秒微基准宣传普遍倍速。新版若找到更快，不人为拖长它。每次任务同输入、EPS、线程数、时间上限；不同seed和执行顺序都保留。超时是删失数据，只能给成功率或改善下限，不能把60秒当作旧版真实完成耗时。本次没有测试T6。

## 新算法

- **有见证的反向会合搜索**：预计算一笔可生成的点及其真实构造见证，将目标载体上的候选交点与已知点方向索引配对；得到建议后从原图逐步重放全部合法操作，最后GoalsMet验证。不是免费加入目标交点，也不靠题名/坐标特判。
- **反向结构地标评分**：既有圆的对径关系、目标线与载体交点及有界两层圆弦配对。只用于评分，部分portfolio启用；`--landmarks`可强制启用，`--no-adaptive --landmarks --tail-candidates=0`可测纯beam地标策略。
- **构造家族多样性**：同一元素集合的不同顺序不再挤满全部精英槽；保留选中路径原顺序。这是显式启发式丢弃，不宣称浮点状态等价。
- **真正前置条件预览**：在branch截断前查看候选实际能生成的目标相关交点，而不是只在截断后评分；减弱“点越多越好”的误导。
- **持续后备线程**：多线程、助手启用时，worker0完成首次beam后用一个持续DFS兜底，其余worker继续启发式。不会反复从头跑短切片。成功来源分开计数，不把旧DFS命中伪装成纯启发式收益。

## 主长测结果
'''
labels={'t2_OX_E4_1t':'T2已知圆心求OX，4E/1线程',
        't1_E5_8t':'T1条件构造，5E/8线程','t5_J_E6_8t':'T5辅助点J，6E/8线程',
        't5_J_E6_1t':'T5辅助点J，6E/1线程'}
for group,label in [('windows_long_final','Windows i9-13900HK'),('linux_long_final','Linux 9950X'),
                    ('linux_t5_three_way','Linux T5长时单线程三版对照（120秒预算）')]:
    path=P/'benchmarks/r2'/group/'results.json'
    if not path.exists():continue
    r=json.loads(path.read_text())['runs']
    text+=f'\n### {label}\n\n| 任务 | 版本 | 成功/运行 | 成功耗时（秒） | 结果来源 |\n|---|---|---:|---|---|\n'
    for name in dict.fromkeys(x['name'] for x in r):
        for version in ['v10','v11-r1','v11-r2']:
            rows=[x for x in r if x['name']==name and x['label']==version]
            if not rows:continue
            done=[x for x in rows if x['status']=='QUOTA_REACHED']
            seconds=', '.join(f"{x['seconds']:.3f}" for x in done) or '—'
            failed=[x for x in rows if x['status']!='QUOTA_REACHED']
            if failed: seconds+='；'+', '.join(f"{x['seconds']:.0f}s {x['status']}" for x in failed)
            origins=[]
            if version=='v11-r2':
                for x in done:
                    origins.append('/'.join(k for k in ['beam','helper','rendezvous'] if x.get(k)) or 'unknown')
            text+=f"| {labels.get(name,name)} | {version} | {len(done)}/{len(rows)} | {seconds} | {', '.join(origins) if origins else '—'} |\n"
text+='''
不同算法的Nodes不是同一单位工作量，本报告不用节点比声称求解倍速。T1/T2输入是既有条件模型，T5为辅助点，不应把测试预算当完整原题的新E。

## 结论与没有改善的部分

- T2是本次主要结构突破：旧r1两次各60秒未命中；v10 Linux单线程约49.7秒完成；r2约0.07–0.11秒完成。不是因小任务波动得出的结论，而是旧搜索真正持续几十秒/超时，而新版采用不同结构搜索避免枚举。只对这个结构实例成立，不宣传通用几百倍。
- T1 seed2从r1的60秒未命中变成约4秒（Linux）/10秒（Windows）命中，但主要来自持续DFS后备，不是纯beam变聪明的证明；seed1反而可能慢于旧r1。
- T5单线程三版对照：v10稳定在10.87/10.84秒；v11-r1与r2都在seed1约29秒、seed2约0.72秒，两者几乎重合。r2相对r1没有普遍改善，v10在该类输入上仍是最稳的。初版r2曾退化到43.98秒，已按目标类型禁用无收益的新评分，保留失败日志 `linux_t5_single_long`，不抹掉退化记录。
- 主表8线程T5是该开关之前的诊断版本，不能当作最终单线程性能承诺；最终三版对照单列，算法是否更好以成功率及同预算证书为准。

## 证据与复现

- `benchmarks/r2/windows_long_final`、`linux_long_final`、`linux_t5_three_way`：所有轮次原始结果、日志、RSS采样与构造证书。
- `tools/compare_revisions.py`：交错r1/r2/v10测量，可选seed与重复次数。
- `tests/r2_tests.cpp`：家族池顺序、容量、不同排列非等价反例、会合重放、配额、取消与初始图契约。
- `tools/verify_certificate.py`：独立高精度回放，不调用C++搜索。

## 边界

r2仍是不完备启发式，不保证所有输入加速、找到最短解或找到任何给定解。方向检索窗口、候选上限、地标上限都只影响建议覆盖率；所有命中都真实重放后验证。会合方向索引payload上限32MiB，不是整个进程RSS上限。搜索失败仍HEURISTIC_STOPPED/TIMEOUT_PARTIAL，不是数学无解。

`--no-adaptive --no-landmarks`禁用r2新增策略，用于消融；这不保证与旧二进制逐时间/机器行为完全一致。
'''
(P/'docs/R2_PERFORMANCE.md').write_text(text,encoding='utf-8')
print('Wrote docs/R2_PERFORMANCE.md')
