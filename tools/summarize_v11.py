#!/usr/bin/env python3
"""Publish a bounded-benchmark comparison, retaining all censored failures."""
from pathlib import Path
import json
import statistics

P=Path(__file__).resolve().parents[1]
labels={'t1_conditional_E5':'T1条件模型5E','t5_aux_J_E6':'T5辅助点J，6E',
        't2_givenO_OX_E4':'T2已知圆心求OX，4E','t8_centers_tail4':'T8双圆心尾部4E',
        't8_axis_centers_tail4':'T8已知轴后双圆心4E',
        't7_point_tail3':'T7目标点尾部3E','t7_point_line_tail3':'T7点+目标线尾部3E'}
text='''# v11 实测：求解速度、成功率和边界

v10是先前优化DFS版，v10.1是其9950X native构建，v11是新的启发式搜索。以下不是“穷尽速度”比较，而是同预算内实际达到解配额的耗时与成功率。Windows本机i9-13900HK、Linux9950X；GCC严格浮点portable构建；相同EPS与输入。线程数均按各表说明。种子1/2/3，交错次序，未命中运行保留为超时，不从统计中藏掉。

成功耗时是Search Time（包含启动/退出），接近首次解时间而非单独插桩的精确发现时刻。只有双方全部成功时给中位数比；少一次成功也不把成功样本均值称作普遍倍速。
'''
for directory,title in [('windows_final_solvable','Windows，8线程，15秒预算'),
                         ('linux_final_solvable','Linux，8线程，15秒预算'),
                         ('windows_final_t7','Windows T7，8线程'),
                         ('linux_final_t7','Linux T7，8线程'),
                         ('linux_final_pure_beam','Linux纯beam（禁用全部旧尾助手），8线程')]:
    path=P/'benchmarks'/directory/'results.json'
    if not path.exists():continue
    rows=json.loads(path.read_text())['runs']; names=list(dict.fromkeys(x['name'] for x in rows))
    text+=f'\n## {title}\n\n| 输入 | v10成功 | v11成功 | v10成功中位秒 | v11成功中位秒 | 全成功时中位比 |\n|---|---:|---:|---:|---:|---:|\n'
    for name in names:
        a=[x for x in rows if x['name']==name and x['version']=='v10']
        b=[x for x in rows if x['name']==name and x['version']=='v11']
        aa=[x['seconds'] for x in a if x['status']=='QUOTA_REACHED']
        bb=[x['seconds'] for x in b if x['status']=='QUOTA_REACHED']
        ma=statistics.median(aa) if aa else None
        mb=statistics.median(bb) if bb else None
        ratio=f'{ma/mb:.2f}×' if len(aa)==len(a)>0 and len(bb)==len(b)>0 and mb else '不计算'
        sa=f'{ma:.6f}' if ma is not None else '—'
        sb=f'{mb:.6f}' if mb is not None else '—'
        text+=f'| {labels.get(name,name)} | {len(aa)}/{len(a)} | {len(bb)}/{len(b)} | {sa} | {sb} | {ratio} |\n'
text+='''
## 大型问题对照（60–120秒预算，找到解才算成功）

Windows本机8线程、90秒预算；Linux 9950X 32线程、120秒预算。每例单轮，交错次序。这里比的是真实找到解的能力和用时，不是节点吞吐。

| 输入 | 平台 | v10 | v11 |
|---|---|---|---|
| T2已知圆心求OX，4E | Windows 8线程 | **13.0秒找到** | 90秒超时未找到 |
| T1条件模型5E（seed2） | Windows 8线程 | **6.6秒找到** | 90秒超时未找到 |
| T5辅助点J，6E | Windows 8线程 | **2.5秒找到** | 17.2秒找到（慢约7倍） |
| T2已知圆心求OX，4E | Linux 32线程 | **3.2秒找到** | 120秒超时未找到 |
| T1条件模型5E（seed2） | Linux 32线程 | 5.7秒找到 | **3.5秒找到（快1.6倍）** |

同一输入跨平台结论并不一致：T1在Linux上v11更快，在Windows 90秒内未命中，说明启发式结果对seed/线程/预算敏感，不能按单次表现外推。

这批结果明确：**v11在更大、更深的真实任务上并不稳定优于v10**。两个超时说明目标引导在该类结构上会把beam引向错误区域；T5在Windows上找到但比v10慢约7倍；T1在Linux快1.6倍但在Windows超时，波动来自seed/线程/平台。此前小/中型有解实例的3–5倍优势不能外推。原始运行、证书与日志全部保留在 `benchmarks/large/`。

应用建议：先用v11短预算（例如30–60秒）尝试；未命中或深问题再切v10完整搜索。两个引擎共享输入格式，`--search=exhaustive` 即可切换。

用户在测试进行中要求停止T6；本节不含T6数据，避免以中断的残缺对照下结论。

## 如何读结果

- 对有解的某些结构任务，目标引导可以省掉大量不相关搜索；但不同seed成功率并不恒定，且部分输入明显比v10慢或超时。
- T1/T2若干输入来自历史条件模型（包括已经给定的支持线或圆心），只用于计算对照，不能把预算当当前严格原题的新最少E。
- T5 J是中间辅助点任务，T8/T7是合法前缀后的尾部，不是宣称整题4E/3E。
- 默认模式使用beam+限时尾助手；统计单独列Beam/Helper successful state visits。助手成功不包装为纯beam。纯beam表禁用了所有旧助手，展示真实启发式能力。
- `HEURISTIC_STOPPED`表示尝试停止，`TIMEOUT_PARTIAL`表示预算到；两者都不能证明无解。不同算法节点不可当作相同工作量。

## 验证

- Windows MinGW11.5和Linux GCC14.2实际编译、5组CTest/单测通过。最终两平台各34次CLI用例、9个证书、10项CLI契约均通过；单线程/4线程各12个规定有解案例全找到，另外的真实困难基准成功率如上表单列，不混为100%。
- 专用单测检查每个结果的已知点来源与元素原始浮点位，覆盖模式0/1/2/3、E限制、seed、线程1/4、partial quota、超时、资源截断和旧下界两类EPS反例。
- 大量EPS在线点对的尾助手取消检查、linked global stop、quota/timeout互斥和异常join已补充。
- JSON证书导出需为每一步找到真实已知点的bitwise构造见证，回放全部目标后才写文件。
- Linux ASan+UBSan 构建下5组CTest全部通过，无检测报告。
- 独立mpmath证书验证器不调用solver，自测38项；真实T8四步、T7三步及中点四步证书100位回放通过原EPS目标检查，但仍标exact_proof=false，不能当泛型代数证明。
- 原v10/v10.1发行文件保留，命名副本SHA256与原测试字节一致。

## 深问题与资源边界

已在9950X使用32线程、seed1、分别60秒测试T6严格整题8E和11E；v10、v11四次均TIMEOUT_PARTIAL且未找到解。这意味着本版尚未改善该深问题的实际成功结果，不是证明8E无解，也不是否定已有11E人工构造。原始数据位于 `benchmarks/linux_t6_final/results.json`。不同算法节点计数不用于宣称等工作量吞吐倍速。

启发式可以对深问题快速挑选路线，但不保证T6八步解存在或能找到，任何超时仍是未知。beam/branch只约束前沿候选存储，不是进程总内存硬上限；初始图、点数、深度和解收集仍占内存。停止为协作式，单次特别巨大的几何Apply可能延迟响应。

所有原始运行、未命中和初版退化记录保留在benchmarks。最终报告不以初版偶然快例或单个seed的数十倍结果概括所有任务。
'''
(P/'docs/PERFORMANCE.md').write_text(text,encoding='utf-8')
print('Wrote docs/PERFORMANCE.md')
