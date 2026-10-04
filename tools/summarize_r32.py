#!/usr/bin/env python3
"""Summarize prerequisite-planner measurements, including near-tangent caveat."""
from pathlib import Path
import json

P=Path(__file__).resolve().parents[1];D=P/'benchmarks/r32'
labels={'eu10_2_external_tangent':'10.2 外公切线/8E（数值命中）',
        'eu9_7_minimum_perimeter':'9.7 最小周长内接三角形/8E',
        'eu10_2_external_tangent_E9_control':'外公切线/9E高精度控制',
        'eu13_5_equal_distance':'13.5定点与角边等距/8E',
        'eu14_5_apollonius':'14.5三相切圆/7E'}
text='''# v11-r3.2：目标前置结构规划

## 方向与边界

本轮由程序开发者选择方向：不再只按离目标近或产生点多来加分，而是把目标元素所缺的已知点用可付费的几何结构补齐，再只搜索真实可画的目标元素。新增直径圆前缀、镜像/位似/Thales提议和目标锁定尾部。所有结构都逐笔普通known-pair生成，保持root计费和birth，不免费添加解析中点、镜像点或切点。

新图仍采用公开解析代表坐标，不是复原截图游戏内部状态，详情见 `benchmarks/r32/MODEL_SCOPE.md`。两机均8线程、严格浮点portable x64，同输入/EPS/预算，原r3.1二进制冻结。最终两个新题三轮seed1/2/3，旧未解题用60秒对照。新图旧版实际较快，因此本轮不宣传基于新图短时间的普遍倍速；此前十秒以上/超时题继续单列。未测试T6。

## 一个重要发现：EPS成功不是精确外切线

旧r3.1的seed1外切8E证书，120位重放仍能满足原EPS=1e-11，但到两圆的距离残差约5e-12/8e-12，实际选取圆上点距离真切点约5.83e-6。这是合法操作的近切数值解，不能直接宣称精确相切。

随后seed2发现真正的泛型精确8E：先4E取得两圆外相似中心H，再以小圆O/r作圆(O,H)得W=2O-H，圆(P,H)得V=2P-H（P是已知朝H轴端），圆(W,V)与圆(O,H)交X，最后HX。WV=2r，HW是后圆直径，故WX垂直HX；O是HW中点，到HX距离WX/2=r，严格相切。所有P/W/V/X均来自真实已付费对象交点；不是免费中点或长度复制。

最终新版已实现这条通用四笔切线收尾，三个seed在两平台均得到精确结构8E，120位残差约1e-90。旧seed1近切证据、早期只知9E控制均保留，不把数值成功与严格几何混淆。9E控制仍作为旧路线，不能冒充8E；也不证明8E全局最少。

Fagnano的8E有独立的直径圆与反射恒等式支撑，完整证书高精度残差约1e-90。

## 实测结果
'''
for folder,title in [('windows_release_final','Windows i9-13900HK / 8线程 / 最终三种子'),
                     ('linux_release_final','Linux9950X / 8线程 / 最终三种子'),
                     ('windows_holdout_final','Windows旧未解题 / 8线程 / 60秒'),
                     ('linux_holdout_final','Linux旧未解题 / 8线程 / 60秒'),
                     ('linux_exact_control','Linux9E高精度控制')]:
    f=D/folder/'results.json'
    if not f.exists():continue
    runs=json.loads(f.read_text(encoding='utf-8'))['runs']
    text+=f'\n### {title}\n\n| 任务 | 版本 | 命中/次数 | 各轮秒数 | 新版命中来源 |\n|---|---|---:|---|---|\n'
    for name in dict.fromkeys(x['name'] for x in runs):
        for version in ['v10','r3.1','r3.2']:
            rr=[r for r in runs if r['name']==name and r['label']==version]
            if not rr:continue
            hits=[r for r in rr if r['status']=='QUOTA_REACHED']
            times=', '.join(f"{r['seconds']:.6f}"+(' 超时' if r['status']=='TIMEOUT_PARTIAL' else '' if r['status']=='QUOTA_REACHED' else ' '+r['status']) for r in rr)
            origins=', '.join('/'.join(k for k in ['beam','helper','coverage','prerequisite','equal_radius','rendezvous','chain_join','point_join'] if r.get(k)) for r in hits) if version=='r3.2' else '—'
            text+=f"| {labels.get(name,name)} | {version} | {len(hits)}/{len(rr)} | {times} | {origins or '—'} |\n"
text+='''
## 算法与资源控制

- `goal_finish.hpp`：先尝试真实目标元素序列，再枚举至多一个父状态known-pair辅助操作后补目标；不按missing-count硬剪，不把EPS近似当传递关系。失败只有Unknown。
- `diameter_probe.hpp`：互心圆、公共弦、必要的付费中心线、中点圆，最多真实4/5笔，不免费用中点。
- `homothety_probe.hpp` / `tangent_probe.hpp`：两个已知圆心圆的位似前缀、圆外点的四笔双弦切线结构以及Thales后备。最终精确外切为8E，9E控制是较早路线。
- `mirror_probe.hpp`：既有载体上两已知点为圆心作过已知P的圆，以真实另一交点取得镜像；本版保留为可测试前置模块，不宣称单独改善所有任务。
- 根部前置规划最多2秒或总剩余10%；beam只给前2个状态少量goal-finish预算并共用已有6秒累计结构限额。失败不会删除父状态。
- 候选预览先判交点是否能增加目标收益，再做HasPoint查重，省掉无关点扫描；数学/FP几何语义不变。
- 所有成功归因单列 `Prerequisite successful state visits`；不是旧DFS命中冒称新算法。

## 验证

新增前置结构12组和goal_finish正式测试，覆盖原图不可变、付费parent、预算/工具、circle-only镜像、已有圆少付费、finite/ray/segment、回滚、非有限、出生顺序、全局/局部deadline、quota多解、单笔同时满足多个近EPS目标；另覆盖泛型四步切线的半径/距离/旋转/平移及双圆完整8E。

两平台均实际编译，Linux ASan/UBSan下12组CTest全部通过。Windows旧斜边切点长任务单轮13.79秒→10.42秒，既有T1/T5/三等线段都保持合法命中；这些防退化单轮不宣传普遍倍速。

原有r2/r3/coverage/Novelty回归不删。证书采用独立mpmath120位回放，并额外报告1e-60强数值残差判据与切线距离残差；强判据不是符号证明。本版仍是EPS数值搜索器，任何超时不能包装成无解。
'''
(P/'docs/R32_PERFORMANCE.md').write_text(text,encoding='utf-8');print('Wrote docs/R32_PERFORMANCE.md')
