#!/usr/bin/env python3
"""Summarize improved r3's measured outcomes without censoring failures."""
from pathlib import Path
import json

P=Path(__file__).resolve().parents[1]
D=P/'benchmarks/r31'
labels={'eu15_4_three_equal':'15.4 三条相等线段（7E）',
        'eu14_3_hypotenuse_touch':'14.3 斜边切点作直角三角形（9E）',
        'eu13_5_equal_distance':'13.5 定点与角边等距（8E）',
        'eu14_5_apollonius':'14.5 三相切圆（7E）',
        'eu14_1_inscribed_rhombus':'14.1 内接菱形（8E）'}
text='''# r3继续优化：v11-r3.1 对比报告

## 口径

本轮继续改进r3而不是重命名成v12。旧r3源与二进制冻结在build/revision_r3；新程序输出v11-r3.1。Windows为i9-13900HK、Linux为9950X，均8搜索线程，总线程没有偷偷增加。普通portable x64、严格浮点，相同模型/EPS/预算，60秒，轮换顺序和seed1/2。重点选择旧版十秒以上或到时仍未命中的题；新算法若很快命中不人为拖慢。

截图题是公开的解析数值代表实例，不是复原游戏内部精确坐标，固定目标分支；详见benchmarks/euclidea8/MODEL_SCOPE.md。本轮不改输入，不针对题号或坐标硬编码答案，不测试T6。

## 改了什么

1. **同半径依赖链探测**：真实普通圆与直线C–C–L–C产生候选前缀，真实第五笔补出目标/目标元素前置点后短尾求解。全部known-pair、实际付费E，等半径关系来自既有点和圆交点，不是免费compass。关闭tail helper时不会偷用尾搜。
2. **真正按构造家族抽样**：NoveltyPool在入口保留高分家族及稳定随机家族，低分族不会先被top4×beam截掉；同族多种排列不会多买随机票。full raw key判等，保留真实顺序。这仍是启发式资源选择，不证明状态等价。
3. **共享前缀覆盖组**：原一个单线程DFS保险改成有界队列producer/consumers，每子树独占，失败不丢改为继续覆盖。覆盖和启发式合计等于threads；默认高E更多线程留给beam，避免所有核押一种算法。Coverage命中单独计数，不冒充beam。
4. **取消/资源修复**：队列20ms节拍响应纯atomic stop、finite restarts不转持续DFS、结构助手尊重tail开关，参数提前验证，异常全部join后上抛。

## 主要长题结果
'''
for folder,title in [('windows_long_final','Windows 8线程，两种子'),('linux_long_final','Linux 8线程，两种子'),
                     ('windows_unsolved_holdout','Windows 未解题保留测试'),('linux_unsolved_holdout','Linux 未解题保留测试')]:
    f=D/folder/'results.json'
    if not f.exists():continue
    rows=json.loads(f.read_text(encoding='utf-8'))['runs']
    text+=f'\n### {title}\n\n| 题目 | 版本 | 成功/次数 | 各轮秒数/状态 | 新版命中来源 |\n|---|---|---:|---|---|\n'
    for name in dict.fromkeys(r['name'] for r in rows):
        for label in ['v10','r3-old','r3.1']:
            rr=[r for r in rows if r['name']==name and r['label']==label]
            if not rr:continue
            hits=[r for r in rr if r['status']=='QUOTA_REACHED']
            times=', '.join(f"{r['seconds']:.6f}"+(' 超时' if r['status']=='TIMEOUT_PARTIAL' else '' if r['status']=='QUOTA_REACHED' else ' '+r['status']) for r in rr)
            origin=', '.join('/'.join(k for k in ['beam','helper','coverage','equal_radius','rendezvous','chain_join','point_join'] if r.get(k)) for r in hits) if label=='r3.1' else '—'
            text+=f"| {labels.get(name,name)} | {label} | {len(hits)}/{len(rr)} | {times} | {origin} |\n"
text+='''
## 结论与限制

- 三等线段是本轮明确突破：旧r3 seed1超时，新结构探测直接找到可重放7E。新版命中很短，因此只声明解决了旧长搜索瓶颈，不把微秒/毫秒倒数包装成普遍万倍提速。
- 斜边切点题至少一轮旧程序约11–15秒，是有参考价值的长成功任务；根据两seed结果判断，不选最快一次宣传。
- 初版coverage比例过大曾令斜边切点题从约11秒退到43秒；已据目标E降低coverage占比，失败pilot仍保留，不能隐瞒。
- 13.5/14.5仍按原预算单列，全部超时不是无解，也不能用更多节点声称求解加速。
- 默认仍混合启发式与DFS，Coverage/Helper成功均如实标注；新结构模式不保证覆盖全部题型。

## 验证

- 证书由真实存储点逐笔bitwise重建验证；独立mpmath100位回放要求满足原EPS。
- 新coverage测试9组，equal-radius结构测试，NoveltyPool固定seed离线oracle792组及哈希碰撞/重复家族公平性、取消计数测试。
- 两机完整C++/CLI验收；Linux另做ASan/UBSan。原有几何、r2/r3见证回放继续保留。
- 原始轮次、构造证书、二进制SHA256在benchmarks/r31，各版本输入完全相同。

## 用法

```bash
bs_v11.exe --threads=8 --solutions=1 --time-limit=120 < input.in > result.out
```

`--coverage-threads=0`关闭共享覆盖组；显式N包含producer且必须2..threads-1。只有adaptive、tail助手打开、无限重启、E>=4及线程>=4时启用。`--tail-candidates=0`关闭旧尾搜和coverage；`--no-structural`关闭结构探测；`--search=exhaustive`仍为旧DFS路径。`HEURISTIC_STOPPED/TIMEOUT_PARTIAL`不表示无解。
'''
(P/'docs/R31_PERFORMANCE.md').write_text(text,encoding='utf-8')
print('Wrote docs/R31_PERFORMANCE.md')
