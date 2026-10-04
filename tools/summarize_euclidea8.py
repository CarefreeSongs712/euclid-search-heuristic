#!/usr/bin/env python3
"""Report same-input two-machine three-version Euclidea representative results."""
from pathlib import Path
import json

P=Path(__file__).resolve().parents[1]
D=P/'benchmarks/euclidea8'
manifest=json.loads((D/'manifest.json').read_text(encoding='utf-8'))
versions=['v10','v11-r2','v11-r3']
text='''# 八道Euclidea题型：Windows / Linux三版本对照

## 测试口径与必要限制

- Windows：i9-13900HK；Linux：用户指定的9950X。两台机器均使用**8线程**，不是一边8线程另一边32线程。
- 每题每版本60秒，配额1，EPS=1e-11；r2/r3 seed=1，单轮长预算测试。按题轮换版本顺序，同机不同时运行不同版本。
- 对照程序是冻结的v10、v11-r2、v11-r3通用x64构建；不使用native版本，记录二进制SHA256。
- 按截图E预算8/7/8/7/7/9/6/8，新直线或普通圆各1E；不比较L宏工具次数。
- **截图没有精确坐标，因此本次是题意一致的解析数值代表实例，不是恢复游戏内部状态。** 所有三版本、两机器用完全相同输入字节。
- 射线、有限线段不免费扩展；题14.3的灰内切圆不是给定对象；3°题显式提供一个在初始射线上任取的尺度点，因为程序不支持自动取自由点。
- 多解问题固定一整套合法分支，不把镜像分支混作同时目标。成功表示找到这一支；没找到不代表游戏没有别支解。
- 输入定义、初始可选点、解析公式与分支全部公开在 `benchmarks/euclidea8/MODEL_SCOPE.md`；110位目标约束已核验。

## 用时与结果

“超时”表示60秒未达到解配额；不视为无解，不用于计算精确求解倍速。表内秒数为程序Search Time，短任务单轮只能记录，不能靠百分之几差距判断谁更优。
'''
for folder,title in [('windows_8t_60s','Windows i9-13900HK / 8线程'),('linux_8t_60s','Linux 9950X / 8线程')]:
    path=D/folder/'results.json'
    if not path.exists():continue
    report=json.loads(path.read_text(encoding='utf-8'));runs=report['runs']
    lookup={(r['name'],r['label']):r for r in runs}
    text+=f'\n### {title}\n\n| 题型 | E上限 | v10 | r2 | r3 |\n|---|---:|---:|---:|---:|\n'
    for case in manifest:
        cells=[]
        for version in versions:
            r=lookup.get((case['name'],version))
            if r is None:cells.append('未完成');continue
            if r['status']=='QUOTA_REACHED':
                cells.append(f"{r['seconds']:.3f}s / {','.join(map(str,r['returned_E']))}E")
            elif r['status']=='TIMEOUT_PARTIAL':cells.append(f"超时 {r['seconds']:.1f}s")
            else:cells.append(f"{r['status']} {r['seconds']:.3f}s")
        text+=f"| {case['title']} | {case['E']} | {' | '.join(cells)} |\n"
    successes={v:sum(r['status']=='QUOTA_REACHED' for r in runs if r['label']==v) for v in versions}
    text+='\n成功题数：'+ '；'.join(f'{v} {successes[v]}/8' for v in versions)+'。\n'
    text+=f'原始证据：`benchmarks/euclidea8/{folder}/results.json`，完整stdout/stderr同目录。\n'
text+='''
## 结论

- 两平台结论一致：v10成功4/8，r2成功5/8，r3成功5/8。r3在这组新题上没有比r2多解出题目，不能延用此前特定T1/T5实例的数量级提速结论。
- 14.1内接菱形、14.3斜边切点题中，r2/r3命中而v10超时；14.3约11–13秒，是本轮较有意义的成功长任务对照。
- 15.4三等线段由v10在Windows20.04秒、Linux11.75秒找到，r2/r3却均60秒超时，显示启发式可能显著失利。
- 13.5等距点、14.5三相切圆三版均超时；没有完成穷尽，不能说无解。
- 3°、圆桌台球、同心圆等边三角形都命中；前两者v10更快，但这些短任务仅单轮记录，不作总体排名依据。
- r2/r3的成功源统计在原始stdout/JSON保留；同样的命中不等于采用同一构造路径，不能用节点比替代找到解用时。

## 独立验证

对于r2/r3，回放程序导出的17位端点见证；v10不支持JSON证书，将中文可读报告中的17位定义点提取成相同证书格式，并标记source_version。随后以独立mpmath 100位算术从原始given重算所有步骤，检查端点必须先已知、实际E不超预算、最终目标满足原EPS。格式转换不改变被测程序，不使用raw-output的12位坐标作为准确证书。

两台机器总共48次运行，28次命中均已逐条通过100位独立回放及原EPS检验，结果分别在 `windows_verified.json`、`linux_verified.json`。EPS验证通过仍不是任意位置的符号证明；本测试不宣称全局最少E或数学无解。

## 复现

```bash
python tools/make_euclidea8.py
python tools/compare_euclidea8.py --v10 V10 --r2 R2 --r3 R3 --out NEW_DIRECTORY --threads 8 --seconds 60 --seed 1
python tools/verify_euclidea8.py --directory NEW_DIRECTORY --output verified.json
```

Linux用python3。每个版本同题达到配额即停止，未完成则由各自60秒限制退出。`--case NAME`可按题分批，`--resume`只允许同二进制hash与测试参数继续未完成行，不重复覆盖已完成证据。

本轮只测这些新题，不修改搜索算法或针对题目调参，未测试此前用户排除的T6。
'''
(P/'docs/EUCLIDEA8_COMPARISON.md').write_text(text,encoding='utf-8')
print('Wrote docs/EUCLIDEA8_COMPARISON.md')
