#!/usr/bin/env python3
"""Assemble the single release zip: programs, source, docs, evidence; no git."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BS11 = ROOT
BS10 = ROOT.parent / 'bs_optimized'
OUT = ROOT.parent / 'bs_v11_r32_release.zip'
STAGING = 'bs_v11_r32_release'

INCLUDE_V11_DIRS = ['src', 'tests', 'tools', 'docs', 'examples', 'benchmarks', 'dist']
INCLUDE_V11_FILES = ['README.md', 'AGENTS.md', 'CMakeLists.txt']
INCLUDE_BENCH_FILES = ['v11_acceptance.json', 'final_windows_release_acceptance.json',
                       'final_linux_acceptance.json', 'heuristic_manifest.json',
                       't8_axis_initial_certificate.json', 't8_axis_certificate_verified.json',
                       't7_final_certificate.json', 't7_final_certificate_verified.json',
                       'midpoint_certificate_verified.json', 'certificate_verifier_selftest.json',
                       'windows_final_solvable', 'linux_final_solvable', 'windows_final_t7',
                       'linux_final_t7', 'linux_final_pure_beam', 'linux_t6_final']
EXCLUDE_PARTS = {'.git', 'build', '__pycache__', '.github'}

def add_file(zip_handle, arcname, source):
    data = source.read_bytes()
    info = zipfile.ZipInfo(STAGING + '/' + arcname)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o755 << 16 if not arcname.endswith(('.exe', '.md', '.txt', '.json')) else 0o644 << 16
    zip_handle.writestr(info, data)
    return len(data)

total = 0
files = []
added = set()
def add_unique(zip_handle, arcname, source):
    global total
    if arcname in added:
        return 0
    added.add(arcname)
    total_local = add_file(zip_handle, arcname, source)
    total += total_local
    files.append(arcname)
    return total_local

with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
    # Top-level quick guide (Chinese) pointing into the tree.
    guide = f'''# BS v11-r3.2 启发式几何搜索 交付包

生成日期：2026-10-04。本包不含任何 git 数据。

## 最快上手

- Windows 程序：`v11/dist/windows-x64/bs_v11.exe`（静态链接，无需额外 DLL）
- Linux x64 程序：`v11/dist/linux-x64/bs_v11`（先 `chmod +x`）

```bash
bs_v11.exe --threads=8 --solutions=1 --time-limit=120 < input.in > result.out
```

- 进度走 stderr，结果走 stdout；完整参数见 `v11/README.md`。
- v11-r3.2新增目标元素前置结构与目标锁定补全；不保证搜全，未找到不是无解。
- 成功来源按Prerequisite/Beam/Helper/Coverage及既有探测分别统计，所有实际操作真实已知点定义、正常计E。
- `--no-prerequisites`关闭本轮前置规划；`--coverage-threads=0`禁用共享覆盖组；`--no-structural`关闭全部结构探测。
- 外切线旧seed1有EPS近切风险，最终结构版有已证明的精确8E；请读R32报告区分数值验收与几何证明。
- 需要旧的完整枚举：加 `--search=exhaustive`。

## 旧版本

- `v10/windows-x64/bs_v10.exe`、`v10/linux-x64/bs_v10`：先前优化版（完整DFS）。
- `v10/linux-x64/bs_v10.1`：9950X 专用 native+LTO 构建，仅限该类CPU。
- 命名与哈希说明：`v10/VERSION_NAMES.md`。

## 文档

- `v11/README.md`：使用说明、参数、结果状态。
- `v11/docs/R32_PERFORMANCE.md`：本版两新图双平台测试、成功来源与近切精度审核。
- `v11/benchmarks/r32/MODEL_SCOPE.md`：新题解析模型与泛型精确8E见证。
- `v11/docs/R31_PERFORMANCE.md`：此前版本的长题与未解结果，保留作为历史对照。
- `v11/docs/EUCLIDEA8_COMPARISON.md`：原八道截图题的模型和旧版完整结果。
- `v11/docs/R3_PERFORMANCE.md` / `R2_PERFORMANCE.md` / `PERFORMANCE.md`：历史性能报告，保留失败样本。
- `v11/docs/DESIGN.md`：启发式设计与边界。
- `v11/AGENTS.md`：开发接手说明。

## 验证与复现

- `v11/tools/`：构建、回归、版本对比、证书独立高精度回放（需 mpmath）。
- `v11/benchmarks/`：全部输入与保留的原始运行结果（含超时/失败）。
- 校验和：`v11/dist/SHA256SUMS`、`v10/SHA256SUMS`。

重要：本工具是数值几何搜索器；EXHAUSTED 也不是数学无解证明，重要构造请用
`v11/tools/verify_certificate.py` 独立回放。
'''
    info = zipfile.ZipInfo(STAGING + '/使用说明.md')
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    z.writestr(info, guide.encode('utf-8'))
    total += len(guide.encode('utf-8'))
    files.append('使用说明.md')

    for name in INCLUDE_V11_FILES:
        add_unique(z, f'v11/{name}', BS11 / name)
    for folder in INCLUDE_V11_DIRS:
        for path in sorted((BS11 / folder).rglob('*')):
            if not path.is_file():
                continue
            rel = path.relative_to(BS11)
            if EXCLUDE_PARTS & set(rel.parts):
                continue
            if path.suffix in {'.pyc', '.o', '.obj', '.exe'} and folder != 'dist':
                continue
            if rel.parts[0] == 'benchmarks' and 'large' in rel.parts and path.name == 'results.json':
                pass  # keep evidence
            if 'large' in rel.parts and rel.parts[2] == 'windows_8t':
                continue  # aborted T6 partial run, excluded at user request
            add_unique(z, f'v11/{rel.as_posix()}', path)
    for name in INCLUDE_BENCH_FILES:
        path = BS11 / 'benchmarks' / name
        if path.is_file():
            add_unique(z, f'v11/benchmarks/{name}', path)
        elif path.is_dir():
            for sub in sorted(path.rglob('*')):
                if sub.is_file() and sub.suffix == '.json':
                    add_unique(z, f"v11/benchmarks/{name}/{sub.relative_to(path).as_posix()}", sub)

    # Preserved v10/v10.1 binaries and naming note.
    v10_root = BS10 / 'dist'
    for rel in ['windows-x64/bs_v10.exe', 'linux-x64/bs_v10', 'linux-x64/bs_v10.1',
                'VERSION_NAMES.md', 'VERSIONS.json']:
        source = v10_root / rel
        if source.is_file():
            add_unique(z, f'v10/{rel}', source)
    sums=[]
    for rel in ['windows-x64/bs_v10.exe','linux-x64/bs_v10','linux-x64/bs_v10.1']:
        sums.append(hashlib.sha256((v10_root/rel).read_bytes()).hexdigest()+'  '+rel)
    z.writestr(STAGING+'/v10/SHA256SUMS','\n'.join(sums)+'\n')
    files.append('v10/SHA256SUMS')

manifest = {'files': len(files), 'bytes_uncompressed': total,
            'entries': files,
            'zip_sha256': hashlib.sha256(OUT.read_bytes()).hexdigest()}
(BS11 / 'build' / 'release_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
print(f'{OUT} ({OUT.stat().st_size/1048576:.1f} MiB, {len(files)} files)')
