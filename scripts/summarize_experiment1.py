#!/usr/bin/env python3
"""Reduce experiment 1's official runs to the mean +/- sd table in docs/experiment.md.

    scripts/summarize_experiment1.py [backend ...]

Reads measurements/experiment1/ (and its real/ subdirectory) and prints one
table per backend: the three numbers a tape measure gives, averaged over the
repeats of each sequence.

`sweep` is deliberately NOT summarised. It ends at a pose no single measurement
describes -- that is why line/spin/square exist -- so averaging its net pose
says nothing. Its value is the per-phase command -> wheel comparison inside
each file.
"""
import json
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / 'measurements' / 'experiment1'
ORDER = ['line', 'spin_cw', 'spin_ccw', 'square_cw', 'square_ccw']
NOMINAL_YAW = {'line': 0, 'spin_cw': -720, 'spin_ccw': 720,
               'square_cw': -360, 'square_ccw': 360}


def runs(backend):
    """Every official file for one backend, wherever it lives."""
    out = {}
    for f in sorted(list(ROOT.glob(f'*_exp1_{backend}_*.json'))
                    + list((ROOT / 'real').glob(f'*_exp1_{backend}_*.json'))):
        if f.name.endswith('.samples.json'):
            continue
        d = json.loads(f.read_text())
        out.setdefault(d['sequence_name'], []).append((f.name, d))
    return out


def stat(vals):
    # sd of one sample is undefined, not zero -- say so rather than print 0.0000.
    return st.mean(vals), (st.pstdev(vals) if len(vals) < 2 else st.stdev(vals))


def table(backend):
    by_seq = runs(backend)
    if not by_seq:
        print(f'{backend}: no runs\n')
        return
    srcs = {d['net']['source'] for v in by_seq.values() for _, d in v}
    print(f'## {backend}   net.source: {", ".join(sorted(srcs))}')
    print('| sequence | n | forward m | lateral m | yaw deg | nominal yaw deg |')
    print('|---|---|---|---|---|---|')
    for seq in ORDER + sorted(set(by_seq) - set(ORDER) - {'sweep'}):
        if seq not in by_seq:
            continue
        rs = [d['net'] for _, d in by_seq[seq]]
        n = len(rs)
        (fm, fs), (lm, ls), (ym, ys) = (stat([r[k] for r in rs])
                                        for k in ('forward_m', 'lateral_m', 'yaw_deg'))
        print(f'| `{seq}` | {n} | {fm:+.4f} ± {fs:.4f} | {lm:+.4f} ± {ls:.4f} '
              f'| {ym:+.2f} ± {ys:.2f} | {NOMINAL_YAW.get(seq, "")} |')
    if 'sweep' in by_seq:
        print(f'\n`sweep` x{len(by_seq["sweep"])}, not summarised (see the docstring).')
    print()


for b in sys.argv[1:] or ['gazebo', 'isaacsim', 'real']:
    table(b)
