#!/usr/bin/env python3
"""Accuracy benchmark: runs the real firmware headless over many scenarios and
random seeds, in parallel, and prints accuracy for both directions and in total.

    python3 bench.py                       # all scenarios, 3 seeds x 10 min
    python3 bench.py --seeds 5 --seed-start 100 --scenarios sparse,dense
"""
import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
SCENARIOS = ["sparse", "normal", "rush", "dense", "festival", "signal", "waiting", "mixed", "traffic"]
DESCRIPTIONS = {
    "sparse": "4 people/min, walking alone",
    "normal": "14/min, 25% groups, some vehicles",
    "rush": "36/min, groups, some vehicles",
    "dense": "75/min continuous flow, 40% groups",
    "festival": "140/min crowd, 50% groups",
    "signal": "platoons of ~45 every 60 s + traffic",
    "waiting": "30% stop on the strip 3-25 s",
    "mixed": "18 people/min + 10 vehicles/min, 20% stop on strip",
    "traffic": "vehicles only (cars, motos, autos, buses, trucks)",
}


def run_one(fw, scenario, seed, duration):
    exe = os.path.join(HERE, "bin", "zebra_" + fw)
    out = subprocess.run([exe, "--bench", "--scenario", scenario, "--duration", str(duration), "--seed", str(seed)],
                         capture_output=True, text=True, timeout=900)
    line = out.stdout.strip().splitlines()[-1]
    return json.loads(line)


def run_suite(fws, scenarios, seeds, duration, workers=None, progress=None, seed_start=1):
    jobs = [(fw, sc, sd) for fw in fws for sc in scenarios for sd in range(seed_start, seed_start + seeds)]
    results = []
    with ThreadPoolExecutor(max_workers=workers or os.cpu_count() or 4) as ex:
        futs = [ex.submit(run_one, fw, sc, sd, duration) for fw, sc, sd in jobs]
        for i, f in enumerate(futs):
            results.append(f.result())
            if progress:
                progress(i + 1, len(jobs))
    return summarize(results, fws, scenarios)


def summarize(results, fws, scenarios):
    table = []
    for sc in scenarios:
        row = {"scenario": sc, "description": DESCRIPTIONS.get(sc, "")}
        for fw in fws:
            rs = [r for r in results if r["firmware"] == fw and r["scenario"] == sc]
            truth = sum(r["truth"] for r in rs)
            counted = sum(r["counted"] for r in rs)
            veh = sum(r["vehiclesOverStrip"] for r in rs)
            lanes = {}
            for ln in ("A", "B"):
                t = sum(r["truth" + ln] for r in rs)
                c = sum(r["counted" + ln] for r in rs)
                lanes[ln] = {"truth": t, "counted": c, "accuracy": (1 - abs(c - t) / t) if t else None}
            if truth > 0:
                acc = 1 - abs(counted - truth) / truth
                worst = min(r["accuracy"] for r in rs)
            else:
                acc = None
                worst = None
            row[fw] = {"truth": truth, "counted": counted, "accuracy": acc, "worstRun": worst,
                       "vehiclesOverStrip": veh, "lanes": lanes, "runs": rs}
        table.append(row)
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", default="v2")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--duration", type=float, default=600)
    ap.add_argument("--seed-start", type=int, default=1)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    fws = a.fw.split(",")
    scs = a.scenarios.split(",")
    table = run_suite(fws, scs, a.seeds, a.duration, seed_start=a.seed_start)
    if a.json:
        print(json.dumps(table, indent=1))
        return
    hdr = f"{'scenario':10s}" + "".join(f" | {fw:>26s}" for fw in fws)
    print(hdr)
    print("-" * len(hdr))
    for row in table:
        line = f"{row['scenario']:10s}"
        for fw in fws:
            c = row[fw]
            if c["accuracy"] is None:
                line += f" | {'false counts: %d / %d veh' % (c['counted'], c['vehiclesOverStrip']):>26s}"
            else:
                la, lb = c["lanes"]["A"]["accuracy"], c["lanes"]["B"]["accuracy"]
                line += (f" | {c['counted']:5d}/{c['truth']:5d} {c['accuracy']*100:6.1f}% (min {c['worstRun']*100:5.1f})"
                         f"  ->{la*100:5.1f}%  <-{lb*100:5.1f}%")
        print(line)


if __name__ == "__main__":
    sys.exit(main())
