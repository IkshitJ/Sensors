#!/usr/bin/env python3
"""Accuracy benchmark: runs the real firmware headless over many scenarios and
random seeds, in parallel, and prints accuracy of the crossing count, of the
IN / OUT split (into / out of the Ram Ghat premises) and of the "people inside"
counter.

    python3 bench.py                                   # all scenarios, 3 seeds x 10 min
    python3 bench.py --suite tidal                     # bathing rush / return
    python3 bench.py --seeds 5 --seed-start 100 --scenarios sparse,dense
    python3 bench.py --nonlin 0.3                      # 30% non-linear walkers everywhere
"""
import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
# pilgrims walk both ways, anywhere on the marg width, in every scenario
LINEAR_SCENARIOS = ["sparse", "normal", "rush", "dense", "festival", "signal", "waiting", "mixed", "traffic"]
# people who do NOT walk in straight lines (Kumbh crowds)
NONLINEAR_SCENARIOS = ["weave", "drift", "jostle", "uturn", "meander", "kumbh"]
# tidal flows through the ghat gate
TIDAL_SCENARIOS = ["snan", "return"]
SCENARIOS = LINEAR_SCENARIOS + NONLINEAR_SCENARIOS + TIDAL_SCENARIOS
SUITES = {"linear": LINEAR_SCENARIOS, "nonlinear": NONLINEAR_SCENARIOS, "tidal": TIDAL_SCENARIOS, "all": SCENARIOS}
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
    "weave": "6/min, all meander (heading +-10..30 deg)",
    "drift": "6/min, all cut diagonally across the marg",
    "jostle": "6/min, all jostled: random heading, sidesteps",
    "uturn": "6/min, all step onto the strip and turn back",
    "meander": "14/min, 100% non-linear mix, groups, vehicles",
    "kumbh": "140/min crowd, 50% groups, 60% non-linear",
    "snan": "bathing rush: 60/min, 85% into the ghat, 40% non-linear",
    "return": "after the snan: 60/min, 85% out of the ghat, 40% non-linear",
}


def run_one(fw, scenario, seed, duration, nonlin=None):
    exe = os.path.join(HERE, "bin", "zebra_" + fw)
    env = dict(os.environ)
    if nonlin is not None:  # overlay non-linear walkers on any scenario
        env["SIM_NONLIN"] = str(nonlin)
    out = subprocess.run([exe, "--bench", "--scenario", scenario, "--duration", str(duration), "--seed", str(seed)],
                         capture_output=True, text=True, timeout=1800, env=env)
    line = out.stdout.strip().splitlines()[-1]
    return json.loads(line)


def run_suite(fws, scenarios, seeds, duration, workers=None, progress=None, seed_start=1, nonlin=None):
    jobs = [(fw, sc, sd) for fw in fws for sc in scenarios for sd in range(seed_start, seed_start + seeds)]
    results = []
    with ThreadPoolExecutor(max_workers=workers or os.cpu_count() or 4) as ex:
        futs = [ex.submit(run_one, fw, sc, sd, duration, nonlin) for fw, sc, sd in jobs]
        for i, f in enumerate(futs):
            results.append(f.result())
            if progress:
                progress(i + 1, len(jobs))
    return summarize(results, fws, scenarios)


def acc_of(counted, truth):
    return (1 - abs(counted - truth) / truth) if truth else None


def summarize(results, fws, scenarios):
    table = []
    for sc in scenarios:
        row = {"scenario": sc, "description": DESCRIPTIONS.get(sc, "")}
        for fw in fws:
            rs = [r for r in results if r["firmware"] == fw and r["scenario"] == sc]
            truth = sum(r["truth"] for r in rs)
            counted = sum(r["counted"] for r in rs)
            dirs = {}
            for d in ("In", "Out"):
                t = sum(r["truth" + d] for r in rs)
                c = sum(r["counted" + d] for r in rs)
                dirs[d.lower()] = {"truth": t, "counted": c, "accuracy": acc_of(c, t)}
            # people inside: error at the end of each run (after everyone left the strip),
            # and the average error while the crowd was flowing, relative to the people who crossed
            inside_end = [abs(r["countedInside"] - r["truthInside"]) for r in rs]
            row[fw] = {
                "truth": truth, "counted": counted,
                "accuracy": acc_of(counted, truth),
                "worstRun": min(r["accuracy"] for r in rs) if truth else None,
                "vehiclesOverStrip": sum(r["vehiclesOverStrip"] for r in rs),
                "dirs": dirs,
                "inside": {
                    "truthEnd": [r["truthInside"] for r in rs],
                    "countedEnd": [r["countedInside"] for r in rs],
                    "errEndMean": sum(inside_end) / len(rs) if rs else 0,
                    "errEndMax": max(inside_end) if rs else 0,
                    "errLiveMean": sum(r["insideErrMean"] for r in rs) / len(rs) if rs else 0,
                    "errPctOfCrossed": (sum(inside_end) / truth) if truth else None,
                },
                "turnedBack": sum(r.get("touchedNotCrossed", 0) for r in rs),
                "runs": rs,
            }
        table.append(row)
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", default="v2")
    ap.add_argument("--scenarios", default=None, help="comma list (default: the chosen --suite)")
    ap.add_argument("--suite", default="all", choices=sorted(SUITES), help="linear, nonlinear, tidal, all")
    ap.add_argument("--nonlin", type=float, default=None,
                    help="make this fraction of people walk non-linearly in EVERY scenario (0..1)")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--duration", type=float, default=600)
    ap.add_argument("--seed-start", type=int, default=1)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    fws = a.fw.split(",")
    scs = a.scenarios.split(",") if a.scenarios else SUITES[a.suite]
    table = run_suite(fws, scs, a.seeds, a.duration, seed_start=a.seed_start, nonlin=a.nonlin)
    if a.json:
        print(json.dumps(table, indent=1))
        return
    pc = lambda v: "  n/a" if v is None else f"{v*100:5.1f}%"
    print(f"{'scenario':9s} | {'crossed (counted/true)':>29s} | {'<- IN to ghat':>19s} | {'-> OUT':>19s} | inside: true vs counted at end")
    print("-" * 130)
    for row in table:
        for fw in fws:
            c = row[fw]
            if c["accuracy"] is None:
                line = f"{row['scenario']:9s} | {'false counts: %d / %d veh' % (c['counted'], c['vehiclesOverStrip']):>29s} |"
                if c["turnedBack"]:
                    line += f" (turned back {c['turnedBack']})"
                print(line)
                continue
            di, do, ins = c["dirs"]["in"], c["dirs"]["out"], c["inside"]
            ends = ", ".join(f"{t}/{k}" for t, k in zip(ins["truthEnd"], ins["countedEnd"]))
            line = (f"{row['scenario']:9s} | {c['counted']:5d}/{c['truth']:5d} {pc(c['accuracy'])} (min {pc(c['worstRun'])}) |"
                    f" {di['counted']:5d}/{di['truth']:5d} {pc(di['accuracy'])} |"
                    f" {do['counted']:5d}/{do['truth']:5d} {pc(do['accuracy'])} |"
                    f" {ends}  (err {ins['errEndMean']:.1f}, live err {ins['errLiveMean']:.1f})")
            if c["turnedBack"]:
                line += f"  [turned back {c['turnedBack']}]"
            print(line)


if __name__ == "__main__":
    sys.exit(main())
