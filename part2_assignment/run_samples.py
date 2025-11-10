
#!/usr/bin/env python3
import json, subprocess, os, sys, tempfile, textwrap

def run_tool(cmd, payload):
    p = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = p.communicate(json.dumps(payload))
    if err:
        # do not print on tools, but this runner can show stderr for debugging if needed
        pass
    return json.loads(out)

def main(factory_cmd, belts_cmd):
    # Sample for factory (simple consistent case)
    factory_in = {
        "machines": {"asm": {"crafts_per_min": 1}},
        "recipes": {
            "p": {"machine":"asm","time_s":60.0,"in":{"ore":2},"out":{"p":2}},
            "t": {"machine":"asm","time_s":60.0,"in":{"p":2},"out":{"t":1}}
        },
        "modules": {},
        "limits": {
            "raw_supply_per_min": {"ore": 1000},
            "max_machines": {"asm": 1000}
        },
        "target": {"item":"t","rate_per_min": 30.0}
    }
    out1 = run_tool(factory_cmd, factory_in)
    assert out1["status"] == "ok"
    assert abs(out1["per_recipe_crafts_per_min"]["t"] - 30.0) < 1e-6
    assert abs(out1["raw_consumption_per_min"]["ore"] - 60.0) < 1e-6

    # Sample for belts
    belts_in = {
        "edges": [
            {"from":"s1","to":"a","lo":0,"hi":300},
            {"from":"s2","to":"a","lo":0,"hi":400},
            {"from":"a","to":"b","lo":100,"hi":500},
            {"from":"b","to":"sink","lo":0,"hi":500}
        ],
        "sources": {"s1": 200, "s2": 200},
        "sink": "sink",
        "nodes": {
            "b": {"cap_in": 500}
        }
    }
    out2 = run_tool(belts_cmd, belts_in)
    assert out2["status"] == "ok"
    assert abs(out2["max_flow_per_min"] - 400.0) < 1e-6

    print("sample runs passed")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python run_samples.py '<FACTORY_CMD>' '<BELTS_CMD>'")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
