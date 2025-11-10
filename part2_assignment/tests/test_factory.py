
import json
from factory.main import solve_factory

def test_simple_no_modules():
    """Test basic factory production without any modules."""
    data = {
        "machines": {"m1": {"crafts_per_min": 1}},
        "recipes": {
            "iron_plate": {"machine":"m1","time_s":60.0,"in":{"iron_ore":1},"out":{"iron_plate":1}},
            "copper_plate": {"machine":"m1","time_s":60.0,"in":{"copper_ore":1},"out":{"copper_plate":1}},
            "gc": {"machine":"m1","time_s":60.0,"in":{"iron_plate":1,"copper_plate":1},"out":{"green_circuit":1}}
        },
        "limits": {
            "raw_supply_per_min": {"iron_ore": 100, "copper_ore": 100},
            "max_machines": {"m1": 1000}
        },
        "target": {"item":"green_circuit","rate_per_min":10.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    # crafts per minute equal to target for gc, and equal needs for plates
    assert abs(out["per_recipe_crafts_per_min"]["gc"] - 10.0) < 1e-6
    assert abs(out["per_recipe_crafts_per_min"]["iron_plate"] - 10.0) < 1e-6
    assert abs(out["per_recipe_crafts_per_min"]["copper_plate"] - 10.0) < 1e-6
    # raw consumption equals 10 each
    assert abs(out["raw_consumption_per_min"]["iron_ore"] - 10.0) < 1e-6
    assert abs(out["raw_consumption_per_min"]["copper_ore"] - 10.0) < 1e-6

def test_with_productivity_and_speed():
    """Test factory with productivity and speed modules."""
    data = {
        "machines": {"asm": {"crafts_per_min": 2}, "chem": {"crafts_per_min": 3}},
        "recipes": {
            "plate": {"machine":"chem","time_s":30.0,"in":{"ore":2},"out":{"plate":3}},
            "gc": {"machine":"asm","time_s":60.0,"in":{"plate":2},"out":{"gc":1}}
        },
        "modules": {
            "asm": {"prod": 0.25, "speed": 0.0},
            "chem": {"prod": 0.0, "speed": 0.0}
        },
        "limits": {
            "raw_supply_per_min": {"ore": 1000},
            "max_machines": {"asm": 1000, "chem": 1000}
        },
        "target": {"item":"gc","rate_per_min":50.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    # gc recipe crafts per minute should be 50 / (1+prod) because productivity increases outputs
    assert abs(out["per_recipe_crafts_per_min"]["gc"] - 50.0/1.25) < 1e-6

def test_infeasible_then_maximize():
    """Test infeasible target and compute maximum achievable rate."""
    data = {
        "machines": {"m": {"crafts_per_min": 1}},
        "recipes": {
            "plate": {"machine":"m","time_s":60.0,"in":{"ore":1},"out":{"plate":1}},
        },
        "limits": {
            "raw_supply_per_min": {"ore": 5},
            "max_machines": {"m": 1}
        },
        "target": {"item":"plate","rate_per_min":10.0}
    }
    out = solve_factory(data)
    assert out["status"] == "infeasible"
    # maximum feasible is min(machine eff, raw supply) = min(1,5)=1
    assert abs(out["max_feasible_target_per_min"] - 1.0) < 1e-6

def test_machine_bottleneck():
    """Test scenario where machine capacity is the limiting factor."""
    data = {
        "machines": {"assembler": {"crafts_per_min": 2}},
        "recipes": {
            "gear": {"machine":"assembler","time_s":30.0,"in":{"iron":2},"out":{"gear":1}}
        },
        "limits": {
            "raw_supply_per_min": {"iron": 1000},
            "max_machines": {"assembler": 2}  # Can only use 2 machines
        },
        "target": {"item":"gear","rate_per_min":10.0}
    }
    out = solve_factory(data)
    assert out["status"] == "infeasible"
    # Max machines = 2, each machine can do 2*60/30 = 4 gears/min, so max is 8
    assert abs(out["max_feasible_target_per_min"] - 8.0) < 1e-6
    assert "assembler cap" in out["bottleneck_hint"]

def test_raw_material_bottleneck():
    """Test scenario where raw material supply is the limiting factor."""
    data = {
        "machines": {"furnace": {"crafts_per_min": 5}},
        "recipes": {
            "steel": {"machine":"furnace","time_s":60.0,"in":{"iron":5},"out":{"steel":1}}
        },
        "limits": {
            "raw_supply_per_min": {"iron": 50},  # Limited iron supply
            "max_machines": {"furnace": 100}
        },
        "target": {"item":"steel","rate_per_min":20.0}
    }
    out = solve_factory(data)
    assert out["status"] == "infeasible"
    # Max steel = 50 iron / 5 = 10 steel/min
    assert abs(out["max_feasible_target_per_min"] - 10.0) < 1e-6
    assert "iron supply" in out["bottleneck_hint"]

def test_complex_production_chain():
    """Test complex production chain with multiple intermediate products."""
    data = {
        "machines": {"m1": {"crafts_per_min": 2}, "m2": {"crafts_per_min": 3}},
        "recipes": {
            "iron_plate": {"machine":"m1","time_s":60.0,"in":{"iron_ore":1},"out":{"iron_plate":1}},
            "gear": {"machine":"m1","time_s":30.0,"in":{"iron_plate":2},"out":{"gear":1}},
            "engine": {"machine":"m2","time_s":120.0,"in":{"gear":5,"iron_plate":1},"out":{"engine":1}}
        },
        "limits": {
            "raw_supply_per_min": {"iron_ore": 200},
            "max_machines": {"m1": 50, "m2": 20}
        },
        "target": {"item":"engine","rate_per_min":5.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    # 5 engines need 25 gears and 5 iron plates
    # 25 gears need 50 iron plates
    # Total: 55 iron plates needed, so 55 iron ore
    assert abs(out["raw_consumption_per_min"]["iron_ore"] - 55.0) < 1e-6
    assert abs(out["per_recipe_crafts_per_min"]["engine"] - 5.0) < 1e-6

def test_speed_module_effect():
    """Test that speed modules correctly increase production rate."""
    data = {
        "machines": {"fast_machine": {"crafts_per_min": 1}},
        "recipes": {
            "product": {"machine":"fast_machine","time_s":60.0,"in":{"input":1},"out":{"product":1}}
        },
        "modules": {
            "fast_machine": {"prod": 0.0, "speed": 0.5}  # 50% speed bonus
        },
        "limits": {
            "raw_supply_per_min": {"input": 100},
            "max_machines": {"fast_machine": 10}
        },
        "target": {"item":"product","rate_per_min":10.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    # With 50% speed bonus, effective rate = 1 * 1.5 = 1.5 crafts/min
    # So we need 10/1.5 = 6.67 crafts, using 6.67 machine units
    assert out["per_machine_counts"]["fast_machine"] < 7.0

def test_productivity_module_effect():
    """Test that productivity modules correctly increase output."""
    data = {
        "machines": {"prod_machine": {"crafts_per_min": 1}},
        "recipes": {
            "boosted": {"machine":"prod_machine","time_s":60.0,"in":{"raw":2},"out":{"boosted":1}}
        },
        "modules": {
            "prod_machine": {"prod": 0.4, "speed": 0.0}  # 40% productivity bonus
        },
        "limits": {
            "raw_supply_per_min": {"raw": 100},
            "max_machines": {"prod_machine": 50}
        },
        "target": {"item":"boosted","rate_per_min":10.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    # With 40% prod, each craft gives 1.4 output instead of 1
    # To get 10 output, need 10/1.4 = 7.14 crafts
    # Each craft needs 2 raw, so 7.14 * 2 = 14.29 raw
    assert abs(out["raw_consumption_per_min"]["raw"] - 10.0/1.4*2.0) < 0.5

def test_zero_target_feasible():
    """Test that zero production target is always feasible."""
    data = {
        "machines": {"m": {"crafts_per_min": 1}},
        "recipes": {
            "item": {"machine":"m","time_s":60.0,"in":{"ore":1},"out":{"item":1}}
        },
        "limits": {
            "raw_supply_per_min": {"ore": 0},  # No raw materials
            "max_machines": {"m": 0}  # No machines
        },
        "target": {"item":"item","rate_per_min":0.0}
    }
    out = solve_factory(data)
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] if "max_flow_per_min" in out else 0.0) < 1e-6
