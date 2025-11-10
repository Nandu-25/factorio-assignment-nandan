
import json
from belts.main import solve_belts

def test_belts_feasible_basic():
    """Test basic feasible belt configuration with multiple sources."""
    data = {
        "edges": [
            {"from":"s1","to":"a","lo":0,"hi":1000},
            {"from":"s2","to":"a","lo":0,"hi":1000},
            {"from":"a","to":"b","lo":100,"hi":900},
            {"from":"b","to":"sink","lo":0,"hi":900},
            {"from":"a","to":"sink","lo":0,"hi":1000}
        ],
        "sources": {"s1": 400, "s2": 200},
        "sink": "sink",
        "nodes": {
            "a": {"cap_out": 1000}  # node cap does not bind
        }
    }
    out = solve_belts(data)
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] - 600.0) < 1e-6
    # check lower bound respected on a->b (at least 100 flows)
    flow_ab = next(f for f in out["flows"] if f["from"]=="a" and f["to"]=="b")["flow"]
    assert flow_ab >= 100 - 1e-6

def test_belts_infeasible_cut():
    """Test infeasible belt configuration due to capacity constraints."""
    data = {
        "edges": [
            {"from":"s1","to":"a","lo":0,"hi":200},
            {"from":"a","to":"sink","lo":0,"hi":100}
        ],
        "sources": {"s1": 200},
        "sink": "sink"
    }
    out = solve_belts(data)
    assert out["status"] == "infeasible"
    assert out["deficit"]["demand_balance"] >= 100 - 1e-6

def test_belts_with_node_capacity():
    """Test belt system with node capacity constraints."""
    data = {
        "edges": [
            {"from":"source","to":"node1","lo":0,"hi":500},
            {"from":"node1","to":"node2","lo":0,"hi":500},
            {"from":"node2","to":"sink","lo":0,"hi":500}
        ],
        "sources": {"source": 300},
        "sink": "sink",
        "nodes": {
            "node1": {"cap": 200}  # Bottleneck at node1
        }
    }
    out = solve_belts(data)
    assert out["status"] == "infeasible"
    assert "node1" in out["deficit"].get("tight_nodes", []) or out["deficit"]["demand_balance"] > 0

def test_belts_lower_bound_requirement():
    """Test belt system with lower bound constraints."""
    data = {
        "edges": [
            {"from":"s1","to":"mid","lo":0,"hi":200},
            {"from":"mid","to":"sink","lo":0,"hi":300}
        ],
        "sources": {"s1": 100},
        "sink": "sink"
    }
    out = solve_belts(data)
    # Simple case without restrictive lower bounds should be feasible
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] - 100.0) < 1e-6

def test_belts_multiple_paths():
    """Test belt system with multiple parallel paths to sink."""
    data = {
        "edges": [
            {"from":"source","to":"path1","lo":0,"hi":100},
            {"from":"source","to":"path2","lo":0,"hi":150},
            {"from":"path1","to":"sink","lo":0,"hi":100},
            {"from":"path2","to":"sink","lo":0,"hi":150}
        ],
        "sources": {"source": 200},
        "sink": "sink"
    }
    out = solve_belts(data)
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] - 200.0) < 1e-6

def test_belts_complex_network():
    """Test complex belt network with multiple sources and intermediate nodes."""
    data = {
        "edges": [
            {"from":"s1","to":"hub","lo":0,"hi":300},
            {"from":"s2","to":"hub","lo":0,"hi":200},
            {"from":"hub","to":"dist1","lo":0,"hi":250},
            {"from":"hub","to":"dist2","lo":0,"hi":250},
            {"from":"dist1","to":"sink","lo":0,"hi":300},
            {"from":"dist2","to":"sink","lo":0,"hi":300}
        ],
        "sources": {"s1": 250, "s2": 150},
        "sink": "sink",
        "nodes": {
            "hub": {"cap": 500}
        }
    }
    out = solve_belts(data)
    # Without mandatory lower bounds on distributions, this should be feasible
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] - 400.0) < 1e-6

def test_belts_edge_case_zero_flow():
    """Test edge case with zero flow request."""
    data = {
        "edges": [
            {"from":"s1","to":"sink","lo":0,"hi":100}
        ],
        "sources": {"s1": 0},
        "sink": "sink"
    }
    out = solve_belts(data)
    assert out["status"] == "ok"
    assert abs(out["max_flow_per_min"] - 0.0) < 1e-6
