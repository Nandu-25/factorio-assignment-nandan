# Design Notes: Factory & Belts CLI Tools

This document outlines the design and implementation of two standalone command-line interface (CLI) tools: factory and belts.

## CLI Usage

Both tools function as standard UNIX-style filters. They read a single JSON object from stdin, perform a calculation, and write a single JSON object to stdout. All logging or auxiliary output is suppressed to ensure a clean data pipeline.

```bash
python factory/main.py < input_factory.json > output_factory.json
python belts/main.py < input_belts.json > output_belts.json
```

## A) Factory Steady State Solver (factory)

This tool calculates the steady-state recipe craft rates required to meet a specific target output, subject to resource and machine constraints.

### 1. Model Definition

The problem is modeled as a Linear Program (LP).

**Variables:** `x_r ≥ 0` represents the crafts per minute for each recipe `r`.

**Effective Crafts/min (eff_r):** The maximum craft rate for a single machine of type `m` running recipe `r`.

```
eff_r = machines[m].crafts_per_min · (1 + speed_m) · 60 / time_s(r)
```

**Net Item Contribution (a_ir):** The net amount of item `i` produced by one craft of recipe `r`, accounting for productivity bonuses (which apply to outputs only).

```
a_ir = output_{r,i} · (1 + prod_m) − input_{r,i}
```

### 2. Constraints

The LP is defined by the following constraints:

**Conservation (Target):** For the specified target item `t` and `target_rate`:

```
Σ_r a_{t r} x_r = target_rate
```

**Conservation (Intermediates):** For all non-raw, non-target items, net production must be zero.

```
Σ_r a_{ir} x_r = 0
```

**Raw Item Caps:** For raw items `i` with a cap `cap_i`:

- Net production must be non-positive (raws cannot be created): `Σ_r a_{ir} x_r ≤ 0`
- Net consumption must not exceed the cap: `−Σ_r a_{ir} x_r ≤ cap_i`

**Machine Caps:** For each machine type `m`:

```
Σ_{r uses m} x_r / eff_r ≤ max_machines[m]
```

### 3. Objective Function

A two-phase objective is used:

- **Phase I (Feasibility):** Find any solution `x` that satisfies all constraints.
- **Phase II (Optimization):** If a feasible solution exists, minimize the total machine count.

```
min Σ_r x_r / eff_r
```

### 4. Solver Implementation

The solver is a custom, deterministic two-phase simplex implementation.

- It operates on the standard LP form: `A_eq x = b_eq, A_ub x ≤ b_ub, x ≥ 0`.
- Bland's rule is used for selecting entering and leaving variables to guarantee determinism and prevent cycling.
- All numeric comparisons use a tolerance of 1e-9.

### 5. Handling Infeasibility

If Phase I fails to find a feasible solution for the requested `target_rate`, the tool solves a second LP to find the maximum possible production:

- **Objective:** `max c_target^T x` (where `c_target` is the vector of net coefficients for the target item).
- **Output:** The solver reports `max_feasible_target_per_min` and provides simple bottleneck hints based on saturated machine caps and raw supply constraints.

## B) Bounded Belts & Node Caps Solver (belts)

This tool solves a maximum flow problem in a network with edge bounds (lower and upper) and node capacities.

### 1. Input Specification

The tool expects a JSON object describing the graph, supplies, and constraints.

```json5
{
  "edges": [{"from": "u", "to": "v", "lo": 0, "hi": 100},],
  "sources": {"s1": 900, "s2": 600},
  "sink": "sink",
  "nodes": { "a": {"cap": 500} },
  //can also use "nodes": { "a": {"cap_in":500, "cap_out":700} }
  //can also use or "node_caps": { "a": 500 }
}
```

- Node caps on sources or the sink are ignored, as per convention.
- If both `cap_in` and `cap_out` are provided for a node, the minimum of the two is enforced as the node's capacity.

### 2. Solution Method

The problem is solved using a multi-stage max-flow approach.

1. **Node Capacity:** Each capped node `v` is split into two nodes, `v_in` and `v_out`, connected by an internal edge (`v_in → v_out`) with capacity `cap(v)`. All incoming edges to `v` are redirected to `v_in`, and all outgoing edges from `v` originate from `v_out`.

2. **Lower Bounds:** Each edge (`u→v`) with bounds `[lo, hi]` is transformed. The edge's capacity in the residual graph becomes `hi − lo`. The required lower bound flow `lo` is handled by adding a `+lo` imbalance (demand) to node `v` and a `−lo` imbalance (supply) to node `u`.

3. **Feasibility of Lower Bounds:** A circulation problem is solved to satisfy the imbalances created in step 2. A super-source `s*` is connected to all nodes with negative imbalance (supply), and all nodes with positive imbalance (demand) are connected to a super-sink `t*`. A max-flow is run from `s*` to `t*`. If this flow fully saturates all edges from `s*`, the lower bounds are feasible.

4. **Main Flow:** A primary super-source `S` is added and connected to each original source `s` with capacity `supply(s)`. A max-flow is run from `S` to the original sink on the residual network remaining from step 3.

5. **Flow Reconstruction:** The final flow on an original edge `e` is `f_e = f'_e + lo_e`, where `f'_e` is the flow from step 4 and `lo_e` is its lower bound.

6. **Infeasibility Certificate:** If the flow is infeasible, the solver returns the reachable side of the min-cut from `S`, the remaining demand deficit, and lists of saturated node caps and crossing edges that form the bottleneck.

### 3. Algorithm Implementation

The solver uses a deterministic Dinic's algorithm(https://cp-algorithms.com/graph/dinic.html) for max-flow.

- Numeric precision tolerance is set to 1e-9.
