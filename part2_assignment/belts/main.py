#!/usr/bin/env python3
"""
Belt system flow optimization solver using max-flow algorithms.
This module implements a solution for conveyor belt network flow problems
with node and edge capacity constraints.
"""
import sys
import json
from typing import List, Tuple

# Floating-point comparison tolerance for numerical stability
TOLERANCE = 1e-9

class Edge:
    """
    Represents a directed edge in the flow network.
    Uses __slots__ for memory efficiency in large graphs.
    """
    __slots__ = ("to","rev","cap","flow","orig_from","orig_to","is_original","lo","hi","name")
    
    def __init__(self, to, rev, cap):
        """
        Initialize a flow network edge.
        
        Args:
            to: Destination node index
            rev: Index of reverse edge in adjacency list
            cap: Current available capacity for flow
        """
        self.to = to
        self.rev = rev
        self.cap = cap
        self.flow = 0.0
        self.orig_from = None
        self.orig_to = None
        self.is_original = False
        self.lo = 0.0
        self.hi = 0.0
        self.name = None

class Dinic:
    """
    Dinic's algorithm implementation for computing maximum flow.
    Uses level graph and blocking flow concepts for O(V^2 * E) complexity.
    """
    def __init__(self, n:int):
        """Initialize flow network with n nodes."""
        self.n = n
        self.adj: List[List[Edge]] = [[] for _ in range(n)]
    
    def add_edge(self, u:int, v:int, cap:float)->Tuple[Edge,Edge]:
        """
        Add a directed edge with residual capacity tracking.
        
        Args:
            u: Source node
            v: Destination node  
            cap: Edge capacity
            
        Returns:
            Tuple of (forward_edge, backward_edge)
        """
        fwd = Edge(v, len(self.adj[v]), cap)
        bwd = Edge(u, len(self.adj[u]), 0.0)
        self.adj[u].append(fwd)
        self.adj[v].append(bwd)
        return fwd, bwd
    
    def bfs(self, s:int, t:int)->List[int]:
        """
        Build level graph using breadth-first search.
        
        Args:
            s: Source node
            t: Sink node
            
        Returns:
            Distance levels from source (-1 if unreachable)
        """
        from collections import deque
        level = [-1]*self.n
        q = deque([s]); level[s] = 0
        while q:
            u = q.popleft()
            for e in self.adj[u]:
                if level[e.to] < 0 and e.cap > TOLERANCE:
                    level[e.to] = level[u] + 1
                    q.append(e.to)
        return level
    
    def dfs(self, u:int, t:int, f:float, level:List[int], it:List[int])->float:
        """
        Find blocking flow using depth-first search with iterators.
        
        Args:
            u: Current node
            t: Target sink node
            f: Available flow to push
            level: Pre-computed level graph
            it: Iterator positions for each node
            
        Returns:
            Amount of flow successfully pushed
        """
        if u == t: return f
        i = it[u]
        while i < len(self.adj[u]):
            e = self.adj[u][i]
            if e.cap > TOLERANCE and level[u] < level[e.to]:
                pushed_flow = self.dfs(e.to, t, min(f, e.cap), level, it)
                if pushed_flow > TOLERANCE:
                    e.cap -= pushed_flow
                    self.adj[e.to][e.rev].cap += pushed_flow
                    e.flow += pushed_flow
                    self.adj[e.to][e.rev].flow -= pushed_flow
                    return pushed_flow
            i += 1; it[u] = i
        return 0.0
    
    def maxflow(self, s:int, t:int)->float:
        """
        Compute maximum flow from source to sink.
        
        Args:
            s: Source node
            t: Sink node
            
        Returns:
            Maximum achievable flow value
        """
        total_flow = 0.0
        INFINITY = 1e100
        while True:
            level = self.bfs(s, t)
            if level[t] < 0: return total_flow
            it = [0]*self.n
            while True:
                augment = self.dfs(s, t, INFINITY, level, it)
                if augment <= TOLERANCE: break
                total_flow += augment

def solve_belts(data:dict)->dict:
    """
    Solve the belt network flow problem with lower/upper bounds on edges.
    
    Args:
        data: Input dictionary containing edges, sources, sink, and node capacities
        
    Returns:
        Dictionary with solution status, flow values, or infeasibility diagnosis
    """
    edges_in = data["edges"]
    sources = data.get("sources", {})
    sink = data["sink"]

    # Parse node capacity constraints from multiple possible input formats
    node_caps_map = {}
    if "node_caps" in data:
        for k, v in data["node_caps"].items():
            node_caps_map[k] = float(v)
    if "nodes" in data:
        for k, v in data["nodes"].items():
            cap = None
            if isinstance(v, dict):
                cin = v.get("cap_in", None)
                cout = v.get("cap_out", None)
                # Use the minimum of input and output capacities
                if cin is not None or cout is not None:
                    if cin is None: cin = float('inf')
                    if cout is None: cout = float('inf')
                    cap = float(min(cin, cout))
                if "cap" in v:
                    cap = float(v["cap"])
            # Keep the most restrictive capacity
            if cap is not None and (k not in node_caps_map or cap < node_caps_map[k]):
                node_caps_map[k] = cap

    # Collect all nodes referenced in the network
    nodes = set()
    for e in edges_in:
        nodes.add(e["from"]); nodes.add(e["to"])
    for s in sources.keys():
        nodes.add(s)
    nodes.add(sink)

    # Nodes requiring capacity splitting (excluding sources and sink)
    split_nodes = {v for v in node_caps_map.keys() if v not in sources and v != sink}

    # Build node index mapping
    name_list = []
    idx = {}
    def add_node(name):
        """Register a node in the index if not already present."""
        if name not in idx:
            idx[name] = len(name_list)
            name_list.append(name)

    # Create virtual in/out nodes for capacity-constrained nodes
    for v in sorted(nodes):
        if v in split_nodes:
            add_node(v+"_in"); add_node(v+"_out")
        else:
            add_node(v)

    din = Dinic(len(name_list))

    def map_from(u):
        """Map logical node to its outgoing connection point."""
        return idx[u+"_out"] if u in split_nodes else idx[u]
    def map_to(v):
        """Map logical node to its incoming connection point."""
        return idx[v+"_in"] if v in split_nodes else idx[v]

    # Add internal capacity edges for split nodes
    for v in sorted(split_nodes):
        cap = float(node_caps_map[v])
        u = idx[v+"_in"]; w = idx[v+"_out"]
        fwd, bwd = din.add_edge(u, w, cap)
        fwd.name = f"{v}__internal"

    # Add original network edges with lower/upper bound transformation
    orig_edge_refs = []
    for e in sorted(edges_in, key=lambda x: (x["from"], x["to"])):
        u = map_from(e["from"]); v = map_to(e["to"])
        lo = float(e.get("lo", 0.0)); hi = float(e.get("hi", 0.0))
        # Check for infeasible bounds
        if hi < lo - 1e-12:
            return {"status":"infeasible","cut_reachable":[],"deficit":{"demand_balance":float(sum(sources.values())),"tight_nodes":[],"tight_edges":[]}}
        fwd, bwd = din.add_edge(u, v, hi - lo)
        fwd.is_original = True; fwd.orig_from = e["from"]; fwd.orig_to = e["to"]
        fwd.lo = lo; fwd.hi = hi; fwd.name = f"{e['from']}->{e['to']}"
        orig_edge_refs.append(fwd)

    # Calculate flow imbalance due to lower bounds
    imbalance = {name: 0.0 for name in name_list}
    for fwd in orig_edge_refs:
        if fwd.lo > 0:
            imbalance[name_list[map_to(fwd.orig_to)]] += fwd.lo
            imbalance[name_list[map_from(fwd.orig_from)]] -= fwd.lo

    # Create temporary circulation edges from sink to sources
    extra_pairs = []
    sink_from = map_from(sink)
    for sname, supply in sorted(sources.items()):
        u = sink_from
        v = map_from(sname)
        ef, eb = din.add_edge(u, v, float(supply))
        extra_pairs.append((ef, din.adj[v][ef.rev]))

    # Add auxiliary nodes for checking lower bound feasibility
    SS = len(name_list); TT = SS + 1
    din.adj.append([]); din.adj.append([]); din.n += 2

    demand_total = 0.0
    for vname, val in sorted(imbalance.items()):
        if val > TOLERANCE:
            demand_total += val
            din.add_edge(SS, idx[vname], val)
        elif val < -TOLERANCE:
            din.add_edge(idx[vname], TT, -val)

    # Check if lower bounds are satisfiable
    flow_feas = din.maxflow(SS, TT)
    if flow_feas < demand_total - 1e-7:
        level = din.bfs(SS, TT)
        reached = [name_list[i] for i, lvl in enumerate(level) if i < len(name_list) and lvl >= 0]
        return {
            "status":"infeasible",
            "cut_reachable": sorted({n.replace("_in","").replace("_out","") for n in reached}),
            "deficit":{"demand_balance":float(demand_total - flow_feas),"tight_nodes":[],"tight_edges":[]}
        }

    # Clean up auxiliary structure
    for u in range(len(din.adj)):
        for e in din.adj[u]:
            if e.to == SS or e.to == TT:
                e.cap = 0.0
                e.flow = 0.0
    din.adj[SS] = []; din.adj[TT] = []

    # Remove temporary circulation edges
    for ef, eb in extra_pairs:
        ef.cap = 0.0; ef.flow = 0.0
        eb.cap = 0.0; eb.flow = 0.0

    # Solve main max-flow problem from sources to sink
    S = len(din.adj); din.adj.append([]); din.n += 1
    total_supply = 0.0
    for sname, supply in sorted(sources.items()):
        total_supply += float(supply)
        din.add_edge(S, map_from(sname), float(supply))
    T = map_to(sink)

    delivered = din.maxflow(S, T)
    # Check if all supply can be delivered
    if delivered < total_supply - 1e-7:
        level = din.bfs(S, T)
        reachable_idxs = [i for i, lvl in enumerate(level) if lvl >= 0 and i < len(name_list)]
        reachable = [name_list[i] for i in reachable_idxs]
        cut_set = {v.replace("_in","").replace("_out","") for v in reachable}
        # Identify edges at full capacity in the min-cut
        tight_edges = []
        for u_idx in reachable_idxs:
            for e in din.adj[u_idx]:
                if e.to < len(name_list) and level[e.to] < 0:
                    if e.is_original and e.cap <= 1e-7:
                        tight_edges.append({"from": e.orig_from, "to": e.orig_to, "flow_needed": 0.0})
        # Identify capacity-saturated nodes in the min-cut
        tight_nodes = []
        for v in sorted(set(node_caps_map.keys()) - set(sources.keys()) - {sink}):
            u = idx[v+"_in"] if v in split_nodes else None
            if u is not None:
                for e in din.adj[u]:
                    if e.name == f"{v}__internal" and e.cap <= 1e-7:
                        tight_nodes.append(v)
        return {
            "status":"infeasible",
            "cut_reachable": sorted(cut_set),
            "deficit":{"demand_balance":float(total_supply - delivered),"tight_nodes":sorted(tight_nodes),"tight_edges":tight_edges}
        }

    # Extract final flow values (add back lower bounds)
    flows = []
    for e in orig_edge_refs:
        f = e.flow + e.lo
        # Handle numerical precision issues
        if f < 0 and f > -1e-7: f = 0.0
        flows.append({"from": e.orig_from, "to": e.orig_to, "flow": float(f)})

    return {"status":"ok","max_flow_per_min":float(delivered),"flows":flows}

def main():
    """
    Entry point: Read JSON input from stdin, solve the problem, and output JSON result.
    """
    data = json.load(sys.stdin)
    out = solve_belts(data)
    json.dump(out, sys.stdout, separators=(",", ":"), ensure_ascii=False)

if __name__ == "__main__":
    main()

