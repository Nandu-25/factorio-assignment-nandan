"""
Belt system flow optimization solver using max-flow algorithms.
This module implements a solution for conveyor belt network flow problems
with node and edge capacity constraints.
"""
import sys
import json
from typing import List, Tuple

# Floating-point comparison tolerance for numerical stability
EPSILON = 1e-9

class FlowEdge:
    """
    Represents a directed edge in the flow network.
    Uses __slots__ for memory efficiency in large graphs.
    """
    __slots__ = ("dest","back_idx","capacity","current_flow","source_node","target_node","is_core_edge","lower_bound","upper_bound","label")
    
    def __init__(self, dest, back_idx, capacity):
        """
        Initialize a flow network edge.
        
        Args:
            dest: Destination node index
            back_idx: Index of reverse edge in adjacency list
            capacity: Current available capacity for flow
        """
        self.dest = dest
        self.back_idx = back_idx
        self.capacity = capacity
        self.current_flow = 0.0
        self.source_node = None
        self.target_node = None
        self.is_core_edge = False
        self.lower_bound = 0.0
        self.upper_bound = 0.0
        self.label = None

class MaxFlowSolver:
    """
    Dinic's algorithm implementation for computing maximum flow.
    Uses level graph and blocking flow concepts for O(V^2 * E) complexity.
    """
    def __init__(self, node_count:int):
        """Initialize flow network with node_count nodes."""
        self.node_count = node_count
        self.graph: List[List[FlowEdge]] = [[] for _ in range(node_count)]
    
    def add_directed_edge(self, src:int, dst:int, cap:float)->Tuple[FlowEdge,FlowEdge]:
        """
        Add a directed edge with residual capacity tracking.
        
        Args:
            src: Source node
            dst: Destination node  
            cap: Edge capacity
            
        Returns:
            Tuple of (forward_edge, backward_edge)
        """
        forward = FlowEdge(dst, len(self.graph[dst]), cap)
        backward = FlowEdge(src, len(self.graph[src]), 0.0)
        self.graph[src].append(forward)
        self.graph[dst].append(backward)
        return forward, backward
    
    def build_level_graph(self, source:int, sink:int)->List[int]:
        """
        Build level graph using breadth-first search.
        
        Args:
            source: Source node
            sink: Sink node
            
        Returns:
            Distance levels from source (-1 if unreachable)
        """
        from collections import deque
        distances = [-1]*self.node_count
        queue = deque([source]); distances[source] = 0
        while queue:
            current = queue.popleft()
            for edge in self.graph[current]:
                if distances[edge.dest] < 0 and edge.capacity > EPSILON:
                    distances[edge.dest] = distances[current] + 1
                    queue.append(edge.dest)
        return distances
    
    def push_flow(self, current:int, sink:int, available:float, distances:List[int], edge_pointers:List[int])->float:
        """
        Find blocking flow using depth-first search with iterators.
        
        Args:
            current: Current node
            sink: Target sink node
            available: Available flow to push
            distances: Pre-computed level graph
            edge_pointers: Iterator positions for each node
            
        Returns:
            Amount of flow successfully pushed
        """
        if current == sink: return available
        pointer = edge_pointers[current]
        while pointer < len(self.graph[current]):
            edge = self.graph[current][pointer]
            if edge.capacity > EPSILON and distances[current] < distances[edge.dest]:
                flow_pushed = self.push_flow(edge.dest, sink, min(available, edge.capacity), distances, edge_pointers)
                if flow_pushed > EPSILON:
                    edge.capacity -= flow_pushed
                    self.graph[edge.dest][edge.back_idx].capacity += flow_pushed
                    edge.current_flow += flow_pushed
                    self.graph[edge.dest][edge.back_idx].current_flow -= flow_pushed
                    return flow_pushed
            pointer += 1; edge_pointers[current] = pointer
        return 0.0
    
    def compute_max_flow(self, source:int, sink:int)->float:
        """
        Compute maximum flow from source to sink.
        
        Args:
            source: Source node
            sink: Sink node
            
        Returns:
            Maximum achievable flow value
        """
        total = 0.0
        LARGE_VALUE = 1e100
        while True:
            distances = self.build_level_graph(source, sink)
            if distances[sink] < 0: return total
            edge_pointers = [0]*self.node_count
            while True:
                augmentation = self.push_flow(source, sink, LARGE_VALUE, distances, edge_pointers)
                if augmentation <= EPSILON: break
                total += augmentation

def solve_belts(input_data:dict)->dict:
    """
    Solve the belt network flow problem with lower/upper bounds on edges.
    
    Args:
        input_data: Input dictionary containing edges, sources, sink, and node capacities
        
    Returns:
        Dictionary with solution status, flow values, or infeasibility diagnosis
    """
    edge_list = input_data["edges"]
    source_nodes = input_data.get("sources", {})
    sink_node = input_data["sink"]

    # Parse node capacity constraints from multiple possible input formats
    capacity_map = {}
    if "node_caps" in input_data:
        for node_name, cap_value in input_data["node_caps"].items():
            capacity_map[node_name] = float(cap_value)
    if "nodes" in input_data:
        for node_name, node_spec in input_data["nodes"].items():
            final_cap = None
            if isinstance(node_spec, dict):
                cap_in = node_spec.get("cap_in", None)
                cap_out = node_spec.get("cap_out", None)
                # Use the minimum of input and output capacities
                if cap_in is not None or cap_out is not None:
                    if cap_in is None: cap_in = float('inf')
                    if cap_out is None: cap_out = float('inf')
                    final_cap = float(min(cap_in, cap_out))
                if "cap" in node_spec:
                    final_cap = float(node_spec["cap"])
            # Keep the most restrictive capacity
            if final_cap is not None and (node_name not in capacity_map or final_cap < capacity_map[node_name]):
                capacity_map[node_name] = final_cap

    # Collect all nodes referenced in the network
    all_nodes = set()
    for edge_spec in edge_list:
        all_nodes.add(edge_spec["from"]); all_nodes.add(edge_spec["to"])
    for src_name in source_nodes.keys():
        all_nodes.add(src_name)
    all_nodes.add(sink_node)

    # Nodes requiring capacity splitting (excluding sources and sink)
    nodes_to_split = {node for node in capacity_map.keys() if node not in source_nodes and node != sink_node}

    # Build node index mapping
    node_names = []
    node_indices = {}
    def register_node(name):
        """Register a node in the index if not already present."""
        if name not in node_indices:
            node_indices[name] = len(node_names)
            node_names.append(name)

    # Create virtual in/out nodes for capacity-constrained nodes
    for node in sorted(all_nodes):
        if node in nodes_to_split:
            register_node(node+"_in"); register_node(node+"_out")
        else:
            register_node(node)

    solver = MaxFlowSolver(len(node_names))

    def get_outgoing_node(node):
        """Map logical node to its outgoing connection point."""
        return node_indices[node+"_out"] if node in nodes_to_split else node_indices[node]
    def get_incoming_node(node):
        """Map logical node to its incoming connection point."""
        return node_indices[node+"_in"] if node in nodes_to_split else node_indices[node]

    # Add internal capacity edges for split nodes
    for node in sorted(nodes_to_split):
        node_cap = float(capacity_map[node])
        in_idx = node_indices[node+"_in"]; out_idx = node_indices[node+"_out"]
        forward, backward = solver.add_directed_edge(in_idx, out_idx, node_cap)
        forward.label = f"{node}__internal"

    # Add original network edges with lower/upper bound transformation
    core_edge_list = []
    for edge_spec in sorted(edge_list, key=lambda x: (x["from"], x["to"])):
        src_idx = get_outgoing_node(edge_spec["from"]); dst_idx = get_incoming_node(edge_spec["to"])
        lo_bound = float(edge_spec.get("lo", 0.0)); hi_bound = float(edge_spec.get("hi", 0.0))
        # Check for infeasible bounds
        if hi_bound < lo_bound - 1e-12:
            return {"status":"infeasible","cut_reachable":[],"deficit":{"demand_balance":float(sum(source_nodes.values())),"tight_nodes":[],"tight_edges":[]}}
        forward, backward = solver.add_directed_edge(src_idx, dst_idx, hi_bound - lo_bound)
        forward.is_core_edge = True; forward.source_node = edge_spec["from"]; forward.target_node = edge_spec["to"]
        forward.lower_bound = lo_bound; forward.upper_bound = hi_bound; forward.label = f"{edge_spec['from']}->{edge_spec['to']}"
        core_edge_list.append(forward)

    # Calculate flow imbalance due to lower bounds
    node_balance = {name: 0.0 for name in node_names}
    for forward in core_edge_list:
        if forward.lower_bound > 0:
            node_balance[node_names[get_incoming_node(forward.target_node)]] += forward.lower_bound
            node_balance[node_names[get_outgoing_node(forward.source_node)]] -= forward.lower_bound

    # Create temporary circulation edges from sink to sources
    temp_edge_pairs = []
    sink_outgoing = get_outgoing_node(sink_node)
    for src_name, supply_value in sorted(source_nodes.items()):
        src_out = get_outgoing_node(src_name)
        circ_forward, circ_backward = solver.add_directed_edge(sink_outgoing, src_out, float(supply_value))
        temp_edge_pairs.append((circ_forward, solver.graph[src_out][circ_forward.back_idx]))

    # Add auxiliary nodes for checking lower bound feasibility
    aux_source = len(node_names); aux_sink = aux_source + 1
    solver.graph.append([]); solver.graph.append([]); solver.node_count += 2

    total_demand = 0.0
    for node_name, balance_value in sorted(node_balance.items()):
        if balance_value > EPSILON:
            total_demand += balance_value
            solver.add_directed_edge(aux_source, node_indices[node_name], balance_value)
        elif balance_value < -EPSILON:
            solver.add_directed_edge(node_indices[node_name], aux_sink, -balance_value)

    # Check if lower bounds are satisfiable
    feasibility_flow = solver.compute_max_flow(aux_source, aux_sink)
    if feasibility_flow < total_demand - 1e-7:
        distances = solver.build_level_graph(aux_source, aux_sink)
        reachable_indices = [idx for idx, dist in enumerate(distances) if idx < len(node_names) and dist >= 0]
        reachable_names = [node_names[idx] for idx in reachable_indices]
        cut_nodes = {n.replace("_in","").replace("_out","") for n in reachable_names}
        
        # Identify saturated edges crossing the cut (lower bound infeasibility)
        saturated_edges_lb = []
        for node_idx in reachable_indices:
            for edge in solver.graph[node_idx]:
                if edge.dest < len(node_names) and distances[edge.dest] < 0:
                    if edge.is_core_edge and edge.capacity <= 1e-7:
                        # This edge is saturated in the transformed network
                        flow_deficit = edge.lower_bound - edge.current_flow
                        saturated_edges_lb.append({
                            "from": edge.source_node, 
                            "to": edge.target_node, 
                            "flow_needed": float(max(0, flow_deficit))
                        })
        
        # Identify saturated node capacities in the cut
        saturated_nodes_lb = []
        for node in sorted(set(capacity_map.keys()) - set(source_nodes.keys()) - {sink_node}):
            in_idx = node_indices[node+"_in"] if node in nodes_to_split else None
            if in_idx is not None and distances[in_idx] >= 0:
                for edge in solver.graph[in_idx]:
                    if edge.label == f"{node}__internal" and edge.capacity <= 1e-7:
                        saturated_nodes_lb.append(node)
        
        return {
            "status":"infeasible",
            "cut_reachable": sorted(cut_nodes),
            "deficit":{
                "demand_balance":float(total_demand - feasibility_flow),
                "tight_nodes":sorted(saturated_nodes_lb),
                "tight_edges":saturated_edges_lb
            }
        }

    # Clean up auxiliary structure
    for node_idx in range(len(solver.graph)):
        for edge in solver.graph[node_idx]:
            if edge.dest == aux_source or edge.dest == aux_sink:
                edge.capacity = 0.0
                edge.current_flow = 0.0
    solver.graph[aux_source] = []; solver.graph[aux_sink] = []

    # Remove temporary circulation edges
    for circ_fwd, circ_bwd in temp_edge_pairs:
        circ_fwd.capacity = 0.0; circ_fwd.current_flow = 0.0
        circ_bwd.capacity = 0.0; circ_bwd.current_flow = 0.0

    # Solve main max-flow problem from sources to sink
    super_source = len(solver.graph); solver.graph.append([]); solver.node_count += 1
    total_supply = 0.0
    for src_name, supply_value in sorted(source_nodes.items()):
        total_supply += float(supply_value)
        solver.add_directed_edge(super_source, get_outgoing_node(src_name), float(supply_value))
    target_sink = get_incoming_node(sink_node)

    flow_delivered = solver.compute_max_flow(super_source, target_sink)
    # Check if all supply can be delivered
    if flow_delivered < total_supply - 1e-7:
        distances = solver.build_level_graph(super_source, target_sink)
        reachable_indices = [idx for idx, dist in enumerate(distances) if dist >= 0 and idx < len(node_names)]
        reachable_names = [node_names[idx] for idx in reachable_indices]
        cut_nodes = {v.replace("_in","").replace("_out","") for v in reachable_names}
        
        # Identify edges at full capacity in the min-cut
        saturated_edges = []
        for node_idx in reachable_indices:
            for edge in solver.graph[node_idx]:
                if edge.dest < len(node_names) and distances[edge.dest] < 0:
                    if edge.is_core_edge and edge.capacity <= EPSILON:
                        # Edge is at upper bound; it's a bottleneck
                        saturated_edges.append({
                            "from": edge.source_node, 
                            "to": edge.target_node, 
                            "flow_needed": 0.0
                        })
        
        # Identify capacity-saturated nodes in the min-cut
        saturated_nodes = []
        for node in sorted(set(capacity_map.keys()) - set(source_nodes.keys()) - {sink_node}):
            in_idx = node_indices.get(node+"_in")
            if in_idx is not None and in_idx < len(distances) and distances[in_idx] >= 0:
                for edge in solver.graph[in_idx]:
                    if edge.label == f"{node}__internal" and edge.capacity <= EPSILON:
                        saturated_nodes.append(node)
                        break
        return {
            "status":"infeasible",
            "cut_reachable": sorted(cut_nodes),
            "deficit":{"demand_balance":float(total_supply - flow_delivered),"tight_nodes":sorted(saturated_nodes),"tight_edges":saturated_edges}
        }

    # Extract final flow values (add back lower bounds)
    final_flows = []
    for edge in core_edge_list:
        flow_value = edge.current_flow + edge.lower_bound
        # Handle numerical precision issues
        if flow_value < 0 and flow_value > -1e-7: flow_value = 0.0
        final_flows.append({"from": edge.source_node, "to": edge.target_node, "flow": float(flow_value)})

    return {"status":"ok","max_flow_per_min":float(flow_delivered),"flows":final_flows}

def main():
    """
    Entry point: Read JSON input from stdin, solve the problem, and output JSON result.
    """
    input_data = json.load(sys.stdin)
    result = solve_belts(input_data)
    json.dump(result, sys.stdout, separators=(",", ":"), ensure_ascii=False)

if __name__ == "__main__":
    main()
