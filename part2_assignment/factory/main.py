#!/usr/bin/env python3
"""
Factory production optimization solver using linear programming.
This module implements a two-phase simplex algorithm to solve factory production
planning problems with resource and machine constraints.
"""
import sys
import json
import math
from typing import Dict, List, Tuple

# Numerical tolerance for floating-point comparisons
EPSILON = 1e-9

# ========== Two-Phase Simplex Algorithm (Deterministic with Bland's Rule) ==========
class SimplexResult:
    """
    Container for linear programming solution results.
    
    Attributes:
        status: Solution status ("optimal", "infeasible", or "unbounded")
        x: Solution vector (variable values)
        obj: Objective function value
    """
    def __init__(self, status:str, x=None, obj=None):
        self.status = status
        self.x = x
        self.obj = obj

class TwoPhaseSimplex:
    """
    Two-phase simplex solver for linear programming.
    
    Solves: minimize c^T x
    Subject to: A_eq x = b_eq, A_ub x <= b_ub, x >= 0
    
    Uses Bland's rule for deterministic pivot selection to prevent cycling.
    """
    def __init__(self, A_eq, b_eq, A_ub, b_ub, c):
        """
        Initialize the linear program.
        
        Args:
            A_eq: Equality constraint matrix
            b_eq: Equality constraint RHS vector
            A_ub: Inequality constraint matrix
            b_ub: Inequality constraint RHS vector
            c: Objective function coefficient vector
        """
        import copy
        self.A_eq = [row[:] for row in A_eq]
        self.b_eq = b_eq[:]
        self.A_ub = [row[:] for row in A_ub]
        self.b_ub = b_ub[:]
        self.c = c[:]
        self.n_orig = len(c)
        # Ensure all equality constraint RHS values are non-negative
        for i, b in enumerate(self.b_eq):
            if b < 0:
                self.b_eq[i] = -b
                self.A_eq[i] = [-a for a in self.A_eq[i]]
        # Ensure all inequality constraint RHS values are non-negative
        for i, b in enumerate(self.b_ub):
            if b < 0:
                # Convert to >= constraint by multiplying by -1
                self.b_ub[i] = -b
                self.A_ub[i] = [-a for a in self.A_ub[i]]

    def solve(self)->SimplexResult:
        """
        Execute two-phase simplex algorithm.
        
        Phase I: Find feasible basic solution using artificial variables
        Phase II: Optimize original objective from feasible starting point
        
        Returns:
            SimplexResult with status and solution if found
        """
        # Construct standard form tableau
        m_ub = len(self.A_ub)
        m_eq = len(self.A_eq)
        n = self.n_orig

        # Introduce slack variables for inequality constraints
        # Introduce artificial variables for equality constraints
        # Variable ordering: [original_vars (n)] + [slacks (m_ub)] + [artificials (m_eq)]
        total_vars = n + m_ub + m_eq

        # Construct constraint matrix rows with basic variables
        rows = []
        basis = []  # Track which variable is basic in each row
        
        # Process inequality constraints: add slack variables as initial basis
        for i in range(m_ub):
            row = [0.0]*total_vars
            # Copy original variable coefficients
            Ai = self.A_ub[i]
            for j in range(n):
                row[j] = Ai[j]
            # Add slack variable
            slack_idx = n + i
            row[slack_idx] = 1.0
            rows.append((row, float(self.b_ub[i])))
            basis.append(slack_idx)
            
        # Process equality constraints: add artificial variables as initial basis
        for i in range(m_eq):
            row = [0.0]*total_vars
            Ai = self.A_eq[i]
            for j in range(n):
                row[j] = Ai[j]
            artificial_idx = n + m_ub + i
            row[artificial_idx] = 1.0
            rows.append((row, float(self.b_eq[i])))
            basis.append(artificial_idx)

        # Phase I objective: minimize sum of artificial variables
        obj = [0.0]*total_vars
        for i in range(m_eq):
            artificial_idx = n + m_ub + i
            obj[artificial_idx] = 1.0

        # Construct initial tableau (each row includes RHS as last element)
        tableau = [r[0] + [r[1]] for r in rows]
        
        # Compute reduced cost row: objective minus weighted sum of basic rows
        obj_row = obj[:] + [0.0]
        # Adjust for current basis (slacks have cost 0, artificials have cost 1)
        for ri in range(len(tableau)):
            bvar = basis[ri]
            cost = obj[bvar]
            if abs(cost) > 0:
                # Subtract cost times constraint row from objective row
                row = tableau[ri]
                for k in range(len(obj_row)):
                    obj_row[k] -= cost * row[k]
                    
        # Execute Phase I to find feasible solution
        res = self._simplex(tableau, obj_row, basis, n, m_ub, m_eq)
        if res is None:
            return SimplexResult("unbounded", None, None)
        status, tableau, obj_row, basis = res
        if status != "optimal" or obj_row[-1] < -1e-8:
            return SimplexResult("infeasible", None, None)

        # Prepare for Phase II: remove artificial variables from tableau
        art_cols = list(range(n + m_ub, n + m_ub + m_eq))
        
        # Pivot out any artificial variables still in basis
        for ri, bvar in enumerate(basis):
            if bvar in art_cols:
                # Find a non-artificial column with non-zero coefficient to pivot
                enter = None
                # Use Bland's rule: select smallest indexed non-artificial column
                for j in range(n + m_ub):
                    if abs(tableau[ri][j]) > 1e-12:
                        enter = j
                        break
                if enter is not None:
                    self._pivot(tableau, obj_row, basis, ri, enter)

        # Remove artificial variable columns from tableau structure
        keep_cols = [j for j in range(n + m_ub + m_eq) if j < n + m_ub]
        # Create index mapping from old to new column indices
        idx_map = {}
        for new_j, old_j in enumerate(keep_cols):
            idx_map[old_j] = new_j
            
        def shrink_row(row):
            """Extract non-artificial columns plus RHS."""
            return [row[j] for j in keep_cols] + [row[-1]]
            
        tableau = [shrink_row(row) for row in tableau]
        obj_row = [obj_row[j] for j in keep_cols] + [obj_row[-1]]
        
        # Update basis indices after column removal
        new_basis = []
        for bvar in basis:
            if bvar in idx_map:
                new_basis.append(idx_map[bvar])
            else:
                # Row with artificial basic variable (should be all zeros except RHS)
                new_basis.append(None)
                
        # Remove redundant rows (those with None basis)
        filtered_tableau = []
        filtered_basis = []
        for ri, bvar in enumerate(new_basis):
            if bvar is None:
                # Verify RHS is essentially zero; otherwise problem is infeasible
                if abs(tableau[ri][-1]) > 1e-8:
                    return SimplexResult("infeasible", None, None)
                # Skip this redundant row
                continue
            filtered_tableau.append(tableau[ri])
            filtered_basis.append(bvar)
        tableau = filtered_tableau
        basis = filtered_basis

        # Phase II: optimize original objective function
        total_vars_phase2 = n + m_ub
        c_full = [0.0]*total_vars_phase2
        for j in range(n):
            c_full[j] = self.c[j]
        # Construct reduced cost row for Phase II
        obj_row = c_full[:] + [0.0]
        for ri in range(len(tableau)):
            bvar = basis[ri]
            cb = c_full[bvar]
            if abs(cb) > 0:
                row = tableau[ri]
                for k in range(len(obj_row)):
                    obj_row[k] -= cb * row[k]

        # Execute Phase II to find optimal solution
        res2 = self._simplex(tableau, obj_row, basis, n, m_ub, 0)
        if res2 is None:
            return SimplexResult("unbounded", None, None)
        status2, tableau, obj_row, basis = res2
        if status2 != "optimal":
            return SimplexResult(status2, None, None)

        # Extract solution vector from final basis
        x = [0.0]*n
        for ri, bvar in enumerate(basis):
            if bvar is not None and bvar < n:
                x[bvar] = tableau[ri][-1]
        obj_val = obj_row[-1]
        return SimplexResult("optimal", x, obj_val)

    def _simplex(self, tableau, obj_row, basis, n, m_ub, m_eq):
        """
        Core simplex algorithm iteration using Bland's rule.
        
        Args:
            tableau: Current tableau matrix
            obj_row: Reduced cost row
            basis: List of basic variable indices
            n: Number of original variables
            m_ub: Number of inequality constraints
            m_eq: Number of equality constraints
            
        Returns:
            Tuple of (status, tableau, obj_row, basis) or None if unbounded
        """
        # Tableau structure: each row is [coefficients..., rhs]
        num_rows = len(tableau)
        num_vars = len(tableau[0]) - 1  # Exclude RHS column
        max_iterations = 10000 + 10*num_vars + 10*num_rows  # Prevent infinite loops
        iteration_count = 0
        
        while True:
            iteration_count += 1
            if iteration_count > max_iterations:
                # Should not occur with Bland's rule preventing cycling
                break
                
            # Select entering variable using Bland's rule: smallest index with negative reduced cost
            entering_var = None
            for j in range(num_vars):
                if obj_row[j] < -1e-12:
                    entering_var = j
                    break
            if entering_var is None:
                # Current solution is optimal
                return ("optimal", tableau, obj_row, basis)
                
            # Determine leaving variable using minimum ratio test with Bland's tie-breaking
            min_ratio = None
            leaving_row = None
            for i in range(num_rows):
                pivot_elem = tableau[i][entering_var]
                if pivot_elem > 1e-12:
                    ratio = tableau[i][-1] / pivot_elem
                    # Handle negative ratios (shouldn't occur in Phase I structure)
                    if ratio < -1e-12:
                        ratio = float('inf')
                    # Select minimum ratio, break ties by smallest basis index
                    if (min_ratio is None) or (ratio < min_ratio - 1e-12) or \
                       (abs(ratio - min_ratio) <= 1e-12 and (basis[i] is None or \
                        (basis[leaving_row] is not None and basis[i] < basis[leaving_row]))):
                        min_ratio = ratio
                        leaving_row = i
            if leaving_row is None:
                # Problem is unbounded
                return None
                
            # Perform pivot operation
            self._pivot(tableau, obj_row, basis, leaving_row, entering_var)

        return ("iteration_limit", tableau, obj_row, basis)

    def _pivot(self, tableau, obj_row, basis, leave_row, enter_col):
        """
        Perform a simplex pivot operation.
        
        Args:
            tableau: Current tableau matrix
            obj_row: Reduced cost row
            basis: List of basic variable indices
            leave_row: Row index of leaving variable
            enter_col: Column index of entering variable
        """
        # Normalize pivot row by dividing by pivot element
        pivot_element = tableau[leave_row][enter_col]
        reciprocal = 1.0 / pivot_element
        row_length = len(tableau[0])  # Includes RHS column
        for k in range(row_length):
            tableau[leave_row][k] *= reciprocal
            
        # Eliminate entering column from all other rows
        num_rows = len(tableau)
        for i in range(num_rows):
            if i == leave_row:
                continue
            multiplier = tableau[i][enter_col]
            if abs(multiplier) > 1e-12:
                for k in range(row_length):
                    tableau[i][k] -= multiplier * tableau[leave_row][k]
                    
        # Update objective row (reduced costs)
        obj_multiplier = obj_row[enter_col]
        if abs(obj_multiplier) > 1e-12:
            for k in range(row_length):
                obj_row[k] -= obj_multiplier * tableau[leave_row][k]
                
        # Update basis tracking
        basis[leave_row] = enter_col

# ========== Factory Production Solver ==========

def solve_factory(data:dict)->dict:
    """
    Solve factory production optimization problem.
    
    Determines optimal recipe usage to achieve target production rate
    while minimizing machine usage and respecting resource constraints.
    
    Args:
        data: Input dictionary with machines, recipes, modules, limits, and target
        
    Returns:
        Dictionary with solution status and production plan or infeasibility analysis
    """
    machines = data["machines"]
    recipes = data["recipes"]
    modules = data.get("modules", {})
    limits = data.get("limits", {})
    raw_caps = limits.get("raw_supply_per_min", {})
    max_machines = limits.get("max_machines", {})
    target = data["target"]
    target_item = target["item"]
    target_rate = float(target["rate_per_min"])

    # Ensure deterministic processing order
    recipe_names = sorted(recipes.keys())
    
    # Extract productivity and speed modifiers per machine type
    productivity_modifiers = {}
    speed_modifiers = {}
    for machine_name in machines.keys():
        mod = modules.get(machine_name, {})
        productivity_modifiers[machine_name] = float(mod.get("prod", 0.0))
        speed_modifiers[machine_name] = float(mod.get("speed", 0.0))

    # Calculate effective crafting rates for each recipe
    effective_crafts = {}
    recipe_to_machine = {}
    for recipe in recipe_names:
        recipe_def = recipes[recipe]
        machine = recipe_def["machine"]
        craft_time = float(recipe_def["time_s"])
        base_crafts_per_min = float(machines[machine]["crafts_per_min"])
        # Apply speed modifier and time to get effective rate
        effective_rate = base_crafts_per_min * (1.0 + speed_modifiers[machine]) * 60.0 / craft_time
        effective_crafts[recipe] = effective_rate
        recipe_to_machine[recipe] = machine

    # Collect all items involved in the production network
    items_set = set()
    for recipe in recipe_names:
        for item in recipes[recipe].get("in", {}).keys():
            items_set.add(item)
        for item in recipes[recipe].get("out", {}).keys():
            items_set.add(item)
    items_set |= set(raw_caps.keys())
    items = sorted(items_set)

    # Build net production coefficients: output*(1+prod_bonus) - input
    item_coefficients = {}  # Maps item to list of coefficients per recipe
    for item in items:
        row = []
        for recipe in recipe_names:
            recipe_def = recipes[recipe]
            machine = recipe_def["machine"]
            prod_bonus = productivity_modifiers[machine]
            output_qty = recipe_def.get("out", {}).get(item, 0.0)
            input_qty = recipe_def.get("in", {}).get(item, 0.0)
            # Net production coefficient
            coeff = output_qty * (1.0 + prod_bonus) - input_qty
            row.append(float(coeff))
        item_coefficients[item] = row

    # Construct equality constraints: target item and intermediate items
    equality_matrix = []
    equality_rhs = []
    
    # Handle target item constraint
    if target_item not in items:
        # Target item not in production network - check if achievable
        if abs(target_rate) <= EPSILON:
            pass  # Zero production is trivially achievable
        else:
            # May be available as raw material if capacity permits
            pass
    equality_matrix.append(item_coefficients.get(target_item, [0.0]*len(recipe_names)))
    equality_rhs.append(target_rate)

    # Add balance constraints for intermediate items
    for item in items:
        if item == target_item:
            continue
        if item in raw_caps:
            # Raw materials handled via inequality constraints
            continue
        # Net production must equal zero (flow balance)
        equality_matrix.append(item_coefficients[item])
        equality_rhs.append(0.0)

    # Construct inequality constraints: raw material and machine capacity limits
    inequality_matrix = []
    inequality_rhs = []
    
    # Raw material constraints: cannot produce raw, consumption limited by capacity
    for item in sorted(raw_caps.keys()):
        row = item_coefficients.get(item, [0.0]*len(recipe_names))
        # Constraint 1: net production <= 0 (cannot create raw materials)
        inequality_matrix.append(row[:])
        inequality_rhs.append(0.0)
        # Constraint 2: consumption <= capacity (flip sign: -net_production <= cap)
        inequality_matrix.append([-coef for coef in row])
        inequality_rhs.append(float(raw_caps[item]))

    # Machine capacity constraints
    for machine in sorted(max_machines.keys()):
        cap = float(max_machines[machine])
        row = [0.0]*len(recipe_names)
        for idx, recipe in enumerate(recipe_names):
            if recipe_to_machine[recipe] == machine:
                row[idx] = 1.0 / effective_crafts[recipe]
        inequality_matrix.append(row)
        inequality_rhs.append(cap)

    # Solve LP: Check feasibility of target rate and minimize machine usage
    # Objective: minimize total machine count = sum(recipe_rate / effective_rate)
    objective_coeffs = [1.0/effective_crafts[recipe] for recipe in recipe_names]
    lp_solver = TwoPhaseSimplex(equality_matrix, equality_rhs, inequality_matrix, inequality_rhs, objective_coeffs)
    result = lp_solver.solve()
    
    if result.status == "optimal":
        solution = result.x
        # Compute per-recipe production rates
        per_recipe = {recipe_names[i]: float(solution[i]) for i in range(len(recipe_names))}
        
        # Aggregate machine usage by type
        per_machine = {}
        for recipe in recipe_names:
            machine = recipe_to_machine[recipe]
            per_machine[machine] = per_machine.get(machine, 0.0) + per_recipe[recipe] / effective_crafts[recipe]
            
        # Calculate raw material consumption
        raw_use = {}
        for item in sorted(raw_caps.keys()):
            coeffs = item_coefficients.get(item, [0.0]*len(recipe_names))
            net_production = sum(coeffs[j]*solution[j] for j in range(len(recipe_names)))
            consumption = max(0.0, -net_production)  # Positive consumption (inputs minus outputs)
            raw_use[item] = consumption
            
        # Return successful solution
        return {
            "status": "ok",
            "per_recipe_crafts_per_min": per_recipe,
            "per_machine_counts": {k: float(v) for k, v in per_machine.items()},
            "raw_consumption_per_min": {k: float(v) for k, v in raw_use.items()},
        }
    else:
        # Target is infeasible - compute maximum achievable rate
        # Maximize net target production subject to all constraints except target equality
        target_coeffs = [item_coefficients.get(target_item, [0.0]*len(recipe_names))[j] for j in range(len(recipe_names))]
        # Formulate as minimization: minimize -target_coeffs^T x
        lp_solver2 = TwoPhaseSimplex(
            A_eq=[row[:] for k,row in enumerate(equality_matrix) if k != 0],  # Exclude target equality
            b_eq=[b for k,b in enumerate(equality_rhs) if k != 0],
            A_ub=inequality_matrix[:],
            b_ub=inequality_rhs[:],
            c=[-coef for coef in target_coeffs],
        )
        result2 = lp_solver2.solve()
        max_achievable_rate = 0.0
        fallback_solution = [0.0]*len(recipe_names)
        
        if result2.status == "optimal":
            max_achievable_rate = sum(target_coeffs[j]*result2.x[j] for j in range(len(recipe_names)))
            fallback_solution = result2.x
        else:
            max_achievable_rate = 0.0

        # Diagnose bottlenecks: identify saturated constraints
        bottleneck_hints = []
        
        # Check machine capacity constraints
        for machine_idx, machine in enumerate(sorted(max_machines.keys())):
            constraint_row = inequality_matrix[len(raw_caps)*2 + machine_idx]
            machines_used = sum(constraint_row[j]*fallback_solution[j] for j in range(len(recipe_names)))
            capacity = inequality_rhs[len(raw_caps)*2 + machine_idx]
            if capacity - machines_used <= 1e-7:
                bottleneck_hints.append(f"{machine} cap")
                
        # Check raw material supply constraints
        for item_idx, item in enumerate(sorted(raw_caps.keys())):
            # Check net production constraint (should not produce raw materials)
            row = inequality_matrix[2*item_idx]
            net_prod = sum(row[j]*fallback_solution[j] for j in range(len(recipe_names)))
            if net_prod >= -1e-7:
                # Acceptable: not producing raw materials
                pass
            # Check consumption capacity constraint
            consumption_row = inequality_matrix[2*item_idx + 1]
            consumed = sum(consumption_row[j]*fallback_solution[j] for j in range(len(recipe_names)))
            capacity = inequality_rhs[2*item_idx + 1]
            if capacity - consumed <= 1e-7:
                bottleneck_hints.append(f"{item} supply")

        bottleneck_hints = sorted(set(bottleneck_hints))
        return {
            "status": "infeasible",
            "max_feasible_target_per_min": float(max_achievable_rate),
            "bottleneck_hint": bottleneck_hints
        }

def main():
    """
    Entry point: Read JSON input, solve factory problem, output JSON result.
    """
    data = json.load(sys.stdin)
    out = solve_factory(data)
    
    # Ensure deterministic output by sorting dictionary keys
    def sort_dict(d):
        """Sort dictionary by keys for consistent output."""
        return {k: d[k] for k in sorted(d.keys())}
        
    if out.get("per_recipe_crafts_per_min"):
        out["per_recipe_crafts_per_min"] = sort_dict(out["per_recipe_crafts_per_min"])
    if out.get("per_machine_counts"):
        out["per_machine_counts"] = sort_dict(out["per_machine_counts"])
    if out.get("raw_consumption_per_min"):
        out["raw_consumption_per_min"] = sort_dict(out["raw_consumption_per_min"])
    json.dump(out, sys.stdout, separators=(",", ":"), ensure_ascii=False)

if __name__ == "__main__":
    main()
