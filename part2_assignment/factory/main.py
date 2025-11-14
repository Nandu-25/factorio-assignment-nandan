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
TOLERANCE = 1e-9

# ========== Two-Phase Simplex Algorithm (Deterministic with Bland's Rule) ==========
class LinearProgramSolution:
    """
    Container for linear programming solution results.
    
    Attributes:
        status: Solution status ("optimal", "infeasible", or "unbounded")
        solution_vector: Solution vector (variable values)
        objective_value: Objective function value
    """
    def __init__(self, status: str, solution_vector=None, objective_value=None):
        self.status = status
        self.solution_vector = solution_vector
        self.objective_value = objective_value

class SimplexSolver:
    """
    Two-phase simplex solver for linear programming.
    
    Solves: minimize c^T x
    Subject to: A_eq x = b_eq, A_ub x <= b_ub, x >= 0
    
    Uses Bland's rule for deterministic pivot selection to prevent cycling.
    """
    def __init__(self, equality_matrix, equality_rhs, inequality_matrix, inequality_rhs, objective_coefficients):
        """
        Initialize the linear program.
        
        Args:
            equality_matrix: Equality constraint matrix
            equality_rhs: Equality constraint RHS vector
            inequality_matrix: Inequality constraint matrix
            inequality_rhs: Inequality constraint RHS vector
            objective_coefficients: Objective function coefficient vector
        """
        import copy
        self.equality_matrix = [row[:] for row in equality_matrix]
        self.equality_rhs = equality_rhs[:]
        self.inequality_matrix = [row[:] for row in inequality_matrix]
        self.inequality_rhs = inequality_rhs[:]
        self.objective_coefficients = objective_coefficients[:]
        self.num_original_variables = len(objective_coefficients)
        
        # Ensure all equality constraint RHS values are non-negative
        for i, rhs_value in enumerate(self.equality_rhs):
            if rhs_value < 0:
                self.equality_rhs[i] = -rhs_value
                self.equality_matrix[i] = [-coeff for coeff in self.equality_matrix[i]]
        
        # Ensure all inequality constraint RHS values are non-negative
        for i, rhs_value in enumerate(self.inequality_rhs):
            if rhs_value < 0:
                # Convert to >= constraint by multiplying by -1
                self.inequality_rhs[i] = -rhs_value
                self.inequality_matrix[i] = [-coeff for coeff in self.inequality_matrix[i]]

    def solve(self) -> LinearProgramSolution:
        """
        Execute two-phase simplex algorithm.
        
        Phase I: Find feasible basic solution using artificial variables
        Phase II: Optimize original objective from feasible starting point
        
        Returns:
            LinearProgramSolution with status and solution if found
        """
        # Construct standard form tableau
        num_inequality_constraints = len(self.inequality_matrix)
        num_equality_constraints = len(self.equality_matrix)
        num_vars = self.num_original_variables

        # Introduce slack variables for inequality constraints
        # Introduce artificial variables for equality constraints
        # Variable ordering: [original_vars (n)] + [slacks (m_ub)] + [artificials (m_eq)]
        total_variables = num_vars + num_inequality_constraints + num_equality_constraints

        # Construct constraint matrix rows with basic variables
        constraint_rows = []
        basic_variables = []  # Track which variable is basic in each row
        
        # Process inequality constraints: add slack variables as initial basis
        for i in range(num_inequality_constraints):
            row = [0.0] * total_variables
            # Copy original variable coefficients
            constraint_coeffs = self.inequality_matrix[i]
            for j in range(num_vars):
                row[j] = constraint_coeffs[j]
            # Add slack variable
            slack_variable_index = num_vars + i
            row[slack_variable_index] = 1.0
            constraint_rows.append((row, float(self.inequality_rhs[i])))
            basic_variables.append(slack_variable_index)
            
        # Process equality constraints: add artificial variables as initial basis
        for i in range(num_equality_constraints):
            row = [0.0] * total_variables
            constraint_coeffs = self.equality_matrix[i]
            for j in range(num_vars):
                row[j] = constraint_coeffs[j]
            artificial_variable_index = num_vars + num_inequality_constraints + i
            row[artificial_variable_index] = 1.0
            constraint_rows.append((row, float(self.equality_rhs[i])))
            basic_variables.append(artificial_variable_index)

        # Phase I objective: minimize sum of artificial variables
        phase1_objective = [0.0] * total_variables
        for i in range(num_equality_constraints):
            artificial_variable_index = num_vars + num_inequality_constraints + i
            phase1_objective[artificial_variable_index] = 1.0

        # Construct initial tableau (each row includes RHS as last element)
        tableau = [r[0] + [r[1]] for r in constraint_rows]
        
        # Compute reduced cost row: objective minus weighted sum of basic rows
        reduced_cost_row = phase1_objective[:] + [0.0]
        # Adjust for current basis (slacks have cost 0, artificials have cost 1)
        for row_index in range(len(tableau)):
            basic_var = basic_variables[row_index]
            variable_cost = phase1_objective[basic_var]
            if abs(variable_cost) > 0:
                # Subtract cost times constraint row from objective row
                row = tableau[row_index]
                for k in range(len(reduced_cost_row)):
                    reduced_cost_row[k] -= variable_cost * row[k]
                    
        # Execute Phase I to find feasible solution
        phase1_result = self._run_simplex_iterations(tableau, reduced_cost_row, basic_variables, 
                                                       num_vars, num_inequality_constraints, num_equality_constraints)
        if phase1_result is None:
            return LinearProgramSolution("unbounded", None, None)
        
        status, tableau, reduced_cost_row, basic_variables = phase1_result
        if status != "optimal" or reduced_cost_row[-1] < -1e-8:
            return LinearProgramSolution("infeasible", None, None)

        # Prepare for Phase II: remove artificial variables from tableau
        artificial_column_indices = list(range(num_vars + num_inequality_constraints, 
                                               num_vars + num_inequality_constraints + num_equality_constraints))
        
        # Pivot out any artificial variables still in basis
        for row_index, basic_var in enumerate(basic_variables):
            if basic_var in artificial_column_indices:
                # Find a non-artificial column with non-zero coefficient to pivot
                entering_variable = None
                # Use Bland's rule: select smallest indexed non-artificial column
                for j in range(num_vars + num_inequality_constraints):
                    if abs(tableau[row_index][j]) > 1e-12:
                        entering_variable = j
                        break
                if entering_variable is not None:
                    self._perform_pivot(tableau, reduced_cost_row, basic_variables, row_index, entering_variable)

        # Remove artificial variable columns from tableau structure
        columns_to_keep = [j for j in range(num_vars + num_inequality_constraints + num_equality_constraints) 
                          if j < num_vars + num_inequality_constraints]
        # Create index mapping from old to new column indices
        column_index_map = {}
        for new_j, old_j in enumerate(columns_to_keep):
            column_index_map[old_j] = new_j
            
        def extract_kept_columns(row):
            """Extract non-artificial columns plus RHS."""
            return [row[j] for j in columns_to_keep] + [row[-1]]
            
        tableau = [extract_kept_columns(row) for row in tableau]
        reduced_cost_row = [reduced_cost_row[j] for j in columns_to_keep] + [reduced_cost_row[-1]]
        
        # Update basis indices after column removal
        updated_basic_variables = []
        for basic_var in basic_variables:
            if basic_var in column_index_map:
                updated_basic_variables.append(column_index_map[basic_var])
            else:
                # Row with artificial basic variable (should be all zeros except RHS)
                updated_basic_variables.append(None)
                
        # Remove redundant rows (those with None basis)
        filtered_tableau = []
        filtered_basic_variables = []
        for row_index, basic_var in enumerate(updated_basic_variables):
            if basic_var is None:
                # Verify RHS is essentially zero; otherwise problem is infeasible
                if abs(tableau[row_index][-1]) > 1e-8:
                    return LinearProgramSolution("infeasible", None, None)
                # Skip this redundant row
                continue
            filtered_tableau.append(tableau[row_index])
            filtered_basic_variables.append(basic_var)
        tableau = filtered_tableau
        basic_variables = filtered_basic_variables

        # Phase II: optimize original objective function
        phase2_num_variables = num_vars + num_inequality_constraints
        phase2_objective = [0.0] * phase2_num_variables
        for j in range(num_vars):
            phase2_objective[j] = self.objective_coefficients[j]
        
        # Construct reduced cost row for Phase II
        reduced_cost_row = phase2_objective[:] + [0.0]
        for row_index in range(len(tableau)):
            basic_var = basic_variables[row_index]
            basic_var_cost = phase2_objective[basic_var]
            if abs(basic_var_cost) > 0:
                row = tableau[row_index]
                for k in range(len(reduced_cost_row)):
                    reduced_cost_row[k] -= basic_var_cost * row[k]

        # Execute Phase II to find optimal solution
        phase2_result = self._run_simplex_iterations(tableau, reduced_cost_row, basic_variables, 
                                                       num_vars, num_inequality_constraints, 0)
        if phase2_result is None:
            return LinearProgramSolution("unbounded", None, None)
        
        status2, tableau, reduced_cost_row, basic_variables = phase2_result
        if status2 != "optimal":
            return LinearProgramSolution(status2, None, None)

        # Extract solution vector from final basis
        solution_vector = [0.0] * num_vars
        for row_index, basic_var in enumerate(basic_variables):
            if basic_var is not None and basic_var < num_vars:
                solution_vector[basic_var] = tableau[row_index][-1]
        objective_value = reduced_cost_row[-1]
        return LinearProgramSolution("optimal", solution_vector, objective_value)

    def _run_simplex_iterations(self, tableau, reduced_cost_row, basic_variables, 
                                 num_vars, num_inequality_constraints, num_equality_constraints):
        """
        Core simplex algorithm iteration using Bland's rule.
        
        Args:
            tableau: Current tableau matrix
            reduced_cost_row: Reduced cost row
            basic_variables: List of basic variable indices
            num_vars: Number of original variables
            num_inequality_constraints: Number of inequality constraints
            num_equality_constraints: Number of equality constraints
            
        Returns:
            Tuple of (status, tableau, reduced_cost_row, basic_variables) or None if unbounded
        """
        # Tableau structure: each row is [coefficients..., rhs]
        num_rows = len(tableau)
        num_columns = len(tableau[0]) - 1  # Exclude RHS column
        max_iterations = 10000 + 10 * num_columns + 10 * num_rows  # Prevent infinite loops
        iteration_count = 0
        
        while True:
            iteration_count += 1
            if iteration_count > max_iterations:
                # Should not occur with Bland's rule preventing cycling
                break
                
            # Select entering variable using Bland's rule: smallest index with negative reduced cost
            entering_variable = None
            for j in range(num_columns):
                if reduced_cost_row[j] < -1e-12:
                    entering_variable = j
                    break
            if entering_variable is None:
                # Current solution is optimal
                return ("optimal", tableau, reduced_cost_row, basic_variables)
                
            # Determine leaving variable using minimum ratio test with Bland's tie-breaking
            minimum_ratio = None
            leaving_row = None
            for i in range(num_rows):
                pivot_coefficient = tableau[i][entering_variable]
                if pivot_coefficient > 1e-12:
                    ratio = tableau[i][-1] / pivot_coefficient
                    # Handle negative ratios (shouldn't occur in Phase I structure)
                    if ratio < -1e-12:
                        ratio = float('inf')
                    # Select minimum ratio, break ties by smallest basis index
                    if (minimum_ratio is None) or (ratio < minimum_ratio - 1e-12) or \
                       (abs(ratio - minimum_ratio) <= 1e-12 and (basic_variables[i] is None or \
                        (basic_variables[leaving_row] is not None and basic_variables[i] < basic_variables[leaving_row]))):
                        minimum_ratio = ratio
                        leaving_row = i
            if leaving_row is None:
                # Problem is unbounded
                return None
                
            # Perform pivot operation
            self._perform_pivot(tableau, reduced_cost_row, basic_variables, leaving_row, entering_variable)

        return ("iteration_limit", tableau, reduced_cost_row, basic_variables)

    def _perform_pivot(self, tableau, reduced_cost_row, basic_variables, leaving_row_index, entering_column_index):
        """
        Perform a simplex pivot operation.
        
        Args:
            tableau: Current tableau matrix
            reduced_cost_row: Reduced cost row
            basic_variables: List of basic variable indices
            leaving_row_index: Row index of leaving variable
            entering_column_index: Column index of entering variable
        """
        # Normalize pivot row by dividing by pivot element
        pivot_element = tableau[leaving_row_index][entering_column_index]
        reciprocal = 1.0 / pivot_element
        row_length = len(tableau[0])  # Includes RHS column
        for k in range(row_length):
            tableau[leaving_row_index][k] *= reciprocal
            
        # Eliminate entering column from all other rows
        num_rows = len(tableau)
        for i in range(num_rows):
            if i == leaving_row_index:
                continue
            multiplier = tableau[i][entering_column_index]
            if abs(multiplier) > 1e-12:
                for k in range(row_length):
                    tableau[i][k] -= multiplier * tableau[leaving_row_index][k]
                    
        # Update objective row (reduced costs)
        objective_multiplier = reduced_cost_row[entering_column_index]
        if abs(objective_multiplier) > 1e-12:
            for k in range(row_length):
                reduced_cost_row[k] -= objective_multiplier * tableau[leaving_row_index][k]
                
        # Update basis tracking
        basic_variables[leaving_row_index] = entering_column_index

# ========== Factory Production Solver ==========

def solve_factory(input_data: dict) -> dict:
    """
    Solve factory production optimization problem.
    
    Determines optimal recipe usage to achieve target production rate
    while minimizing machine usage and respecting resource constraints.
    
    Args:
        input_data: Input dictionary with machines, recipes, modules, limits, and target
        
    Returns:
        Dictionary with solution status and production plan or infeasibility analysis
    """
    machines = input_data["machines"]
    recipes = input_data["recipes"]
    modules = input_data.get("modules", {})
    limits = input_data.get("limits", {})
    raw_material_capacities = limits.get("raw_supply_per_min", {})
    machine_capacity_limits = limits.get("max_machines", {})
    target_specification = input_data["target"]
    target_item_name = target_specification["item"]
    target_production_rate = float(target_specification["rate_per_min"])

    # Ensure deterministic processing order
    recipe_names_sorted = sorted(recipes.keys())
    
    # Extract productivity and speed modifiers per machine type
    productivity_bonus_by_machine = {}
    speed_bonus_by_machine = {}
    for machine_name in machines.keys():
        module_config = modules.get(machine_name, {})
        productivity_bonus_by_machine[machine_name] = float(module_config.get("prod", 0.0))
        speed_bonus_by_machine[machine_name] = float(module_config.get("speed", 0.0))

    # Calculate effective crafting rates for each recipe
    effective_crafts_per_min = {}
    recipe_machine_mapping = {}
    for recipe_name in recipe_names_sorted:
        recipe_definition = recipes[recipe_name]
        machine_type = recipe_definition["machine"]
        craft_time_seconds = float(recipe_definition["time_s"])
        base_crafts_per_minute = float(machines[machine_type]["crafts_per_min"])
        # Apply speed modifier and time to get effective rate
        effective_rate = base_crafts_per_minute * (1.0 + speed_bonus_by_machine[machine_type]) * 60.0 / craft_time_seconds
        effective_crafts_per_min[recipe_name] = effective_rate
        recipe_machine_mapping[recipe_name] = machine_type

    # Collect all items involved in the production network
    all_items_set = set()
    for recipe_name in recipe_names_sorted:
        for item_name in recipes[recipe_name].get("in", {}).keys():
            all_items_set.add(item_name)
        for item_name in recipes[recipe_name].get("out", {}).keys():
            all_items_set.add(item_name)
    all_items_set |= set(raw_material_capacities.keys())
    all_items_sorted = sorted(all_items_set)

    # Build net production coefficients: output*(1+prod_bonus) - input
    net_production_coefficients = {}  # Maps item to list of coefficients per recipe
    for item_name in all_items_sorted:
        coefficient_row = []
        for recipe_name in recipe_names_sorted:
            recipe_definition = recipes[recipe_name]
            machine_type = recipe_definition["machine"]
            productivity_multiplier = productivity_bonus_by_machine[machine_type]
            output_quantity = recipe_definition.get("out", {}).get(item_name, 0.0)
            input_quantity = recipe_definition.get("in", {}).get(item_name, 0.0)
            # Net production coefficient
            net_coefficient = output_quantity * (1.0 + productivity_multiplier) - input_quantity
            coefficient_row.append(float(net_coefficient))
        net_production_coefficients[item_name] = coefficient_row

    # Construct equality constraints: target item and intermediate items
    equality_constraint_matrix = []
    equality_constraint_rhs = []
    
    # Handle target item constraint
    if target_item_name not in all_items_sorted:
        # Target item not in production network - check if achievable
        if abs(target_production_rate) <= TOLERANCE:
            pass  # Zero production is trivially achievable
        else:
            # May be available as raw material if capacity permits
            pass
    equality_constraint_matrix.append(net_production_coefficients.get(target_item_name, [0.0] * len(recipe_names_sorted)))
    equality_constraint_rhs.append(target_production_rate)

    # Add balance constraints for intermediate items
    for item_name in all_items_sorted:
        if item_name == target_item_name:
            continue
        if item_name in raw_material_capacities:
            # Raw materials handled via inequality constraints
            continue
        # Net production must equal zero (flow balance)
        equality_constraint_matrix.append(net_production_coefficients[item_name])
        equality_constraint_rhs.append(0.0)

    # Construct inequality constraints: raw material and machine capacity limits
    inequality_constraint_matrix = []
    inequality_constraint_rhs = []
    
    # Raw material constraints: cannot produce raw, consumption limited by capacity
    for item_name in sorted(raw_material_capacities.keys()):
        coefficient_row = net_production_coefficients.get(item_name, [0.0] * len(recipe_names_sorted))
        # Constraint 1: net production <= 0 (cannot create raw materials)
        inequality_constraint_matrix.append(coefficient_row[:])
        inequality_constraint_rhs.append(0.0)
        # Constraint 2: consumption <= capacity (flip sign: -net_production <= cap)
        inequality_constraint_matrix.append([-coef for coef in coefficient_row])
        inequality_constraint_rhs.append(float(raw_material_capacities[item_name]))

    # Machine capacity constraints
    for machine_type in sorted(machine_capacity_limits.keys()):
        capacity_limit = float(machine_capacity_limits[machine_type])
        constraint_row = [0.0] * len(recipe_names_sorted)
        for recipe_index, recipe_name in enumerate(recipe_names_sorted):
            if recipe_machine_mapping[recipe_name] == machine_type:
                constraint_row[recipe_index] = 1.0 / effective_crafts_per_min[recipe_name]
        inequality_constraint_matrix.append(constraint_row)
        inequality_constraint_rhs.append(capacity_limit)

    # Solve LP: Check feasibility of target rate and minimize machine usage
    # Objective: minimize total machine count = sum(recipe_rate / effective_rate)
    minimization_objective = [1.0 / effective_crafts_per_min[recipe_name] for recipe_name in recipe_names_sorted]
    
    lp_solver = SimplexSolver(equality_constraint_matrix, equality_constraint_rhs, 
                               inequality_constraint_matrix, inequality_constraint_rhs, 
                               minimization_objective)
    solution_result = lp_solver.solve()
    
    if solution_result.status == "optimal":
        solution_values = solution_result.solution_vector
        # Compute per-recipe production rates
        recipe_crafts_per_minute = {recipe_names_sorted[i]: float(solution_values[i]) 
                                    for i in range(len(recipe_names_sorted))}
        
        # Aggregate machine usage by type
        machine_counts_by_type = {}
        for recipe_name in recipe_names_sorted:
            machine_type = recipe_machine_mapping[recipe_name]
            machines_needed = recipe_crafts_per_minute[recipe_name] / effective_crafts_per_min[recipe_name]
            machine_counts_by_type[machine_type] = machine_counts_by_type.get(machine_type, 0.0) + machines_needed
            
        # Calculate raw material consumption
        raw_material_consumption = {}
        for item_name in sorted(raw_material_capacities.keys()):
            coefficients = net_production_coefficients.get(item_name, [0.0] * len(recipe_names_sorted))
            net_production = sum(coefficients[j] * solution_values[j] for j in range(len(recipe_names_sorted)))
            consumption_amount = max(0.0, -net_production)  # Positive consumption (inputs minus outputs)
            raw_material_consumption[item_name] = consumption_amount
            
        # Return successful solution
        return {
            "status": "ok",
            "per_recipe_crafts_per_min": recipe_crafts_per_minute,
            "per_machine_counts": {machine_type: float(count) for machine_type, count in machine_counts_by_type.items()},
            "raw_consumption_per_min": {item: float(amount) for item, amount in raw_material_consumption.items()},
        }
    else:
        # Target is infeasible - compute maximum achievable rate
        # Maximize net target production subject to all constraints except target equality
        target_production_coefficients = [net_production_coefficients.get(target_item_name, [0.0] * len(recipe_names_sorted))[j] 
                                          for j in range(len(recipe_names_sorted))]
        # Formulate as minimization: minimize -target_coeffs^T x
        maximization_solver = SimplexSolver(
            equality_matrix=[row[:] for k, row in enumerate(equality_constraint_matrix) if k != 0],  # Exclude target equality
            equality_rhs=[rhs for k, rhs in enumerate(equality_constraint_rhs) if k != 0],
            inequality_matrix=inequality_constraint_matrix[:],
            inequality_rhs=inequality_constraint_rhs[:],
            objective_coefficients=[-coef for coef in target_production_coefficients],
        )
        maximization_result = maximization_solver.solve()
        maximum_achievable_rate = 0.0
        fallback_solution_values = [0.0] * len(recipe_names_sorted)
        
        if maximization_result.status == "optimal":
            maximum_achievable_rate = sum(target_production_coefficients[j] * maximization_result.solution_vector[j] 
                                          for j in range(len(recipe_names_sorted)))
            fallback_solution_values = maximization_result.solution_vector
        else:
            maximum_achievable_rate = 0.0

        # Diagnose bottlenecks: identify saturated constraints
        bottleneck_descriptions = []
        
        # Check machine capacity constraints
        for machine_index, machine_type in enumerate(sorted(machine_capacity_limits.keys())):
            constraint_coefficients = inequality_constraint_matrix[len(raw_material_capacities) * 2 + machine_index]
            machines_utilized = sum(constraint_coefficients[j] * fallback_solution_values[j] 
                                   for j in range(len(recipe_names_sorted)))
            capacity = inequality_constraint_rhs[len(raw_material_capacities) * 2 + machine_index]
            if capacity - machines_utilized <= TOLERANCE:
                bottleneck_descriptions.append(f"{machine_type} cap")
                
        # Check raw material supply constraints
        for item_index, item_name in enumerate(sorted(raw_material_capacities.keys())):
            # Check net production constraint (should not produce raw materials)
            net_production_row = inequality_constraint_matrix[2 * item_index]
            net_production_value = sum(net_production_row[j] * fallback_solution_values[j] 
                                      for j in range(len(recipe_names_sorted)))
            if net_production_value >= -TOLERANCE:
                # Acceptable: not producing raw materials
                pass
            # Check consumption capacity constraint
            consumption_constraint_row = inequality_constraint_matrix[2 * item_index + 1]
            consumption_amount = sum(consumption_constraint_row[j] * fallback_solution_values[j] 
                                    for j in range(len(recipe_names_sorted)))
            capacity = inequality_constraint_rhs[2 * item_index + 1]
            if capacity - consumption_amount <= TOLERANCE:
                bottleneck_descriptions.append(f"{item_name} supply")

        bottleneck_descriptions = sorted(set(bottleneck_descriptions))
        return {
            "status": "infeasible",
            "max_feasible_target_per_min": float(maximum_achievable_rate),
            "bottleneck_hint": bottleneck_descriptions
        }

def main():
    """
    Entry point: Read JSON input, solve factory problem, output JSON result.
    """
    input_data = json.load(sys.stdin)
    output_result = solve_factory(input_data)
    
    # Ensure deterministic output by sorting dictionary keys
    def sort_dictionary_keys(dictionary):
        """Sort dictionary by keys for consistent output."""
        return {key: dictionary[key] for key in sorted(dictionary.keys())}
        
    if output_result.get("per_recipe_crafts_per_min"):
        output_result["per_recipe_crafts_per_min"] = sort_dictionary_keys(output_result["per_recipe_crafts_per_min"])
    if output_result.get("per_machine_counts"):
        output_result["per_machine_counts"] = sort_dictionary_keys(output_result["per_machine_counts"])
    if output_result.get("raw_consumption_per_min"):
        output_result["raw_consumption_per_min"] = sort_dictionary_keys(output_result["raw_consumption_per_min"])
    
    json.dump(output_result, sys.stdout, separators=(",", ":"), ensure_ascii=False)

if __name__ == "__main__":
    main()
