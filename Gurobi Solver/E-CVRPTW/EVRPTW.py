import gurobipy as gp
from gurobipy import GRB
import numpy as np

# Load data from file
customers = []
recharging_stations = []
deport = None
vehicle_capacity = None
battery_capacity = None
fuel_consumption_rate = None
refueling_rate = None
velocity = None

with open("evrptw_instances/r102C15.txt") as file:
    next(file)  # Skip header line
    for line in file:
        parts = line.split()
        if len(parts) < 2:
            continue  # Skip empty or invalid lines
        if parts[1] == 'c':
            customers.append({
                'id': parts[0],
                'x': float(parts[2]),
                'y': float(parts[3]),
                'demand': float(parts[4]),
                'ready_time': float(parts[5]),
                'due_date': float(parts[6]),
                'service_time': float(parts[7])
            })
        elif parts[1] == 'f':
            recharging_stations.append({
                'id': parts[0],
                'x': float(parts[2]),
                'y': float(parts[3])
            })
        elif parts[1] == 'd':
            depot = {
                'id': parts[0],
                'x': float(parts[2]),
                'y': float(parts[3]),
                'due_date': float(parts[6])
            }
        elif parts[0] == 'Q':
            battery_capacity = float(parts[-1].strip('/'))
        elif parts[0] == 'C':
            vehicle_capacity = float(parts[-1].strip('/'))
        elif parts[0] == 'r':
            fuel_consumption_rate = float(parts[-1].strip('/'))
        elif parts[0] == 'g':
            refueling_rate = float(parts[-1].strip('/'))
        elif parts[0] == 'v':
            velocity = float(parts[-1].strip('/'))

# Create Gurobi model
model = gp.Model("E-VRPTW")

# Sets
nodes = [depot] + customers + recharging_stations
customers_only = customers

# Parameters
n = len(nodes)
distance = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        distance[i, j] = np.hypot(nodes[i]['x'] - nodes[j]['x'], nodes[i]['y'] - nodes[j]['y'])

# Variables
x = model.addVars(n, n, vtype=GRB.BINARY, name="x")
t = model.addVars(n, vtype=GRB.CONTINUOUS, name="t")
u = model.addVars(n, vtype=GRB.CONTINUOUS, name="u")
y = model.addVars(n, vtype=GRB.CONTINUOUS, name="y")

# Objective: Minimize total distance traveled
model.setObjective(gp.quicksum(distance[i, j] * x[i, j] for i in range(n) for j in range(n) if i != j), GRB.MINIMIZE)

# Constraints
for i in range(1, len(customers) + 1):  # Visit each customer exactly once
    model.addConstr(gp.quicksum(x[i, j] for j in range(n) if i != j) == 1, name=f"visit_{i}")
    model.addConstr(gp.quicksum(x[j, i] for j in range(n) if i != j) == 1, name=f"arrive_{i}")

# Flow constraints
for j in range(n):
    model.addConstr(gp.quicksum(x[0, j] for j in range(1, n)) <= 1, name="departure_from_depot")
    model.addConstr(gp.quicksum(x[j, 0] for j in range(1, n)) <= 1, name="return_to_depot")

# Time window constraints
for i in range(1, len(customers) + 1):
    customer = customers[i - 1]
    model.addConstr(t[i] >= customer['ready_time'], name=f"ready_time_{i}")
    model.addConstr(t[i] <= customer['due_date'], name=f"due_date_{i}")

# Battery capacity and cargo load constraints
for i in range(1, len(customers) + 1):
    model.addConstr(u[i] <= vehicle_capacity, name=f"vehicle_capacity_{i}")
    model.addConstr(y[i] <= battery_capacity, name=f"battery_capacity_{i}")

# Subtour elimination constraints (using MTZ formulation)
for i in range(1, len(customers) + 1):
    for j in range(1, len(customers) + 1):
        if i != j:
            model.addConstr(u[i] - u[j] + vehicle_capacity * x[i, j] <= vehicle_capacity - customers[j - 1]['demand'], name=f"subtour_elim_{i}_{j}")

# Solve the model
model.optimize()

# Extract the solution
if model.status == GRB.OPTIMAL:
    routes = []
    for i in range(n):
        for j in range(n):
            if i != j and x[i, j].X > 0.5:
                routes.append((nodes[i]['id'], nodes[j]['id']))
    
    # Format the output similar to Table 5
    total_distance = model.objVal
    num_vehicles = len(set(route[0] for route in routes if route[0] == depot['id']))
    num_recharges = len([node for node in nodes if node['id'].startswith('f')])
    
    print("Optimal Solution Summary:")
    print(f"Number of Vehicles Used: {num_vehicles}")
    print(f"Total Distance Traveled: {total_distance}")
    print(f"Number of Recharges: {num_recharges}")
    print("Routes:")
    for route in routes:
        print(f"{route[0]} -> {route[1]}")
else:
    print("No optimal solution found.")
