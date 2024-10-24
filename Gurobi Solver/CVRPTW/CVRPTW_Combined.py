import pandas as pd
import numpy as np
import folium
import matplotlib.pyplot as plt
from gurobipy import Model, GRB, quicksum
from geopy.distance import geodesic

# Load the location data
locations_df = pd.read_csv("Data/locations.csv")  # Replace with the path to your 'locations.csv'
time_windows_df = pd.read_csv("Data/time_windows.csv")  # Load time windows

# Define demand and vehicle capacity
demands = pd.to_numeric(locations_df['demand'], errors='coerce').fillna(0).astype(int).values
vehicle_capacity = 100  # Vehicle capacity, adjust as needed

# Extract time windows for each node
time_windows = time_windows_df[['start_time', 'end_time']].values

# Create a distance matrix based on latitude and longitude
def create_distance_matrix(locations_df):
    num_locations = len(locations_df)
    distance_matrix = np.zeros((num_locations, num_locations))
    for i in range(num_locations):
        for j in range(num_locations):
            if i != j:
                loc1 = (locations_df.iloc[i]['latitude'], locations_df.iloc[i]['longitude'])
                loc2 = (locations_df.iloc[j]['latitude'], locations_df.iloc[j]['longitude'])
                distance_matrix[i][j] = geodesic(loc1, loc2).kilometers
    return distance_matrix

# Create the distance matrix
distance_matrix = create_distance_matrix(locations_df)

# Function to set up and solve the VRP model with dynamic number of vehicles and time windows
def solve_vrp(num_locations, max_vehicles, demands, vehicle_capacity, distance_matrix, time_windows, depot=0):
    # Create the Gurobi model
    model = Model()

    # Decision variables
    x = model.addVars(num_locations, num_locations, max_vehicles, vtype=GRB.BINARY, name="x")
    u = model.addVars(num_locations, max_vehicles, lb=0, ub=num_locations, vtype=GRB.CONTINUOUS, name="u")
    y = model.addVars(max_vehicles, vtype=GRB.BINARY, name="y")
    t = model.addVars(num_locations, vtype=GRB.CONTINUOUS, name="t")  # Time variables for time windows

    # Objective: Minimize the number of vehicles used + total time across routes
    model.setObjective(
        quicksum(y[k] for k in range(max_vehicles)) +
        0.001 * quicksum(t[j] for j in range(num_locations)),
        GRB.MINIMIZE
    )

    # Add constraints
    add_constraints(model, x, u, y, t, num_locations, max_vehicles, demands, vehicle_capacity, time_windows, depot)

    # Optimize the model
    model.optimize()

    if model.status == GRB.OPTIMAL:
        routes, capacity_usage = extract_routes(model, x, num_locations, max_vehicles, demands, t)
        save_routes_as_csv(routes, locations_df, time_windows)
        plot_routes_on_map(routes, locations_df, capacity_usage)
        plot_routes_matplotlib(routes, locations_df, capacity_usage)
    else:
        print("No optimal solution found.")

    return model

# Function to add constraints to the model
def add_constraints(model, x, u, y, t, num_locations, max_vehicles, demands, vehicle_capacity, time_windows, depot):
    # 1. Flow conservation - Vehicle leaves node that it enters
    model.addConstrs(
        quicksum(x[i, j, k] for j in range(num_locations) if i != j) ==
        quicksum(x[j, i, k] for j in range(num_locations) if i != j)
        for i in range(num_locations) for k in range(max_vehicles) if i != depot
    )

    # 2. Ensure that every node is entered exactly once (except depot)
    model.addConstrs(
        quicksum(x[i, j, k] for k in range(max_vehicles) for i in range(num_locations) if i != j) == 1
        for j in range(1, num_locations)
    )

    # 3. Every vehicle leaves the depot only if it is used
    model.addConstrs(quicksum(x[depot, j, k] for j in range(1, num_locations)) == y[k] for k in range(max_vehicles))

    # 4. Every vehicle returns to the depot only if it is used
    model.addConstrs(quicksum(x[i, depot, k] for i in range(1, num_locations)) == y[k] for k in range(max_vehicles))

    # 5. MTZ subtour elimination constraints
    for k in range(max_vehicles):
        for i in range(1, num_locations):
            model.addConstr(u[i, k] >= 1)
            model.addConstr(u[i, k] <= num_locations - 1)

            for j in range(1, num_locations):
                if i != j:
                    model.addConstr(u[i, k] - u[j, k] + (num_locations) * x[i, j, k] <= num_locations - 1)

    # 6. Capacity constraint
    model.addConstrs(
        quicksum(demands[i] * quicksum(x[i, j, k] for j in range(num_locations) if i != j) for i in range(1, num_locations)) <= vehicle_capacity * y[k]
        for k in range(max_vehicles)
    )

    # 7. Time window constraints
    for i in range(num_locations):
        model.addConstr(t[i] >= time_windows[i][0])  # Start time
        model.addConstr(t[i] <= time_windows[i][1])  # End time

    # 8. Ensure sequential usage of vehicles
    model.addConstrs(y[k] <= y[k-1] for k in range(1, max_vehicles))

# Function to extract and print routes from the solution and calculate capacity usage
def extract_routes(model, x, num_locations, max_vehicles, demands, t):
    routes = []
    capacity_usage = []
    for k in range(max_vehicles):
        if model.getVarByName(f"y[{k}]").X > 0.5:
            route = []
            current_location = 0
            visited_locations = {current_location}
            total_demand = 0
            print(f"\nRoute for Vehicle {k + 1}:")
            while True:
                for j in range(num_locations):
                    if x[current_location, j, k].X > 0.5 and j not in visited_locations:
                        route.append((current_location, j, k))
                        visited_locations.add(j)
                        total_demand += demands[j]
                        # Print the scheduled time for each location
                        visit_time = t[j].X
                        print(f"From {locations_df['name'].iloc[current_location]} to {locations_df['name'].iloc[j]}, Time: {visit_time:.2f}")
                        current_location = j
                        break
                else:
                    if x[current_location, 0, k].X > 0.5:
                        route.append((current_location, 0, k))
                        print(f"From {locations_df['name'].iloc[current_location]} back to {locations_df['name'].iloc[0]}")
                    break
                if current_location == 0:
                    break
            routes.append(route)
            capacity_percentage = (total_demand / vehicle_capacity) * 100
            capacity_usage.append((k + 1, capacity_percentage))
            print(f"Vehicle {k + 1} Capacity Usage: {capacity_percentage:.2f}%")
    return routes, capacity_usage

# Function to save the routes as a CSV file
def save_routes_as_csv(routes, locations_df, time_windows):
    route_data = []
    for vehicle_route in routes:
        for (i, j, k) in vehicle_route:
            route_data.append({
                "Vehicle": k + 1,
                "From": locations_df["name"].iloc[i],
                "To": locations_df["name"].iloc[j],
                "Latitude_From": locations_df["latitude"].iloc[i],
                "Longitude_From": locations_df["longitude"].iloc[i],
                "Latitude_To": locations_df["latitude"].iloc[j],
                "Longitude_To": locations_df["longitude"].iloc[j],
                "Start_Time": time_windows[j][0],
                "End_Time": time_windows[j][1]
            })

    route_df = pd.DataFrame(route_data)
    route_df.to_csv("Gurobi Solver/CVRPTW/Vehicle_Assignment.csv", index=False)
    print("Optimal routes saved to 'Gurobi Solver/CVRPTW/Vehicle_Assignment.csv'.")

# Function to plot routes on a Folium map with capacity usage
def plot_routes_on_map(routes, locations_df, capacity_usage):
    # Create a folium map centered at the depot
    depot_lat = locations_df.loc[locations_df['node_id'] == 0, 'latitude'].values[0]
    depot_lon = locations_df.loc[locations_df['node_id'] == 0, 'longitude'].values[0]
    m = folium.Map(location=[depot_lat, depot_lon], zoom_start=12)

    # Define colors for each vehicle
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'cadetblue']

    # Plot smaller circles at each location
    for idx, row in locations_df.iterrows():
        folium.CircleMarker(
            location=[row['latitude'], row['longitude']],
            radius=4,
            color='black',
            fill=True,
            fill_color='white',
            fill_opacity=0.7,
            popup=row['name']
        ).add_to(m)

    # Add routes
    for idx, route in enumerate(routes):
        vehicle_num, capacity_percentage = capacity_usage[idx]
        color = colors[vehicle_num % len(colors)]
        for (i, j, k) in route:
            folium.PolyLine(
                locations=[(locations_df['latitude'].iloc[i], locations_df['longitude'].iloc[i]),
                           (locations_df['latitude'].iloc[j], locations_df['longitude'].iloc[j])],
                color=color, weight=5, opacity=0.8
            ).add_to(m)

    # Custom legend for capacity usage
    legend_html = '''
     <div style="position: fixed; 
                 top: 10px; right: 10px; width: 200px; height: auto; 
                 background-color: white; border:2px solid grey; z-index:9999; font-size:14px;
                 padding: 10px;">
     <b>Vehicle Capacity Usage</b><br>
     '''
    
    for idx, (vehicle_num, capacity_percentage) in enumerate(capacity_usage):
        legend_html += f'<i style="background:{colors[(vehicle_num - 1) % len(colors)]};width:15px;height:15px;display:inline-block;margin-right:5px;"></i>'
        legend_html += f'Vehicle {vehicle_num}: {capacity_percentage:.2f}%<br>'
    
    legend_html += '</div>'
    m.get_root().html.add_child(folium.Element(legend_html))
    m.save("FoliumViz/CVRPTW.html")
    print("Map saved to 'FoliumViz/CVRPTW.html'.")

# Function to plot routes using Matplotlib with capacity usage
def plot_routes_matplotlib(routes, locations_df, capacity_usage):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Vehicle Routing Problem (Matplotlib)")
    colors = ['red', 'blue', 'green', 'purple', 'orange']

    for idx, route in enumerate(routes):
        vehicle_num, capacity_percentage = capacity_usage[idx]
        vehicle_color = colors[route[0][2] % len(colors)]
        for (i, j, k) in route:
            ax.plot([locations_df['longitude'].iloc[i], locations_df['longitude'].iloc[j]],
                    [locations_df['latitude'].iloc[i], locations_df['latitude'].iloc[j]],
                    marker='o', color=vehicle_color, label=f"Vehicle {vehicle_num} ({capacity_percentage:.2f}%)" if i == 0 else "")
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    handles, labels = ax.get_legend_handles_labels()
    unique_labels = dict(zip(labels, handles))
    ax.legend(unique_labels.values(), unique_labels.keys())
    plt.show()

# Main function to run everything
def main():
    max_vehicles = 6  # Maximum vehicles allowed
    depot = 0  # First location as the depot
    solve_vrp(len(locations_df), max_vehicles, demands, vehicle_capacity, distance_matrix, time_windows, depot)

if __name__ == "__main__":
    main()