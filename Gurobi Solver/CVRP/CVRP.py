import pandas as pd
import numpy as np
import folium
import matplotlib.pyplot as plt
from gurobipy import Model, GRB, quicksum
from geopy.distance import geodesic

# Load the location data
locations_df = pd.read_csv("Data/locations.csv")  # Replace with the path to your 'locations.csv'

# Define demand and vehicle capacity
# Convert demands to integers, handling non-numeric values gracefully
demands = pd.to_numeric(locations_df['demand'], errors='coerce').fillna(0).astype(int).values
vehicle_capacity = 100  # Vehicle capacity, adjust as needed

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

# Function to set up and solve the VRP model with dynamic number of vehicles
def solve_vrp(num_locations, max_vehicles, demands, vehicle_capacity, distance_matrix, depot=0):
    # Create the Gurobi model
    model = Model()

    # Decision variables
    x = model.addVars(num_locations, num_locations, max_vehicles, vtype=GRB.BINARY, name="x")  # Binary decision variables
    u = model.addVars(num_locations, max_vehicles, lb=0, ub=num_locations, vtype=GRB.CONTINUOUS, name="u")  # MTZ variables
    y = model.addVars(max_vehicles, vtype=GRB.BINARY, name="y")  # Binary variable to indicate if a vehicle is used

    # Objective: Minimize the number of vehicles used + total distance
    model.setObjective(
        quicksum(y[k] for k in range(max_vehicles)) + 
        0.001 * quicksum(x[i, j, k] * distance_matrix[i][j] for i in range(num_locations) for j in range(num_locations) for k in range(max_vehicles)),
        GRB.MINIMIZE
    )

    # Add constraints
    add_constraints(model, x, u, y, num_locations, max_vehicles, demands, vehicle_capacity, depot)

    # Optimize the model
    model.optimize()

    if model.status == GRB.OPTIMAL:
        routes, capacity_usage = extract_routes(model, x, num_locations, max_vehicles, demands)
        save_routes_as_csv(routes, locations_df)
        plot_routes_on_map(routes, locations_df, capacity_usage)
        plot_routes_matplotlib(routes, locations_df, capacity_usage)
    else:
        print("No optimal solution found.")

    return model

# Function to add constraints to the model
def add_constraints(model, x, u, y, num_locations, max_vehicles, demands, vehicle_capacity, depot):
    # 1. Flow conservation - Vehicle leaves node that it enters
    model.addConstrs(
        quicksum(x[i, j, k] for j in range(num_locations) if i != j) ==
        quicksum(x[j, i, k] for j in range(num_locations) if i != j)
        for i in range(num_locations) for k in range(max_vehicles) if i != depot
    )

    # 2. Ensure that every node is entered exactly once (except depot)
    model.addConstrs(
        quicksum(x[i, j, k] for k in range(max_vehicles) for i in range(num_locations) if i != j) == 1
        for j in range(1, num_locations)  # Nodes 1 to n must be visited exactly once
    )

    # 3. Every vehicle leaves the depot only if it is used
    model.addConstrs(quicksum(x[depot, j, k] for j in range(1, num_locations)) == y[k] for k in range(max_vehicles))

    # 4. Every vehicle returns to the depot only if it is used
    model.addConstrs(quicksum(x[i, depot, k] for i in range(1, num_locations)) == y[k] for k in range(max_vehicles))

    # 5. MTZ subtour elimination constraints
    for k in range(max_vehicles):
        for i in range(1, num_locations):
            model.addConstr(u[i, k] >= 1)  # u must be at least 1 for each customer
            model.addConstr(u[i, k] <= num_locations - 1)  # u cannot exceed n-1

            for j in range(1, num_locations):
                if i != j:
                    model.addConstr(u[i, k] - u[j, k] + (num_locations) * x[i, j, k] <= num_locations - 1)

    # 6. Capacity constraint
    # Ensure that each vehicle's total load does not exceed its capacity
    model.addConstrs(
        quicksum(demands[i] * quicksum(x[i, j, k] for j in range(num_locations) if i != j) for i in range(1, num_locations)) <= vehicle_capacity * y[k]
        for k in range(max_vehicles)
    )

    # 7. Ensure sequential usage of vehicles
    model.addConstrs(y[k] <= y[k-1] for k in range(1, max_vehicles))

# Function to extract and print routes from the solution and calculate capacity usage
def extract_routes(model, x, num_locations, max_vehicles, demands):
    routes = []
    capacity_usage = []
    for k in range(max_vehicles):
        if model.getVarByName(f"y[{k}]").X > 0.5:  # Only print routes for used vehicles
            route = []
            current_location = 0  # Start from the depot
            visited_locations = {current_location}
            total_demand = 0
            print(f"\nRoute for Vehicle {k + 1}:")  # Print the vehicle route header
            while True:
                for j in range(num_locations):
                    if x[current_location, j, k].X > 0.5 and j not in visited_locations:
                        route.append((current_location, j, k))
                        visited_locations.add(j)
                        total_demand += demands[j]
                        print(f"From {locations_df['name'].iloc[current_location]} to {locations_df['name'].iloc[j]}")
                        current_location = j
                        break
                else:
                    # Check if we need to return to the depot
                    if x[current_location, 0, k].X > 0.5:
                        route.append((current_location, 0, k))
                        print(f"From {locations_df['name'].iloc[current_location]} back to {locations_df['name'].iloc[0]}")
                    break  # No more locations to visit
                if current_location == 0:
                    break  # Returned to depot
            routes.append(route)
            capacity_percentage = (total_demand / vehicle_capacity) * 100
            capacity_usage.append((k + 1, capacity_percentage))
            print(f"Vehicle {k + 1} Capacity Usage: {capacity_percentage:.2f}%")
    return routes, capacity_usage

# Function to save the routes as a CSV file with car assignment
def save_routes_as_csv(routes, locations_df):
    route_data = []
    for vehicle_route in routes:
        for (i, j, k) in vehicle_route:
            route_data.append({
                "Vehicle": k + 1,  # Vehicle number starts from 1
                "From": locations_df["name"].iloc[i],
                "To": locations_df["name"].iloc[j],
                "Latitude_From": locations_df["latitude"].iloc[i],
                "Longitude_From": locations_df["longitude"].iloc[i],
                "Latitude_To": locations_df["latitude"].iloc[j],
                "Longitude_To": locations_df["longitude"].iloc[j]
            })

    route_df = pd.DataFrame(route_data)
    route_df.to_csv("Gurobi Solver/CVRP/Vehicle_Assignment.csv", index=False)
    print("Optimal routes saved to 'Gurobi Solver/CVRP/Vehicle_Assignment.csv'.")

# Function to plot routes on a Folium map with a legend for capacity usage and smaller markers
def plot_routes_on_map(routes, locations_df, capacity_usage):
    # Create a folium map centered at the depot
    m = folium.Map(location=[locations_df['latitude'][0], locations_df['longitude'][0]], zoom_start=12)

    # Define colors for each vehicle
    colors = ['red', 'blue', 'green', 'purple', 'orange']

    # Plot smaller circles at each location instead of the larger markers
    for idx, row in locations_df.iterrows():
        folium.CircleMarker(
            location=[row['latitude'], row['longitude']],
            radius=4,  # Smaller radius for less distraction
            color='black',  # Border color
            fill=True,
            fill_color='white',  # Fill color
            fill_opacity=0.7,
            popup=row['name']
        ).add_to(m)

    # Plot the routes with distinct colors for each vehicle
    for idx, route in enumerate(routes):
        vehicle_num, capacity_percentage = capacity_usage[idx]
        for (i, j, k) in route:
            folium.PolyLine(
                locations=[(locations_df['latitude'].iloc[i], locations_df['longitude'].iloc[i]),
                           (locations_df['latitude'].iloc[j], locations_df['longitude'].iloc[j])],
                color=colors[k % len(colors)], weight=5, opacity=0.8
            ).add_to(m)

    # Create a custom HTML legend
    legend_html = '''
     <div style="position: fixed; 
                 top: 10px; right: 10px; width: 200px; height: 120px; 
                 background-color: white; border:2px solid grey; z-index:9999; font-size:14px;
                 padding: 10px;">
     <b>Vehicle Capacity Usage</b><br>
     '''
    
    # Add each vehicle's usage to the legend
    for idx, (vehicle_num, capacity_percentage) in enumerate(capacity_usage):
        legend_html += f'<i style="background:{colors[vehicle_num - 1]};width:15px;height:15px;display:inline-block;margin-right:5px;"></i>'
        legend_html += f'Vehicle {vehicle_num}: {capacity_percentage:.2f}%<br>'
    
    # Close the div
    legend_html += '</div>'

    # Add the legend to the map
    m.get_root().html.add_child(folium.Element(legend_html))

    # Save the map to an HTML file
    m.save("FoliumViz/CRVP.html")
    print("Map saved to 'FoliumViz/CRVP.html'.")

# Function to plot routes using Matplotlib
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
    max_vehicles = 3  # Maximum vehicles allowed
    depot = 0  # First location as the depot
    solve_vrp(len(locations_df), max_vehicles, demands, vehicle_capacity, distance_matrix, depot)

if __name__ == "__main__":
    main()