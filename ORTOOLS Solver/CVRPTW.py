from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
import folium
import csv

def create_data_model():
    """Stores the data for the problem by reading from CSV files."""
    data = {}
    
    # Read locations
    data["locations"] = []
    data["node_names"] = []
    with open("Data/locations.csv", mode="r", newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            data["locations"].append( (lat, lon) )
            data["node_names"].append(row["name"])
    
    num_locations = len(data["locations"])
    
    # Read time_matrix
    data["time_matrix"] = []
    with open("Data/time_matrix.csv", mode="r", newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        for row in reader:
            # Convert each cell to integer
            data["time_matrix"].append( [int(cell) for cell in row] )
    
    if len(data["time_matrix"]) != num_locations:
        raise ValueError("Time matrix rows do not match number of locations.")
    for row in data["time_matrix"]:
        if len(row) != num_locations:
            raise ValueError("Time matrix columns do not match number of locations.")
    
    # Read time_windows
    data["time_windows"] = [ (0,0) ] * num_locations  # Initialize with dummy values
    with open("Data/time_windows.csv", mode="r", newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            node_id = int(row["node_id"])
            start_time = int(row["start_time"])
            end_time = int(row["end_time"])
            data["time_windows"][node_id] = (start_time, end_time)
    
    data["num_vehicles"] = 4
    data["depot"] = 0
    
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{node} ({data['node_names'][node]}) "
                f"Time({solution.Min(time_var)},{solution.Max(time_var)}) -> "
            )
            index = solution.Value(routing.NextVar(index))
        node = manager.IndexToNode(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{node} ({data['node_names'][node]}) "
            f"Time({solution.Min(time_var)},{solution.Max(time_var)})\n"
        )
        plan_output += f"Time of the route: {solution.Min(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
    print(f"Total time of all routes: {total_time}min")

def plot_routes(data, manager, routing, solution):
    """Plot routes using Matplotlib."""
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Vehicle Routing Problem with Time Windows (Matplotlib)")
    colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta', 'yellow']
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        # No need to append End node separately
        plt.plot(
            [data["locations"][i][1] for i in route],  # Longitude as x-axis
            [data["locations"][i][0] for i in route],  # Latitude as y-axis
            marker="o",
            linestyle="-",
            label=f"Route {vehicle_id}",
            color=colors[vehicle_id % len(colors)],
        )
        # Annotate nodes
        for node in route:
            plt.text(data["locations"][node][1], data["locations"][node][0], str(node))
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_map(data, manager, routing, solution):
    """Plot routes using Folium."""
    depot_location = data["locations"][data["depot"]]
    m = folium.Map(location=[depot_location[0], depot_location[1]], zoom_start=12)
    
    # Add markers for each location
    for idx, (lat, lon) in enumerate(data["locations"]):
        folium.Marker(
            location=[lat, lon],
            popup=f"Node {idx}: {data['node_names'][idx]}",
            icon=folium.Icon(color='blue' if idx == data["depot"] else 'red', icon='info-sign')
        ).add_to(m)
    
    colors = ['blue', 'green', 'red', 'purple', 'orange', 'darkred', 'lightred', 'beige',
              'darkblue', 'darkgreen', 'cadetblue', 'darkpurple', 'white', 'pink', 'lightblue',
              'lightgreen', 'gray', 'black', 'lightgray']
    
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))  # Add end node
        route_points = [(data["locations"][node][0], data["locations"][node][1]) for node in route]
        folium.PolyLine(
            locations=route_points,
            color=colors[vehicle_id % len(colors)],
            weight=5,
            opacity=0.8,
            popup=f"Route {vehicle_id}"
        ).add_to(m)
    
    # Save the map to an HTML file
    m.save("FoliumViz/ortoolsviz_map.html")
    print("Map has been saved to vrptw_map.html")

def main():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    depot_idx = data["depot"]
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
        )

    # Instantiate route start and end times to produce feasible times.
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
        plot_routes(data, manager, routing, solution)
        plot_map(data, manager, routing, solution)
    else:
        print("No solution found!")

if __name__ == "__main__":
    main()