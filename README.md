
# Vehicle Routing Problem (VRP) Optimization with Gurobi

## Overview
This project implements two variations of the Vehicle Routing Problem (VRP) using Python and Gurobi, aimed at optimizing routes for a set of vehicles to cover a network of locations. The project includes:
- **CVRP_Combined.py**: Capacitated Vehicle Routing Problem (CVRP) without time windows.
- **CVRPTW_Combined.py**: Capacitated Vehicle Routing Problem with Time Windows (CVRPTW), adding constraints on the time available for visits to each location.

Each script uses Gurobi to minimize the number of vehicles used and optimize the travel route. Additionally, routes are visualized using Folium maps and Matplotlib, and saved in CSV format for easy reference.

## Features
- **Route Optimization**: Minimize the number of vehicles and optimize paths based on distance and, where applicable, time windows.
- **Capacity Constraints**: Enforce vehicle capacity limits to ensure efficient resource allocation.
- **Time Windows**: In `CVRPTW_Combined.py`, each location has a specific time window for visits.
- **Visualization**: View optimized routes on an interactive Folium map or Matplotlib plot.
- **Export to CSV**: Save optimized routes, including details like vehicle assignments and travel times (where applicable).

## Prerequisites
- Python 3.x
- [Gurobi](https://www.gurobi.com/): Ensure you have a Gurobi license and have installed the `gurobipy` library.
- Python libraries: Install the following dependencies.
  ```bash
  pip install pandas numpy folium matplotlib geopy
  ```

## Setup
1. **Prepare Data Files**:
   - `locations.csv`: List of locations, including latitude, longitude, and demand for each.
   - For `CVRPTW_Combined.py`, add a `time_windows.csv` file defining the time windows for each location.

2. **Configure Parameters**:
   - **Vehicle Capacity**: Default set to 100, but you can adjust this in the code based on your requirements.
   - **Maximum Vehicles**: The maximum number of vehicles is set in the script. You can change this if needed.

## Usage
### Running the Scripts
1. **Capacitated VRP (CVRP)**:
   ```bash
   python CVRP_Combined.py
   ```
   This runs the CVRP optimization, solving for the minimum number of vehicles and optimized routes without considering time windows.

2. **Capacitated VRP with Time Windows (CVRPTW)**:
   ```bash
   python CVRPTW_Combined.py
   ```
   This adds time window constraints, optimizing the schedule alongside the route.

### Output
- **CSV Files**:
  - `CVRP_Combined.py`: Outputs to `Vehicle_Assignment.csv`.
  - `CVRPTW_Combined.py`: Outputs to `Vehicle_Assignment.csv` with added time information.
- **Folium Map**: Visualizes routes with vehicle assignments and capacity usage, saved as HTML (`CVRP.html` or `CVRPTW.html`).
- **Matplotlib Plot**: Provides a quick, static view of routes with assigned colors and labels.

## Key Functions
- **solve_vrp**: Defines and solves the optimization problem using Gurobi.
- **create_distance_matrix**: Calculates distance between each location based on latitude and longitude.
- **extract_routes**: Extracts and prints optimized routes and capacity usage per vehicle.
- **plot_routes_on_map**: Generates a Folium map for interactive viewing of routes.
- **plot_routes_matplotlib**: Creates a static plot using Matplotlib.

## Notes
- Ensure Gurobi license activation and path configuration if running on a new environment.
- Modify vehicle capacity, time windows, and maximum vehicle count as needed to fit specific use cases.

## License
This project uses Gurobi for optimization, which requires a valid license. Check Gurobi’s [license documentation](https://www.gurobi.com/downloads/end-user-license-agreement-eula/) for more information.

