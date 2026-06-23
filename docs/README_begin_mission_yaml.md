# begin_mission_yaml.py

This Python script is a generic mission launcher designed to initiate sophisticated multi-drone missions by reading mission definitions from a structured YAML file.

It acts as a central point to distribute mission instructions (like takeoff plans or auction-based task assignments) to multiple ROS 2 drone nodes, ensuring a choreographed start to the mission.

## Key Functionality

1.  **YAML-Driven Configuration:** All mission parameters—such as which drones participate, rendezvous points, and the sequence of actions—are defined in an external YAML file.
2.  **Mission Distribution:** It reads the configuration and uses ROS 2 services/topics to publish `MissionUpdate` messages.
3.  **Differentiated Plans:** It intelligently distributes different mission plans based on the drone's role:
    *   **Auctioneer Drone:** Receives a sequence of missions, typically `takeoff` followed by `auction`.
    *   **Other Bidders:** Receive a simpler mission plan, usually just `takeoff`.

## YAML Schema Overview

The mission YAML file must adhere to a specific structure. The main sections include:

*   `mission_id`: Unique integer identifier for the mission.
*   `takeoff`: Specifies common takeoff parameters (e.g., `height`, `speed`).
*   `auctioneer`: The ID of the drone responsible for initiating the auction process.
*   `drones`: A list of all drone IDs participating in the mission (including the auctioneer).
*   `auction`: Details for the task assignment auction, including:
    *   `name`: The name of the task (e.g., `point_assignment`).
    *   `auction_type`: The technique used (e.g., `coordinate_item`).
    *   `elements`: A list of available items (e.g., points, zones) that are up for assignment.

## Usage

The script is run from the command line, requiring the path to the mission YAML file.

```bash
python3 begin_mission_yaml.py <path/to/mission.yaml> [-s]
```

*   **`<path/to/mission.yaml>`**: The mandatory path to the configuration file.
*   **`-s`** or **`--use_sim_time`**: Optional flag. When provided, the script assumes the simulation is running and uses the simulated time for publishing.
