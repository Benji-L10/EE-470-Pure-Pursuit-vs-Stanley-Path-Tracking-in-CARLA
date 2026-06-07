import carla
import csv

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
world_map = world.get_map()

spawn = world_map.get_spawn_points()[0]

waypoint = world_map.get_waypoint(spawn.location)

points = []

for _ in range(300):
    points.append([
        round(waypoint.transform.location.x, 3),
        round(waypoint.transform.location.y, 3)
    ])

    next_waypoints = waypoint.next(2.0)

    if not next_waypoints:
        break

    waypoint = next_waypoints[0]

with open("paths/smooth_curvy.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    writer.writerows(points)

print(f"Generated {len(points)} waypoints")