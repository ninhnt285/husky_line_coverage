#!/usr/bin/env python
from geodesy import utm
import json

def load_nodes(node_file="./maps/node_data"):
    nodes = {}
    with open(node_file, "r") as f:
        lines = f.readlines()
        for line in lines:
            params = line.split()
            nodes[params[0]] = {"lat": float(params[3]), "long": float(params[4])}
    return nodes

# Load all routes and convert to UTM
# Note: included last point
def load_routes(route_file="./maps/route", node_file="./map/node_data"):
    routes = []

    if route_file[-4:] == "json":
        with open(route_file, "r") as f:
            data = json.load(f)

            nodes_arr = list(filter(lambda x: (x["type"] == "node"), data["elements"]))
            paths_arr = list(filter(lambda x: (x["type"] == "way"), data["elements"]))
            
            nodes = {}
            for node in nodes_arr:
                nodes[node["id"]] = node

            routes = []
            
            for path in paths_arr:
                for i in range(len(path["nodes"]) - 1):
                    node = nodes[path["nodes"][i]]
                    if len(routes) > 0 and node["id"] == routes[-1]["id"]:
                        continue

                    point = utm.fromLatLong(node["lat"], node["lon"]).toPoint()
                    node["x"] = point.x
                    node["y"] = point.y
                    routes.append(node)

            routes.append(routes[0])
    else:
        nodes = load_nodes(node_file)
        with open(route_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                params = line.split()
                id = params[0]
                lat = nodes[id]["lat"]
                long = nodes[id]["long"]
                point = utm.fromLatLong(lat, long).toPoint()

                routes.append({
                    "id": id,
                    "lat": lat,
                    "long": long,
                    "x": point.x,
                    "y": point.y
                })
            routes.append(routes[0])

    return routes

if __name__ == "__main__":
    print("Helper Functions")
    routes = load_routes("./maps/ww_circle/full.json")
    print(routes)