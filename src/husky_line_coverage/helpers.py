#!/usr/bin/env python
from geodesy import utm

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
    nodes = load_nodes(node_file)
    routes = []
    
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