import sys
import tkinter as tk
from tkinter import ttk
import random

def build_graph(N, M, roads):
    """Build the graph using adjacency list representation."""
    graph = [[] for _ in range(N + 1)]
    for u, v, d, t in roads:
        cost = t  # Use traffic delay as cost
        graph[u].append((v, cost))
    return graph

def dial_algorithm(graph, start, N):
    """Dial's Algorithm for graphs with small integer edge weights."""
    max_distance = 5 * N
    dist = [float('inf')] * (N + 1)
    dist[start] = 0
    prev = [None] * (N + 1)

    buckets = [[] for _ in range(max_distance + 1)]
    buckets[0].append(start)
    idx = 0
    while idx <= max_distance:
        while buckets[idx]:
            u = buckets[idx].pop()
            if dist[u] < idx:
                continue
            for v, w in graph[u]:
                if dist[v] > dist[u] + w:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    if dist[v] <= max_distance:
                        buckets[dist[v]].append(v)
        idx += 1
    return dist, prev

def shortest_delivery(N, M, roads, K, delivery_points, warehouse_node):
    """Compute shortest delivery times using Dial's Algorithm."""
    graph = build_graph(N, M, roads)
    dist, prev = dial_algorithm(graph, start=warehouse_node, N=N)
    results = [dist[point] for point in delivery_points]
    return dist, prev, results

def draw_town(canvas, positions, roads, dist, prev, delivery_points, warehouse_node):
    """Visualize the grid town with intersections, roads, traffic delays, and shortest paths."""
    # To avoid duplicate labels for bidirectional roads, keep track of labeled roads
    labeled_roads = set()

    # Draw roads and traffic delay labels
    for road in roads:
        u, v, _, traffic_delay = road
        # Create a unique identifier for each road (undirected)
        road_id = tuple(sorted((u, v)))
        if road_id in labeled_roads:
            continue  # Skip if already labeled
        labeled_roads.add(road_id)

        x1, y1 = positions[u]
        x2, y2 = positions[v]
        canvas.create_line(x1, y1, x2, y2, fill="gray")

        # Calculate midpoint for the traffic delay label
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        canvas.create_text(mid_x, mid_y, text=str(traffic_delay), fill="purple", font=('Arial', 10, 'bold'))

    # Highlight shortest paths
    for dest in delivery_points:
        node = dest
        while prev[node] is not None:
            u = prev[node]
            x1, y1 = positions[u]
            x2, y2 = positions[node]
            canvas.create_line(x1, y1, x2, y2, fill="red", width=2)
            node = u

    # Draw intersections
    for node, (x, y) in positions.items():
        color = "blue"
        if node == warehouse_node:
            color = "green"  # Warehouse
        elif node in delivery_points:
            color = "red"    # Delivery points
        canvas.create_oval(x-5, y-5, x+5, y+5, fill=color)
        canvas.create_text(x, y-10, text=str(node), fill="black", font=('Arial', 10, 'bold'))

def main():
    grid_rows = 7  # Number of rows in the grid
    grid_cols = 7  # Number of columns in the grid
    N = grid_rows * grid_cols  # Total number of intersections
    K = 8   # Number of delivery points

    # Create positions for intersections in grid layout
    positions = {}
    spacing = 100
    node_positions = {}
    node = 1
    for i in range(grid_rows):
        for j in range(grid_cols):
            x = 50 + j * spacing
            y = 50 + i * spacing
            positions[node] = (x, y)
            node_positions[(i, j)] = node
            node += 1

    # Determine the warehouse node (center of the grid)
    center_i = grid_rows // 2
    center_j = grid_cols // 2
    warehouse_node = node_positions.get((center_i, center_j))
    if warehouse_node is None:
        # If grid has even dimensions, choose one of the central nodes
        warehouse_node = node_positions.get((center_i - 1, center_j - 1), 1)

    # Generate roads between adjacent intersections (grid edges)
    roads = []
    for i in range(grid_rows):
        for j in range(grid_cols):
            u = node_positions[(i, j)]
            # Right neighbor
            if j < grid_cols - 1:
                v = node_positions[(i, j + 1)]
                traffic_delay = random.randint(0, 5)
                roads.append((u, v, 1, traffic_delay))  # u -> v
                roads.append((v, u, 1, traffic_delay))  # v -> u
            # Down neighbor
            if i < grid_rows - 1:
                v = node_positions[(i + 1, j)]
                traffic_delay = random.randint(0, 5)
                roads.append((u, v, 1, traffic_delay))  # u -> v
                roads.append((v, u, 1, traffic_delay))  # v -> u

    M = len(roads)  # Total number of roads

    # Random delivery points (excluding warehouse)
    all_nodes = set(range(1, N + 1))
    all_nodes.remove(warehouse_node)
    delivery_points = random.sample(list(all_nodes), K)  # Convert set to list

    dist, prev, results = shortest_delivery(N, M, roads, K, delivery_points, warehouse_node)

    # Create tkinter window with dynamic resizing and scrollbars
    root = tk.Tk()
    root.title("Grid Town Map with Shortest Delivery Paths")

    # Create a main frame
    main_frame = ttk.Frame(root)
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Create a canvas with scrollbars
    canvas_frame = ttk.Frame(main_frame)
    canvas_frame.pack(fill=tk.BOTH, expand=True)

    # Add horizontal and vertical scrollbars
    h_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL)
    h_scroll.pack(side=tk.BOTTOM, fill=tk.X)
    v_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL)
    v_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    # Create the canvas
    canvas = tk.Canvas(canvas_frame, bg="white",
                       xscrollcommand=h_scroll.set,
                       yscrollcommand=v_scroll.set)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Configure the scrollbars
    h_scroll.config(command=canvas.xview)
    v_scroll.config(command=canvas.yview)

    # Update the scrollregion after drawing
    def on_draw():
        draw_town(canvas, positions, roads, dist, prev, delivery_points, warehouse_node)
        # Calculate the bounding box of all items
        canvas.update_idletasks()
        bbox = canvas.bbox("all")
        if bbox:
            canvas.config(scrollregion=bbox)

    on_draw()

    # Bind the configure event to update the scrollregion
    def resize_event(event):
        canvas.config(scrollregion=canvas.bbox("all"))

    canvas.bind("<Configure>", resize_event)

    root.mainloop()

if __name__ == "__main__":
    sys.setrecursionlimit(1 << 25)
    main()