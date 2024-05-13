
import random
import math
import matplotlib.pyplot as plt

class Vertex:
    def __init__(self, coords):
        self.x, self.y = coords
        self.neighbors = []
        self.predecessor = None
        self.cost = float('inf')

def euclidean_distance(v1, v2):
    return math.sqrt((v1.x - v2.x)**2 + (v1.y - v2.y)**2)

def check_obstacle_collision(vertex, matrix_map, clearance=0.75):
    x, y = int(vertex.x), int(vertex.y)
    if 0 <= x < len(matrix_map) and 0 <= y < len(matrix_map[0]):
        if matrix_map[x][y] == 100:
            return True
    return False

def check_edge_collision(v1, v2, matrix_map, clearance=0.75):
    dx = v2.x - v1.x
    dy = v2.y - v1.y
    steps = 10
    for i in range(steps + 1):
        x = int(v1.x + dx * i / steps)
        y = int(v1.y + dy * i / steps)
        if check_obstacle_collision(Vertex((x, y)), matrix_map, clearance):
            return True
    return False

def prm(start, goal, matrix_map, num_samples, neighborhood_size):
    vertices = [Vertex(start), Vertex(goal)]
    for _ in range(num_samples):
        sample = Vertex((random.randint(0, len(matrix_map)-1), random.randint(0, len(matrix_map[0])-1)))
        if not check_obstacle_collision(sample, matrix_map):
            vertices.append(sample)

    for v1 in vertices:
        neighbors = sorted(vertices, key=lambda v: euclidean_distance(v1, v))[:neighborhood_size + 1]
        for v2 in neighbors:
            if v2 != v1 and not check_edge_collision(v1, v2, matrix_map):
                v1.neighbors.append(v2)

    start_vertex = vertices[0]
    goal_vertex = vertices[1]
    return a_star(start_vertex, goal_vertex, vertices)

def a_star(start, goal, vertices):
    open_set = {start}
    closed_set = set()
    start.cost = 0

    while open_set:
        current = min(open_set, key=lambda v: v.cost + euclidean_distance(v, goal))
        if current == goal:
            path = []
            while current.predecessor:
                path.append((current.x, current.y))
                current = current.predecessor
            path.append((start.x, start.y))
            return path[::-1]

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in current.neighbors:
            if neighbor in closed_set:
                continue
            tentative_cost = current.cost + euclidean_distance(current, neighbor)
            if neighbor not in open_set or tentative_cost < neighbor.cost:
                neighbor.cost = tentative_cost
                neighbor.predecessor = current
                open_set.add(neighbor)

    return None

def visualize_path(start, goal, matrix_map, path):
    fig, ax = plt.subplots()

    # Display the matrix map
    ax.imshow(matrix_map, cmap='gray', origin='lower')

    ax.plot(start[1], start[0], 'go', markersize=10, label='Start')
    ax.plot(goal[1], goal[0], 'bo', markersize=10, label='Target')

    if path is not None:
        path_x, path_y = zip(*[(y, x) for x, y in path])
        ax.plot(path_y, path_x, '-', linewidth=2, label='Path')

    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.set_title('PRM Path')
    ax.legend()
    plt.tight_layout()
    plt.show()

def parse_input(input_str):
    lines = input_str.split('\n')
    dimensions = tuple(map(int, lines[0].split(',')))
    matrix_map = [list(map(int, line.split(','))) for line in lines[1:dimensions[0]+1]]
    start = tuple(map(int, lines[dimensions[0]+1].split(',')))
    goal = tuple(map(int, lines[dimensions[0]+2].split(',')))
    return start, goal, matrix_map

inp = input()
input_str=''
while inp != '-1':
    input_str += inp + '\n'
    inp = input()

start, goal, obstacles = parse_input(input_str)

path = prm(start, goal, obstacles, num_samples=1000, neighborhood_size=25)
if path is not None:
    waypoints = '\n'.join([f"{x},{y}" for x, y in path])
    print(waypoints)
    visualize_path(start, goal, obstacles, path)
else:
    print("No path found.")