import heapq

class Node:
    def __init__(self, x, y, cost=0, heuristic=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def find_path(self, start, goal):
        open_list = []
        heapq.heappush(open_list, Node(start[0], start[1], 0, self.heuristic(Node(start[0], start[1]), Node(goal[0], goal[1]))))
        closed_list = set()
        
        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.add((current_node.x, current_node.y))
            
            if (current_node.x, current_node.y) == goal:
                path = []
                while current_node:
                    path.append((current_node.x, current_node.y))
                    current_node = current_node.parent
                return path[::-1]

            neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            for dx, dy in neighbors:
                neighbor_x = current_node.x + dx
                neighbor_y = current_node.y + dy

                if 0 <= neighbor_x < self.rows and 0 <= neighbor_y < self.cols and self.grid[neighbor_x][neighbor_y] == 0:
                    if (neighbor_x, neighbor_y) in closed_list:
                        continue
                    
                    neighbor_node = Node(neighbor_x, neighbor_y, current_node.cost + 1, self.heuristic(Node(neighbor_x, neighbor_y), Node(goal[0], goal[1])), current_node)
                    
                    if not any(neighbor_node == n and neighbor_node.cost >= n.cost for n in open_list):
                        heapq.heappush(open_list, neighbor_node)
        
        return None

    def print_grid_with_path(self, path):
        for x, y in path:
            self.grid[x][y] = '*'
        for row in self.grid:
            print(' '.join(str(cell) for cell in row))

grid = [
    [0, 0, 0, 1, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (7, 5)

astar = AStar(grid)
path = astar.find_path(start, goal)

if path:
    print("Path found:")
    astar.print_grid_with_path(path)
else:
    print("No path found.")
