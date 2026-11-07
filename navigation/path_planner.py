import heapq
import math
import numpy as np

class AStarPlanner:
    def __init__(self, resolution_m=0.10, grid_size_m=(20.0, 20.0)):
        self.res = resolution_m
        self.width = int(grid_size_m[0] / resolution_m)
        self.height = int(grid_size_m[1] / resolution_m)
        self.grid = np.zeros((self.width, self.height), dtype=np.uint8)

    def set_obstacle(self, x_m, y_m, radius_m=0.25):
        gx, gy = int(x_m / self.res), int(y_m / self.res)
        inflation = int(math.ceil(radius_m / self.res))

        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if math.hypot(dx, dy) <= inflation:
                        self.grid[nx, ny] = 1

    def plan(self, start_xy, goal_xy):
        start = (int(start_xy[0] / self.res), int(start_xy[1] / self.res))
        goal = (int(goal_xy[0] / self.res), int(goal_xy[1] / self.res))

        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height) or \
           not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height): return []

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from, g_score = {}, {start: 0.0}

        motions = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                   (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [(p[0] * self.res, p[1] * self.res) for p in path]

            for dx, dy, cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[0], neighbor[1]] == 1: continue 

                    tentative_g = g_score[current] + cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + math.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1])
                        heapq.heappush(open_set, (f_score, neighbor))
        return []