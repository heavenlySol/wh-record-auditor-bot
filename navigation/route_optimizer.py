import math
import itertools

class RouteOptimizer:
    def __init__(self, dock_pos=(0.0, 0.0)):
        self.dock_pos = dock_pos

    def get_perfect_path(self, start_pos, queried_racks, rack_database):
        valid_racks = {k: v for k, v in rack_database.items() if k in queried_racks}
        if not valid_racks: return []

        best_distance = float('inf')
        best_sequence = []

        for perm in itertools.permutations(valid_racks.keys()):
            current_dist = 0.0
            current_pos = start_pos
            
            for rack in perm:
                target_pos = valid_racks[rack]
                current_dist += math.hypot(target_pos[0] - current_pos[0], target_pos[1] - current_pos[1])
                current_pos = target_pos
                
            current_dist += math.hypot(self.dock_pos[0] - current_pos[0], self.dock_pos[1] - current_pos[1])

            if current_dist < best_distance:
                best_distance = current_dist
                best_sequence = list(perm)

        print(f"[*] Optimized Route: {' -> '.join(best_sequence)} -> DOCK ({best_distance:.2f}m)")
        return best_sequence