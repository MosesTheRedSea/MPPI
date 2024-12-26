import numpy as np
import matplotlib.pyplot as plt

class MPPI():
    def __init__(self, grid_size):
        self.cost_map = np.ones([grid_size,grid_size])

    def make_obstacles(self, num_obstacles,cost_map,obstacle_size):
        
        # add obstacles until num_obstacles are added
        while num_obstacles != 0:
            
            # Randomly select coordinates for obstacles
            rand_idx = np.random.choice(cost_map.shape[0]**2)
            obs_center_x,obs_center_y = np.unravel_index(rand_idx, cost_map.shape)
            
            x_min = int(obs_center_x-obstacle_size/2)
            x_max = int(obs_center_x+obstacle_size/2)
            y_min = int(obs_center_y-obstacle_size/2)
            y_max = int(obs_center_y+obstacle_size/2)
            
            # Ensure obstacles are not going over sides
            if x_min < 0 or x_max > cost_map.shape[0] or y_min < 0 or y_max > cost_map.shape[0]:
                continue
            
            # Assign extra cost to obstacle area
            cost_map[x_min:x_max,y_min:y_max] = 10
            num_obstacles -= 1
            
        self.cost_map = cost_map
        
    def draw_path(self,path):
        plt.plot(path[:,0],path[:,1])

    def set_goal(self, x, y):
        self.goal = (x, y)
        
    def set_start(self, x, y):
        self.start = (x, y)

    def cost_function(self, position, control):
        distance_cost = np.linalg.norm(position - self.goal)  # linalg.norm is the euclidean distance

        collision_cost = 0
        if position[0] < 0 or position[0] >= self.cost_map.shape[0] or position[1] < 0 or position[1] >= self.cost_map.shape[1]:
            collision_cost = 1e6
        else:
            collision_cost = self.cost_map[position[0], position[1]]

        control_cost = np.linalg.norm(control) ** 2  # penalize large control inputs

        return distance_cost + collision_cost + 0.1 * control_cost

    def generate_path(self):
        # MPPI implementation
        test=1
        
        
def main():
    
    # Generate costmap and make obstacles
    mppi_obj = MPPI(grid_size=100)
    mppi_obj.make_obstacles(num_obstacles=10,cost_map=mppi_obj.cost_map, obstacle_size=10)
    plt.matshow(mppi_obj.cost_map)
    plt.colorbar()  # Add a color scale for reference
    
    # Generate the map
    path = np.array([[0, 0],[1, 2],[4,4]])
    
    mppi_obj.draw_path(path)
    plt.show()
    
if __name__ == "__main__":
    main()