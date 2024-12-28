import numpy as np
import matplotlib.pyplot as plt
from animation import animate

class MPPI():
    def __init__(self, grid_size, num_samples=10, num_iterations=20, lambda_ = 1):
        self.cost_map = np.zeros([grid_size,grid_size])
        self.num_samples = num_samples
        self.num_iterations = num_iterations
        self.lambda_ = lambda_
        self.grid_size = grid_size
        self.obstacle_cost = 100
        self.collision_weight = 1000
        self.direction_weight = 1000

    def make_obstacles(self, num_obstacles,cost_map,obstacle_size,build_wall):
        
        # add obstacles until num_obstacles are added
        while num_obstacles != 0:
            
            # Randomly select coordinates for obstacles
            rand_idx = np.random.choice(cost_map.shape[0]**2)
            obs_center_x,obs_center_y = np.unravel_index(rand_idx, cost_map.shape)
            
            x_min = int(np.ceil(obs_center_x-obstacle_size/2))
            x_max = int(np.floor(obs_center_x+obstacle_size/2))
            y_min = int(np.ceil(obs_center_y-obstacle_size/2))
            y_max = int(np.floor(obs_center_y+obstacle_size/2))
            
            # Ensure obstacles are not going over sides
            if x_min < 0 or x_max > cost_map.shape[0] or y_min < 0 or y_max > cost_map.shape[0]:
                continue
            
            # Assign extra cost to obstacle area
            cost_map[x_min:x_max,y_min:y_max] = self.obstacle_cost
            num_obstacles -= 1
            
        # Build a heavy wall in the middle of the path
        if build_wall == True:
            cost_map[round(self.grid_size/2)-3:round(self.grid_size/2)+3,round(self.grid_size/10):self.grid_size - round(self.grid_size/10)] = self.obstacle_cost * 5
            
        self.cost_map = cost_map
        
    def draw_path(self,path):
        plt.plot(path[:,0],path[:,1],color='red')
        plt.xlim([0,self.grid_size])
        plt.ylim([0,self.grid_size])

    def set_goal(self, x, y):
        # Check if the goal is on the map and not on an obstacle
        assert (0 <= x < self.grid_size) and (0 <= y < self.grid_size), 'Goal is not on the map.'
        assert self.cost_map[x,y] != self.obstacle_cost, 'Goal cannot be an obstacle.'
        
        # Assign goal
        # plt.scatter(x,y,color='red', linewidth = 3)
        self.goal = (x, y)
        
    def set_start(self, x, y):
        # Check if the start is on the map and not on an obstacle
        assert (0 <= x < self.grid_size) and (0 <= y < self.grid_size), 'Start is not on the map.'
        assert self.cost_map[x,y] != self.obstacle_cost, 'Start cannot be an obstacle.'
        
        # Assign the goal
        # plt.scatter(x,y,color='green',linewidth = 3)
        self.start = (x, y)

    def cost_function(self, position, former_position, control):
        
        # For some reason the costmap is flipped. Spent 2 hours trying to figure out the plotting.
        x = position[1]
        y = position[0]
        
        distance_cost = np.linalg.norm(position - self.goal)  # linalg.norm is the euclidean distance

        collision_cost = 0
        
        # Case where robot is out of bounds, punished more than normal
        if x < 0 or x >= self.cost_map.shape[0] or y < 0 or y >= self.cost_map.shape[1]:
            collision_cost = self.obstacle_cost * 10
        
        # Case where robot goes through an obstacle, but doesn't land on it for this prediction
        # elif np.any(self.cost_map[np.round(np.linspace(former_position, position)).astype(int)] != 0):
        #     collision_cost = self.obstacle_cost
            
        # Case where an obstacle is hit in this prediction
        else:
            collision_cost = self.cost_map[int(x),int(y)]
        
        # Direction based on dot product of unit vectors of direction to goal and and control direction
        direction_cost = -np.dot((self.goal - position) / np.linalg.norm(position - self.goal), control / np.linalg.norm(control))

        return distance_cost + self.collision_weight * collision_cost + self.direction_weight * direction_cost
    
    def robot_model(self, old_state, control):
        
        # Model
        control_gain = 0.5
        state = old_state + control_gain*control
        
        return state

    def generate_path(self):
        # MPPI implementation
        position = self.start
        path = [position]
        prediction_horizon = 5 # Was num_samples
        monte_carlo = 30 # Was num_iterations
        control_sequence = np.zeros((monte_carlo, prediction_horizon, 2)) # 2 is one control for each direction
        
        # Initialize annimation
        ann = animate(self.cost_map, self.start, self.goal)
        
        # While the robot is not within some tolerance of the goal position
        while abs(int(position[0]) - self.goal[0]) > 1 or abs(int(position[1]) - self.goal[1]) > 1:
            
            # Simulate the newest state of the robot
            ann.move(position=position)
            
            # Random simulated noise generated before the loop
            noise = np.random.normal(0, 2, (monte_carlo, prediction_horizon, 2))
            controls = control_sequence + noise # 5 monte carlos for 30 steps into the future
            
            # Build cost vector
            cost = np.array([0 for _ in range(monte_carlo)])
            
            # Build a number of monte_carlo estimations
            for i in range(monte_carlo):
                simulated_position = position
                ann.reset(position) # Reset the variables in the annimation

                # Over some prediction horizon
                for j in range(prediction_horizon):
                    
                    # Sample possible controls with noise
                    control = controls[i,j] 
                    
                    # Apply control to model to find simulated movement
                    former_simulated_position = simulated_position
                    simulated_position = self.robot_model(simulated_position, control)
                    
                    # Accumulate rollout cost over the prediction horizon for this monte carlo
                    cost[i] += self.cost_function(simulated_position, former_simulated_position, control)
                    
                    # Plot predictions
                    ann.predict(simulated_position, cost)
                    
            # weights cannot be too small thus add np.min(cost)
            weights = np.exp(((-cost)+np.min(cost)) / self.lambda_)
            total_weight = np.sum(weights)

            if total_weight == 0:  # Handle division by zero
                weights = np.ones_like(weights) / len(weights)  # Assign uniform weights
            else:
                weights /= total_weight
            # for i in range(prediction_horizon):
            control_sequence = np.sum(weights[:, None, None] * controls, axis=0)
                
            # Apply first control in the sequence
            position = self.robot_model(position, control_sequence[0])
            path.append(position)
            control_sequence = np.roll(control_sequence, -1, axis=0)
            control_sequence[-1] = (0, 0)
        return np.array(path)
        
        
def main():
    
    # Generate costmap and make obstacles
    mppi_obj = MPPI(grid_size=100)
    mppi_obj.make_obstacles(num_obstacles=100,cost_map=mppi_obj.cost_map, obstacle_size=7,build_wall=False)
    # plt.matshow(mppi_obj.cost_map)
    # plt.colorbar()  # Add a color scale for reference
    
    mppi_obj.set_start(10, 10)
    mppi_obj.set_goal(90, 90)

    # Generate the map
    path = mppi_obj.generate_path()
    
    mppi_obj.draw_path(path)
    plt.show()
    
    
if __name__ == "__main__":
    main()