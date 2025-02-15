import numpy as np
import matplotlib.pyplot as plt
from animation import animate
from obstacle_hit import *

class MPPI():

    def __init__(self, grid_size, prediction_horizon, monte_carlo_iters, num_samples=10, num_iterations=20, lambda_ = 1, draw_preds=True):
        
        self.cost_map = np.zeros([grid_size,grid_size])
        self.num_samples = num_samples
        self.num_iterations = num_iterations
        self.lambda_ = lambda_
        self.grid_size = grid_size
        self.draw_preds = draw_preds
        self.prediction_horizon = prediction_horizon # Was num_samples
        self.monte_carlo_iters = monte_carlo_iters # Was num_iterations
        self.distance = 0
        
        # Variables for getting the robot unstuck
        self.stuck_counter = 0
        self.stuck_multiplier = 1
        
        # Variables to modify
        self.obstacle_cost = 100
        self.collision_weight = 100
        self.direction_weight = 1000
        self.distance_weight = 1

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
            
            # Ensure the obstacles aren't placed on the start or end goal. Again, the axes are flipped and idk why
            if (self.start[0] < x_max and self.start[0] > x_min and self.start[1] < y_max and self.start[1] > y_min) or (self.goal[0] < x_max and self.goal[0] > x_min and self.goal[1] < y_max and self.goal[1] > y_min):
                continue
            
            # Assign extra cost to obstacle area
            cost_map[x_min:x_max,y_min:y_max] = self.obstacle_cost
            num_obstacles -= 1
            
        # Build a heavy wall in the middle of the path
        if build_wall == True:
            cost_map[round(self.grid_size/2)-3:round(self.grid_size/2)+3,round(self.grid_size/10):self.grid_size - round(self.grid_size/10)] = self.obstacle_cost
            
        self.cost_map = cost_map
        
    def draw_path(self,path):
        plt.plot(path[:,0],path[:,1],color='red')
        plt.xlim([0,self.grid_size])
        plt.ylim([0,self.grid_size])

    def set_goal(self, x, y):
        # Check if the goal is on the map
        assert (0 <= x < self.grid_size) and (0 <= y < self.grid_size), 'Goal is not on the map.'
        
        # Assign goal
        self.goal = (x, y)
    
    def set_start(self, x, y):
        # Check if the start is on the map
        assert (0 <= x < self.grid_size) and (0 <= y < self.grid_size), 'Start is not on the map.'
        
        # Assign the goal
        self.start = (x, y)

    """
    ▀█▀ ░█▄─░█ ░█▀▀█ ░█▀▀▀█ ░█▀▀█ ░█▀▀█ ░█▀▀▀ ░█▀▀█ ▀▀█▀▀ 　 ▀█▀ ░█▄─░█ ░█▀▀▄ ░█▀▀▀ ▀▄░▄▀ ░█▀▀▀ ░█▀▀▀█ 
    ░█─ ░█░█░█ ░█─── ░█──░█ ░█▄▄▀ ░█▄▄▀ ░█▀▀▀ ░█─── ─░█── 　 ░█─ ░█░█░█ ░█─░█ ░█▀▀▀ ─░█── ░█▀▀▀ ─▀▀▀▄▄ 
    ▄█▄ ░█──▀█ ░█▄▄█ ░█▄▄▄█ ░█─░█ ░█─░█ ░█▄▄▄ ░█▄▄█ ─░█── 　 ▄█▄ ░█──▀█ ░█▄▄▀ ░█▄▄▄ ▄▀░▀▄ ░█▄▄▄ ░█▄▄▄█
        """
    
    def cost_function(self, position, former_position, control):
        
        # For some reason the costmap is flipped. Spent 2 hours trying to figure out the plotting.
        # position = position[[1,0]]
        # former_position = np.array(former_position)[[1,0]]
        x = position[1]
        y = position[0]
        
        # Get distance cost
        self.distance_cost = np.linalg.norm(position - self.goal)  # linalg.norm is the euclidean distance

        # Get collision cost
        collision_cost = 0
        
        # Collision cost case 2 precaclulations
        coords = np.round(np.linspace(former_position, position)).astype(int)
        x_coords = coords[:,1]
        y_coords = coords[:,0]
        
        # Case where robot is out of bounds, punished more than normal
        if x < 0 or x >= self.cost_map.shape[0] or y < 0 or y >= self.cost_map.shape[1]:
            collision_cost = self.obstacle_cost * 10
            
        # Case where an obstacle is in the path from one point to another, but not explicitly hit. This case will make the agent take corners and not jump over obstacles
        elif np.any(self.cost_map[x_coords,y_coords] != 0):
            collision_cost = self.obstacle_cost
            
        # Case where an obstacle is hit
        elif ObstacleHit(self.cost_map, position):
            collision_cost = self.obstacle_cost
            
        # Case where no obstacle is hit
        else:
            collision_cost = 0
        
        # POSSIBLE CONDITIONAL FOR GETTING UNSTUCK
        # Direction based on dot product of unit vectors of direction to goal and and control direction
        # goal = self.goal
        # if self.stuck_counter > 10:
        #     goal = (-self.goal[0], self.goal[1])
        # direction_cost = -np.dot((goal - position) / np.linalg.norm(position - goal), control / np.linalg.norm(control))

        direction_cost = -np.dot((self.goal - position) / np.linalg.norm(position - self.goal), control / np.linalg.norm(control))

        # POSSIBLE CONDITIONAL FOR GETTING UNSTUCK
        # if direction_cost < 0: # if it is going in the right direction, penalize it if it is stuck 
        #     weighted_direction_cost = self.direction_weight * direction_cost / self.stuck_multiplier
        # else: # If it is going the other way, reward it so it gets unstuck
        weighted_direction_cost = self.direction_weight * direction_cost

        weighted_distance_cost = self.distance_weight * self.distance_cost
        
        weighted_collision_cost = self.collision_weight * collision_cost
        
        return weighted_distance_cost + weighted_collision_cost + weighted_direction_cost
    
    def robot_model(self, old_state, control):
        
        # Model
        control_gain = 0.5
        state = old_state + control_gain*control
        
        return state
    
    # WORK IN PROGRESS
    def check_stuck(self,position):
        position = position[[1,0]]
        # Check if the robot is stuck by comparing it's former distance cost to the current. Tune the tolerable change.
        former_distance = self.distance
        self.distance = np.linalg.norm(position - self.goal)
        print(self.stuck_counter)
        
        if abs(self.distance - former_distance) < 1.5:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            
        # Make the robot more afraid of obstacles if it is stuck
        if self.stuck_counter > 10:
            print('Robot is stuck! Increasing prediction horizon.')
            # self.prediction_horizon += 2
            self.stuck_multiplier = -1
        else:
            self.stuck_multiplier = 1
    
    def generate_path(self):
        # MPPI implementation
        position = self.start
        path = [position]
        control_sequence = np.zeros((self.monte_carlo_iters, self.prediction_horizon, 2)) # 2 is one control for each direction
        
        # Initialize annimation
        ann = animate(self.cost_map, self.start, self.goal)
        
        # While the robot is not within some tolerance of the goal position
        while abs(int(position[0]) - self.goal[0]) > 2 or abs(int(position[1]) - self.goal[1]) > 2:
            
            # Simulate the newest state of the robot
            ann.move(self.cost_map, position)
            
            # Random simulated noise generated before the loop
            noise = np.random.normal(0, 1, (self.monte_carlo_iters, self.prediction_horizon, 2))
            controls = control_sequence + noise # 5 monte carlos for 30 steps into the future
            
            # Build cost vector
            cost = np.array([0 for _ in range(self.monte_carlo_iters)])
            
            # Build a number of monte_carlo estimations
            for i in range(self.monte_carlo_iters):
                simulated_position = position
                ann.reset(position) # Reset the variables in the annimation

                # Over some prediction horizon
                for j in range(self.prediction_horizon):
                    
                    # Sample possible controls with noise
                    control = controls[i,j] 
                    
                    # Apply control to model to find simulated movement
                    former_simulated_position = simulated_position
                    simulated_position = self.robot_model(simulated_position, control)
                    
                    # Accumulate rollout cost over the prediction horizon for this monte carlo
                    cost[i] += self.cost_function(simulated_position, former_simulated_position, control)
                    
                    # Plot predictions
                    if self.draw_preds == True:
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
            # self.check_stuck(position)
            path.append(position)
            control_sequence = np.roll(control_sequence, -1, axis=0)
            control_sequence[-1] = (0, 0)
    
        # Final simulated movement
        ann.move(self.cost_map, position)
        
        return np.array(path)

def main():
    
    # Generate costmap and make obstacles
    mppi_obj = MPPI(grid_size=100,prediction_horizon=7, monte_carlo_iters=50,draw_preds=False,lambda_=1)
    
    mppi_obj.set_start(10, 10)
    mppi_obj.set_goal(90, 90)
    
    mppi_obj.make_obstacles(num_obstacles=75,cost_map=mppi_obj.cost_map, obstacle_size=7,build_wall=False)

    # Generate the map
    path = mppi_obj.generate_path()
    
    mppi_obj.draw_path(path)
    plt.show()
    
    
if __name__ == "__main__":
    main()