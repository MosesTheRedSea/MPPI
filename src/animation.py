import matplotlib.pyplot as plt
import numpy as np

'''
This class generates a matplotlib annimation of the MPPI controller. It can easily be generalized to other motion planning, given a costmap and a discrete state space.
'''

class animate():
    def __init__(self, cost_map, start, goal):
        '''
        Plot the initial map and view for 5 seconds
        
        Args:
            cost_map: numpy matrix of the map's weights
            start: tuple of starting (x,y) position
            goal: tuple of goal (x,y) position
        '''

        self.cost_map = cost_map
        self.position = np.array(start)
        self.monte_carlo = []
        
        # Show initial map
        self.fig, self.ax = plt.subplots()
        self.ax.matshow(self.cost_map, origin='upper', cmap='viridis')  # Ensure proper origin
        plt.colorbar(self.ax.imshow(self.cost_map, origin='upper', cmap='viridis'), ax=self.ax)
        self.ax.scatter(start[1], start[0], color='green', linewidth=3)  # Swap x and y
        self.ax.scatter(goal[1], goal[0], color='red', linewidth=3)  # Swap x and y
        plt.pause(5)
        
    def move(self, position):
        
        # Limit the viewing window to around the robot
        x = position[0]
        y = position[1]
        self.ax.scatter(x,y,color='green',linewidth = 2)
        plt.xlim(x-20,x+20)
        plt.ylim(y-20,y+20)
        plt.pause(.5)
        
    def predict(self, prediction, cost):
        
        self.monte_carlo = np.concatenate((self.position.reshape(1,2), prediction.reshape(1,2)),axis=0)
        x_coord = self.monte_carlo[:,0]
        y_coord = self.monte_carlo[:,1]
        self.ax.plot(x_coord,y_coord,color='yellow')
        
        # Set the prediction as the new position
        self.position = prediction
        
    def reset(self, position):
        '''
        Reset the prediction for each monte carlo
        '''
        
        self.monte_carlo = []
        self.position = np.array(position)