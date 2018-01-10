from environment import Environment
from region import Region, Obstacle
from searchNode import SearchNode
from collision_check import is_collision
import matplotlib.collections as collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random

class Simulation():
    def __init__(self, environment=None, start_state=None, end=None, speed=10, sight_distance=None, dist_max=5):
        self.environment = environment
        self.state = start_state
        self.end = end
        self.speed = speed
        self.sight_distance = sight_distance
        self.dist_max = dist_max
        self.rrt_star_max_sq = 25

        if environment is None:
            self.environment = Environment()

        if sight_distance is None:
            y_range = self.environment.bounds[3] - self.environment.bounds[2]
            self.sight_distance = int(y_range/10.0)

        if start_state is None:
            self.state = (self.environment.bounds[0], self.environment.bounds[2])

        if end is None:
            self.end = Region([
                (self.environment.bounds[1], self.environment.bounds[3]),
                (self.environment.bounds[1], self.environment.bounds[3]-self.sight_distance),
                (self.environment.bounds[1]-self.sight_distance, self.environment.bounds[3]-self.sight_distance),
                (self.environment.bounds[1]-self.sight_distance, self.environment.bounds[3]),])

        self.node_state = SearchNode(self.state)
        self.nodes = [self.node_state]

        self.searching = True
        self.counter = 0

        self.search_lines = []
        self.path_lines = []

        self.discovered_lines = []
        self.discovered_obstacles = set()
        self.goal_lines = self.end.line_segments()

        self.goal_found = False
        
        # Initialize with seen obstacles
        seen_obstacles = self.checkNewObstacles()
        if len(seen_obstacles) > 0:
            for obs in seen_obstacles:
                self.discovered_lines += obs.line_segments()
                self.discovered_obstacles.add(id(obs))

    def checkNewObstacles(self):
        new_obstacles = []
        for obs in self.environment.obstacles:
            if id(obs) not in self.discovered_obstacles:

                for line_seg in obs.line_segments():

                    start = np.array(line_seg[0])
                    end = np.array(line_seg[1])
                    diff = np.subtract(end,start)
                    len_sq = np.sum(diff*diff, axis=0)
                    nx = ((self.state[0] - start[0])*diff[0] + (self.state[1] - start[1])*diff[1])/len_sq
                    nx = min(1, max(0, nx))

                    proj = (diff[0]*nx + start[0], diff[1]*nx + start[1])

                    diff2 = np.subtract(proj, self.state)
                    dist = np.sqrt(np.sum(diff2*diff2, axis=0))
                    if dist < self.sight_distance:
                        new_obstacles.append(obs)
                        break

        return new_obstacles

    def step(self):

        def findNearest(nodes, node_random):
            dist_min_sq = 1000000
            node_min = None
            for node in nodes:
                (x, y) = node.state
                (x_r, y_r) = node_random
                dist_sq = (x_r-x)**2 + (y_r-y)**2
                if dist_sq < dist_min_sq:
                    dist_min_sq = dist_sq
                    node_min = node
            return node_min, dist_min_sq

        def newPoint(node_start, point_random, mag_sq):
            mag = np.sqrt(mag_sq)

            if mag > self.dist_max:
                length = self.dist_max
            else:
                length = mag

            point_start = node_start.state

            (x_s, y_s) = point_start
            (x_r, y_r) = point_random
        
            (x_d, y_d) = (x_r-x_s, y_r-y_s)
            (x_d_n, y_d_n) = (x_d/mag, y_d/mag)

            (x_n, y_n) = (x_s + (length*x_d_n), y_s + (length*y_d_n))
    
            x_n = max(min(self.environment.bounds[1], x_n), self.environment.bounds[0])
            y_n = max(min(self.environment.bounds[3], y_n), self.environment.bounds[2])

            return (x_n, y_n), length

        def checkCollision(start_node, end_point, line_segs):
            test_segment = [start_node.state, end_point]
            for line_seg in line_segs:
                if is_collision(test_segment, line_seg):
                    return True
            return False

        def newLineSegment(node):
            start_point = node.parent.state
            end_point = node.state
            line_seg = [start_point, end_point]
            return line_seg

        def nodeLineCost(node_near, node_new):
            start = node_near.state
            end = node_new.state

            diff = np.subtract(start, end)
            return np.sqrt(np.sum(diff*diff, axis=0))

        def nearestNodes(nodes, new_point, max_dist):
            nodes_nearest = []

            for node in nodes:
                diff = np.subtract(node.state, new_point)
                dist = np.sum(diff*diff, axis=0)
                if dist < max_dist:
                    nodes_nearest.append(node)

            return nodes_nearest

        def createGoalPath(goal_node):
            node = goal_node
            path_lines = []
            while node.parent is not None:
                line_seg = newLineSegment(node)
                path_lines.append(line_seg)
                
                node.parent.child = node
                node = node.parent

            return path_lines

        def isPathCollision(start_node, discovered_lines):
            node = start_node
            while node.child is not None:
                if checkCollision(node, node.child.state, discovered_lines):
                    return True
                node = node.child
            return False

        # Run RRT*
        if self.searching:

            if self.counter > 1000:
                self.searching = False

            (x_r, y_r) = (random.uniform(self.environment.bounds[0], self.environment.bounds[1]), 
                random.uniform(self.environment.bounds[2], self.environment.bounds[3]))

            node_nearest, mag_sq = findNearest(self.nodes, (x_r, y_r))
            new_point, length = newPoint(node_nearest, (x_r, y_r), mag_sq)

            if not checkCollision(node_nearest, new_point, self.discovered_lines):

                nodes_nearest = nearestNodes(self.nodes, new_point, self.rrt_star_max_sq)

                newNode = SearchNode(new_point, node_nearest, node_nearest.cost + length)
                self.nodes.append(newNode)

                node_min = node_nearest
                cost_min = newNode.cost

                for node_near in nodes_nearest:
                    if (not checkCollision(node_near, new_point, self.discovered_lines)) and (node_near.cost + 
                        nodeLineCost(node_near, newNode) < cost_min):
                        node_min = node_near
                        cost_min = node_near.cost + nodeLineCost(node_near, newNode)

                newNode.parent = node_min
                self.search_lines.append(newLineSegment(newNode))

                for node_near in nodes_nearest:
                    if not checkCollision(node_near, new_point, self.discovered_lines) and (newNode.cost + 
                        nodeLineCost(newNode, node_near) < node_near.cost):
                        node_near.parent = newNode

                if checkCollision(node_nearest, new_point, self.goal_lines):
                    self.goal_found = True
                    self.searching = False
                    self.path_lines = createGoalPath(newNode)

        # Move robot
        if (self.counter % self.speed == 0) and self.goal_found:
            if self.node_state.child is not None:
                self.node_state = self.node_state.child
                self.state = self.node_state.state

            # Check for unseen obstacles
            seen_obstacles = self.checkNewObstacles()

            if len(seen_obstacles) > 0:
                for obs in seen_obstacles:
                    self.discovered_lines += obs.line_segments()
                    self.discovered_obstacles.add(id(obs))

                # Check if new obstacles interfere with path
                if isPathCollision(self.node_state, self.discovered_lines):
                    self.goal_found = False
                    self.searching = True
                    self.node_state.parent = None
                    self.nodes = [self.node_state]
                    self.search_lines = []
                    self.path_lines = []

        self.counter += 1

#------------------------------------------------------------
# Set up simulation
env = Environment(yaml_file="sample_obstacles.yaml")
start_state=(2,2)
end = Region([
    (70, 70),
    (70, 90),
    (90, 90),
    (90, 70)])
sim = Simulation(env, start_state=start_state, end = end, sight_distance=10)

#------------------------------------------------------------
# Set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=sim.environment.bounds[:2], 
    ylim=sim.environment.bounds[2:])
ax.grid()

# Add goal lines
goal_collection = collections.LineCollection(sim.goal_lines, colors=(0,0.8,0.4,1))
ax.add_collection(goal_collection)

# Add initial obstacles
obstacle_collection = collections.LineCollection(sim.discovered_lines, colors=(1,0,0,1))
ax.add_collection(obstacle_collection)

# Add initial search lines
search_line_collection = collections.LineCollection([])
ax.add_collection(search_line_collection)

# Add initial path lines
path_line_collection = collections.LineCollection([], colors=(0,0.8,0.4,1))
ax.add_collection(path_line_collection)

# Add initial state
state_line, = ax.plot(sim.state[0], sim.state[1], 'bo')

def animate(i):
    # Perform search and movement
    global sim, ax, fig
    sim.step()
    
    # update  animation
    obstacle_collection.set_segments(sim.discovered_lines)
    search_line_collection.set_segments(sim.search_lines)
    path_line_collection.set_segments(sim.path_lines)
    state_line.set_data(sim.state[0], sim.state[1])


    return obstacle_collection, search_line_collection, state_line

ani = animation.FuncAnimation(fig, animate,
                              interval=10, blit=False)


plt.show()






