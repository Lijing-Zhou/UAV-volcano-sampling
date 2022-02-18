import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import math
# a star
class PathPlanning(Node):

    def __init__(self):
        """
        ox: x position list of Obstacles
        oy: y position list of Obstacles
        resolution: grid resolution
        rr: robot radiums

        """
        super.__init__('path_planning')
        self.resolution = 1                         #提前设置
        self.rr = 1                                 #提前设置
        self.ox = []                                #提前设置
        self.oy = []                                #提前设置
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(self.ox, self.oy)              
        self.message = "AStar init"
        # 3----2
        # |    |
        # 0----1
        self.fly_boundary_position = []   #之后确定边界的顶点坐标后，应设定这个值, 按逆时针, 必须为长方形且与Gazebo中的经纬度平行

        self.position_list_pub = self.create_publisher(String, '/position_list/sub', 10)
        

    class Node_:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def calc_heuristic(self, node1, node2):
        w = 1.0 # weight of heuristic
        d = w * math.hypot(node1.x - node2.x, node1.y - node2.y)
        return d

    def tf_xy_global_position(self, index, min_position):
        # tf xy glocal position
        pos = index * self.resolution + min_position
        return pos
    
    def tf_xy_local_position(self, position, min_pos):
        # tf xy local position, e.g. index
        return (position - min_pos) / self.resolution
    
    def tf_dict_index(self, node):
        # dict index: tf 2-D to 1-D dict
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.tf_xy_global_position(node.x, self.min_x)
        py = self.tf_xy_global_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True


    def calc_final_path(self, goal_node, closed_set):
        rx = [self.tf_xy_global_position(goal_node.x, self.min_x)]
        ry = [self.tf_xy_global_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.tf_xy_global_position(n.x, self.min_x))
            ry.append(self.tf_xy_global_position(n.y, self.min_y))
            parent_index = n.parent_index
        
        return rx, ry

    def planning(self, sx, sy, gx, gy):
        """
        input:
            sx: start x position
            sy: start y position
            gx: goal x position
            gy: goal y position
        
        output:
            rx: x position list of the fianl path
            ry: y position list of the final path
        """

        start_node = self.Node_(self.tf_xy_local_position(sx, self.min_x),
                                self.tf_xy_local_position(sy, self.min_y), 0.0, -1)
        goal_node = self.Node_(self.tf_xy_local_position(gx, self.min_x),
                                self.tf_xy_local_position(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.tf_dict_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                self.message = "Open set is empty.."
                break

            # get the min value's index in open_set
            c_id = min(open_set, 
                        key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                self.message = "Find goal"
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]

            closed_set[c_id] = current

            # expend the grid search based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node_(current.x + self.motion[i][0], current.y + self.motion[i][1],
                                current.cost + self.motion[i][2], c_id)
                n_id = self.tf_dict_index(node)

                # if the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
        
        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_obstacle_map(self, ox, oy):
        self.min_x = min(ox)
        self.min_y = min(oy)
        self.max_x = max(ox)
        self.max_y = max(oy)

        self.x_width = (self.max_x - self.min_x) / self.resolution
        self.y_width = (self.max_y - self.min_y) / self.resolution

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]

        for ix in range(self.x_width):
            x = self.tf_xy_global_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.tf_xy_global_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    def get_motion_model(self):
        # x, y, cost
        motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, -1],
                    [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                    [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]

        return motion

    # 假设禁飞区的顶点坐标为no_fly_zone, 逆时针
    # 设定飞行边界fly_boundary为顶点坐标
    def obstacle_map_init(self, fly_boundary, no_fly_zone):
        ox, oy = [], []
        # set fly boundary
        for i in range(fly_boundary[0][0], fly_boundary[1][0]):
            ox.append(i)
            oy.append(fly_boundary[0][1])
        for i in range(fly_boundary[1][1], fly_boundary[2][1]):
            ox.append(fly_boundary[1][0])
            oy.append(i)
        for i in range(fly_boundary[3][0], fly_boundary[2][0]):
            ox.append(i)
            oy.append(fly_boundary[2][1])
        for i in range(fly_boundary[0][1], fly_boundary[3][1]):
            ox.append(fly_boundary[0][0])
            oy.append(i)                
        # set no fly zone
        for i in range(len(no_fly_zone)):
            if i == len(no_fly_zone) - 1:
                ...
            else:
                # Y = aX + b
                # a: slope 斜率
                if (no_fly_zone[i+1][0] - no_fly_zone[i][0]) != 0:
                    a = (no_fly_zone[i+1][1] - no_fly_zone[i][1]) / (no_fly_zone[i+1][0] - no_fly_zone[i][0])
                    b = no_fly_zone[i][1] - (a * no_fly_zone[i][0])
                    no_fly_start = min(no_fly_zone[i][0], no_fly_zone[i+1][0])
                    no_fly_end = max(no_fly_zone[i][0], no_fly_zone[i+1][0])
                    for j in range(no_fly_start, no_fly_end):
                        ox.append(j)
                        oy.append(a * j + b)
                else:
                    no_fly_start = min(no_fly_zone[i][1], no_fly_zone[i+1][1])
                    no_fly_end = max(no_fly_zone[i][1], no_fly_zone[i+1][1])
                    for j in range(no_fly_start, no_fly_end):
                        ox.append(no_fly_zone[i][0])
                        oy.append(j)
    def start(self):
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        rx, ry = self.planning(1, 1, 2, 2)              #未调整数值
        if len(rx) == len(ry):
            for i in range(len(rx)):
                position_x = rx[i] + 51.423             #未调整数值
                position_y = ry[i] - 2.671              #未调整数值
                position_xy = "x=" + str(position_x) + " " + "y=" + str(position_y) 
                position_msg = String()
                position_msg.data = position_xy
                self.position_list_pub.publish(position_msg)
def main(args=None):
    rclpy.init(args=args)

    pathplanning_node = PathPlanning()
    pathplanning_node.start()
    rclpy.spin(pathplanning_node)

if __name__ == '__main__':
    main()

# 用法
# a_star = AStar(ox, oy, resolution, rr)
# rx, ry = a_star.planning(sx, sy, gx, gy)
