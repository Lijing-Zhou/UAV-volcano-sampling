import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
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
        super().__init__("path_planning")

        self.position_list_pub = self.create_publisher(String, '/position_list', 10)
        self.pos_list_pub = self.create_publisher(Int32MultiArray, '/pos_test', 10)
        self.resolution = 1                         
        self.rr = 100                                 
        # 3----2
        # |    |
        # 0----1
        #之后确定边界的顶点坐标后，应设定这个值, 按逆时针, 必须为长方形且与Gazebo中的经纬度平行
        # self.fly_boundary_position = [[-26717208, 514234260], [-26701340, 514212462], 
        #                                 [-26656878, 514224401], [-26670602, 514246918]]
        # self.no_fly_zone = [[-26711717, 514226717], [-26673723, 514235482],
        #                     [-26670013, 514228490], [-26706367, 514219368]]          
        self.fly_boundary_position = [[-267172, 5142342], [-267013, 5142124], 
                                        [-266568, 5142244], [-266706, 5142469]]
        self.no_fly_zone = [[-267117, 5142267], [-266737, 5142354],
                            [-266700, 5142284], [-267063, 5142193]]   
        # A[-2.6711717, 51.4226717], D[-2.6706367, 51.4219368]

        self.ox = []
        self.oy = []
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()         
        self.message = "AStar init" 

        self.pos_t = [[1, 1], [2, 2]]

    class PathNode:
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
        return (int)(node.y - self.min_y) * self.x_width + (node.x - self.min_x)

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

        if self.obstacle_map[(int)(node.x)][(int)(node.y)]:
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

        start_node = self.PathNode(self.tf_xy_local_position(sx, self.min_x),
                                self.tf_xy_local_position(sy, self.min_y), 0.0, -1)

        goal_node = self.PathNode(self.tf_xy_local_position(gx, self.min_x),
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
                node = self.PathNode(current.x + self.motion[i][0], current.y + self.motion[i][1],
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

    def calc_obstacle_map(self):
        self.min_x = self.fly_boundary_position[0][0]
        self.min_y = self.fly_boundary_position[1][1]
        self.max_x = self.fly_boundary_position[2][0]
        self.max_y = self.fly_boundary_position[3][1]     
        
        self.x_width = self.max_x - self.min_x
        self.y_width = self.max_y - self.min_y

        self.obstacle_map = [[True for _ in range(self.y_width)] for _ in range(self.x_width)]

        obstacle_para = 1 # 1 = 1 m 1*1.414 = 1.41

        for ix in range(self.x_width):
            x = self.tf_xy_global_position(ix, self.min_x)
            flight_ab = round((-137107 * x + 477603048596) / 100000) + obstacle_para
            flight_bc = round((  26966 * x + 521412672558) / 100000) + obstacle_para
            flight_cd = round((-163043 * x + 470762353576) / 100000) - obstacle_para
            flight_da = round((  27253 * x + 521515438516) / 100000) - obstacle_para
            no_fly_ab = round((  23077 * x + 520390889785) / 100000) + obstacle_para
            no_fly_bc = round((-189189 * x + 463771693707) / 100000) + obstacle_para
            no_fly_cd = round((  25000 * x + 520895900000) / 100000) - obstacle_para
            for iy in range(self.y_width):
                y = self.tf_xy_global_position(iy, self.min_y)
                if x < self.no_fly_zone[0][0]:
                    if y > flight_ab and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.no_fly_zone[3][0]:
                    if y > no_fly_ab and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.fly_boundary_position[1][0]:
                    if y > flight_ab and y < no_fly_cd:
                        self.obstacle_map[ix][iy] = False
                    elif y > no_fly_ab and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.no_fly_zone[1][0]:
                    if y > flight_bc and y < no_fly_cd:
                        self.obstacle_map[ix][iy] = False
                    elif y > no_fly_ab and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.no_fly_zone[2][0]:
                    if y > flight_bc and y < no_fly_cd:
                        self.obstacle_map[ix][iy] = False
                    elif y > no_fly_bc and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.fly_boundary_position[3][0]:
                    if y > flight_bc and y < flight_da:
                        self.obstacle_map[ix][iy] = False
                elif x < self.fly_boundary_position[2][0]:
                    if y > flight_bc and y < flight_cd:
                        self.obstacle_map[ix][iy] = False
                        
        # self.min_x = (int)(min(ox))
        # self.min_y = (int)(min(oy))
        # self.max_x = (int)(max(ox))
        # self.max_y = (int)(max(oy))
        
        # self.x_width = (self.max_x - self.min_x) / self.resolution
        # self.y_width = (self.max_y - self.min_y) / self.resolution
        # self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]

        # for ix in range(self.x_width):
        #     x = self.tf_xy_global_position(ix, self.min_x)
        #     for iy in range(self.y_width):
        #         y = self.tf_xy_global_position(iy, self.min_y)
        #         for iox, ioy in zip(ox, oy):
        #             d = math.hypot(iox - x, ioy - y)
        #             if d <= self.rr:
        #                 self.obstacle_map[ix][iy] = True
        #                 break        

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
        #fly boundary bc
        for i in range(fly_boundary[1][0], fly_boundary[0][0]+1):
            ox.append(i)
            oy.append(round((-26852 * i * 100000 + 52138230581680) / 1000000000000, 7))
        #fly boundary cd
        for i in range(fly_boundary[1][0], fly_boundary[2][0]+1):
            ox.append(i)
            oy.append(round((164070 * i * 100000 + 47048846126540) / 1000000000000, 7))
        #fly boundary da
        for i in range(fly_boundary[2][0], fly_boundary[3][0]+1):
            ox.append(i)
            oy.append(round((-27160 * i * 100000 + 52149065369280) / 1000000000000, 7))
        #fly boundary ab
        for i in range(fly_boundary[0][0], fly_boundary[3][0]+1):
            ox.append(i)
            oy.append(round((137371 * i * 100000 + 47753256419832) / 1000000000000, 7))                
        
        # set no fly zone
        #no fly zone dc
        for i in range(no_fly_zone[1][0], no_fly_zone[0][0]+1):
            ox.append(i)
            oy.append(round((-25089 * i * 100000 + 52091972956157) / 1000000000000, 7))
        #no fly zone bc
        for i in range(no_fly_zone[1][0], no_fly_zone[2][0]+1):
            ox.append(i)
            oy.append(round((188464 * i * 100000 + 46396511668528) / 1000000000000, 7))
        #no fly zone ab
        for i in range(no_fly_zone[2][0], no_fly_zone[3][0]+1):
            ox.append(i)
            oy.append(round((-23068 * i * 100000 + 52038857631596) / 1000000000000, 7))

        return ox, oy            
        
    def start(self):
        pos = Int32MultiArray()
        pos_layout = MultiArrayLayout()
        pos_dim = MultiArrayDimension()
        pos.data = [v for nested in self.pos_t for v in nested]
        pos_dim.label = 'pos'
        pos_dim.size = len(self.pos_t)
        pos_dim.stride = len(pos.data)
        pos_layout.dim = [pos_dim]
        pos_layout.data_offset = 0
        pos.layout = pos_layout

        # self.ox, self.oy = self.obstacle_map_init(self.fly_boundary_position, self.no_fly_zone) 
        self.calc_obstacle_map() 
        rx, ry = self.planning(-267155, 5142341, -266877, 5142192)
        if len(rx) == len(ry):
            self.get_logger().info('this is path_planning {}'.format(len(rx)))
            current_pos = [-2.67155, 51.42341]

            for i in range(len(rx) - 1 , -1, -1):
                self.get_logger().info('this is path_planning pub xy pos {}'.format(i))

                position_x = rx[i] / 100000             
                position_y = ry[i] / 100000             
                if math.sqrt((current_pos[0] - position_x)**2 + (current_pos[1] - position_y)**2) > 0.0005:
                    current_pos = [position_x, position_y]
                    position_xy = str(position_x) + "," + str(position_y) 
                    position_msg = String()
                    position_msg.data = position_xy
                    self.get_logger().info('x={}, y={}'.format(position_x, position_y))
                    self.position_list_pub.publish(position_msg)

                    self.pos_list_pub.publish(pos)
        # self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('this is path_planning time_callback')          

        # if len(rx) == len(ry):
        #     for i in range(len(rx)):
        #         self.get_logger().info('this is path_planning pub xy pos {}'.format(i))

        #         position_x = rx[i]             
        #         position_y = ry[i]              
        #         position_xy = "x=" + str(position_x) + " " + "y=" + str(position_y) 
        #         position_msg = String()
        #         position_msg.data = position_xy
        #         self.get_logger().info('x={}, y={}'.format(position_x, position_y))
        #         self.position_list_pub.publish(position_msg)

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
