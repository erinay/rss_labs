from graph import *
import numpy as np
import rospy
from geometry_msgs.msg import Point

class RRT:
    def __init__(self, start, end, maps, bound, max_dist, resolution, map_origin_pos):
        self.start = start
        self.goal = end
        self.maps = maps
        self.bound = bound
        self.max_dist = max_dist
        self.target_range = 15
        self.map_resolution = resolution
        self.map_origin_pos=map_origin_pos

        gr = Graph()
        gr.add_node(start)
        reached = (self.goal==self.start)
        searchNodes = [SearchNode(self.start)]
        self.count = 0        

        self.node_pub = rospy.Publisher('node_tree', Point, queue_size=10)

        # # # See if using list is faster
        # tree_nodes = []
        # tree_parent = []
        # tree_nodes.append(start)
        

        # while(not reached):
        while(not reached and self.count<=10000):
            rospy.loginfo(self.count)
            rand_pt = self.random_path()
            xnear = gr.nearest(rand_pt)
            new_x = self.step(xnear, rand_pt, self.target_range) #steering

            xy_pt = self.pixel2Map(new_x)
            # map_path.append(xy_pt)

            point = Point()
            point.x = xy_pt[0]
            point.y = xy_pt[1]

            self.trajectory.addPoint(point)
            self.node_pub.publish(self.trajectory.toPoseArray())


            # rospy.loginfo(xnear)
            # rospy.loginfo(new_x)

            # Check path collision
            if not self.collision(xnear, new_x):
                gr.add_edge(xnear, new_x)
                # Find parent node to create SearchNode for Pathing
                for searchnode in searchNodes:
                    # If existing search node states is the same as the closest node
                    # Make new Search node instance with previous search node as the parent
                    if(searchnode._state == xnear):
                        parent = searchnode
                        break
                newnode = SearchNode(new_x, parent)
                searchNodes.append(newnode)
                    
                # check if point exists in goal
                g = np.asarray(self.goal)
                # c = np.asarray(newnode._state)
                diff = np.linalg.norm(g-new_x)
                # rospy.loginfo(diff)
                if (diff < 10):
                    goalnode = SearchNode(self.goal, newnode)
                    searchNodes.append(goalnode)
                    reached=True
                else: 
                    self.count += 1
            else:
                self.count+=1
        self.path = Path(searchNodes[-1]) 
        self.final_gr = gr._nodes

    def find_nearest(self, x_rand, nodes):
        diff = x_rand - np.array(nodes)

    def random_path(self):
        # Set random node
        if (self.count%200==0):
            x = self.goal[0]
            y = self.goal[1]
        else:
            x = np.random.uniform(self.bound[1])
            y = np.random.uniform(self.bound[0])
        return [x,y]

    def step(self, xnear, xnew, step):
        x1 = np.asarray(xnear)
        x2 = np.asarray(xnew)
        dist = np.linalg.norm(x1-x2)
        xnew = xnear
        if(dist>self.max_dist):
            diff = x2-x1
            # Unit step
            dx = diff[0]/dist
            dy = diff[1]/dist

            theta = np.arctan2(dy, dx)
            x_step = step*np.cos(theta)
            y_step = step*np.sin(theta)
            
            x = xnear[0]+x_step*dx
            y = xnear[1]+y_step*dy
            xnew = (x,y)

        return(xnew)

    def collision(self, xnear, xnew):

        ##AABB collision checking
        # rospy.loginfo(xnear)
        # rospy.loginfo(xnew)
        # rospy.loginfo('goal'+str(self.goal))

        xA = np.round(np.asarray(xnear)).astype(int)
        xB = np.round(np.asarray(xnew)).astype(int)

        mapx_min = min(xA[0], xB[0])
        mapx_max = max(xA[0], xB[0])
        mapy_min = min(xA[1], xB[1])
        mapy_max = max(xA[1], xB[1])

        map_rectangle = self.maps[mapx_min:mapx_max+1, mapy_min:mapy_max+1]
        # rospy.loginfo(map_rectangle)
        # compare data pt, where unknown = -1, occupancy b/t [0,100]
        try:
            if(np.amax(map_rectangle)>10): 
                collision = True
            else:
                collision = False
        except:
            collision=True
            # rospy.loginfo(map_rectangle)

        return collision
    
    def cost(self, parent, xnew):
        pass
    def pixel2Map(self, pixel):
        # Convert pixel to coordinate
        u = pixel[1]*self.map_resolution
        v = pixel[0]*self.map_resolution
        conv = np.array([u, v])-np.array([self.map_origin_pos.x, self.map_origin_pos.y])
        xy_map = np.dot(self.rot_mat_inv, conv)
        # x = np.matmul(self.tr_mat[0][:],np.array([[u],[v],[1]]))
        # y = np.matmul(self.tr_mat[1][:],np.array([[u],[v],[1]]))

        return (xy_map[0].astype(float),xy_map[1].astype(float))
    