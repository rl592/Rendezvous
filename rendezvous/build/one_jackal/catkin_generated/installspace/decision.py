#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Bool, MultiArrayLayout, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import os
from queue import PriorityQueue
import matplotlib
matplotlib.use('Agg')
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pickle
import copy

class Graph:

    def minDistance(self,dist,queue):
        minimum = float("Inf")
        min_index = -1

        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i]
                min_index = i
        return min_index
        
    def findpath(self, j):
        if self.parent[j] == -1 :
            self.path.append(j)
            return
        self.findpath(self.parent[j])
        self.path.append(j)
   
    def getpath(self, i):
        self.path = []
        self.findpath(i)
        return self.path
 
    def dijkstra(self, graph, src):
        row = len(graph)
        col = len(graph[0])
        self.src = src
        dist = [float("Inf")] * row
        self.parent = [-1] * row
        dist[src] = 0
        queue = []
        for i in range(row):
            queue.append(i)
        while queue:
            u = self.minDistance(dist,queue)  
            queue.remove(u)
            for i in range(col):
                if graph[u][i] and i in queue:
                    if dist[u] + graph[u][i] < dist[i]:
                        dist[i] = dist[u] + graph[u][i]
                        self.parent[i] = u
        return dist
        

class NavSelection():

    def __init__(self):

        rospy.init_node('nav_selection')
        param_name = rospy.search_param('robot_name')
        self.robot_name = rospy.get_param(param_name)

        param_name = rospy.search_param('j1x')
        j1x = rospy.get_param(param_name)
        param_name = rospy.search_param('j1y')
        j1y = rospy.get_param(param_name)
        param_name = rospy.search_param('j2x')
        j2x = rospy.get_param(param_name)
        param_name = rospy.search_param('j2y')
        j2y = rospy.get_param(param_name)


        self.j1loc = np.array([j1x-j2x, j1y- j2y])
        self.lamb = []

        self.saveinfo = {'map':'square',
                         'config': '0',
                         'strategy': 'avarage edge length',
                         'lambda': 0,
                         'timesc': 0,
                         'timest': 0,
                         'distance': 0,
                         'total times ofVertices visited': 0,
                         'number of nodesVisited more than once': 0,
                         'Total number Of nodes': 0,
                         'rate1': '',
                         'rate2': '',
                         'result': 'Success'}

        param_name = rospy.search_param('map')
        self.saveinfo['map'] = str(rospy.get_param(param_name))
        param_name = rospy.search_param('config')
        self.saveinfo['config'] = str(rospy.get_param(param_name))
        param_name = rospy.search_param('strategy')
        self.saveinfo['strategy'] = str(rospy.get_param(param_name))

        rospy.loginfo(str(self.robot_name))

        self.pub = rospy.Publisher('nav_goal', Float32MultiArray, queue_size=1)

        self.curPose = None
        self.navi_goal = None
        self.stop = False
        self.distTravel = 0
        self.nodeRadius = 15 * 0.05
        self.nodes = np.empty([0,2])
        self.edges = np.empty([0,0])
        self.step = 0
        self.visited = None
        self.g= Graph()

        self.inittime = rospy.get_time()
        self.timelimit = 1500
        self.farest_node()





    def node_cb(self, data):

        if rospy.get_time() - self.inittime > self.timelimit and self.step > 0:
            info = self.robot_name + ": Time Limit Exceed! Time Elapsed: " + str(rospy.get_time() - self.inittime) + "seconds, Distance traveled: " + str(self.distTravel) + 'meters, vertices visited: '+ str(np.sum(self.visited))
            info += ', more than once: ' + str(sum(self.visited > 1)) + ', totol nodes: '+ str(self.visited.shape[0])
            rospy.loginfo(info)
            self.updatesaveinfo('Time Expired')
            self.output2txt()
            self.savedata('final')
            rospy.signal_shutdown(self.robot_name + ": Time Limit Exceed! Time Elapsed: " + str(rospy.get_time() - self.inittime) + "seconds, Distance traveled: " + str(self.distTravel) + 'meters, vertices visited: '+ str(np.sum(self.visited)))



        self.preNumOfNode = self.nodes.shape[0]
        graphinfo = np.array(data.data)
        graphinfo = graphinfo.reshape(1,-1)
        numOfNodes = (graphinfo[0][0]).astype(int)
        newnodes = graphinfo[0][1:numOfNodes*2+1].reshape(numOfNodes, 2)

        newEdges = graphinfo[0][numOfNodes*2+1:].reshape(numOfNodes,numOfNodes)
        newEdges = newEdges + newEdges.transpose()
        newEdges = self.preprocessedge(newEdges)
        self.indexOfnewNode = self.updateNodes(newnodes)
        self.updateEdges(newEdges)

        self.collisioncheck()

        self.covertrim()

        self.obtusetrim()

        self.validcheck()

        self.pathplaning()

        # self.plot_graph()
        
        self.savedata(str(self.step))

        self.cwd = os.getcwd()
        filename = str(self.cwd) + "/info/Gstar/graph_"+str(self.step) +".png"
        img = cv2.imread(filename, cv2.IMREAD_COLOR)
        imS = cv2.resize(img, (1200, 900))  
        cv2.imshow(str(self.robot_name) + ": Graph", imS)
        cv2.waitKey(6000)
        self.step += 1
    
    def ab_cb(self,msg):
        if msg.data:
            waypoints = self.nodes[self.curPose]
            newGoal = Float32MultiArray()
            newGoal.data = waypoints.ravel().tolist()
            self.pub.publish(newGoal)
            self.navi_goal = self.curPose
            cutedge = np.array(self.wyidx[-2:])
            self.edges[cutedge[0]][cutedge[1]] = 0
            self.edges[cutedge[1]][cutedge[0]] = 0

    def map_cb(self, msg):
        height = msg.info.height
        width = msg.info.width
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        tmp = np.array(msg.data).reshape(height, width)
        self.mapr = tmp
        self.mapr = np.transpose(self.mapr)
        tmp1 = np.append(self.mapr[:,1:], self.mapr[:,0].reshape(-1,1), axis = 1)
        tmp11 = np.append(tmp1[:,1:], tmp1[:,0].reshape(-1,1), axis = 1)
        tmp111 = np.append(tmp11[:,1:], tmp11[:,0].reshape(-1,1), axis = 1)
        tmp2 = np.append(self.mapr[:,-1].reshape(-1,1), self.mapr[:,:-1], axis = 1)
        tmp22 = np.append(tmp2[:,-1].reshape(-1,1), tmp2[:,:-1], axis = 1)
        tmp222 = np.append(tmp22[:,-1].reshape(-1,1), tmp22[:,:-1], axis = 1)
        tmp3 = np.append(self.mapr[1:,:], self.mapr[0].reshape(1,-1), axis = 0)
        tmp33 = np.append(tmp3[1:,:], tmp3[0].reshape(1,-1), axis = 0)
        tmp333 = np.append(tmp33[1:,:], tmp33[0].reshape(1,-1), axis = 0)
        tmp4 = np.append(self.mapr[-1].reshape(1,-1), self.mapr[:-1,:], axis = 0)
        tmp44 = np.append(tmp4[-1].reshape(1,-1), tmp4[:-1,:], axis = 0)
        tmp444 = np.append(tmp44[-1].reshape(1,-1), tmp44[:-1,:], axis = 0)
        self.mapr = self.mapr + tmp1 + tmp2 + tmp3 + tmp4 + tmp11 + tmp22 + tmp33 + tmp44 + tmp111 + tmp222 + tmp333 + tmp444

        # self.mapr = self.mapr + tmp1 + tmp2 + tmp3 + tmp4 

        tmp[tmp == 100] = 50
        tmp[tmp == 0] = 100
        tmp[tmp == -1] = 0
        tmp = np.transpose(tmp)
        tmp = np.flipud(tmp)

        self.tmap = np.zeros([160, 160])
        for i in range(0, 800, 5):
            for j in range(0, 800, 5):
                self.tmap[int(i/5)][int(j/5)] = tmp[i][j]

    def farest_node(self):
        self.sub2node = rospy.Subscriber("nodes", Float32MultiArray, self.node_cb)
        self.sub2ab = rospy.Subscriber('abortion', Bool, self.ab_cb)
        self.map = rospy.Subscriber("map", OccupancyGrid, self.map_cb)

        rospy.spin()

    def updateNodes(self, newNodes):
        indexOfnewNode = np.zeros(len(newNodes))
        if self.nodes.shape[0]:
            for i in range(len(newNodes)):
                dist = np.linalg.norm(self.nodes - newNodes[i], axis=1)
                if min(dist) < self.nodeRadius * 2:
                    indexOfnewNode[i] = np.argmin(dist)
                else:
                    self.nodes = np.append(self.nodes,[newNodes[i]],axis= 0)
                    self.edges = np.append(self.edges,np.zeros([1, self.edges.shape[1]]),axis= 0)
                    self.edges = np.append(self.edges,np.zeros([self.edges.shape[0], 1]),axis= 1)
                    indexOfnewNode[i] = len(self.nodes) - 1
                    self.visited = np.append(self.visited,0)
        else:
            self.nodes = newNodes
            indexOfnewNode = np.arange(len(newNodes))
            self.visited = np.zeros(len(newNodes))
            dist = np.linalg.norm(self.nodes, axis=1)
            self.navi_goal = np.argmin(dist)
            self.navi_point = self.nodes[self.navi_goal]
            self.inittime = rospy.get_time()


        return indexOfnewNode.astype(int)

    def updateEdges(self, newEdges):
        if self.edges.shape[0]:
            for i in range(newEdges.shape[0]):
                for j in range(newEdges.shape[1]):
                    if self.indexOfnewNode[i] >= self.preNumOfNode or self.indexOfnewNode[j] >= self.preNumOfNode:
                        if newEdges[i][j] > 0.1:
                            self.edges[self.indexOfnewNode[i]][self.indexOfnewNode[j]] =  np.linalg.norm(self.nodes[self.indexOfnewNode[i]] - self.nodes[self.indexOfnewNode[j]])
        else:
            self.edges = newEdges
   
    def bres(self, x1, y1, x2, y2):
        x11 = x1
        y11 = y1
        x, y = x1, y1
        dx = abs(x2- x1)
        dy = abs(y2- y1)
        gradient = dy/(float(dx) + 0.01)
        
        if gradient > 1:
            dx, dy = dy, dx
            x, y = y, x
            x1, y1 = y1, x1
            x2, y2 = y2, x2
            
        p = 2 * dy - dx
        cxy = [[y, x]]
        for k in range(dx):
            if p > 0:
                y = y + 1 if y <y2 else y-1
                p = p + 2 * (dy- dx)
            else:
                p = p + 2 * dy
            
            x = x + 1 if x < x2 else x - 1
            cxy.append([y,x])
        if cxy[0] == [x11,y11]:
            return np.array(cxy)
        else:
            return np.fliplr(np.array(cxy))

    def collisioncheck(self):
        nodes_to_delete = []
        for i in range(self.nodes.shape[0]):
            if self.visited[i] == 0:
                nix = int(( self.nodes[i][0] - self.origin_x) / self.resolution)
                niy = int(( self.nodes[i][1] - self.origin_y) / self.resolution)
                for j in range(50):
                    angle = 7.2 * j * np.pi / 180
                    cx = np.cos(angle) * 7 + nix
                    cy = np.sin(angle) * 7 + niy
                    if self.mapr[int(cx)][int(cy)] != 0:
                        nodes_to_delete.append(i)
                        break

        self.edges = np.delete(self.edges, nodes_to_delete, 0)
        self.edges = np.delete(self.edges, nodes_to_delete, 1)
        self.nodes = np.delete(self.nodes, nodes_to_delete, 0)
        self.visited = np.delete(self.visited, nodes_to_delete)


        self.navi_goal = np.argmin(np.linalg.norm(self.nodes - self.navi_point, axis=1))
        self.navi_point = self.nodes[self.navi_goal]


        edges_to_delete = []
        for i in range(self.nodes.shape[0]):      
            nix = int((self.nodes[i][0] - self.origin_x) / self.resolution)
            niy = int((self.nodes[i][1] - self.origin_y) / self.resolution)

            for j in range(i+1, self.nodes.shape[0]):
                if self.edges[i][j] >0:
                    njx = int((self.nodes[j][0] - self.origin_x) / self.resolution)
                    njy = int((self.nodes[j][1] - self.origin_y) / self.resolution)
                    cds = self.bres(nix,niy,njx, njy)

                    sumOfmp = 0            

                    for k in range(cds.shape[0]):
                        sumOfmp += self.mapr[cds[k][0]][cds[k][1]]     
                        if self.mapr[cds[k][0]][cds[k][1]] > 0 or sumOfmp < - 300:
                            edges_to_delete.append([i, j])
                            # rospy.loginfo('found edges to delete (collision): (' + str(i)+ ',' + str(j) + ')')
                            break
        for i in range(len(edges_to_delete)):
            self.edges[edges_to_delete[i][0]][edges_to_delete[i][1]] = 0
            self.edges[edges_to_delete[i][1]][edges_to_delete[i][0]] = 0

    def validcheck(self):

        D = {v:float('inf') for v in range(self.edges.shape[0])}
        
        D[0] = 0
        # D[self.navi_goal] = 0

        pq = PriorityQueue()
        # pq.put((0, self.navi_goal))
        pq.put((0, 0))
        visited = []
        while not pq.empty():
            (dist, current_vertex) = pq.get()
            visited.append(current_vertex)

            for neighbor in range(self.edges.shape[0]):
                if self.edges[current_vertex][neighbor] > 0:
                    distance = self.edges[current_vertex][neighbor]
                    if neighbor not in visited:
                        old_cost = D[neighbor]
                        new_cost = D[current_vertex] + distance
                        if new_cost < old_cost:
                            pq.put((new_cost, neighbor))
                            D[neighbor] = new_cost

        to_delete = []
        tm = []
        for i in range(len(D)):
            if np.isinf(D[i]):
                to_delete.append(i)
            tm.append(D[i])
        # rospy.loginfo("to_delete(valid): " + str(to_delete) + " \n tm: " +str(tm))
        self.edges = np.delete(self.edges, to_delete, 0)
        self.edges = np.delete(self.edges, to_delete, 1)
        self.nodes = np.delete(self.nodes, to_delete, 0)
        self.visited = np.delete(self.visited, to_delete)

        self.navi_goal = np.argmin(np.linalg.norm(self.nodes - self.navi_point, axis=1))
        self.navi_point = self.nodes[self.navi_goal]

    def covertrim(self):
        for i in range(self.edges.shape[0]):
            for j in range(self.edges.shape[0]):
                if self.edges[i][j] > 0:
                    for k in range(self.edges.shape[0]):
                        if k!=i and k!=j:
                            v1 = self.nodes[i] - self.nodes[k]
                            v2 = self.nodes[j] - self.nodes[k]                            

                            if np.inner(v1,v2) <0:
                                v1d = np.linalg.norm(v1)
                                v2d = np.linalg.norm(v2)
                                v3d = np.linalg.norm(self.nodes[i] - self.nodes[j])

                                p = (v1d+v2d+v3d)/2
                                s = (p*(p-v1d)*(p-v2d)*(p-v3d))**0.5
                                h = 2*s/v3d

                                if h <= self.nodeRadius:
                                    # rospy.loginfo('found edges to delete (cover): (' + str(i)+ ',' + str(j) + ') connect: ('+str(i) +','+str(k)+')('+str(j)+ ','+str(k)+') order: ' + str(i)+ ','+ str(j) + ',' + str(k))
                                    self.edges[i][j] = 0
                                    self.edges[j][i] = 0
                                    self.edges[i][k] = np.linalg.norm(self.nodes[i] - self.nodes[k])
                                    self.edges[k][i] = self.edges[i][k]                        
                                    self.edges[j][k] = np.linalg.norm(self.nodes[j] - self.nodes[k])
                                    self.edges[k][j] = self.edges[j][k]

    def obtusetrim(self):
        for i in range(self.nodes.shape[0]):
            for j in range(i+1, self.nodes.shape[0]):
                if self.edges[i][j] > 0:
                    for k in range(j+1, self.nodes.shape[0]):
                        if self.edges[j][k] > 0 and self.edges[i][k] > 0 :
                            
                            v1 = self.nodes[j] - self.nodes[i]
                            v2 = self.nodes[k] - self.nodes[i]
                            if np.inner(v1,v2) < 0:
                                self.edges[j][k] = 0
                                self.edges[k][j] = 0
                                # rospy.loginfo('found edges to delete (obtuse): (' + str(j)+ ',' + str(k) + ') order: ' + str(i)+ ','+ str(j) + ',' + str(k))

                            v1 = self.nodes[i] - self.nodes[j]
                            v2 = self.nodes[k] - self.nodes[j]
                            if np.inner(v1,v2) < 0:
                                self.edges[i][k] = 0
                                self.edges[k][i] = 0
                                # rospy.loginfo('found edges to delete (obtuse): (' + str(i)+ ',' + str(k) + ') order: ' + str(i)+ ','+ str(j) + ',' + str(k))


                            v1 = self.nodes[j] - self.nodes[k]
                            v2 = self.nodes[i] - self.nodes[k]
                            if np.inner(v1,v2) < 0:
                                self.edges[j][i] = 0
                                self.edges[i][j] = 0

    def pathplaning(self):
        if self.saveinfo['strategy'] == 'avarage edge length':
            self.lamb.append(self.get_avglength(self.edges))
        elif self.saveinfo['strategy'] == 'far':
            self.lamb.append(0)
        elif self.saveinfo['strategy'] == 'exploration':
            self.lamb.append(100)
        else:
            self.lamb.append(float(self.saveinfo['strategy']))

        self.curPose = self.navi_goal

        dist = np.array(self.g.dijkstra(self.edges,self.curPose))
        self.distedge = copy.deepcopy(dist)
        weight = np.zeros(dist.shape[0])
        self.visitweight = np.zeros(dist.shape[0])
        for i in range(dist.shape[0]):
            if i!=self.curPose:
                path = self.g.getpath(i)
                weight[i] = dist[i]
                for j in range(len(path)):
                    weight[i] -= self.lamb[-1] * self.visited[path[j]]
                    self.visitweight[i] += self.visited[path[j]]
                    # pass
            else:
                weight[i] = float('-inf')
        
        self.weighted_dist = copy.deepcopy(weight)

        if self.stop:
            info = "Finish! Time Elapsed: " + str(rospy.get_time() - self.inittime) + "seconds, Distance traveled: " + str(self.distTravel) + 'meters, vertices visited: '+ str(np.sum(self.visited))
            info += ', more than once: ' + str(sum(self.visited > 1)) + ', totol nodes: '+ str(self.visited.shape[0])
            rospy.loginfo(info)
            self.updatesaveinfo('Success')
            self.output2txt()
            self.savedata('final')
            self.plot_graph()



        if not self.stop:
            if  self.found():
                self.navi_goal = np.argmin(np.linalg.norm(self.nodes - self.j1loc, axis=1))
                self.stop = True
            else:
                frontier = []
                trap = []
                for i in range(self.nodes.shape[0]):
                    idx = np.nonzero(self.edges[i])
                    neibor = self.nodes[idx]
                    avgLoc = np.mean(neibor, 0)
                    if np.linalg.norm(avgLoc - self.nodes[i]) >= 2 * self.nodeRadius:
                        frontier.append(i)
                    
                    if any(vis >= 3 for vis in self.visited[idx]):
                        trap.append(i)
                    
                numOfoption = 0    
                for nodeidx in frontier:
                    if nodeidx not in trap:
                        numOfoption += 1
                if numOfoption > 0:
                    for i in range(self.nodes.shape[0]):
                        if i not in frontier or i in trap:
                            weight[i] = float('-inf')

                
                self.navi_goal = np.argmax(weight)
                self.stop = False           


            self.distTravel += dist[self.navi_goal]

            self.navi_point = self.nodes[self.navi_goal]

            self.wyidx = self.g.getpath(self.navi_goal)
            waypoints = self.nodes[self.wyidx]
            for i in range(len(self.wyidx)-1):
                self.visited[self.wyidx[i]] += 1


            newGoal = Float32MultiArray()
            newGoal.layout.dim.append(MultiArrayDimension())
            newGoal.layout.dim.append(MultiArrayDimension())
            newGoal.layout.dim[0].size = len(waypoints.ravel().tolist()[2:])
            newGoal.data = waypoints.ravel().tolist()[2:]
            self.plot_graph()


            if len(newGoal.data) > 1 :
                self.pub.publish(newGoal)

            else:
                if self.found():
                    info = "Finish! Time Elapsed: " + str(rospy.get_time() - self.inittime) + "seconds, Distance traveled: " + str(self.distTravel) + 'meters, vertices visited: '+ str(np.sum(self.visited))
                    info += ', more than once: ' + str(sum(self.visited > 1)) + ', totol nodes: '+ str(self.visited.shape[0])
                    rospy.loginfo(info)
                    self.updatesaveinfo('Success')
                    self.output2txt()
                    self.savedata('final')
                    filename = str(self.cwd) + "/info/Gstar/graph_"+str(self.step) +".png"
                    while not os.path.exists(filename):
                        pass
                    rospy.signal_shutdown(self.robot_name + ": Finish! Time Elapsed: " + str(rospy.get_time() - self.inittime) + " seconds; Distance traveled: " + str(self.distTravel) + ' meters')


        else:
            filename = str(self.cwd) + "/info/Gstar/graph_"+str(self.step) +".png"
            while not os.path.exists(filename):
                pass
            rospy.signal_shutdown(self.robot_name + ": Finish! Time Elapsed: " + str(rospy.get_time() - self.inittime) + " seconds; Distance traveled: " + str(self.distTravel) + ' meters')

    def plot_graph(self):
        

        fig = plt.figure(num = self.step, figsize=(12, 12), dpi=200)
        ax = fig.gca()

        for i in range(self.nodes.shape[0]):
            plt.text(self.nodes[i][1]-0.5, -self.nodes[i][0]-0.08 + 0.3, str(round(self.distedge[i], 2))      + ',' + str(int(self.visitweight[i])) , fontsize = 5)
            plt.text(self.nodes[i][1]-0.5, -self.nodes[i][0]-0.08 - 0.3, str(round(self.weighted_dist[i], 2)) + ',' +  str(i), fontsize = 5)

            if i == self.curPose:
                col = [0,0,1]
                lg = 'Current Node'
            elif i == self.navi_goal:
                col = [1,0,0]
                lg = 'Goal'
            else:
                col = [0.9-0.3*self.visited[i] for j in (0,1,2)] 
                if col[0] < 0:
                    col = [0,0,0]
                if self.visited[i] == 0:
                    lg = 'Unvisited Nodes'
                elif self.visited[i] == 1:
                    lg = "Nodes visited once"
                elif self.visited[i] == 2:
                    lg = "Nodes visited twice"
                else:
                    lg = "Nodes visited more than twice"
            
            circle = plt.Circle((self.nodes[i][1], -self.nodes[i][0]), self.nodeRadius, color=col,fill=True, label=lg)
            ax.add_patch(circle)

        for i in range(self.edges.shape[0]):
            for j in range(self.edges.shape[1]):
                if self.edges[i][j] > 0 :
                    plt.plot([self.nodes[i][1], self.nodes[j][1]],[-self.nodes[i][0], -self.nodes[j][0]],color='c',linestyle="dotted", label='Edges')
        
        for i in range(len(self.wyidx)-1):
            plt.plot([self.nodes[self.wyidx[i]][1], self.nodes[self.wyidx[i+1]][1]],[-self.nodes[self.wyidx[i]][0], -self.nodes[self.wyidx[i+1]][0]],color='r',linestyle="solid", label='Path')

        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(),fontsize=15, bbox_to_anchor=(1.0, 0.71))
        x = np.arange(160) * 0.25 + self.origin_x
        y = np.arange(160) * 0.25 + self.origin_y
        x, y = np.meshgrid(x, y)
        plt.scatter(x,y,c=self.tmap, marker='s', cmap='hot', s=55)


        xmin, xmax= -20, 20
        ymin, ymax= -20, 20
        
        plt.xlim( xmin, xmax)
        plt.ylim( ymin, ymax)
        
        plt.text(xmax+1,ymax,  s='sim time: ' + str(round(rospy.get_time() - self.inittime, 2)) + ' seconds',c='red', fontsize = 12)
        plt.text(xmax+1,ymax-1,s='distance traveled: ' + str(round(self.distTravel, 2)) + ' meters',         c='red', fontsize = 12)
        plt.text(xmax+1,ymax-2,s='lambda: ' + str(round(self.lamb[-1],2)),                                      c='red', fontsize = 12)
        plt.text(xmax+1,ymax-3,s='vertices visited: '+ str(int(np.sum(self.visited))) ,                 c='red', fontsize = 12)
        plt.text(xmax+1,ymax-4,s='more than once: ' + str(sum(self.visited > 1)) ,                 c='red', fontsize = 12)
        plt.text(xmax+1,ymax-5,s='totol nodes: '+ str(self.visited.shape[0]) ,                     c='red', fontsize = 12)

        plt.xlabel("x [m]")    
        plt.ylabel("y [m]")  
        plt.title("Graph* at time k="+str(self.step), fontsize = 15)
        plt.close(fig)
        fig.savefig("info/Gstar/graph_"+str(self.step) +".png", bbox_inches='tight')
  
    def printedgeinfo(self):
        edgeinfo = ""
        for i in range(self.edges.shape[0]):
            edgeinfo += str(i) + ': '
            for j in range(i+1, self.edges.shape[1]):
                if self.edges[i][j] > 0:
                    edgeinfo += str(j) + ','
            edgeinfo += '\n'
        rospy.loginfo('edges: \n' + edgeinfo)

    def savedata(self, name):
        siminfo = {}
        siminfo['nodes'] = self.nodes
        siminfo['edges'] = self.edges
        siminfo['visited'] = self.visited
        siminfo['disttravel'] = self.distTravel
        siminfo['simtime'] = rospy.get_time() - self.inittime
        siminfo['distedge'] = self.distedge
        siminfo['sumofvisit'] = self.visitweight
        siminfo['weighted_dist'] = self.weighted_dist

        pickle.dump(siminfo, open( "info/dict/" + name, "wb" ) )

    def found(self):
        nix = int(( self.j1loc[0] - self.origin_x) / self.resolution)
        niy = int(( self.j1loc[1] - self.origin_y) / self.resolution)
        freecnt = 0
        for i in np.arange(0, 8):
            for j in range(50):
                angle = 7.2 * j * np.pi / 180
                cx = np.cos(angle) * i + nix 
                cy = np.sin(angle) * i + niy
                if self.mapr[int(cx)][int(cy)] == 0:
                    freecnt += 1
                
                if freecnt >=200:
                    return True
        return False

    def get_avglength(self, edges):
        numOfedge = 0
        sumOflength = 0
        for i in range(edges.shape[0]):
            for j in range(edges.shape[1]):
                if edges[i][j] > 0:
                    numOfedge += 1
                sumOflength += edges[i][j]
        return sumOflength / numOfedge

    def preprocessedge(self, edges):
        newedges = copy.deepcopy(edges)
        numOfedge = 0
        sumOfpowlength = 0
        avg = self.get_avglength(edges)
        for i in range(edges.shape[0]):
            for j in range(i+1, edges.shape[0]):
                if edges[i][j] > 0:
                    numOfedge += 1
                sumOfpowlength += (avg - edges[i][j]) ** 2                
        stddev = (sumOfpowlength / numOfedge) ** 0.5
        cutoff = avg + stddev / 2

        for i in range(edges.shape[0]):
            for j in range(edges.shape[0]):
                if edges[i][j] > cutoff:
                    newedges[i][j] = 0
                    # rospy.loginfo('cut')
        return newedges


    def updatesaveinfo(self, res):
        self.saveinfo['lambda'] = str(round(np.mean(self.lamb),2))
        self.saveinfo['timesc'] = str(round(rospy.get_time() - self.inittime, 2))
        self.saveinfo['timest'] = self.step
        self.saveinfo['distance'] = str(round(self.distTravel, 2))
        self.saveinfo['total times ofVertices visited'] = str(np.sum(self.visited))
        self.saveinfo['number of nodesVisited more than once'] = str(sum(self.visited > 1))
        self.saveinfo['Total number Of nodes'] = str(self.visited.shape[0])
        
        self.saveinfo['rate1'] = str(round(np.sum(self.visited)/self.visited.shape[0], 2))
        self.saveinfo['rate2'] = str(round(sum(self.visited > 1)/self.visited.shape[0], 2))

        self.saveinfo['result'] = res

    def output2txt(self):
        testfile =  open("test_file.txt", "a")
        for key in self.saveinfo:
            testfile.write(str(self.saveinfo[key]) + '\t')
        testfile.write('\n')
        testfile.close()


if __name__ == '__main__':
    try:

        NavSelection()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")