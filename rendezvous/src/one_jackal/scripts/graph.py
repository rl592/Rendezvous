#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pandas as pd 
import matplotlib.pyplot as plt
# import matplotlib


test_map = OccupancyGrid()
bridge = CvBridge()



def bres(x1, y1, x2, y2):
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

def callback(data):
    global test_map
    global bridge
    pub = rospy.Publisher("/test_map", OccupancyGrid, queue_size=10)
    height = data.info.height
    width = data.info.width
    tmp = np.array(data.data).reshape(height, width)
    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y
    origin_z = data.info.origin.position.z
    resolution = data.info.resolution
    rospy.loginfo(str(origin_x)+','+str(origin_y)+','+str(origin_z))
    rospy.loginfo('resolution: '+str(resolution))

    # mapr = tmp
    # mapr = np.transpose(mapr)
    # tmp1 = np.append(mapr[:,1:], mapr[:,0].reshape(-1,1), axis = 1)
    # tmp11 = np.append(tmp1[:,1:], tmp1[:,0].reshape(-1,1), axis = 1)
    # tmp111 = np.append(tmp11[:,1:], tmp11[:,0].reshape(-1,1), axis = 1)
    # tmp2 = np.append(mapr[:,-1].reshape(-1,1), mapr[:,:-1], axis = 1)
    # tmp22 = np.append(tmp2[:,-1].reshape(-1,1), tmp2[:,:-1], axis = 1)
    # tmp222 = np.append(tmp22[:,-1].reshape(-1,1), tmp22[:,:-1], axis = 1)
    # tmp3 = np.append(mapr[1:,:], mapr[0].reshape(1,-1), axis = 0)
    # tmp33 = np.append(tmp3[1:,:], tmp3[0].reshape(1,-1), axis = 0)
    # tmp333 = np.append(tmp33[1:,:], tmp33[0].reshape(1,-1), axis = 0)
    # tmp4 = np.append(mapr[-1].reshape(1,-1), mapr[:-1,:], axis = 0)
    # tmp44 = np.append(tmp4[-1].reshape(1,-1), tmp4[:-1,:], axis = 0)
    # tmp444 = np.append(tmp44[-1].reshape(1,-1), tmp44[:-1,:], axis = 0)
    # mapr = mapr + tmp1 + tmp2 + tmp3 + tmp4 + tmp11 + tmp22 + tmp33 + tmp44+ tmp111 + tmp222 + tmp333 + tmp444


    # nodes = np.array([[-4.84999943 , 3.85000038],
    #                   [-7.94999981 , 3.20000076]])
    # nodes = np.array([ [ 2.          ,5.39999962],
    #                     [ 4.39999962  ,5.64999962]])

    # nodes = np.array([[ 3.64999962 , 0.5       ],
    #                  [ 0.14999962 , 1.        ]])   


    # nodes = np.array([ [-4.89999962 , 9.39999962],
    #                     [-7.5       ,  7.25      ]])
    
    # nix = int((nodes[0][0] + 20)/resolution)
    # niy = int((nodes[0][1] + 20)/resolution)        
    # rospy.loginfo('(' + str(nix) + ',' + str(niy)+')')

    # njx = int((nodes[1][0] + 20)/resolution)
    # njy = int((nodes[1][1] + 20)/resolution)        
    # rospy.loginfo('(' + str(njx) + ',' + str(njy)+')')

    # cds = bres(nix,niy,njx, njy)
    # mm = []
    # for k in range(cds.shape[0]):            
    #     mm.append(mapr[cds[k][0]][cds[k][1]])
    #     # mapr[cds[k][0]][cds[k][1]] = 200
    # rospy.loginfo('\n'+ str(mm) )


    # nodes = np.array([ [ 3.25      ,  0.80000114],
    #                     [ 2.75        ,3.70000076] ])

    # nix = int((nodes[0][0] + 20)/resolution)
    # niy = int((nodes[0][1] + 20)/resolution)        
    # rospy.loginfo('(' + str(nix) + ',' + str(niy)+')')

    # njx = int((nodes[1][0] + 20)/resolution)
    # njy = int((nodes[1][1] + 20)/resolution)        
    # rospy.loginfo('(' + str(njx) + ',' + str(njy)+')')
    # cds = bres(nix,niy,njx, njy)
    # mm = []
    # for k in range(cds.shape[0]):            
    #     mm.append(mapr[cds[k][0]][cds[k][1]])
    #     mapr[cds[k][0]][cds[k][1]] = 200
    # rospy.loginfo('\n'+ str(cds) )


    # for k in range(cds.shape[0]):            
    #     if mapr[cds[k][0]][cds[k][1]] > 0:
    #         rospy.loginfo('Delete')
    #         break


    # nodes = np.array([[-2 , -1     ],
    #                  [ 6 , 8.        ]])   
    # nix = int((nodes[0][0] + 20)/resolution)
    # niy = int((nodes[0][1] + 20)/resolution)  
    # njx = int((nodes[1][0] + 20)/resolution)
    # njy = int((nodes[1][1] + 20)/resolution)     
    # winshow = np.zeros([max(njx-nix + 1, njy-niy + 1), max(njx-nix + 1, njy-niy + 1)])
    # for i in range(winshow.shape[0]):
    #     for j in range(winshow.shape[1]):
    #         if mapr[nix +i][niy + j] == -1:
    #             winshow[i][j] = 0
    #         elif mapr[nix +i][niy + j] == 100:
    #             # winshow[i][j] = mapr[nix +i][niy + j]
    #             winshow[i][j] = 200
    #         elif mapr[nix +i][niy + j] == 200:
    #             winshow[i][j] = 200

    #         else:
    #             winshow[i][j] = 0



    # rospy.loginfo('map value: \n'+ str(mm) + 'sum: ' + str(np.sum(mm)))
    # x = np.arange(max(njx-nix + 1, njy-niy + 1))
    # y = np.arange(max(njx-nix + 1, njy-niy + 1)) 
    # x, y = np.meshgrid(x, y) 

    # cv2.imshow('RGB Image',winshow )        
    # # cv2.imshow('haha', winshow)
    # # plt.show()
    # cv2.waitKey(0)
    
    rospy.loginfo(rospy.get_time())



    # nodes_to_delete = []
    # for i in range(nodes.shape[0]):
    #     nix = 800 -  int((-nodes[i][0] + 20)*20)
    #     niy = int(( nodes[i][1] + 20)*20)
    #     for j in range(50):
    #         angle = 7.2 * j * np.pi / 180
    #         cx = np.cos(angle) * 13 + nix
    #         cy = np.sin(angle) * 13 + niy
    #         if mapr[int(cx)][int(cy)] != 0:
    #             nodes_to_delete.append(i)
    #             break
    # rospy.loginfo("j2: to_delete\n:"+str(nodes_to_delete)+ '\n'+ str(edges_to_delete))

    # for i in range(map.shape[0]):
    #     for j in range(map.shape[1]):
    #         if map[i, j] >=0:
    #             map[i, j]  =50
    # for i in range(100):
    #     for j in range(100):
    #         map[i, j]  =100
    # tmap = map.reshape(-1,)
    # test_map = data
    # test_map.data = tmap
    # pub.publish(test_map)

    # try:
    #     # We select bgr8 because its the OpenCV encoding by default
    #     cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # except CvBridgeError as e:
    #     print(e)
    #Converted the datatype to np.uint8
    new_image = tmp.astype(np.uint8)
    # new_image = map
    new_image = np.transpose(new_image)
    # # new_image = np.flipud(new_image)    

    pd.DataFrame(new_image).to_csv("mapupson.csv")
    # cv2.imwrite("map.jpg", new_image ) 
    # imS = cv2.resize(new_image, (900, 900))  

    # cv2.imshow('image',imS)
    # cv2.waitKey(0)
    # fig = plt.figure(figsize=(10, 10), dpi=80)
    # map[map == -1] = 100
    # map = np.transpose(map)
    # map = np.flipud(map)
    # map[map == 0] = 1
    # map[map == 100] = 0

    # tmap = np.zeros([200, 200])
    # for i in range(0, 800, 4):
    #     for j in range(0, 800, 4):
    #         tmap[int(i/4)][int(j/4)] = map[i][j]
    # x = np.arange(200) * 0.2 - 20
    # y = np.arange(200) * 0.2 - 20
    # x, y = np.meshgrid(x, y)

    # plt.scatter(x,y,c=tmap, cmap = 'gray')
    # plt.xlim( -2,19)
    # plt.ylim( -6,12)
    # plt.close

    # fig.savefig("map.png")

    # # plt.show()
    rospy.loginfo('Done')



def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/j2/map", OccupancyGrid, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()