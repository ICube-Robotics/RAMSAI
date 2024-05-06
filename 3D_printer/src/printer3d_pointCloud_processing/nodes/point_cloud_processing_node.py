#!/usr/bin/env python3
# Copyright 2023 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from printer3d_msgs.srv import GcodeCommand
import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import matplotlib as mpl
import random
import pyransac3d as pyrsc
from math import sqrt

""" parametres du noeuds ROS, à mettre dans un fichier de config a la fin"""
basic_limit_for_reflexions = 12


coords = []

def onclick(event):
    ix, iy = event.xdata, event.ydata
    print ('x = ',ix,', y = ',iy)
    global coords
    coords.append((ix, iy))
    if len(coords) == 2:
        print('ok')
        plt.close()
    return coords

class PointCloudProcessingNode(Node):
    def __init__(self):
        """PointCloudProcessingNode class constructor."""
        super().__init__('point_cloud_processing_node')
        self.xinf = -100000
        self.xsup = 100000
        self.x1 = -100000
        self.x2 = 100000
        self.y1 = -100000
        self.y2 = 100000

    def load_scan(self,fileName):
        data = np.squeeze(np.load(fileName))
        xDim = data.shape[1]
        yDim = data.shape[0]
        zDim = data.shape[2]
        for i in range(0,yDim):
            for j in range(0,xDim):
                data[i,j,1] = i*0.3
        return data

    def flatten_point_cloud(self,inputPointCloud):
        if len(inputPointCloud.shape) == 2:
            outputPointCloud = inputPointCloud
        else:
            outputPointCloud = np.reshape(inputPointCloud,(inputPointCloud.shape[0]*inputPointCloud.shape[1],inputPointCloud.shape[2]))
        return outputPointCloud

    def remove_non_used_plate_points(self,inputPointCloud):
        treatedPointCloud = self.remove_inf_value(inputPointCloud)
        shapes = treatedPointCloud.shape
        savedPoints = []
        for i in range(0,shapes[0]):
            if treatedPointCloud[i,0]>self.xinf and treatedPointCloud[i,0]<self.xsup:
                savedPoints.append(treatedPointCloud[i,:])
        savedPoints = np.array(savedPoints)
        return savedPoints

    def find_layer_barycentre(self, inputPointCloud, epsilon):

        treatedPointCloud = self.remove_inf_value(inputPointCloud)
        shapes = treatedPointCloud.shape
        savedPoints = []

        for i in range(0,shapes[0]):
            if treatedPointCloud[i,0]>self.x1 and treatedPointCloud[i,0]<self.x2 and treatedPointCloud[i,1]>self.y1 and treatedPointCloud[i,1]<self.y2:
                savedPoints.append(treatedPointCloud[i,:])
        savedPoints = np.array(savedPoints)

        zMoy = 0
        newShape = savedPoints.shape
        for i in range(0, newShape[0]):
            zMoy += savedPoints[i,2]
        zMoy /= newShape[0]

        print(savedPoints.shape)
        print(zMoy)

        xg = 0
        yg = 0
        nPts = 0

        for i in range(0,shapes[0]):
            if treatedPointCloud[i,2]>zMoy-epsilon and treatedPointCloud[i,2]<zMoy+epsilon:
                nPts += 1
                xg += treatedPointCloud[i,0]
                yg += treatedPointCloud[i,1]

        xg /= nPts
        yg /= nPts
        return (xg, yg)

    def select_points_of_layer(self, inputPointCloud, xDomain, yDomain):

        treatedPointCloud = self.remove_inf_value(inputPointCloud)
        shapes = treatedPointCloud.shape
        savedPoints = []
        for i in range(0,shapes[0]):
            if      treatedPointCloud[i,0] > xDomain[0] \
                and treatedPointCloud[i,0] < xDomain[1] \
                and treatedPointCloud[i,1] > yDomain[0] \
                and treatedPointCloud[i,1] < yDomain[1]:
                savedPoints.append(treatedPointCloud[i,:])
        savedPoints = np.array(savedPoints)
        return savedPoints

    def get_Mj(self,layerPointCloud, layerNumber, layerHeight):
        shapes = layerPointCloud.shape
        Mj = 0
        for i in range(0,shapes[0]):
            Mj += layerPointCloud[i,2]
        return (Mj/shapes[0])-(layerHeight*layerNumber)

    def get_Sj(self,layerPointCloud,Mj):
        shapes = layerPointCloud.shape
        Sj = 0
        for i in range(0,shapes[0]):
            Sj += (layerPointCloud[i,2]-Mj)**2
        Sj /= shapes[0]
        return sqrt(Sj)

    def remove_inf_value(self,inputPointCloud):
        inputPointCloud = self.flatten_point_cloud(inputPointCloud)
        shapes = inputPointCloud.shape
        outputPointCloud = []
        for i in range(0,shapes[0]):
            if inputPointCloud[i,2]<10000 and inputPointCloud[i,2]>-10000:
                outputPointCloud.append(inputPointCloud[i,:])
        return np.array(outputPointCloud)

    def remove_value_above(self,inputPointCloud,threshold):
        inputPointCloud = self.flatten_point_cloud(inputPointCloud)
        shapes = inputPointCloud.shape
        outputPointCloud = []
        for i in range(0,shapes[0]):
            if inputPointCloud[i,2]<threshold:
                outputPointCloud.append(inputPointCloud[i,:])
        return np.array(outputPointCloud)

    def plot_point_cloud(self,inputPointCloud):
        data = self.remove_inf_value(inputPointCloud)
        data = self.decimate_point_cloud(data, 0.95)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d',aspect='auto')
        im = ax.scatter(x, y, z, c=z, cmap = mpl.colormaps['jet'])
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')
        fig.colorbar(im)
        ax.set_xlim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        ax.set_ylim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        ax.set_zlim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        plt.show()
        return 0
    
    def barycentre_calculation(self,layer_point_cloud):
        self.xg = 0
        self.yg = 0
        layerShape = layer_point_cloud.shape
        for i in range(0,layerShape[0]):
            self.xg += layer_point_cloud[i][0]
            self.yg += layer_point_cloud[i][1]
        self.xg /= layerShape[0]
        self.yg /= layerShape[0]
        return 0

    def transformation_creation(self,plate_point_cloud):
        self.rotation_matrix = np.zeros((3,3))
        self.translation_matrix = np.array([0, 0, 0])

        plane1 = pyrsc.Plane()
        best_eq, best_inliers = plane1.fit(plate_point_cloud, 0.01)
        if best_eq[3] > 0:
            best_eq[3] = -best_eq[3]

        if best_eq[2]<0:
            self.rotation_matrix[2,0] = -best_eq[0]
            self.rotation_matrix[2,1] = -best_eq[1]
            self.rotation_matrix[2,2] = -best_eq[2]
            self.translation_matrix[2] = best_eq[3]
            self.zVector = best_eq[0:3]
            self.zVector[0] = -self.zVector[0]
            self.zVector[1] = -self.zVector[1]
            self.zVector[2] = -self.zVector[2] 
        else:
            self.rotation_matrix[2,0] = best_eq[0]
            self.rotation_matrix[2,1] = best_eq[1]
            self.rotation_matrix[2,2] = best_eq[2]
            self.translation_matrix[2] = best_eq[3]
            self.zVector = best_eq[0:3]

        self.xVector = plate_point_cloud[10,:]-plate_point_cloud[0,:]
        self.xVector /= np.sqrt(self.xVector[0]**2+self.xVector[1]**2+self.xVector[2]**2)

        self.yVector = [0, 0, 0]

        self.yVector[0] = self.zVector[1]*self.xVector[2] - self.zVector[2]*self.xVector[1]
        self.yVector[1] = self.zVector[2]*self.xVector[0] - self.zVector[0]*self.xVector[2]
        self.yVector[2] = self.zVector[0]*self.xVector[1] - self.zVector[1]*self.xVector[0]

        self.rotation_matrix[0,0] = self.xVector[0]
        self.rotation_matrix[0,1] = self.xVector[1]
        self.rotation_matrix[0,2] = self.xVector[2]

        self.rotation_matrix[1,0] = self.yVector[0]
        self.rotation_matrix[1,1] = self.yVector[1]
        self.rotation_matrix[1,2] = self.yVector[2]

        return 0

    def tranform_point_cloud(self,inputPointCloud):
        outputPointCloud = np.zeros(inputPointCloud.shape)
        inputShape = inputPointCloud.shape
        for i in range(0,inputShape[0]):
            outputPointCloud[i,:] = np.dot(self.rotation_matrix, inputPointCloud[i,:]) - self.translation_matrix
        return outputPointCloud

    def decimate_point_cloud(self,inputPointCloud,decimationRate):
        shapes = inputPointCloud.shape
        interPointCloud = self.flatten_point_cloud(inputPointCloud)
        indexList = [i for i in range(0,interPointCloud.shape[0])]
        deletedElementsIndex = random.sample(indexList,int(len(indexList)*decimationRate))
        outputPointCloud = np.delete(interPointCloud, deletedElementsIndex,axis=0)
        return outputPointCloud

    def ask_for_plate_limits(self,inputPointCloud):
        data = self.remove_inf_value(inputPointCloud)
        data = self.decimate_point_cloud(data, 0.95)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111,aspect='equal')
        im = ax.scatter(x, y, c=z,s=1)
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')
        fig.colorbar(im)
        ax.set_xlim(min(x)-10, max(x)+10)
        ax.set_ylim(min(y)-10, max(y)+10)
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
        global coords
        self.xinf = coords[0][0]
        self.xsup = coords[1][0]
        self.get_logger().info("xinf : "+str(self.xinf))
        self.get_logger().info("xsup : "+str(self.xsup))
        coords = []
        return 0

    def ask_for_points_of_plate(self,inputPointCloud):
        data = self.remove_inf_value(inputPointCloud)
        data = self.decimate_point_cloud(data, 0.95)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111,aspect='equal')
        im = ax.scatter(x, y, c=z,s=1)
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')
        fig.colorbar(im)
        ax.set_xlim(min(x)-10, max(x)+10)
        ax.set_ylim(min(y)-10, max(y)+10)
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
        global coords
        self.x1 = coords[0][0]
        self.x2 = coords[1][0]
        self.y1 = coords[0][1]
        self.y2 = coords[1][1]
        self.get_logger().info("x1 : "+str(self.x1))
        self.get_logger().info("x2 : "+str(self.x2))
        self.get_logger().info("y1 : "+str(self.y1))
        self.get_logger().info("y2 : "+str(self.y2))
        coords = []
        return 0

    def extract_reference_base_change(self):
        return 0

if __name__ == '__main__':
    rclpy.init()
    point_cloud_processing_node = PointCloudProcessingNode()
    reference_layer = point_cloud_processing_node.load_scan("/home/gulltor/Ramsai_Robotics/history/impression_base_avec_retraction_20_pourcents/scan/layer_scan_0.npy")
    layer_3 = point_cloud_processing_node.load_scan("/home/gulltor/Ramsai_Robotics/history/impression_base_avec_retraction_20_pourcents/scan/layer_scan_3.npy")
    layer_3 = point_cloud_processing_node.remove_value_above(layer_3, -70)

    #point_cloud_processing_node.ask_for_plate_limits(layer_3)
    point_cloud_processing_node.xinf = -40.311554355006095
    point_cloud_processing_node.xsup = 4

    filtered_scan               = point_cloud_processing_node.remove_non_used_plate_points(layer_3)

    reference_layer_filtered    = point_cloud_processing_node.remove_non_used_plate_points(reference_layer)

    point_cloud_processing_node.transformation_creation(reference_layer_filtered)

    layer_3_transformed         = point_cloud_processing_node.tranform_point_cloud(filtered_scan)

    point_cloud_processing_node.plot_point_cloud(layer_3_transformed)

    point_cloud_processing_node.ask_for_points_of_plate(filtered_scan)
    [xg, yg] = point_cloud_processing_node.find_layer_barycentre(filtered_scan, 0.04)
    print("( ",xg," , ",yg," )")


    point_cloud_list = [None]*122
    Mj = [None]*122
    Sj = [None]*122

    for i in range(0,122):
        point_cloud_list[i] = point_cloud_processing_node.load_scan('/home/gulltor/Ramsai_Robotics/history/impression_base_avec_retraction_20_pourcents/scan/layer_scan_'+str(i)+'.npy')
        point_cloud_list[i] = point_cloud_processing_node.remove_non_used_plate_points(point_cloud_list[i])
        point_cloud_list[i] = point_cloud_processing_node.select_points_of_layer(point_cloud_list[i], [xg-7, xg+7], [yg-7, yg+7])
        Mj[i] = point_cloud_processing_node.get_Mj(point_cloud_list[i], i, 0.41)
        Sj[i] = point_cloud_processing_node.get_Sj(point_cloud_list[i],Mj[i])
        point_cloud_processing_node.get_logger().info('layer : '+str(i))

    plt.figure()
    plt.plot(Mj)
    plt.show()

    plt.figure()
    plt.plot(Sj)
    plt.show()


    rclpy.shutdown()
