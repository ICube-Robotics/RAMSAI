import matplotlib.pyplot as plt
import numpy as np




class StlRepresentation():
    def __init__(self, filename):
        self.filename = filename
        self.__import_stl_file__()


    def __import_stl_file__(self):
        file = open(self.filename,'r')
        lines = file.readlines()
        file.close()
        print(len(lines))
        print(lines[1:10])
        return 0

    def slice_stl_representation(self, plane_parameters):
        return 0

    def get_layer_internal_contour(self, stl_slice):
        return 0

    def get_layer_external_contour(self, stl_slice):
        return 0
    
if __name__ == '__main__':
    print("demarrage")
    STL_representation = StlRepresentation("/home/gulltor/Documents/stl_files/cylindre_sacrificiel.stl")
