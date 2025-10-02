import matplotlib.pyplot as plt
import numpy as np
from stl import mesh
from math import sqrt

def intersection_segment_plan(p1, p2, a, b, c, d):
    """Calcule l'intersection entre un segment [p1, p2] et le plan ax + by + cz + d = 0.
    Retourne le point d'intersection s'il existe, sinon None."""
    # Vecteur directeur de l'arête
    u = p2 - p1
    # Produit scalaire avec la normale du plan
    denom = np.dot(u, np.array([a, b, c]))
    if abs(denom) < 1e-10:  # Arête parallèle au plan
        return None
    # Paramètre t de l'intersection
    t = -(a * p1[0] + b * p1[1] + c * p1[2] + d) / denom
    if 0 <= t <= 1:  # Intersection dans le segment
        return p1 + t * u
    return None

def find_neighbor(segment, triangle_intersions, tolerance):
    neighbor = []
    neighbor_indices = []
    for j in range(0,len(triangle_intersions)):
            test_P1_segment_P1_intersection = sqrt((segment[0][0]-triangle_intersions[j][0][0])**2+(segment[0][1]-triangle_intersions[j][0][1])**2) < tolerance
            test_P1_segment_P2_intersection = sqrt((segment[0][0]-triangle_intersions[j][1][0])**2+(segment[0][1]-triangle_intersions[j][1][1])**2) < tolerance
            
            test_P2_segment_P1_intersection = sqrt((segment[1][0]-triangle_intersions[j][0][0])**2+(segment[1][1]-triangle_intersions[j][0][1])**2) < tolerance
            test_P2_segment_P2_intersection = sqrt((segment[1][0]-triangle_intersions[j][1][0])**2+(segment[1][1]-triangle_intersions[j][1][1])**2) < tolerance
            
            if test_P1_segment_P1_intersection != test_P2_segment_P2_intersection:
                neighbor.append(triangle_intersions[j])
                neighbor_indices.append(j)
            if test_P1_segment_P2_intersection != test_P2_segment_P1_intersection:
                neighbor.append(triangle_intersions[j])
                neighbor_indices.append(j)
    return (neighbor, neighbor_indices)

class StlRepresentation():
    def __init__(self, filename):
        self.filename = filename
        self.__import_stl_file__()

    def __import_stl_file__(self):
        stl_mesh = mesh.Mesh.from_file(self.filename)
        self.vertices = stl_mesh.vectors
        return 0
    
    def __identify_sliced_vertices__(self,a,b,c,d):
        intersections = set()
        triangle_intersions = []
        for triangle in self.vertices:
            triangle_intersion = []
            # Parcourir les 3 arêtes du triangle
            for i in range(3):
                p1 = triangle[i]
                p2 = triangle[(i + 1) % 3]
                # Calculer les valeurs signées de p1 et p2 par rapport au plan
                val1 = a * p1[0] + b * p1[1] + c * p1[2] + d
                val2 = a * p2[0] + b * p2[1] + c * p2[2] + d
                # Si les signes sont opposés, il y a intersection
                if val1 * val2 < 0:
                    intersection = intersection_segment_plan(p1, p2, a, b, c, d)
                    if intersection is not None:
                        intersections.add(tuple(intersection))
                        triangle_intersion.append(tuple(intersection))
            if len(triangle_intersion) >0:
                triangle_intersions.append(triangle_intersion)
        return (intersections,triangle_intersions)

    def slice_stl_representation(self, a,b,c,d):
        (intersections,triangle_intersions) = self.__identify_sliced_vertices__(a,b,c,d)
        intersections = list(intersections)
        intersections_x = [intersections[i][0] for i in range(0, len(intersections))]
        intersections_y = [intersections[i][1] for i in range(0, len(intersections))]
        
        contours = {}
        
        contours[1] = [triangle_intersions[0]]
        (neighbor, neighbor_indices) = find_neighbor(triangle_intersions[0], triangle_intersions, 0.001)
        for i in range(0,len(neighbor)):
            contours[1].append(neighbor[i])
        test = False
        while test == False:
            indices_to_be_tested = neighbor_indices.copy()
            neighbor_indices = [None]*len(indices_to_be_tested)
            neighbor = [None]*len(indices_to_be_tested)
            for i in range(0,len(indices_to_be_tested)):
                (neighbor[i], neighbor_indices[i]) = find_neighbor(triangle_intersions[0], triangle_intersions, 0.001)
        
        
        
        
        plt.figure(figsize=(16, 12))
        plt.plot(intersections_x,intersections_y,'r*')
        for i in range(0,len(contours[1])):
            plt.plot([contours[1][i][0][0], contours[1][i][1][0]],
                     [contours[1][i][0][1], contours[1][i][1][1]],
                     'b-')
        plt.plot([triangle_intersions[0][0][0], triangle_intersions[0][1][0]],
                 [triangle_intersions[0][0][1], triangle_intersions[0][1][1]],
                 'g-')
        plt.axis("equal")
        plt.show()
        
        
        return 0
    
    def plot_sliced_stl(self,a,b,c,d):
        return 0
            

    def get_layer_internal_contour(self, stl_slice):
        return 0

    def get_layer_external_contour(self, stl_slice):
        return 0
    
if __name__ == '__main__':
    print("demarrage")
    STL_representation = StlRepresentation("/home/mosserloic/Documents/example_cylinder.stl")
    STL_representation.slice_stl_representation(0,0,1,-25)
