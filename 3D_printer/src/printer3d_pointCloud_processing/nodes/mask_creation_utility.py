import matplotlib.pyplot as plt
import numpy as np
from stl import mesh


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
        for triangle in self.vertices:
            # Parcourir les 3 arêtes du triangle
            for i in range(3):
                p1 = triangle[i]
                p2 = triangle[(i + 1) % 3]
                # Calculer les valeurs signées de p1 et p2 par rapport au plan
                val1 = a * p1[0] + b * p1[1] + c * p1[2] + d
                val2 = a * p2[0] + b * p2[1] + c * p2[2] + d
                print("val1 * val2 : ",val1 * val2)
                # Si les signes sont opposés, il y a intersection
                if val1 * val2 < 0:
                    print("val1 * val2")
                    intersection = intersection_segment_plan(p1, p2, a, b, c, d)
                    if intersection is not None:
                        intersections.add(tuple(intersection))
        return intersections

    def slice_stl_representation(self, plane_parameters):
        return 0

    def get_layer_internal_contour(self, stl_slice):
        return 0

    def get_layer_external_contour(self, stl_slice):
        return 0
    
if __name__ == '__main__':
    print("demarrage")
    STL_representation = StlRepresentation("/home/gulltor/Téléchargements/example_cylinder.stl")
    print(STL_representation.__identify_sliced_vertices__(0,0,1,-25))