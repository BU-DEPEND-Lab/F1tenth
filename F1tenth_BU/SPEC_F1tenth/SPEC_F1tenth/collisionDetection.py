import numpy as np


"""
Detect whether two rectangles are overlapping
"""
def do_polygons_intersect(a, b):
    """
 * Helper function to determine whether there is an intersection between the two polygons described
 * by the lists of vertices. Uses the Separating Axis Theorem
 *
 * @param a an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
 * @param b an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
 * @return true if there is any intersection between the 2 polygons, false otherwise
    """

    polygons = [a, b];
    minA, maxA, projected, i, i1, j, minB, maxB = None, None, None, None, None, None, None, None
    for i in range(len(polygons)):

        # for each polygon, look at each edge of the polygon, and determine if it separates
        # the two shapes
        polygon = polygons[i];
        for i1 in range(len(polygon)):

            # grab 2 vertices to create an edge
            i2 = (i1 + 1) % len(polygon);
            p1 = polygon[i1];
            p2 = polygon[i2];

            # find the line perpendicular to this edge
            normal = { 'x': p2[1] - p1[1], 'y': p1[0] - p2[0] };

            minA, maxA = None, None
            # for each vertex in the first shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            for j in range(len(a)):
                projected = normal['x'] * a[j][0] + normal['y'] * a[j][1];
                if (minA is None) or (projected < minA):
                    minA = projected

                if (maxA is None) or (projected > maxA):
                    maxA = projected

            # for each vertex in the second shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            minB, maxB = None, None
            for j in range(len(b)):
                projected = normal['x'] * b[j][0] + normal['y'] * b[j][1]
                if (minB is None) or (projected < minB):
                    minB = projected

                if (maxB is None) or (projected > maxB):
                    maxB = projected

            # if there is no overlap between the projects, the edge we are looking at separates the two
            # polygons, and we know there is no overlap
            if (maxA < minB) or (maxB < minA):
                print("polygons don't intersect!")
                return False;

    #print minA, maxA, minB, maxB
    #print normal
    #print polygon
    #print b
    print a
    return True

class Rectangle(object):
    """
    define a rectangle region that parallel to axises
    with left bottom point and right top point
    """
    def __init__(self, left_bottom, right_top):
        self.left = left_bottom[0]
        self.right = right_top[0]
        self.top = right_top[1]
        self.bottom = left_bottom[1]
        self.center_x = 0.5 * (self.left + self.right)
        self.center_y = 0.5 * (self.bottom + self.top)
        self.length = self.right - self.left
        self.width = self.top - self.bottom

    def closest_distance(self, x, y):
        dx = max(abs(x - self.center_x) - self.length/2, 0)
        dy = max(abs(y - self.center_y) - self.width/2, 0)
        return np.sqrt(dx**2 + dy**2)

    @property
    def polygon(self):
        return np.array([[self.left, self.bottom], [self.left, self.top],
                         [self.right, self.top], [self.right, self.bottom]])
class car_shape(object):
    """
    define a rotated rectangle to represent the car
    """
    def __init__(self, length=1., width=0.5):
        self.length = length
        self.width = width

    def polygon(self, x, y, theta):
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        left_bottom = (0.5 * np.matmul(rotation_matrix,
                       np.array([[-self.length], [-self.width]])) +
                       np.array([[x], [y]])).T
        right_bottom = (0.5 * np.matmul(rotation_matrix,
                       np.array([[self.length], [-self.width]])) +
                       np.array([[x], [y]])).T
        left_top = (0.5 * np.matmul(rotation_matrix,
                       np.array([[-self.length], [self.width]])) +
                       np.array([[x], [y]])).T
        right_top = (0.5 * np.matmul(rotation_matrix,
                       np.array([[self.length], [self.width]])) +
                       np.array([[x], [y]])).T
        #print left_bottom.diagonal(), right_bottom.diagonal(), left_top.diagonal(), right_top.diagonal()
        return np.array([left_bottom[0], left_top[0], right_top[0], right_bottom[0]])
