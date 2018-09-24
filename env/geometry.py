from __future__ import print_function
import math
import numpy as np


def distance(p0, p1):
    """
    Compute the euclidean distance between two points.
    :param p0: tuple, the initial point
    :param p1: tuple, the final point
    :return: the distance between points
    """
    if len(p0) != len(p1):
        return None
    return math.sqrt(sum(
        [(p1[i] - p0[i]) ** 2 for i in range(len(p0))]
    ))


def orientation(p, q, r):
    """
    Determine the orientation of three points
    :param p: tuple<float>; a point
    :param q:
    :param r:
    :return: 1 if clockwise, -1 if counterclockwise, 0 if collinear
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val > 0:
        return 1  # clockwise
    elif val < 0:
        return -1  # counterclockwise
    else:
        return 0  # collinear


def on_segment(p, q, r):
    """
    Checks if point q lies on line pr.
    :param p:
    :param q:
    :param r:
    :return: whether q lies on pr
    """
    return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and \
        min(p[1], r[1]) <= q[1] <= max(p[1], r[1])


def point_from_polar(c, r, theta):
    point = (c[0] + r * math.cos(theta), c[1] + r * math.sin(theta))
    return point


class Line2D(object):

    def __init__(self, p, q):
        """
        Constructor
        :param p: tuple<float> the start point
        :param q: tuple<float> the end point
        """
        self.p = p
        self.q = q

    def intersects(self, line):
        """
        Determine if this line intersects the given line.
        :param line: a line to check intersection with
        :return: True if the lines intersect
        """
        o1 = orientation(self.p, self.q, line.p)
        o2 = orientation(self.p, self.q, line.q)
        o3 = orientation(line.p, line.q, self.p)
        o4 = orientation(line.p, line.q, self.q)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Collinear cases
        if o1 == 0 and on_segment(self.p, line.p, self.q):
            return True
        if o2 == 0 and on_segment(self.p, line.q, self.q):
            return True
        if o3 == 0 and on_segment(line.p, self.p, line.q):
            return True
        if o4 == 0 and on_segment(line.p, self.q, line.q):
            return True

        return False

    def intersects_special(self, line):
        """
        Determine if this line intersects the given line, assuming this line is
        horizontal and extends to positive infinity.
        :param line: a line to check intersection with
        :return: True if the lines intersect
        """
        m, c = line.grad_intercept()
        # Intersection if the y value of the horizontal line is between the
        # y range of line, and line is to the right of the horizontal's origin
        return min(line.p[1], line.q[1]) <= self.p[1] \
            <= max(line.p[1], line.q[1]) and (self.p[1] - c) / m >= self.p[0]

    def grad_intercept(self):
        """
        Get the parameters of this line in gradient-intercept form.
        :return:
        """
        diff = float(self.q[0] - self.p[0])
        # Handle 0 division
        if abs(diff) < 1e-9:
            if diff > 0:
                diff = 1e-9
            else:
                diff = -1e-9
        m = (self.q[1] - self.p[1]) / diff
        c = self.p[1] - m * self.p[0]
        return m, c

    def general(self):
        """
        Get the parameters of this line in general form.
        :return:
        """
        m, c = self.grad_intercept()
        return -m, 1, -c

    def p_distance(self, p):
        """
        Compute the shortest distance from this line to the point p.
        :param p:
        :return:
        """
        g = self.general()
        return abs(g[0] * p[0] + g[1] * p[1] + g[2]) \
               / math.sqrt(g[0] ** 2 + g[1] ** 2)

    def p_point(self, p):
        """
        Find the point on this line closest to the point p.
        :param p:
        :return:
        """
        m, c = self.grad_intercept()
        x = (p[0] + m * (p[1] - c)) / (m ** 2 + 1)
        return x, m * x + c

    def get_vertex(self, edge2):
        if self.p == edge2.p or self.p == edge2.q:
            return self.p
        elif self.q == edge2.p or self.q == edge2.q:
            return self.q

    def find_normal_at_point(self, point):
        m0, c0 = self.grad_intercept()
        m1 = -1 / (m0 + 1e-9)

        c1 = -m1 * point[0] + point[1]

        return m1, c1


class Shape2D(object):
    """
    A general 2D shape, e.g. a polygon or a circle.
    """

    def __init__(self):
        pass

    def intersects(self, shape):
        """
        Checks if this shape overlaps another.
        :param shape: the other shape
        :return: True if the shapes overlap, i.e. share coordinates
        """
        pass


class Circle(Shape2D):

    def __init__(self, centre, radius):
        super(Circle, self).__init__()
        self.centre = centre
        self.radius = radius
        self.type = "circle"

    def intersects(self, shape):

        # Check circle case first
        if shape is None:
            return False
        if shape.type == "circle" and distance(self.centre, shape.centre) < \
                self.radius + shape.radius:
            # Centres within the sum of radii implies edges overlap
            return True

        if shape.type == "rectangle" or shape.type == "triangle":
            # Check if the centre of the circle is inside or outside shape
            # Create line from puck centre to the right
            # The end of the line must extend somewhere beyond the workspace to
            # guarantee all intersections are found, so we choose 1000

            # angle = np.arctan((shape.centre[1] - self.centre[1])/(shape.centre[0] - self.centre[0]))
            # centre = (self.centre[0] + self.radius*np.cos(angle),
            #         self.centre[1] + self.radius*np.sin(angle))

            r_line = Line2D(self.centre, (1000.0, self.centre[1]))
            num_intersections = 0
            for e in shape.edges:
                if r_line.intersects_special(e):
                    num_intersections += 1
            if num_intersections % 2 != 0:
                # Odd number implies centre is inside shape
                return True
            # Now check if edges overlap
            corner_cases = []
            for e in shape.edges:
                # Find the point on line collinear to e that is closest to this
                # circle's centre
                nearest_point = e.p_point(self.centre)
                # Find the distance from that nearest point to this circle's
                # centre
                min_distance = e.p_distance(self.centre)
                if min_distance < self.radius:
                    if on_segment(e.p, nearest_point, e.q):
                        # This circle intersects the line equation of edge e
                        # and that point is on the actual edge
                        return True
                    # Corner case: point does not lie on a line normal to any
                    # edge, but could intersect at the corners, so keep track
                    corner_cases.append(e)
            if len(corner_cases) > 1:
                # 2 edges share a corner case - check if vertex is inside
                v = corner_cases[0].get_vertex(corner_cases[1])
                if v is not None and distance(v, self.centre) < self.radius:
                    return True
        return False

    def __str__(self):
        return "{0}: {1}, {2}".format(self.type, self.centre, self.radius)


class Polygon(Shape2D):

    def __init__(self, vertices, edges=None):
        """
        Constructor
        :param vertices: list<tuple<float>> the vertices of the polygon
        :param edges: list<Line2D> the edges of the polygon
        """
        super(Polygon, self).__init__()
        self.type = None
        self.vertices = vertices
        # xTotal= 0
        # yTotal = 0
        # for v in self.vertices:
        #     xTotal += v[0]
        #     yTotal += v[1]
        # self.centre = (xTotal/len(self.vertices), yTotal/len(self.vertices))

        if not edges:
            # Assume anticlockwise order
            self.edges = []
            for i in range(len(vertices) - 1):
                self.edges.append(Line2D(vertices[i], vertices[i + 1]))
            self.edges.append(Line2D(vertices[-1], vertices[0]))
        else:
            self.edges = edges

    def intersects(self, shape):
        pass

    def get_vertex(self, edge1, edge2):
        return edge1.get_vertex(edge2)

    def get_edge(self, vertex1, vertex2):
        pass

    def __str__(self):
        return "{0}: {1}".format(self.type, self.vertices)


class Rectangle(Polygon):

    def __init__(self, vertices, edges=None):
        super(Rectangle, self).__init__(vertices, edges)
        self.type = "rectangle"


class Triangle(Polygon):

    def __init__(self, vertices, edges=None):
        super(Triangle, self).__init__(vertices, edges)
        self.type = "triangle"


def main():
    circle1 = Circle((0.0, 0.0), 0.5)
    circle2 = Circle((1.0, 0.0), 0.75)
    print(distance(circle1.centre, circle2.centre))
    print(circle1.intersects(circle2))
    line1 = Line2D((4.0, 4.0), (7.0, 5.5))
    print(line1.p_distance((0.0, 0.0)))


if __name__ == '__main__':
    main()
