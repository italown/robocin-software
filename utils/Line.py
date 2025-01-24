from utils.Point import Point
from math import sqrt
from utils.Vector import Vector
from utils.Direction import Direction

class Line:
    def __init__(self, p1:Point, p2:Point):
        self.p1: Point = p1
        self.p2: Point = p2
    
    def __str__(self):
        return f"({self.p1}, {self.p2})"
    
    def __repr__(self):
        return f"Line({self.p1}, {self.p2})"
    
    def __eq__(self, other):
        return self.p1 == other.p1 and self.p2 == other.p2
    
    def point_on_line(self, t):
        return self.p1 + (self.p2 - self.p1) * t
    
    def length(self):
        return self.p1.dist_to(self.p2)
    
    def belongs(self, point):
        return self.p1.dist_to(point) + self.p2.dist_to(point) == self.length()
    

    def intersect_robot(self, center: Point, radius : float) -> tuple:
        '''Função que calcula a interseção da Linha com o círculo (robô) e retorna 
        '''
                           
        ab = Vector(self.p2.x - self.p1.x, self.p2.y - self.p1.y)
        ac = Vector(center.x - self.p1.x, center.y - self.p1.y)  
        
        t = (ac.dot(ab)) / (ab.magnitude() ** 2)
        
        if t < 0:
            nearest_point = self.p1
        elif t > 1:
            nearest_point = self.p2
        else:
            nearest_point = self.p1 + ab * t
        
        dist = nearest_point.dist_to(center)

        cross_product = ab.cross_product(ac)

        if cross_product > 0:
            position = Direction.LEFT.value
        elif cross_product < 0:
            position = Direction.RIGHT.value
        else:
            position = Direction.CONTINUE.value


        return dist <= radius, position
    