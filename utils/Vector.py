from math import sqrt

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)
    
    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)
    
    def __str__(self):
        return f"<{self.x}, {self.y}>"
    
    def length(self) -> float:
        return (self.x ** 2 + self.y ** 2)**(1/2)
    
    def normalize(self):
        return self/self.length()

    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def projection(self, other):
        if self.length() == 0:
            return Vector(0, 0)
        return (self.dot(other)/self.length()) * self
    
    def cross_product(self, other):
        return self.x * other.y - self.y * other.x

    def magnitude(self):
        return (self.x ** 2 + self.y ** 2)**(1/2)