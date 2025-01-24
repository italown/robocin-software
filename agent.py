from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.Line import Line
from utils.Vector import Vector
from utils.Direction import Direction

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

        self.robot_collided = None
        self.direction_swerve = None
        self.last_robot_collided = None
        self.last_swerve_target = None

        self.size_robot = 0.18

        self.last_target = None
        self.t = 0
        self.dist_to_target = None
        

    def select_target(self) -> Point:
        lower_dist = float('inf')
        best_target = None

        for i in range(len(self.targets)):
            target = self.targets[i]
            dist = self.pos.dist_to(target)
            
            if dist < lower_dist:
                lower_dist = dist
                best_target = self.targets[i]

                for j in range(len(self.teammates)):
                    teammate = self.teammates[j]
                    
                    point_teammate = Point(teammate.x, teammate.y)
                    dist_teammate = point_teammate.dist_to(target)

                    if dist_teammate < lower_dist:
                        best_target = None
                        break

        return best_target


    def evaluate_collision(self, target) -> bool:
        '''
        Verifica se houve colisão com o robô ou com os colegas
        '''
        robot_radius = self.size_robot + 0.1       # Tamanho do robô + margem
        line = Line(self.pos, target)

        closest_distance = float('inf')
        closest_robot = None
        closest_swerve = None

        #Verificação de colisão com oponentes
        for _, opponent in self.opponents.items():
            point_opponent = Point(opponent.x, opponent.y)

            colide, direction = line.intersect_robot(point_opponent, robot_radius)

            if colide:
                distance = point_opponent.dist_to(self.pos)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_robot = opponent
                    closest_swerve = direction

        #Verificação de colisão com os colegas
        for _, teammate in self.teammates.items():
            if teammate.id == self.id:  
                continue

            point_teammate = Point(teammate.x, teammate.y)

            colide, direction = line.intersect_robot(point_teammate, robot_radius)

            if colide:
                distance = point_teammate.dist_to(self.pos)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_robot = teammate
                    closest_swerve = direction

        if closest_robot is not None:

            if target.dist_to(closest_robot) < self.size_robot:     # Verifica se o alvo está dentro do obstáculo
                return False
            
            self.robot_collided = closest_robot
            self.direction_swerve = closest_swerve
            return True

        return False

    def swerve(self, target):   
        
        obstacle_vector = Vector(self.robot_collided.x - self.pos.x, self.robot_collided.y - self.pos.y)
        
        perpendicular_vector = Vector(-obstacle_vector.y, obstacle_vector.x).normalize()

        if self.direction_swerve == Direction.LEFT.value:
            perpendicular_vector = perpendicular_vector * -1

        intermediate_point = Point(
            self.robot_collided.x + perpendicular_vector.x,
            self.robot_collided.y + perpendicular_vector.y
        )

        self.t = 0.65
        smooth_target = self.calculate_bezier(self.pos, intermediate_point, target, self.t)

        self.last_swerve_target = smooth_target
        return smooth_target
    
    def calculate_bezier(self, p0, p1, p2, t):
        """
        Calcula um ponto na curva Bézier quadrática.
        """
        x = (1 - t) ** 2 * p0.x + 2 * (1 - t) * t * p1.x + t ** 2 * p2.x
        y = (1 - t) ** 2 * p0.y + 2 * (1 - t) * t * p1.y + t ** 2 * p2.y
        return Point(x, y)    

    def decision(self):
        if len(self.targets) == 0:
            return

        best_target = self.select_target()

        if best_target is None:
            return

        collide = False
        if self.evaluate_collision(best_target):
            collide = True
            best_target = self.swerve(best_target)

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, best_target, collide)
        
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    def post_decision(self):
        
        self.last_robot_collided = self.robot_collided

        #Reseta variaveis
        self.robot_collided = None
        self.direction_swerve = None
