from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.Line import Line
from utils.Vector import Vector

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

        self.robot_collided = None
        self.direction_swerve = None
        self.last_robot_collided = None
        self.last_swerve_target = None
        

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
        robot_radius = 0.30
        line = Line(self.pos, target)

        closest_distance = float('inf')
        closest_robot = None
        closest_swerve = None

        for _, opponent in self.opponents.items():
            point_opponent = Point(opponent.x, opponent.y)

            colide, direction = line.intersect_robot(point_opponent, robot_radius)

            if colide:
                distance = point_opponent.dist_to(self.pos)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_robot = opponent
                    closest_swerve = direction


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
            self.robot_collided = closest_robot
            self.direction_swerve = closest_swerve
            return True

        return False

    def swerve(self, target):   

        # Não ficar recalculando a linha de colisão para cada iteração
        if self.last_robot_collided == self.robot_collided:
            return self.last_swerve_target
        
        obstacle_vector = Vector(self.robot_collided.x - self.pos.x, self.robot_collided.y - self.pos.y)
        
        perpendicular_vector = Vector(-obstacle_vector.y, obstacle_vector.x).normalize()

        if self.direction_swerve == 'left':
            perpendicular_vector = perpendicular_vector * -1

        intermediate_point = Point(
            self.pos.x + perpendicular_vector.x * 0.7,
            self.pos.y + perpendicular_vector.y * 0.7
        )

        smooth_target = Point(
            (intermediate_point.x + target.x) / 2,
            (intermediate_point.y + target.y) / 2
        )

        self.last_swerve_target = smooth_target
        return smooth_target

    def buffer_velocity(self, target, target_velocity, target_angle_velocity):
        '''Se a distância para o alvo for maior que 0.5, aumenta a velocidade em 50%'''
        distance = self.pos.dist_to(target)

        if distance < 0.5:
            return target_velocity , target_angle_velocity
    

        target_velocity = target_velocity * 1.2
        return target_velocity, target_angle_velocity
        

    def decision(self):
        if len(self.targets) == 0:
            return

        best_target = self.select_target()

        if best_target is None:
            return

        if self.evaluate_collision(best_target):
            best_target = self.swerve(best_target)

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, best_target)
        
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)
        return

    def post_decision(self):
        
        self.last_robot_collided = self.robot_collided

        #Reseta variaveis
        self.robot_collided = None
        self.direction_swerve = None
