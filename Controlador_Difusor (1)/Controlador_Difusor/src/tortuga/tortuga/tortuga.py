
import rclpy
from rclpy.node import Node
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
# Código para Ejercicio 1 seguidor
#para hacer ejercicio 1 descomentar esta primera sección y comentar la 2nda
"""class FuzzyController(Node):
   def __init__(self):
       super().__init__('tortuga')
       self.subscription1 = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
       self.subscription2 = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)
       self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

       self.turtle1_pose = None
       self.turtle2_pose = None

       self.control_surface = self.calculate_control_surface()

   def turtle1_pose_callback(self, msg):
       self.turtle1_pose = msg
       print("pose 1")
       self.control_turtle()

   def turtle2_pose_callback(self, msg):
       self.turtle2_pose = msg
       print("pose 2")
       self.control_turtle()

   def calculate_control_surface(self):
       print("calculate control once")
       distance = ctrl.Antecedent(np.arange(0, 11, 0.1), 'distance')
       angle_diff = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.1), 'angle_diff')

       linear_velocity = ctrl.Consequent(np.arange(0, 2, 0.1), 'linear_velocity')
       angular_velocity = ctrl.Consequent(np.arange(-2, 2, 0.1), 'angular_velocity')

       # Definir las funciones de membresía
       distance['muycerca'] = fuzz.trimf(distance.universe, [0, 0, 2.5])
       distance['cerca'] = fuzz.trimf(distance.universe, [0, 2.5, 5])
       distance['medio'] = fuzz.trimf(distance.universe, [2.5, 5, 7.5])
       distance['lejos'] = fuzz.trimf(distance.universe, [5, 7.5, 10])
       distance['muylejos'] = fuzz.trimf(distance.universe, [7.5, 10, 10])

       angle_diff['muyizquierda'] = fuzz.trimf(angle_diff.universe, [-np.pi, -np.pi/2, 0])
       angle_diff['izquierda'] = fuzz.trimf(angle_diff.universe, [-np.pi/2, -np.pi/4, 0])
       angle_diff['centrado'] = fuzz.trimf(angle_diff.universe, [-np.pi/4, 0, np.pi/4])
       angle_diff['derecha'] = fuzz.trimf(angle_diff.universe, [0, np.pi/4, np.pi/2])
       angle_diff['muyderecha'] = fuzz.trimf(angle_diff.universe, [0, np.pi/2, np.pi])

       linear_velocity['muylento'] = fuzz.trimf(linear_velocity.universe, [0, 0.25, 0.5])
       linear_velocity['lento'] = fuzz.trimf(linear_velocity.universe, [0.25, 0.5, 0.75])
       linear_velocity['medio'] = fuzz.trimf(linear_velocity.universe, [0.5, 1, 1.5])
       linear_velocity['rapido'] = fuzz.trimf(linear_velocity.universe, [1, 1.25, 1.5])
       linear_velocity['muyrapido'] = fuzz.trimf(linear_velocity.universe, [1.5, 1.75, 2])

       angular_velocity['rapidoderecha'] = fuzz.trimf(angular_velocity.universe, [-2, -1, 0])
       angular_velocity['lentoderecha'] = fuzz.trimf(angular_velocity.universe, [-1, -0.5, 0])
       angular_velocity['centrado'] = fuzz.trimf(angular_velocity.universe, [-0.5, 0, 0.5])
       angular_velocity['lentoizquierda'] = fuzz.trimf(angular_velocity.universe, [0, 0.5, 1])
       angular_velocity['rapidoizquierda'] = fuzz.trimf(angular_velocity.universe, [0, 1, 2])

       # Definir las reglas
       rules = [
           ctrl.Rule(distance['muycerca'] & angle_diff['muyizquierda'], (linear_velocity['muylento'], angular_velocity['rapidoderecha'])),
           ctrl.Rule(distance['muycerca'] & angle_diff['izquierda'], (linear_velocity['muylento'], angular_velocity['lentoderecha'])),
           ctrl.Rule(distance['muycerca'] & angle_diff['centrado'], (linear_velocity['muylento'], angular_velocity['centrado'])),
           ctrl.Rule(distance['muycerca'] & angle_diff['derecha'], (linear_velocity['muylento'], angular_velocity['lentoizquierda'])),
           ctrl.Rule(distance['muycerca'] & angle_diff['muyderecha'], (linear_velocity['muylento'], angular_velocity['rapidoizquierda'])),

           ctrl.Rule(distance['cerca'] & angle_diff['muyizquierda'], (linear_velocity['lento'], angular_velocity['rapidoderecha'])),
           ctrl.Rule(distance['cerca'] & angle_diff['izquierda'], (linear_velocity['lento'], angular_velocity['lentoderecha'])),
           ctrl.Rule(distance['cerca'] & angle_diff['centrado'], (linear_velocity['lento'], angular_velocity['centrado'])),
           ctrl.Rule(distance['cerca'] & angle_diff['derecha'], (linear_velocity['lento'], angular_velocity['lentoizquierda'])),
           ctrl.Rule(distance['cerca'] & angle_diff['muyderecha'], (linear_velocity['lento'], angular_velocity['rapidoizquierda'])),

           ctrl.Rule(distance['medio'] & angle_diff['muyizquierda'], (linear_velocity['medio'], angular_velocity['rapidoderecha'])),
           ctrl.Rule(distance['medio'] & angle_diff['izquierda'], (linear_velocity['medio'], angular_velocity['lentoderecha'])),
           ctrl.Rule(distance['medio'] & angle_diff['centrado'], (linear_velocity['medio'], angular_velocity['centrado'])),
           ctrl.Rule(distance['medio'] & angle_diff['derecha'], (linear_velocity['medio'], angular_velocity['lentoizquierda'])),
           ctrl.Rule(distance['medio'] & angle_diff['muyderecha'], (linear_velocity['medio'], angular_velocity['rapidoizquierda'])),

           ctrl.Rule(distance['lejos'] & angle_diff['muyizquierda'], (linear_velocity['rapido'], angular_velocity['rapidoderecha'])),
           ctrl.Rule(distance['lejos'] & angle_diff['izquierda'], (linear_velocity['rapido'], angular_velocity['lentoderecha'])),
           ctrl.Rule(distance['lejos'] & angle_diff['centrado'], (linear_velocity['rapido'], angular_velocity['centrado'])),
           ctrl.Rule(distance['lejos'] & angle_diff['derecha'], (linear_velocity['rapido'], angular_velocity['lentoizquierda'])),
           ctrl.Rule(distance['lejos'] & angle_diff['muyderecha'], (linear_velocity['rapido'], angular_velocity['rapidoizquierda'])),

           ctrl.Rule(distance['muylejos'] & angle_diff['muyizquierda'], (linear_velocity['muyrapido'], angular_velocity['rapidoderecha'])),
           ctrl.Rule(distance['muylejos'] & angle_diff['izquierda'], (linear_velocity['muyrapido'], angular_velocity['lentoderecha'])),
           ctrl.Rule(distance['muylejos'] & angle_diff['centrado'], (linear_velocity['muyrapido'], angular_velocity['centrado'])),
           ctrl.Rule(distance['muylejos'] & angle_diff['derecha'], (linear_velocity['muyrapido'], angular_velocity['lentoizquierda'])),
           ctrl.Rule(distance['muylejos'] & angle_diff['muyderecha'], (linear_velocity['muyrapido'], angular_velocity['rapidoizquierda']))
       ]

       control_system = ctrl.ControlSystem(rules)
       fuzzy_controller = ctrl.ControlSystemSimulation(control_system)

       control_surface = np.zeros((len(distance.universe), len(angle_diff.universe), 2))

       for i, d in enumerate(distance.universe):
           for j, a in enumerate(angle_diff.universe):
               fuzzy_controller.input['distance'] = d
               fuzzy_controller.input['angle_diff'] = a
               try:
                   fuzzy_controller.compute()
                   control_surface[i, j, 0] = fuzzy_controller.output['linear_velocity']
                   control_surface[i, j, 1] = fuzzy_controller.output['angular_velocity']
               except ValueError:
                   # Asignar valores predeterminados si no se puede calcular la salida
                   control_surface[i, j, 0] = 0.0
                   control_surface[i, j, 1] = 0.0

       return control_surface

   def get_control_signals(self, distance, angle_diff):
       # Buscar los índices más cercanos en la superficie de control
       distance_idx = np.abs(np.arange(0, 11, 0.1) - distance).argmin()
       angle_diff_idx = np.abs(np.arange(-np.pi, np.pi, 0.1) - angle_diff).argmin()

       # Obtener las velocidades lineal y angular de la superficie de control
       linear_velocity = self.control_surface[distance_idx, angle_diff_idx, 0]
       angular_velocity = self.control_surface[distance_idx, angle_diff_idx, 1]

       return linear_velocity, angular_velocity

   def control_turtle(self):
       print("Control")
       if self.turtle1_pose is None or self.turtle2_pose is None:
           return

       distance = self.calculate_distance(self.turtle1_pose, self.turtle2_pose)
       angle_diff = self.calculate_angle_diff(self.turtle1_pose, self.turtle2_pose)
       linear_velocity, angular_velocity = self.get_control_signals(distance, angle_diff)
       print("Linear velocitu")
       print(linear_velocity)

        # Publicar las velocidades
       cmd = Twist()
       cmd.linear.x = linear_velocity
       cmd.angular.z = angular_velocity
       self.publisher_.publish(cmd)

   def calculate_distance(self, pose1, pose2):
       print("disntacia calculation")
       return math.sqrt((pose2.x - pose1.x) ** 2 + (pose2.y - pose1.y) ** 2)
   

   def calculate_angle_diff(self, pose1, pose2):
       print("angle calculation")
       angle_to_target = math.atan2(pose2.y - pose1.y, pose2.x - pose1.x)
       angle_diff = angle_to_target - pose1.theta

       while angle_diff > np.pi:
           angle_diff -= 2 * np.pi

       while angle_diff < -np.pi:
           angle_diff += 2 * np.pi

       return angle_diff

def main(args=None):
   rclpy.init(args=args)
   fuzzy_controller = FuzzyController()

   try:
       rclpy.spin(fuzzy_controller)
   except KeyboardInterrupt:
       pass

   fuzzy_controller.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()

"""
#################################################################################################################################################


# Código para Ejercicio 2 Orientación
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


# Definir las variables del sistema difuso
distance = ctrl.Antecedent(np.arange(0, 11, 1), 'distance')
angle_diff = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.1), 'angle_diff')

linear_velocity = ctrl.Consequent(np.arange(0, 3, 0.1), 'linear_velocity')
angular_velocity = ctrl.Consequent(np.arange(-2, 2, 0.1), 'angular_velocity')

# Definir los conjuntos difusos
distance['close'] = fuzz.gaussmf(distance.universe, 0, 2)
distance['far'] = fuzz.gaussmf(distance.universe, 10, 2)
angle_diff['left'] = fuzz.gaussmf(angle_diff.universe, -np.pi/2, np.pi/4)
angle_diff['right'] = fuzz.gaussmf(angle_diff.universe, np.pi/2, np.pi/4)

linear_velocity['slow'] = fuzz.trimf(linear_velocity.universe, [0, 0, 1.5])
linear_velocity['fast'] = fuzz.trimf(linear_velocity.universe, [0, 1.5, 3])
angular_velocity['left'] = fuzz.trimf(angular_velocity.universe, [-2, -1, 0])
angular_velocity['right'] = fuzz.trimf(angular_velocity.universe, [0, 1, 2])

# Definir las reglas difusas
rule1 = ctrl.Rule(distance['close'] & angle_diff['left'], (linear_velocity['slow'], angular_velocity['left']))
rule2 = ctrl.Rule(distance['close'] & angle_diff['right'], (linear_velocity['slow'], angular_velocity['right']))
rule3 = ctrl.Rule(distance['far'] & angle_diff['left'], (linear_velocity['fast'], angular_velocity['left']))
rule4 = ctrl.Rule(distance['far'] & angle_diff['right'], (linear_velocity['fast'], angular_velocity['right']))

# Crear el sistema de control
control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
fuzzy_controller = ctrl.ControlSystemSimulation(control_system)

# Calcular la superficie de control
control_surface = []

for i, d in enumerate(distance.universe):
    for j, a in enumerate(angle_diff.universe):
        fuzzy_controller.input['distance'] = d
        fuzzy_controller.input['angle_diff'] = a
        fuzzy_controller.compute()
        linear_velocity_output = fuzzy_controller.output['linear_velocity']
        angular_velocity_output = fuzzy_controller.output['angular_velocity']
        control_surface.append([d, a, linear_velocity_output, angular_velocity_output])

# Aquí tienes la superficie de control almacenada en un arreglo llamado control_surface
print(control_surface)

# Nodo de ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        self.subscription1 = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.subscription2 = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.turtle1_pose = None
        self.turtle2_pose = None

        # Cargar la superficie de control en memoria
        self.control_surface = control_surface

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg
        self.control_turtle()

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg
        self.control_turtle()

    def control_turtle(self):
        if self.turtle1_pose is None or self.turtle2_pose is None:
            return

        # Calcular la distancia y el ángulo
        distance = self.calculate_distance(self.turtle1_pose, self.turtle2_pose)
        angle_diff = self.calculate_angle_difference(self.turtle1_pose, self.turtle2_pose)

        # Obtener las velocidades usando la superficie de control
        linear_velocity, angular_velocity = self.get_control_signals(distance, angle_diff)

        # Publicar las velocidades
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.publisher_.publish(cmd)

    def calculate_distance(self, pose1, pose2):
        return np.sqrt((pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2)

    def calculate_angle_difference(self, pose1, pose2):
        desired_angle = np.arctan2(pose2.y - pose1.y, pose2.x - pose1.x)
        angle_diff = desired_angle - pose1.theta
        return np.arctan2(np.sin(angle_diff), np.cos(angle_diff))  # Normalize angle

    def get_control_signals(self, distance, angle_diff):
        # Implementar la lógica para seleccionar las velocidades usando la superficie de control
        # Aquí se debe buscar el valor más cercano en la superficie de control para angle_diff
        print("Obteniendo el control")
        closest_match = min(self.control_surface, key=lambda x: abs(x[1] - angle_diff))
        linear_velocity = 0.0  # Mantener la velocidad lineal en 0
        angular_velocity = closest_match[3]  # Usar la velocidad angular correspondiente
        return linear_velocity, angular_velocity

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyController()
    rclpy.spin(fuzzy_controller)
    fuzzy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()#""" 
