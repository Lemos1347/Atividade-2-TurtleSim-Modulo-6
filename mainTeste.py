from rclpy.executors import MultiThreadedExecutor
import threading
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill
from time import sleep
import random
from itertools import permutations
from math import sqrt


class Router_optimizer():

    @staticmethod
    def distance(p1: tuple, p2: tuple):
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    @classmethod
    def get_route_distance(cls, route: list):
        return sum(cls.distance(route[i], route[i-1]) for i in range(len(route)))
    
    @classmethod
    def get_best_route(cls, points: list):
        routes = permutations(points)
        start_point = (0,0)
        routes = [(start_point,) + route for route in routes]
        best_route = min(routes, key=cls.get_route_distance)
        return best_route


class PoseSubscriber(Node):
    last_pose = None

    def __init__(self):
        super().__init__('pose_subscriber')
        print("Waiting for pose topic...")
        self.subscription = self.create_subscription(
            Pose,
            '/turtle5/pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        PoseSubscriber.last_pose = (msg.x, msg.y)
        print(f"Last pose: {PoseSubscriber.last_pose}")
        # self.get_logger().info('Pose: %.2f %.2f %.2f' % (msg.x, msg.y, msg.theta))

    def get_last_pose(self):
        return PoseSubscriber.last_pose


# Classe Turtle que contem todas as informações necessárias para uma tartaruga ser criada
class Turtle:
    amount = 0
    def __init__(self):
        self.name = f"turtle{Turtle.amount}"
        Turtle.amount += 1

# Classe Turtle_controller que contem todos os métodos necessários para controlar uma tartaruga. Essa classe é um nó do ROS para que possa se comunicar com o nó do TurtleSim
class Turtle_controller(Node):
    def __init__(self):
        # Criando um nó no ROS com o nome "turtle_controller"
        super().__init__('turtle_controller')
        sleep(0.5)
        # Limpando a tela do TurtleSim
        self._kill_first_turtle()

        self.points = [(random.randint(0,10), random.randint(0, 10)) for _ in range(5)]

        # Criando as tartarugas nos pontos aleatorios
        for point in self.points:  
            turtle = Turtle()
            self.spawn_turtle(turtle, x=float(point[0]), y=float(point[1]))
        
        # Criando a tartaruga principal para percorrer os pontos
        self.main_turtle = Turtle()
        self.spawn_turtle(self.main_turtle, x=0.0, y=0.0)
        print(f'/{self.main_turtle.name}/pose')

        # Analisando para descobrir a rota mais curta para passar em todos os pontos
        self.best_route = Router_optimizer.get_best_route(self.points)

        self.trajectory()

        self.destroy_node()

    def trajectory(self):
        for i in range(len(self.best_route)):
            try:
                print(f"Moving to {self.best_route[i+1]}")
                x = self.best_route[i+1][0] - self.best_route[i][0] 
                y = self.best_route[i+1][1] - self.best_route[i][1] 
            except IndexError:
                break

            self.move_turtle(self.main_turtle, x=float(x), y=float(y))
            sleep(2)
            try:
                print(f"Last pose: {PoseSubscriber.last_pose[0]}, {PoseSubscriber.last_pose[1]}")
            except:
                print("Last pose: None")

    # Metodo para uso interno da classe para matar a primeira tartaruga criada
    def _kill_first_turtle(self):
        # Utilizando um método da classe herdada para utilizar o serviço kill do TurtleSim
        kill_client = self.create_client(Kill, 'kill')
        # Configurando a chamada do serviço
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        # Chamando o serviço
        kill_client.call_async(kill_request)
        sleep(1)
    
    # Metodo para mover uma tartaruga
    def move_turtle(self, turtle:Turtle, x=None, y=None, z=None):
        # Utilizando um método da classe herdada para conseguir publicar no tópico cmd_vel do TurtleSim
        move_publisher = self.create_publisher(Twist, f'{turtle.name}/cmd_vel', 10)
        # Criando uma mensagem do tipo Twist (o que vai ser publicado)
        twist_msg = Twist()
        # Realizando verificações para modificar a mensagem que vai ser passada de acordo com os parâmetros requisitados
        if x:
            twist_msg.linear.x = x
        if y:
            twist_msg.linear.y = y
        if z:
            twist_msg.angular.z = z
        # Publicando a mensagem no tópico
        move_publisher.publish(twist_msg)
        sleep(1)
    
    # Metodo para criar um tartaruga
    def spawn_turtle(self, turtle:Turtle, x=None, y=None, theta=None):
        # Utilizando um método da classe herdada para utilizar o serviço spawn do TurtleSim
        spawn_client = self.create_client(Spawn, 'spawn')
        # Configurando a chamada do serviço
        spawn_request = Spawn.Request()
        # Realizando verificações para modificar a mensagem que vai ser passada de acordo com os parâmetros requisitados
        if x:
            spawn_request.x = x
        if y:
            spawn_request.y = y
        if theta:
            spawn_request.theta = theta
        spawn_request.name = turtle.name
        # Chamando o serviço
        spawn_client.call_async(spawn_request)
        sleep(1)

rclpy.init(args=None)
global_turtle_controller = Turtle_controller()
global_pose_subscriber = PoseSubscriber()

# Função principal de execução
def main(args=None):
    # Inicializando o ROS
    # rclpy.init(args=args)
    # # Instanciando um objeto Turtle_controller
    # turtle_controller = Turtle_controller()
    # pose_subscriber = PoseSubscriber()

    # executor = MultiThreadedExecutor()
    # executor.add_node(pose_subscriber)
    # executor.add_node(turtle_controller)
    # # add more nodes to the executor as needed

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass

    # # remove nodes and shutdown rclpy library
    # executor.remove_node(turtle_controller)
    # executor.remove_node(pose_subscriber)

    thread1 = threading.Thread(target=turtle_controller)
    thread2 = threading.Thread(target=pose)

    # Inicie os threads
    thread1.start()
    thread2.start()

    # Espere os threads terminarem
    thread1.join()
    thread2.join()


    # turtle_controller.destroy_node()
    # pose_subscriber.destroy_node()
    rclpy.shutdown()

def turtle_controller():
    rclpy.spin(global_turtle_controller)

def pose():
    rclpy.spin(global_pose_subscriber)

# # Funcão principal de execução
# def main(args=None):
#     # Inicializando o ROS
#     rclpy.init(args=args)
#     # Instanciando um objeto Turtle_controller
#     turtle_controller = Turtle_controller()

#     # Chamando as funções para desenhar no TurtleSim
#     clean_screen(turtle_controller)
#     draw_first_mountain(turtle_controller)
#     draw_second_mountain(turtle_controller)
#     draw_floor(turtle_controller)
#     draw_sky(turtle_controller)
#     draw_sun(turtle_controller)

#     # Finalizando o ROS e finalizando o nó criado para comunicar com o nó do TurtleSim
#     turtle_controller.destroy_node()
#     rclpy.shutdown()

# Ponto de inicialização do programa
if __name__ == '__main__':
    main()
