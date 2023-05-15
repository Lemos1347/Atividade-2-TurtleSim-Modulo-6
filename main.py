import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
from time import sleep
import random
from itertools import permutations
from math import sqrt

# Classe contento todas as funcoes para escolher a rota mais curta para passar em todos os pontos
class Router_optimizer():
    # Metodo estatico da classe para calcular a distancia entre dois pontos
    @staticmethod
    def distance(p1: tuple, p2: tuple):
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    # Metodo da clase para calcular a distancia total de todos os pontos
    @classmethod
    def get_route_distance(cls, route: list):
        return sum(cls.distance(route[i], route[i-1]) for i in range(len(route)))
    # Metodo da classe para calcular o conjunto de pontos que possui a menor trajetoria
    @classmethod
    def get_best_route(cls, points: list):
        # Utilizo a funcao permutations da biblioteca 'itertools'. Essa funcao ira criar um array com arrays de tuplas que possuem todas as permutacoes de todos os pontos que e necessario passar, ou seja se temos os seguinte pontos (0,1), (0,2) e (0,3) termos:
        # [[(0,1), (0,2), (0,3)], [(0,1), (0,3), (0,2)], [(0,2), (0,1), (0,3)], [(0,2), (0,3), (0,1)], [(0,3), (0,2), (0,1)], [(0,3), (0,1), (0,2)]]
        routes = permutations(points)
        # Para que ele nao permute o ponto de inicio tambem, estabelecemos ele fora da funcao da biblioteca e o adiconamos depois em cada um dos arrays resultantes da permutacao
        start_point = (0,0)
        routes = [(start_point,) + route for route in routes]
        # Utilizamos da funcao 'min' para pegar o menor resultado de todas as possiveis rotas
        best_route = min(routes, key=cls.get_route_distance)
        # Retorno a rota mais 'curta'
        return best_route

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

    def trajectory(self, best_route: list, turtle: Turtle):
        # Para cada um dos pontos que a tartaruga deve se mover, comparo com o ponto que ela esta com o que ela deveria ir. O resultado disso sao os pontos x e y que devem ser utilizados no topico cmd_vel
        for i in range(len(best_route)):
            # Coloco um try-except para que ele pare o loop e nao todo o programa quando chegar na ultima posicao
            try:
                print(f"Moving to {best_route[i+1]}")
                x = best_route[i+1][0] - best_route[i][0] 
                y = best_route[i+1][1] - best_route[i][1] 
            except IndexError:
                break

            self.move_turtle(turtle, x=float(x), y=float(y))
            sleep(3)

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


# Função principal de execução
def main(args=None):
    # Inicializando o ROS
    rclpy.init(args=args)
    # # Instanciando um objeto Turtle_controller
    turtle_controller = Turtle_controller()

    # Criando os pontos aleatorios entre 0 e 10 que a tartaruga principal deve passar
    points = [(random.randint(0,10), random.randint(0, 10)) for _ in range(5)]

    # Criando as tartarugas nos pontos aleatorios para uma melhor visuzlizacao de onde a tartaruga principal deve passar 
    for point in points:  
        turtle = Turtle()
        turtle_controller.spawn_turtle(turtle, x=float(point[0]), y=float(point[1]))
    
    # Criando a tartaruga principal para percorrer os pontos
    main_turtle = Turtle()
    turtle_controller.spawn_turtle(main_turtle, x=0.0, y=0.0)

    # Analisando para descobrir a rota mais curta para passar em todos os pontos
    best_route = Router_optimizer.get_best_route(points)

    # Executando a movimentacao da tartaruga com base na fila dos pontos que acabamos de criar
    turtle_controller.trajectory(best_route, main_turtle)

    # Apagando o no, assumindo que todas as tarefas ja foram executadas com sucessso
    turtle_controller.destroy_node()
    
    # Finalizando o rclpy
    rclpy.shutdown()

# Ponto de inicialização do programa
if __name__ == '__main__':
    main()
