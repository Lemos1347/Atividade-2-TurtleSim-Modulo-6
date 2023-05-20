import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from time import sleep
import random
from itertools import permutations
from math import sqrt, atan2, degrees, radians

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
    # Variável de controle para nunca tentar criar uma tartaruga que já existe
    amount = 0
    # Para o constructor da classe precisamos passa um turtle_controller para já criar a tartaruga no turtlesim e as coordenadas que desejamos passar
    def __init__(self, turtel_controller, x=0, y=0):
        self.name = f"turtle{Turtle.amount}"
        Turtle.amount += 1
        # Criando a tartaruga no turtlesim
        turtel_controller.spawn_turtle(self, x=float(x), y=float(y))
        # Variável para sabermos a posição que a tartaruga se encontra
        self.last_pose = Pose(x=float(x), y=float(y))

# Classe Turtle_controller que contem todos os métodos necessários para controlar uma tartaruga. Essa classe é um nó do ROS para que possa se comunicar com o nó do TurtleSim
class Turtle_controller(Node):
    # last_pose = Pose(x=float(0), y=float(0))
    def __init__(self) -> None:
        # Criando um nó no ROS com o nome "turtle_controller"
        super().__init__('turtle_controller')
        sleep(0.5)
        # Limpando a tela do TurtleSim
        self._kill_first_turtle()
        # Dicionário de tartarugas que deseja atualizar a last_pose
        self.turtle_to_look_up: dict = {}
    
    def create_turtle_monitor(self, turtle: Turtle) -> None:
        # Verificando se o nó já está como subscriber no tópico da tartaruga em que se deseja monitorar a posição
        try:
            if self.turtle_to_look_up[f'{turtle.name}']:
                print(f'Already subscribed to {turtle.name}/pose!')
        except:
            # Inscrevendo o nó turtle_controller no tópico /turtle5/pose para que possamos pegar a posição da tartaruga principal (obs.: a turtle 5 é a principal que irá percorrer os pontos)
            self.create_subscription(
                Pose,
                f'/{turtle.name}/pose',
                self.generate_callback(f'{turtle.name}'),
                10)
            # Adicionando ao dicionário o objeto 'turtle' da tartaruga em que acabamos de inscrever no tópico
            self.turtle_to_look_up[f'{turtle.name}'] = turtle

    # Para que seja possível identificar qual tartaruga deseja atualizar a last_pose, crio função encapsulada para que além de receber a msg que está sendo publicada no tópico, também recebe uma mensagem padrão que é definida no momento em que a função 'generate_callback' é chamada, a qual é o nome da tartaruga(assim podemos identificar de qual tópico essa mensagem está vindo)
    def generate_callback(self, topic_name: str):
        def callback(msg: Pose):
            self.listener_callback(msg, topic_name)
        return callback

    # Função para atualizar a 'last_pose' de uma tartaruga
    def listener_callback(self, msg: Pose, topic_name: str) -> None:
        # Utilizando 'try' para exibir uma mensagem de erro caso a tartaruga que desejo atualizar a last_pose não esteja na lista de monitoramento.
        try:
            self.turtle_to_look_up[topic_name].last_pose = msg
        except:
            print(f'Error: {topic_name}/pose not found!')

    def trajectory(self, best_route: list, turtle: Turtle) -> None:
        # Para cada um dos pontos que a tartaruga deve se mover, comparo com o ponto que ela esta com o que ela deveria ir. O resultado disso sao os pontos x e y que devem ser utilizados no topico cmd_vel
        for i in range(1, len(best_route)):
            # Realizamos multiplas vezes o spin do nó para que seja possível pegar as últimas mensagens dos tópicos que esse nó está como subscriber, ou seja, o last_pose das tartarugas serão atualizados
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.5)
            # Coloco um try-except para que ele pare o loop e nao todo o programa quando chegar na ultima posicao
            try:
                print(f"Moving to {best_route[i]}")
                x: float = float(best_route[i][0]) - float(turtle.last_pose.x)
                y: float = float(best_route[i][1]) - float(turtle.last_pose.y)

                # Calcular o ângulo entre a posição atual da tartaruga e o próximo ponto
                angle: float = atan2(y, x)
                # Girar a tartaruga para esse ângulo
                self.rotate_turtle(turtle, target_angle=angle)
                sleep(1)

                distance: float = sqrt(x**2 + y**2)

                self.move_turtle(turtle, x=distance)
                sleep(3)
            except IndexError:
                break
    
    def rotate_turtle (self, turtle:Turtle, target_angle:float) -> None:
        # Converter o ângulo alvo para graus
        target_angle = degrees(target_angle)
        # Criar um 'publisher' para o tópico cmd_vel
        rotate_publisher = self.create_publisher(Twist, f'{turtle.name}/cmd_vel', 10)
        # Criar uma mensagem Twist
        twist_msg = Twist()
        
        while True:
            # Calcular a diferença entre o ângulo atual da tartaruga e o ângulo alvo
            angle_diff = target_angle - degrees(turtle.last_pose.theta)
            # Se a diferença de ângulo for pequena o suficiente, pare de girar
            if abs(angle_diff) < 0.1:
                break
            # Caso contrário, continue girando a tartaruga na direção correta
            twist_msg.angular.z = radians(angle_diff)
        
            # Criando condições para estabeler o limite de 'giro' entre 2.0 e -2.0 radianos. Isso é feito para que a tartaruga não gire muito e acaba não dando tempo do tópico atualizar a last_pose rápido o suficiente, evitndo assim que a tartaruga gire infinitamente
            if radians(angle_diff) < -2.0 or radians(angle_diff) > 2.0:
                if radians(angle_diff) < 0:
                    twist_msg.angular.z = -2.0
                elif radians(angle_diff) > 0:
                    twist_msg.angular.z = 2.0

            print(f"Angle diff: {twist_msg.angular.z}")
            # Publicar a mensagem
            rotate_publisher.publish(twist_msg)
            # Realizamos multiplas vezes o spin do nó para que seja possível pegar as últimas mensagens dos tópicos que esse nó está como subscriber, ou seja, o last_pose das tartarugas serão atualizados
            for _ in range(15):
                rclpy.spin_once(self, timeout_sec=0.5)
            
    # Metodo para uso interno da classe para matar a primeira tartaruga criada
    def _kill_first_turtle(self) -> None:
        # Utilizando um método da classe herdada para utilizar o serviço kill do TurtleSim
        kill_client = self.create_client(Kill, 'kill')
        # Configurando a chamada do serviço
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        # Chamando o serviço
        kill_client.call_async(kill_request)
        sleep(1)
    
    # Metodo para mover uma tartaruga
    def move_turtle(self, turtle:Turtle, x: float=None, y: float=None, z: float=None) -> None:
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
    def spawn_turtle(self, turtle:Turtle, x:float=None, y:float=None, theta:float=None) -> None:
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
def main(args=None) -> None:
    # Inicializando o ROS
    rclpy.init(args=args)
    # # Instanciando um objeto Turtle_controller
    turtle_controller = Turtle_controller()

    # Criando os pontos aleatorios entre 0 e 10 que a tartaruga principal deve passar
    points = [(random.randint(1,10), random.randint(1, 10)) for _ in range(5)]

    # Criando as tartarugas nos pontos aleatorios para uma melhor visuzlizacao de onde a tartaruga principal deve passar 
    for point in points:  
        _ = Turtle(turtel_controller=turtle_controller, x=float(point[0]), y=float(point[1]))
    
    # Criando a tartaruga principal para percorrer os pontos
    main_turtle = Turtle(turtel_controller=turtle_controller)

    # Criando um subscriber para a tartaruga principal
    turtle_controller.create_turtle_monitor(main_turtle)

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
