# Atividade-2-TurtleSim-Modulo-6

O código Python apresentado é um programa para controle de tartarugas em um simulador TurtleSim usando ROS (Robot Operating System).

# Como rodar

Para rodar o programa, é necessário ter o ROS instalado e configurado. Para isso, siga as instruções do site oficial do [ROS](https://docs.ros.org/en/humble/index.html). Também é necessário ter [python3](https://www.python.org/downloads/) instalado.

Agora em seu terminal digite o seguinte comando:

```bash
ros2 run turtlesim turtlesim_node
```

E na root desse projeto digite:

```bash
python3 main.py
```

**Projeto rodando**  

https://github.com/Lemos1347/Atividade-2-TurtleSim-Modulo-6/assets/99190347/1e0d6b58-6974-42f1-938e-389b98ed06ca

## Funcionalidades

Quando o comando "ros2 run turtlesim turtlesim_node" é utilizado, é criado um nó em meu ROS com o nome "turtlesim". Esse nó contém um série de tópicos que está inscrito e serviços. Compreende-se que esses serviços podem ser interpretados como uma interação cliente-servidor, enquanto tópicos é um método de interação que "lanço" dados na rede do ROS e apenas o nó para que é dirigida a informação(os inscritos a aquele tópico) capta a "mensagem". A partir disso, precisei de utilizar os seguintes serviços: "/spawn" e "/kill".

- "/spawn" : cria uma nova tartaruga no simulador, isso é acompanhado com novos serviços com o nome daquela tartaruga e novos tópicos com o nome da tartaruga criada que o nó "turtlesim" fica como "subscriber" e "publisher".
- "/kill": apaga uma tartaruga existente e os serviços e tópicos que são relacionados a ela.

Para fazer o uso desses serviços, foi criado um cliente para cada um deles. O cliente é criado com o nome da tartaruga que será criada ou apagada. Para criar uma tartaruga, é necessário passar como parâmetro o nome da tartaruga, a posição x e y e a orientação. Para apagar uma tartaruga, é necessário passar como parâmetro o nome da tartaruga que será apagada. Isso tudo é feito na classe "Turtle_controller" a qual é um nó chamado "turtle_controller".

Tópico utilizado:
Para movimentar a tartaruga, precisei utilizar um tópico chamado "/{nome_tartaruga}/cmd_vel". Ele recebe como mensagem do tipo "Twist", a qual contempla como a tartaruga deve se locomover. Para isso, foi criado um publisher para esse tópico, o qual envia a mensagem para o nó "turtlesim" que está inscrito nesse tópico. O publisher é criado na classe "Turtle_controller", a qual é um nó chamado "turtle_controller".

Para realizar a locomoção da tartaruga principal criei uma classe `Router_optimizer` com os seguinte métodos:

- **Distance**: dois pontos são dados como entrada e a distância euclidiana entre eles é calculada e retornada, seguindo a seguinte equação: √((x2-x1)² + (y2-y1)²).

- **get_route_distance**: uma lista de pontos é dada como entrada e a distância total da rota é calculada e retornada. Para isso, é utilizado o método "Distance" para calcular a distância entre cada ponto e o próximo ponto na lista.

- **get_best_route**: uma lista de pontos é dada como entrada e a rota mais curta é calculada e retornada. Para isso, é utilizado o método "get_route_distance" para calcular a distância total de cada rota possível. Nessa função é utilizado a biblioteca "itertools" para gerar todas as permutações possíveis dos pontos. Essa função retorna uma lista de pontos que será responsável por guiar a tartaruga principal.

Toda vez que o programa é rodado, crio pontos aleatórios que a tartaruga principal deve passar. Para uma melhor visualização dos pontos, crio novas tartarugas com o serviço "/spawn" nos pontos aleatórios gerados.

Na classe `Turtle_controller` há um método chamado "journey" que é responsável realizar a locomoção da tartaruga principal. Esse método recebe como parâmetro uma lista de pontos e a tartaruga principal é movimentada para cada ponto da lista. Para isso, é utilizado o publisher criado para o tópico "/{nome_tartaruga}/cmd_vel" e é enviado uma mensagem do tipo "Twist" para o nó "turtlesim" que está inscrito nesse tópico. A mensagem é criada com a velocidade linear e angular que a tartaruga deve se locomover. A velocidade linear é calculada com a distância entre o ponto atual e o próximo ponto da lista.

Toda a lógica do código está na função `main`, desde a inicialização do rclpy, instanciação dos nós, criação dos clientes e publisher, até a chamada da função `journey` para realizar a locomoção da tartaruga principal.     
  
**_Observação:_** os arquivos `mainTeste.py` e `subscriberTeste.py` são para desenvolvimento, apenas considere o `main.py`.
