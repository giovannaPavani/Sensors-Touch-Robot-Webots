"""rus_controller controller."""

#by slmm for TI502

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

print("Iniciando")
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64 #int(robot.getBasicTimeStep())

nome = robot.getName()
print("Nome do robo : ", nome)

motor_esq = robot.getDevice("motor roda esquerda")
motor_dir = robot.getDevice("motor roda direita")

motor_esq.setPosition(float('+inf'))
motor_dir.setPosition(float('+inf'))

motor_esq.setVelocity(0.0)
motor_dir.setVelocity(0.0)

# obtem o sensor de distancia
ir0 = robot.getDevice("ir0")
ir0.enable(timestep)

ir1 = robot.getDevice("ir1")
ir1.enable(timestep)

ir2 = robot.getDevice("ir2")
ir2.enable(timestep)


sentido = 0
timer = 0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
      #enquanto o robô estiver andando

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    dist = ir0.getValue()
    dist1 = ir1.getValue()
    dist2 = ir2.getValue()
    print(dist)    
    # Process sensor data here.
    if (dist >= 456): # se a distancia for maior ou igual a 456, ele muda o sentido
        sentido = 1
        
    if (sentido == 0): #indo para frente
        motor_esq.setVelocity(5.0)
        motor_dir.setVelocity(5.0)
    else: # indo para a direita
        motor_esq.setVelocity(0.0)
        motor_dir.setVelocity(5.0)
        sentido = 0
            
    


# Enter here exit cleanup code.
