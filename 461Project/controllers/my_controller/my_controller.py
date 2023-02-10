from controller import Robot

#create instace of Robot
robot = Robot()

time_step=int(robot.getBasicTimeStep())
base_speed=4

#getting rotational motor from hindgejoint
left_motor = robot.getDevice('wheel1')
right_motor = robot.getDevice('wheel2')
right_b_motor = robot.getDevice('wheel3')
left_b_motor = robot.getDevice('wheel4')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
right_b_motor.setPosition(float('inf'))
left_b_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0) 
left_b_motor.setVelocity(0.0)
right_b_motor.setVelocity(0.0)

#getting ir sensors   
ir_left = robot.getDevice('LEFT')
ir_right = robot.getDevice('RIGHT')
ir_middle = robot.getDevice('MID')
ir_left.enable(time_step)   
ir_right.enable(time_step)    
ir_middle.enable(time_step)

#intitializing pid variables
kp = 0.05
kd = 0
ki = 0
last_error = intg = diff = prop = 0

sensor_val = [0,100]   

def PID(error):
    global last_error, intg, diff, prop, kp, ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    last_error = error
    balance = (kp*prop) + (kd*diff) + (ki*intg)
    return balance  
    
while robot.step(time_step) != -1:
    left_ir_value = ir_left.getValue()
    right_ir_value = ir_right.getValue()
    mid_ir_value = ir_middle.getValue()
        
    print(f"left:{left_ir_value}   mid:{mid_ir_value}   right:{right_ir_value}")
        
    left_speed = base_speed
    right_speed = base_speed
    #straight    
    if left_ir_value <1000 and right_ir_value<1000 and mid_ir_value>=1000:
        error = sensor_val[0]
        balance = PID(error)
        left_speed = base_speed+balance
        right_speed = base_speed+balance
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)
    #right turn
    elif left_ir_value<1000 and right_ir_value>=1000 and mid_ir_value>=1000:
        error = sensor_val[1]
        balance = PID(error)
        left_speed = base_speed+balance
        right_speed = base_speed-balance  
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)     
    #left turn
    elif left_ir_value>=1000 and right_ir_value<1000 and mid_ir_value>=1000:
        error = sensor_val[1]
        balance = PID(error)
        left_speed = base_speed-balance
        right_speed = base_speed+balance
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)
    #sharp left turn
    elif left_ir_value>=1000 and right_ir_value<1000 and mid_ir_value<1000:
        error = sensor_val[1]
        balance = PID(error)
        left_speed = base_speed-balance
        right_speed = base_speed+balance
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)
    #sharp right turn
    elif left_ir_value<1000 and right_ir_value>=1000 and mid_ir_value<1000:
        error = sensor_val[1]
        balance = PID(error)
        left_speed = base_speed+balance
        right_speed = base_speed-balance
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)
    #off track
    elif left_ir_value<1000 and right_ir_value<1000 and mid_ir_value<1000:
        error = sensor_val[0]
        balance = PID(error)
        left_motor.setVelocity(left_speed+balance)
        right_motor.setVelocity(right_speed+balance)
        left_b_motor.setVelocity(left_speed)
        right_b_motor.setVelocity(right_speed)