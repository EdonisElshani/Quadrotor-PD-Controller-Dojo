using Dojo
using DojoEnvironments
using LinearAlgebra
i = 0;
help = 0; 
loop_bool = 0; 
loop_start = 0; 
loop_ongoing = 404;
step_counter = nothing 
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=2500)
waypoint_increment = 1;

function calculate_rotor_speeds(thrust, pitch, roll, yaw)
    # Base thrust plus adjustments for pitch, roll, and yaw
    global loop_bool
    global loop_ongoing 
    if loop_bool == 1
        motor1 = 65#1 * (thrust - pitch + roll + yaw)
        motor2 = 65#1* thrust - pitch - roll - yaw
        motor3 =  70#0.5 * (thrust + pitch - roll + yaw)
        motor4 = 70#0.5 * (thrust + pitch + roll - yaw)
    #elseif loop_bool == 0 && loop_ongoing == 1 
     #   motor1 = 55
      #  motor2 = 55
       # motor3 = 55
        #motor4 = 55
        #println("HEEEEEEEYYYYYY")
    else 
        motor1 = thrust - pitch + roll + yaw
        motor2 = thrust - pitch - roll - yaw
        motor3 = thrust + pitch - roll + yaw
        motor4 = thrust + pitch + roll - yaw
end

   
    return [motor1, motor2, motor3, motor4]
end

function orientation_PD(environment, des_vz, des_pitch, des_roll, des_yaw, des_x, des_y)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 
    global loop_bool, loop_ongoing, i, help

    
    roll = orientation[1]
    pitch = orientation[2]
    yaw = orientation[3]

    K_p_pitch = 30
    K_d_pitch = 3
    K_i_pitch = 0.3

    K_p_roll = 30
    K_d_roll = 3
    K_i_roll = 0.3

    if loop_ongoing == 1 
        g_thrust = 12
    else 
        g_thrust = 20
    end 
    #println("roll",orientation[1])
    #println("pitch",orientation[2])
    if loop_ongoing == 1 && abs(orientation[2]) < 0.15
        #println("NOW FALL CONTROL", step_counter)
        #println("time",step_counter)
        #println("linear_velocity:",linear_velocity)
         
        if i == 0
            help = step_counter
        end
        loop_bool = 0
        des_pitch = 0
        des_roll = 0
        des_yaw = 0
        des_vz = 0
        K_p_pitch = 1#30
        K_d_pitch = 3
        K_i_pitch = 0#0.3
    
        K_p_roll = 1#30
        K_d_roll = 3
        K_i_roll = 0#0.3
        
        if step_counter - help > 200 && help > 0 
            loop_ongoing = 0
            K_p_pitch = 1#30
            K_d_pitch = 3
            K_i_pitch = 0#0.3
    
            K_p_roll = 1#30
            K_d_roll = 3
            K_i_roll = 0#0.3
            #g_thrust = 12 
            println("Jetzt Zielpunkt ansteuern",step_counter)
        end 
        i += 1
      
    end  
  

    u_thrust= g_thrust *((10*(des_vz - linear_velocity[3]) - 1*linear_velocity[3] + 5.1)*(1/sqrt(4)))
    u_pitch = (K_p_pitch * (des_pitch - pitch) + K_d_pitch * (0 - angular_velocity[2])+K_i_pitch*(des_x-position[1]))*(1/sqrt(2))
    u_roll = (K_p_roll * (des_roll + roll) + K_d_roll * (0 + angular_velocity[1])+K_i_roll*(des_y-position[2]))*(1/sqrt(2))
    u_yaw = des_yaw=0

    #println("e_roll",des_roll - roll)
    #println("e_angular_velocity", 0 - angular_velocity[1])
    #println("u_roll",u_roll)

    return calculate_rotor_speeds(u_thrust, u_pitch, u_roll, u_yaw)
end

function position_PD(environment, des_x, des_y, des_z)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 
    
    
   
    K_p_pitch = 0.065
    K_d_pitch = 0.15

    K_p_roll = 0.065
    K_d_roll = 0.15

    des_vz = des_z - position[3]

    des_pitch = (K_p_pitch * (des_x - position[1]) + K_d_pitch * (0 - linear_velocity[1]))
    des_roll = (K_p_roll * (des_y - position[2]) + K_d_roll * (0 - linear_velocity[2]))
    des_yaw = 0

    #println("e_pos",des_y - position[2])
    #println("e_velocity",0 - linear_velocity[2])
    #println("des_roll",des_roll)

    return orientation_PD(environment, des_vz, des_pitch, des_roll, des_yaw, des_x, des_y)
end

function controller!(environment, k)
    global step_counter 
    if step_counter === nothing 
        step_counter = 0
    end 

    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 
    global waypoint_increment
    global loop_bool
    global loop_ongoing


    waypoints = 
    [
        [1;-1;0.3],
        [2;0;0.3],
        [1;1;0.3],
        [0;0;3],
    ]

    if norm(position-waypoints[waypoint_increment]) < 0.2
        if waypoint_increment < 4
            println("next_waypoint: ", waypoint_increment)
            waypoint_increment += 1
        end
    end
    
    tmp = waypoints[waypoint_increment]

    if waypoint_increment == 4 
        if norm(position-waypoints[4]) < 0.1
            loop_bool = 1
        end 
    end
    if loop_bool == 1 && norm(position-waypoints[4]) > 0.3
        loop_ongoing = 1
    end

    x = tmp[1]
    y = tmp[2]
    z = tmp[3]

    if waypoint_increment == 4 && loop_bool == 0 && loop_ongoing == 0
        x = 0 
        y = 0 
        z = 0.3 
    end 
    rotor_speeds = position_PD(environment, x, y, z)
    step_counter += 1
    #println("time:",step_counter)
    set_input!(environment, rotor_speeds) # rotor speeds are directly set
end


initialize!(quadrotor_env, :quadrotor; body_orientation=Dojo.RotZ(-pi/4))
simulate!(quadrotor_env, controller!; record=true)
vis = visualize(quadrotor_env) # you can visualize even if the simulation fails
render(vis)
