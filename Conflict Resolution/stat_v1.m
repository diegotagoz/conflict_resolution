%statistical data generator V1. Works with solverTB_v1_stat and UAV_Robot_v1
addpath('./Own_Classes')

%loop simulation parameters
num_robots_i = 5;                                                           %initial number of robots
num_robots_f = 100;                                                         %final number of robots
num_robots_inc = 5;                                                         %increment of the number of robots
timeout_multiplier = 2;

%simulation parameters
seed = 7;                                                                   %seed fot the random numer generator
steps = 4;                                                                  %default UAV's speed
zref_low = 10;                                                              %referential altitudes
zref_high = 25;
distance_tolerance = 6;                                                     %robots close to each other
arrive_tolerance = 2;                                                       %robot arrived to destination
collision_tolerance = 2;                                                    %number of cells to extend robot position to avoid collision (FMredo method)
critical_distance = 1;                                                      %distance to consider a real collisions (collision watchdog)

num_iter = length(num_robots_i:num_robots_inc:num_robots_f);                %total iterations    

%initialization of the statistics results structure. Comment if you have already some statistic results and want to cmplete them.
v1_stat.timeout = zeros(1,num_iter);
v1_stat.num_robots = zeros(1,num_iter);
v1_stat.posible_collisions_detected = zeros(1,num_iter);
v1_stat.arrived_robots_perc = zeros(1,num_iter);
v1_stat.runtime = zeros(1,num_iter);
v1_stat.iterations = zeros(1,num_iter);
v1_stat.collisions = zeros(1,num_iter);
v1_stat.collisiions_avoided_perc = zeros(1,num_iter);
v1_stat.minimal_distance = zeros(1,num_iter);

%Typical configuration for the simulation (confX.m file)
position_distance = 5;                                                      %minimal distance between created UAV positions/destinations.
rng(seed);
%initializations
robot_pos = zeros(num_robots_f,2);
robot_dest = zeros(num_robots_f,2);
robot_priority = zeros(num_robots_f,1);
for i = 1:num_robots_f
    
    %position
    while 1
        robot_pos(i,1)=round(rand*(140-3))+1;
        robot_pos(i,2)=round(rand*(80-3))+1;
        
        %verify that this position is not close to a previous one
        if i > 1
            probe = 1;
            for j = 1:i-1
                probe = probe && norm(robot_pos(i,:) - robot_pos(j,:)) >= position_distance;
            end
            if probe == 1
                break;
            end
        else
            break;
        end
    end
    
    %destination
    while 1
        robot_dest(i,1) = round(rand*(140-2))+1;
        robot_dest(i,2) = round(rand*(80-2))+1;
        
        %verify that this destination is not close to a previous one
        if i > 1
            probe = 1;
            for j = 1:i-1
                probe = probe && norm(robot_dest(i,:) - robot_dest(j,:)) >= position_distance;
            end
            if probe == 1
                break;
            end
        else
            break;
        end
    end
    
    %priority
    robot_priority(i,1) = round(rand*10);
end

%complete structure data wich is general to all the simulations
v1_stat.seed = seed;
v1_stat.robot_pos = robot_pos;
v1_stat.robot_prioirity = robot_priority;

%simulation loop
for num_robots = num_robots_i:num_robots_inc:num_robots_f
    
    disp(['Simulating v1 with ' num2str(num_robots) ' robots']); 
   
    big_iteration = num_robots / num_robots_inc;                            %order number of the actual iteration
    
    timeout = (6.8449*(num_robots/5)^2 - 7.7215*(num_robots/5) + 16.664) * timeout_multiplier       %estimated by a quadratic regression on previous simulations
    
    solverTB_v1_stat;                                                       %THE simulation
    
    %complete statistical results in the structure
    v1_stat.timeout(big_iteration) = timeout;
    v1_stat.num_robots(big_iteration) = num_robots;
    v1_stat.posible_collisions_detected(big_iteration) = posible_collisions_detected;
    v1_stat.arrived_robots_perc(big_iteration) = sum(arrived_robots)/num_robots * 100;
    v1_stat.runtime(big_iteration) = total_runtime;
    v1_stat.iterations(big_iteration) = iteration;
    v1_stat.collisions(big_iteration) = collisions;
    v1_stat.collisiions_avoided_perc(big_iteration) = collisions_avoided / posible_collisions_detected * 100;
    v1_stat.minimal_distance(big_iteration) = minimal_distance;
end

save('v1_stat.mat','v1_stat');                                              %save structure as a .mat file