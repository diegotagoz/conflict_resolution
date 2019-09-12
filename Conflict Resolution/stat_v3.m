%statistical data generator V3. Works with solverTB_v3_stat and UAV_Robot_v3_stat
addpath('./Own_Classes')

%global variables
global posible_collisions_detected;
global collisions_avoided_velmod;
global collisions_avoided_fmredo;
global collisions_avoided_hover;
global velmod_reduction_accum;

%loop simulation parameters
num_robots_i = 30;                                                           %initial number of robots
num_robots_f = 30;                                                         %final number of robots
num_robots_inc = 5;                                                         %increment of the number of robots

%simulation parameters
seed = 7;                                                                   %seed fot the random numer generator
steps = 4;                                                                  %default UAV's speed
% zref_low = 10;                                                              %referential altitudes
% zref_high = 25;
arrive_tolerance = 2;                                                       %robot arrived to destination
comms_distance =  10;                                                       %comms maximum reach                                                                
comm_error = 0.7;                                                           %typical deviation of communication error,if abs(error) < 1 then UAVs communicate
critical_distance = 1;                                                      %distance to consider a real collisions (collision watchdog)

num_iter = length(num_robots_i:num_robots_inc:num_robots_f);                %total iterations 

%initialization of the statistics results structure. Comment if you have already some statistic results and want to complete them.
v3_stat.num_robots = zeros(1,num_iter);
v3_stat.posible_collisions_detected = zeros(1,num_iter);
v3_stat.arrived_robots_perc = zeros(1,num_iter);
v3_stat.runtime = zeros(1,num_iter);
v3_stat.iterations = zeros(1,num_iter);
v3_stat.collisions = zeros(1,num_iter);
v3_stat.collisiions_avoided_perc = zeros(1,num_iter);
v3_stat.collisions_avoided_velmod_perc = zeros(1,num_iter);
v3_stat.collisions_avoided_fmredo_perc = zeros(1,num_iter);
v3_stat.hover_perc_av = zeros(1,num_iter);
v3_stat.velmod_reducedvelocity_av = zeros(1,num_iter);
v3_stat.velocity_av = zeros(1,num_iter);
v3_stat.minimal_distance = zeros(1,num_iter);
v3_stat.total_comms = zeros(1,num_iter);
v3_stat.succesful_comms_perc = zeros(1,num_iter);

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
v3_stat.seed = seed;
v3_stat.robot_pos = robot_pos;
v3_stat.robot_prioirity = robot_priority;

%simulation loop
for num_robots = num_robots_i:num_robots_inc:num_robots_f
    
    disp(['Simulating v3 with ' num2str(num_robots) ' robots']); 
    
    big_iteration = num_robots / num_robots_inc;                            %order number of the actual iteration
    
    solverTB_v3_stat;                                                       %THE simulation
    
    %complete statistical results in the structure
    v3_stat.num_robots(big_iteration) = num_robots;
    v3_stat.posible_collisions_detected(big_iteration) = posible_collisions_detected;
    v3_stat.arrived_robots_perc(big_iteration) = sum(arrived_robots)/num_robots * 100;
    v3_stat.runtime(big_iteration) = total_runtime;
    v3_stat.iterations(big_iteration) = iteration;
    v3_stat.collisions(big_iteration) = collisions;
    v3_stat.collisiions_avoided_perc(big_iteration) = (collisions_avoided_velmod + collisions_avoided_fmredo + collisions_avoided_hover) / posible_collisions_detected * 100;
    v3_stat.collisions_avoided_velmod_perc(big_iteration) = collisions_avoided_velmod / (collisions_avoided_velmod + collisions_avoided_fmredo + collisions_avoided_hover) * 100;
    v3_stat.collisions_avoided_fmredo_perc(big_iteration) = collisions_avoided_fmredo / (collisions_avoided_velmod + collisions_avoided_fmredo + collisions_avoided_hover) * 100;
   	v3_stat.collisions_avoided_hover_perc(big_iteration) = collisions_avoided_hover / (collisions_avoided_velmod + collisions_avoided_fmredo + collisions_avoided_hover) * 100;
  	v3_stat.hover_perc_av(big_iteration) = sum(hover_iterations ./ moving_iterations) / num_robots * 100;
    v3_stat.velmod_reducedvelocity_av(big_iteration) = velmod_reduction_accum / collisions_avoided_velmod;
    v3_stat.velocity_av(big_iteration) = global_vel_acum / iteration;
    v3_stat.minimal_distance(big_iteration) = minimal_distance;
    v3_stat.total_comms(big_iteration) = total_comms;
    v3_stat.succesful_comms_perc(big_iteration) = succesful_comms / total_comms * 100;


end

%save('v3_stat.mat','v3_stat');                                              %save structure as a .mat file