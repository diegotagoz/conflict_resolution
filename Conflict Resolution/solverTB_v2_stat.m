%Simulation script for UAV Collision Solver v2, wich adds distributed control, a priority system, communication simulation, a new detection method and a new solving method. Path planning by Fast Marching method and Collision Solving by Velocity Modification (VelMod) and Fast Marching replanning (FMredo) methods. Uses the UAV class 'UAV_Robot_v2_stat'. Finish when all UAVs arrive their destination or when no UAV moves in a iteration. Intended to extract stadistical data, so it's optimized for computation time, without graphic results. It can't run itself, you need to run the script "stat_v2.m" and it will use this script.
tic;                                                                        

%global variables
global r;
global move_probe;
%statistical parameters
global posible_collisions_detected;
global collisions_avoided_velmod;
global collisions_avoided_fmredo;
global velmod_reduction_accum;

%inizializations
r = UAV_Robot_v2_stat(1,[1;1;1],zeros(2,2,2));                                   
posible_collisions_detected = 0;
collisions_avoided_velmod = 0;
collisions_avoided_fmredo = 0;
velmod_reduction_accum = 0;
Wo = zeros(150,100,40);
arrived_robots = zeros(1,num_robots);
collisions = 0;
iteration = 1;                                                              %iteration counter
minimal_distance = Inf;
global_vel_acum = 0;
total_comms = 0;

%obstacles for closing the environment
Wo(1,:,:) = 1;  
Wo(150,:,:) = 1;
Wo(:,1,:) = 1;
Wo(:,100,:) = 1;
Wo(:,:,1) = 1;
Wo(:,:,40) = 1;

%buildings
Wo(5:10,2:10,1:30)=1; 
Wo(2:10,20:30,1:20)=1; 
Wo(5:10,40:50,1:25)=1; 
Wo(2:10,60:70,1:20)=1; 
Wo(5:10,75:80,1:25)=1; 
Wo(5:10,90:99,1:15)=1; 
Wo(15:25,2:10,1:15)=1; 
Wo(15:25,20:32,1:10)=1; 
Wo(15:25,40:45,1:20)=1; 
Wo(15:25,90:95,1:28)=1; 
Wo(37:42,2:10,1:35)=1; 
Wo(42:52,5:10,1:35)=1; 
Wo(38:52,20:32,1:37)=1; 
Wo(39:51,40:46,1:25)=1; 
Wo(39:51,90:96,1:35)=1; 
Wo(58:70,2:8,1:30)=1; 
Wo(60:67,20:32,1:35)=1; 
Wo(62:67,32:42,1:25)=1; 
Wo(59:69,42:48,1:30)=1;
Wo(62:67,62:70,1:25)=1; 
Wo(59:69,75:80,1:30)=1;
Wo(60:69,91:99,1:30)=1;
Wo(78:85,4:10,1:20)=1; 
Wo(78:88,25:31,1:30)=1; 
Wo(79:87,40:48,1:35)=1; 
Wo(78:85,60:70,1:25)=1; 
Wo(78:88,75:80,1:30)=1; 
Wo(79:87,90:98,1:33)=1; 
Wo(94:100,10:36,1:35)=1;
Wo(94:100,46:86,1:31)=1;
Wo(108:115,5:25,1:25)=1; 
Wo(108:115,36:48,1:29)=1;
Wo(108:115,60:68,1:25)=1; 
Wo(108:112,76:80,1:21)=1;
Wo(109:114,91:96,1:11)=1;
Wo(120:130,2:8,1:30)=1; 
Wo(120:130,20:32,1:35)=1;  
Wo(120:130,42:48,1:30)=1;
Wo(120:130,62:70,1:25)=1; 
Wo(120:130,75:80,1:30)=1;
Wo(120:130,91:99,1:30)=1;

%Plot city
% map = Wo;
% [nn,mm,ll] = size(map);
% Woo = map(2:nn-1,2:mm-1,1:ll-1);
% WooF = isosurface(Woo,0.5);

% plot_map3d(Woo, 0.1, 1,WooF);
% hold on;

% disp('Paused so you can move the map to your best interest. Press any key to continue.');
% pause

for i=1:num_robots
    
    %position
    x = robot_pos(i,1);                                                     
    y = robot_pos(i,2);
    
    k = 3;
    while k<40 && Wo(x,y,k) == 1                                            %altitude in no obstacle
        k = k+1;
    end
    z = k-1;
    
    start_point = [x;y;z];
    
    %destination
    x = robot_dest(i,1);                                                    
    y = robot_dest(i,2);

    k = 3;
    while k<40 && Wo(x,y,k) == 1                                            %altitude in no obstacle
        k = k+1;
    end
    z = k-1;

    end_point = [x;y;z];

%  For activating referential flight altitudes. create UAV object according to his destination altitude
%     if end_point(3,1) <= (zref_low + zref_high) / 2
%         r(i) = UAV_Robot_v2_stat(i, start_point, Wo, robot_priority(i), zref_low);
%     else
%         r(i) = UAV_Robot_v2_stat(i, start_point, Wo, robot_priority(i), zref_high);
%     end

    %Create UAV object with ID equal to i
    r(i) = UAV_Robot_v2_stat(i, start_point, Wo, robot_priority(i));
    
    r(i) = r(i).findPath(end_point);                                        %path planning
%     r(i).plot;                                                              %plot UAV
end

%UAVs moving loop
while 1                                                                     %it will break with internal comprobations
    
    vel_acum = 0;                                                           %reset parameter
    
    move_probe = 0;                                                         %reset UAV move probe
    for i=1:num_robots
        vel_acum = vel_acum + r(i).velocity;                                %accumulate velocities 
        r(i) = r(i).move;                                                   %make the UAVs move (the arrived UAVs won't do anything)                           
        if norm(r(i).destination-r(i).position) <= arrive_tolerance && arrived_robots(i) == 0
            arrived_robots(i) = 1;                                          %acknowledge arrived UAVs 
            r(i).priority = Inf;                                            %priority of arrived robots set to Infinite
        end
    end
    
    global_vel_acum = global_vel_acum + (vel_acum / num_robots);            %accumulate average velocity in statistical parameter
    
    %termination watchdog: if no robot has move, breaks simulation
    if move_probe == 0                                                          
        break;
    end
        
    %double loop for checking if the distance between two UAVs will make them communicate
    for ii=1:num_robots                                                            
        for jj=ii+1:num_robots
               
            distance = norm(r(ii).position - r(jj).position);               %distance between the 2 UAVs
            
            %modify statistical parameter
            if distance < minimal_distance
                minimal_distance = distance;
            end
                
            if( distance < comms_distance)
                
                %collision watchdog
                if (distance < critical_distance)
                    collisions = collisions +1;
%                     disp(['REAL COLLISION DETECTED BETWEEN UAV ' num2str(ii) ' and UAV ' num2str(jj)]);
                end
                
                if arrived_robots(ii) == 0                                  %make 2nd UAV communicate with 1st IF the 1st hasn't reach destination (1st will listen and evaluate)
                    r(jj).communicate(ii);
                    total_comms = total_comms + 1;                          %increment statistical parameter
                end
                
                if arrived_robots(jj) == 0                                  %make 1st UAV communicate with 2nd IF the 2nd hasn't reach destination (2nd will listen and evaluate)
                    r(ii).communicate(jj);
                    total_comms = total_comms + 1;                          %increment statistical parameter
                end                
            end
        end
    end
   
    %plotting
%     plot_map3d(Woo,0.1,1,WooF);                                           %plot map
%     hold on;
%    
%     for ii=1:num_robots                                                   %plot UAVs
%         r(ii).plot;
%     end
    
    %check if all robots have arrived to their destination, if so, breaks simulation
    if sum(arrived_robots) == num_robots
            break;
    end
    iteration = iteration+1;                                                %increment iteration counter
end

total_runtime = toc                                                         %save current timer value and display it
