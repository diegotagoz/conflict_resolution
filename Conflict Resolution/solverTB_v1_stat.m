%Simulation script for UAV Collision Solver v1. Path planning by Fast Marching method and Collision Solving by replanning the path itroducing the other UAV as obstacle also with FM. Uses the UAV class 'UAV_Robot_v1'. Finish when all UAVs arrive their destination or when the simulation reach the 1000 iterations. Intended to extract stadistical data, so it's optimized for computation time, without graphic results. It can't run itself, you need to run the script "stat_v1.m" and it will use this script.
tic;                                                                        %start timer                                  

%obstacle map initialization
Wo = zeros(150,100,40);

%iniziatization of statistical parameters
arrived_robots = zeros(1,num_robots);
collisions = 0;
posible_collisions_detected = 0;
collisions_avoided = 0;
minimal_distance = Inf;

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

W = bwdist(Wo);

%Plot city
% map = Wo;
% [nn,mm,ll] = size(map);
% Woo = map(2:nn-1,2:mm-1,1:ll-1);
% WooF = isosurface(Woo,0.5);

% plot_map3d(Woo, 0.1, 1,WooF);
% hold on;

% pause                                                                     %for the user to move the figure window

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
    
    %create UAV object according to his destination altitude
    if end_point(3,1) <= (zref_low + zref_high) / 2
        r(i) = UAV_Robot_v1(start_point, W, robot_priority(i), zref_low);
    else
        r(i) = UAV_Robot_v1(start_point, W, robot_priority(i), zref_high);
    end
    
    r(i) = r(i).findPath(end_point);                                        %path planning
    %r(i).plot;                                                             %plot UAV
end

%UAVs moving loop
iteration = 1;
while toc <= timeout
    
    iteration;
    
    %make the UAVs move (if they haven't reached their destination)
    for i=1:num_robots
            if norm(r(i).destination-r(i).position) > arrive_tolerance
                r(i) = r(i).move(steps);                                      
            end
    end
        
    %double loop for checking posible future collisions
    for ii=1:num_robots                                                                      
        for jj=ii+1:num_robots
            %distance between the 2 UAVs 
            d_rr = sqrt(    (r(ii).position(1) - r(jj).position(1))^2 + ...  
                            (r(ii).position(2) - r(jj).position(2))^2 + ...
                            (r(ii).position(3) - r(jj).position(3))^2);
                            
            if(d_rr < distance_tolerance)
                
                %if so, change the minimal_distance parameter
                if d_rr < minimal_distance
                    minimal_distance = d_rr;
                end
                                
                %collision wathcdog
                if (d_rr < critical_distance)
                    collisions = collisions +1;                             %increment statistical parameter
                    %disp(['REAL COLLISION DETECTED BETWEEN UAV ' num2str(ii) ' and UAV ' num2str(jj)]);
                end
                
                %disp(['Posible collision detected between robots ' num2str(ii) ' and ' num2str(jj) ' with a separation of ' num2str(d_rr) ' cells.']); 
                
                posible_collisions_detected = posible_collisions_detected+1; %increment statistical parameter

                %FMredo

                %disp("Solving with method 1: FMredo");
                
                %Copy obstacle map and introduce the other UAV as an obstacle (collision_tolerance parameter)
                Wr = Wo;
                %maxes for avoiding index errors
                Wr(max(round(r(jj).position(1))-collision_tolerance,1):min(round(r(jj).position(1))+collision_tolerance,size(Wr,1)), ...
                   max(round(r(jj).position(2))-collision_tolerance,1):min(round(r(jj).position(2))+collision_tolerance,size(Wr,2)), ...
                   max(round(r(jj).position(3))-collision_tolerance,1):min(round(r(jj).position(3))+collision_tolerance,size(Wr,3))) = 1;
                Wnew = bwdist(Wr);                                          %new distances map

                %replace the robot with a new one, with the new distances map
                start_point = r(ii).position;
                end_point = r(ii).destination;

                r_old = r(ii);

                if end_point(3,1) <= (zref_low + zref_high) / 2
                    r(ii) = UAV_Robot_v1(start_point, Wnew, robot_priority(i), zref_low);
                else
                    r(ii) = UAV_Robot_v1(start_point, Wnew, robot_priority(i), zref_high);
                end

                r(ii).path_done = r_old.path_done;
                r(ii) = r(ii).findPath(end_point);                          %path planning of the replaced UAV
                
                if size(r(ii).path,1) == 3
                    collisions_avoided = collisions_avoided + 1;            %if the path is correct, increment statistical parameter
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
    for i=1:num_robots
        if norm(r(i).destination-r(i).position) <= arrive_tolerance && arrived_robots(i) == 0
            arrived_robots(i) = 1;
        end
    end
    
    if sum(arrived_robots) == num_robots
            break;
    end

    iteration = iteration+1;
end


total_runtime = toc                                                         %save current timer value and display it
