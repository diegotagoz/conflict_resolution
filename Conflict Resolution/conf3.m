num_robots = 20;
position_distance = 5;                                                      %minimal distance between created UAV positions/destinations.

seed = 7;                                                                   %seed for the random numbers generator
rng(seed);

%initializations
robot_pos = zeros(num_robots,2);
robot_dest = zeros(num_robots,2);
robot_priority = zeros(num_robots,1);

for i = 1:num_robots
    
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








