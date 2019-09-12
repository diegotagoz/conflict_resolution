% Class/Object of UAV Robot version 2. Works with the simulation file 'solverTB_v2'.
% Constructor parameters:
% ID: numeric identificator
% position = [x;y;z]
% W: double map, first potential for FM
% priority: optional (default is 1)
% level: referential flight altitude (optional)
% saturation: for FM (optional, default is 0.75)
% Methods:
% findPath(destination): Path planning by Fast Marching Square
% move(steps): 'steps' Moves only if hasn't reached destination, 'steps' is optional velocity to be forced in, not recommended
% moveat(newposition): Forces new position, not recommended
% communicate(mate): Send ADS-B information to the UAV with ID equal to 'mate'
% receive(mate): Receives ADS-B information form the UAV with ID equal to 'mate', evaluates for conflicts and if there's one, tries to solve it
% plot(aux): Plot the UAV position, destination, path and path done. 'aux' is auxiliar and optional plotting options for UAV position.
classdef UAV_Robot_v2
    
    properties
        
        ID;                                 %identificator
        position;                           %[x;y;z]
        priority = 1;                    	%(1-10)
        level = [];                         %navigational level (default is none)    
        destination;                        %[x;y;z]
        path = [];                          %[x1, x2, x3... ; y1, y2, y3.. ; z1, z2, z3,,,,]
        path_done = [];                     %[x1, x2, x3... ; y1, y2, y3.. ; z1, z2, z3,,,,]
        Wo=[]                               %obstacle binary map
        W=[];                               %double map, first potential           
        Wf = [];                            %first potential modified
        D = [];                             %double map, second potential        
        saturation = 0.75;                  %for FM potential map
        velocity = 4;                       %velocity of movements, default/maximum is 4 path elements in each iteration
        %solving parameters
        arrive_tolerance = 2;           	%robot arrived to destination
        collision_tolerance = 2;        	%minimum distance to permit betweeen 2 UAVs (VELmod and FMredo)
        future_pos = 1;                     %future positions of the mate UAV to introduce as obstacle in map (FMredo)
        comprobation_length = 7;            %future steps to consider in collision prevission. Considering velocity
        velocity_lock = 0;                  %locks velocity for not increasing (lasts 1 iteration)
    end
    
    methods
    
        %constructor method
        function uav = UAV_Robot_v2(ID, position, Wo, priority, level, saturation)
            if nargin > 3.
             uav.priority = priority;
            end
            
            if nargin > 4
                uav.level = level;
            end
            
            if nargin > 5
                uav.saturation = saturation;
            end
            
            uav.ID = ID;
            uav.position = position;
            uav.Wo = Wo;
            uav.W = bwdist(uav.Wo);
        end
        
        %path planning method
        function uav = findPath(uav, destination)
            uav.destination = destination;
            W = uav.W;
            sat = uav.saturation;
            
            %options structure for FM function
            options.nb_iter_max = Inf;
            options.Tmax = sum(size(W));
            
            % if there is a referential flight altitude
            if ~isempty(uav.level)
                %refraction --> energy
                zref = uav.level;
                We = zeros(size(W));
                M = 1;
                for i=1:size(W,1)
                    for j=1:size(W,2)
                        for k=1:size(W,3)
                            krel = (abs(zref-k)+0.01)^0.1;
                            We(i,j,k) = 1/(M*krel);
                        end
                    end
                end
                Wt = W.*We;
                Wt = rescale(double(Wt));
                Wt = min(Wt,sat);
                uav.Wf = Wt;
            %if there isn't referential flight altitude
            else
                Wt = rescale(double(W));
                Wt = min(Wt, sat);
                uav.Wf = Wt;
            end
            
            %Fast Marching
            [uav.D,S,Q] = perform_fast_marching(uav.Wf, uav.destination, options);  %Compute second potential map
            uav.path = compute_geodesic(uav.D, uav.position);                       %find path
        end
            
        %UAV move method
        function uav = move(uav,steps)
            if norm(uav.destination-uav.position) > uav.arrive_tolerance && size(uav.path,1)==3 && ~isempty(uav.path)  %only moves if UAV hasn't reached destination and the path is correct (to avoid index errors)
                if nargin > 1
                    vel = steps;
                else
                    vel = uav.velocity;
                end
                vel = min(vel, length(uav.path(1,:)));                      %steps of the path to advance
                %move and redefine path
                uav.position = uav.path(:,vel);
                uav.path_done = [uav.path_done  uav.position];
                uav.path = uav.path(:,vel+1:end);
            end
            uav.velocity_lock = 0;                                          %unlock velocity
        end
        
        %UAV move at method
        function uav = moveAt(uav, newpos)
            uav.position = newpos;
            uav.path_done = [uav.path_done   uav.position];
        end
        
%COMMS
        %send ADS-B to 'mate' method
        function communicate(uav, mate)
            global r;                                                       %global UAV objects vector
            r(mate) = r(mate).receive(uav.ID);
        end
        
        %receive ADS-B from 'mate' method
        function uav = receive(uav, mate)
            global r;                                                       %global UAV objects vector
            %global performance parameters
            global posible_collisions_detected;                             
            global collisions_avoided;
            
            %if I have a higher priority or (equal priority and a longer remaining path), breaks and do nothing
            if r(mate).priority < uav.priority || (r(mate).priority == uav.priority && size(r(mate).path,2) < size(uav.path,2))
                return
            end
            
            %conflict detecting method
            detected = 0;
            %first, check the position distance to the other UAV
            if norm(uav.position - r(mate).position) <= uav.collision_tolerance
                detected = 1;
            else
                %if not, we will check the future positions (only if my path is correct)
                if ~isempty(uav.path)
                    if isempty(r(mate).path)                                %if mate's path is incorrect, I will only consider his actual position
                        mate_probe = r(mate).position;
                    end
                    %j is a vector with the mate's future positions that I will compare to mines
                    j = min(r(mate).velocity,size(r(mate).path,2)):r(mate).velocity:min(uav.comprobation_length * r(mate).velocity,size(r(mate).path,2)); %min for preventing invalid indexes
                    index = 1;                                              %for moving through j
                    % this loops moves through my future positions to compare with j
                    for i = min(uav.velocity,size(uav.path,2)):uav.velocity:min(uav.comprobation_length * uav.velocity,size(uav.path,2))         %min for preventing invalid indexes
                        %if mate's path is correct, the future position is extracted from j
                        if ~isempty(r(mate).path)
                            mate_probe = r(mate).path(:,j(index));
                        end
                        %compare my future positions with mate's future
                        %positions (if it has a correct path) or with his
                        %actual position (if it hasn't a correct path)
                        if norm(uav.path(:,i) - mate_probe) <= uav.collision_tolerance 
                            detected = 1;
                            break;
                        end
                        %increment index for j if there's still available elements
                        if index < length(j)
                            index = index+1;
                        end
                    end
                end
            end
            
            %if a conflict was detected
            if detected == 1
                disp(['Detected possible collision between UAV ' num2str(uav.ID) ' and UAV ' num2str(mate)]);
                posible_collisions_detected = posible_collisions_detected+1;        %increment performance parameter
                
                %VelMod METHOD
                %this loop will try to solve it by reducing velocity until it is solved or it reaches the minimum velocity (1)
                while detected == 1 && uav.velocity > 1  
                    uav.velocity = uav.velocity - 1;                        %decreases velocity
                    
                    %it runs again the conflict detector (only if the path is correct)
                    if ~isempty(uav.path)
                        detected = 0;
                        j = min(r(mate).velocity,size(r(mate).path,2)):min(uav.comprobation_length * r(mate).velocity,size(r(mate).path,2));         %min for preventing invalid indexes
                        index = 1;
                        for i = min(uav.velocity,size(uav.path,2)):uav.velocity:min(uav.comprobation_length * uav.velocity,size(uav.path,2))         %min for preventing invalid indexes
                            if ~isempty(r(mate).path)
                                mate_probe = r(mate).path(:,j(index));
                            end
                            if norm(uav.path(:,i) - mate_probe) <= uav.collision_tolerance
                                detected = 1;
                                break;
                            end
                            if index < length(j)
                                index = index+1;
                            end
                        end
                    end
                end     
                
                if detected == 0                                            %if the conflict is no longer detected, it has been resolved by VelMod
                    disp(['Collision avoided by reducing UAV ' num2str(uav.ID) ' speed to ' num2str(uav.velocity)]);
                    collisions_avoided = collisions_avoided + 1;            %increment performance parameter
                    uav.velocity_lock = 1;                                  %lock velocity from increasing
                else                                                        %if the conflict is still there, VelMod couldn't resolve it
                    disp(['Unable to avoid collision by reducing UAV ' num2str(uav.ID) ' speed.']);
                    uav.velocity = 4;                                       %resets velocity to maximum
                    uav.velocity_lock = 0;                                  %unlocks velocity
                    
                    %FMredo METHOD
                    disp('Trying to avoid collision with FMredo');
                    Wr = uav.Wo;                                            %Temporary binary obstacle map
                    %introduce mate position as obstacle (engrossing by 'collision_tolerance' value). Max/min to avoid index errors.
                    Wr(round(max(r(mate).position(1)-uav.collision_tolerance,1)):round(min(r(mate).position(1)+uav.collision_tolerance,size(Wr,1))), ...                
                       round(max(r(mate).position(2)-uav.collision_tolerance,1)):round(min(r(mate).position(2)+uav.collision_tolerance,size(Wr,2))), ...
                       round(max(r(mate).position(3)-uav.collision_tolerance,1)):round(min(r(mate).position(3)+uav.collision_tolerance,size(Wr,3)))) = 1;
                    
                       
                   %introduce mate future positions (a number of 'future_pos') as obstacles. Max/min to avoid index errors.
                    if size(r(mate).path,1) == 3 && size(r(mate).path,2)>= (uav.future_pos * r(mate).velocity) 
                        for i = r(mate).velocity:r(mate).velocity:uav.future_pos * r(mate).velocity
                            Wr(round(max(r(mate).path(1,i)-uav.collision_tolerance,1)):round(min(r(mate).path(1,i)+uav.collision_tolerance,size(Wr,1))), ...
                               round(max(r(mate).path(2,i)-uav.collision_tolerance,1)):round(min(r(mate).path(2,i)+uav.collision_tolerance,size(Wr,2))), ...
                               round(max(r(mate).path(3,i)-uav.collision_tolerance,1)):round(min(r(mate).path(3,i)+uav.collision_tolerance,size(Wr,3)))) = 1;
                        end
                    end
                    
                    Wnew = bwdist(Wr);                                      %new distances map
                    sat = uav.saturation;
                    options.nb_iter_max = Inf;                              %options structure for FM function
                    options.Tmax = sum(size(Wnew));

                    %calculate temporary first potential map with temporary distances map
                    % if there is a referential flight altitude
                    if ~isempty(uav.level)
                        %refraction --> energy
                        zref = uav.level;
                        We = zeros(size(Wnew));
                        M = 1;
                        for i=1:size(Wnew,1)
                            for j=1:size(Wnew,2)
                                for k=1:size(Wnew,3)
                                    krel = (abs(zref-k)+0.01)^0.1;
                                    We(i,j,k) = 1/(M*krel);
                                end
                            end
                        end
                        Wt = Wnew.*We;
                        Wt = rescale(double(Wt));
                        Wt = min(Wt,sat);
                    %if there isn't referential flight altitude
                    else
                        Wt = rescale(double(Wnew));
                        Wt = min(Wt, sat);
                    end

                    [D,S,Q] = perform_fast_marching(Wt, uav.destination, options);  %Compute temporary second potential map
                    uav.path = compute_geodesic(D, uav.position);                   %find path adn save it as the official path
                    %if this calculated path is correct, increment performance parameter
                    if size(uav.path,1) == 3 && ~isempty(uav.path)                 
                        collisions_avoided = collisions_avoided + 1;
                    end
                end
            else
                %every time a communication is received and it's not detected any conflict, if it's unlocked, the velocity resets.
                if uav.velocity_lock == 0
                    uav.velocity = 4;
                end
            end
        end
        
        %UAV plot method, optionally receives position plot configuration string('aux' variable)
        function plot(uav,aux)
            if nargin > 1
                %plot position with aux
                plot3(uav.position(2), uav.position(1), uav.position(3), aux); 
            else
                %plot position as a green circle
                plot3(uav.position(2), uav.position(1), uav.position(3), 'go');
            end
            text(uav.position(2), uav.position(1), uav.position(3), num2str(uav.ID)); %plot ID in UAV position
            plot3(uav.destination(2), uav.destination(1), uav.destination(3), 'rx'); %plot destination as a red cross

            %plot path (if it's correct) as a blue line
            if size(uav.path,1) == 3 && ~isempty(uav.path)
                plot3(uav.path(2,:), uav.path(1,:), uav.path(3,:),'b-');
            end

            %plot path done (if there's one) as a red line
            if ~isempty(uav.path_done)
                plot3(uav.path_done(2,:), uav.path_done(1,:), uav.path_done(3,:), 'r-');
            end
        end  
    end
end