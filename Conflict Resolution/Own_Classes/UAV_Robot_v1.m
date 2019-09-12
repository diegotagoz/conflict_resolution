% Class/Object of UAV Robot version 1. Works with the simulation file 'solverTB_v1'.
% Constructor parameters:
% position = [x;y;z]
% W (double map, first potential for FM)
% priority (optional)
% level (referential flight altitude)(optional)
% saturation (for FM)(optional)
% Methods:
% findPath(destination)
% move(steps)
% moveat(new position)
% plot (aux plot options)

classdef UAV_Robot_v1
    
    properties
        
        position;                           %[x;y;z]
        priority = 1;                    	%(1-10) Not used
        level = [];                         %navigational level    
        destination;                       	%[x;y;z]
        path = [];                          %[x1, x2, x3... ; y1, y2, y3.. ; z1, z2, z3,,,,]
        path_done = [];                     %[x1, x2, x3... ; y1, y2, y3.. ; z1, z2, z3,,,,]
        W=[];                               %double map, first potential           
        Wf = [];                            %first potential modified
        D = [];                             %double map, second potential        
        saturation = 0.75;                  %for FM potential map
    end
    
    methods
    
        %constructor method
        function uav = UAV_Robot_v1(position, W, priority, level, saturation)
            if nargin > 2
             uav.priority = priority;
            end
            
            if nargin > 3 
                uav.level = level;
            end
            
            if nargin > 4
                uav.saturation = saturation;
            end
            
            uav.position = position;
            uav.W = W;
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
                Wt = rescale(Double(W));
                Wt = min(Wt, sat);
                uav.Wf = Wt;
            end
            
            %Fast Marching
            [uav.D,S,Q] = perform_fast_marching(uav.Wf, uav.destination, options);  %Compute second potential map
            uav.path = compute_geodesic(uav.D, uav.position);                       %find path
        end
            
        %UAV move method
        function uav = move(uav,steps)
            if size(uav.path,1)==3 && ~isempty(uav.path)                    %only moves if the path is correct (to avoid index errors)
                steps = min(steps, length(uav.path(1,:)));                  %steps of the path to advance
                %move and redefine path
                uav.position = uav.path(:,steps);                           
                uav.path_done = [uav.path_done  uav.position];
                uav.path = uav.path(:,steps+1:end);
            end
        end
        
        %UAV move at method
        function uav = moveAt(uav, newpos)
            uav.position = newpos;
            uav.path_done = [uav.path_done   uav.position];
        end
        
        %UAV plot method, optionally receives position plot configuration string('aux' variable)
        function plot(uav,aux)
            %plot position as a green circle
            if nargin > 1                   
                plot3(uav.position(2), uav.position(1), uav.position(3), aux);
            else
                plot3(uav.position(2), uav.position(1), uav.position(3), 'go');
            end
            
            %plot destination as a red cross
            plot3(uav.destination(2), uav.destination(1), uav.destination(3), 'rx');

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