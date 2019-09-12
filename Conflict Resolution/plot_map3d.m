%Dibuja en la figura 'nfigure' el mapa 3D binario con transparencia
%'alpha_val', F es la isosurface asociada al mapa.
function plot_map3d(map, alpha_val, nfigure, F)

    figure(nfigure); 
    clf;
    
    if nargin < 4
        F = isosurface(map, 0.5 ); %To make the code faster when it is iterative.
    end
    p = patch(F);
    isonormals( map,p );
    set(p, 'FaceColor', 'black', 'EdgeColor', 'none');
    box on;
    lighting phong;
    alpha(alpha_val);           %transparencia: 0 es transpartente, 1 es solido
    view(3);
end