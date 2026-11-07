function scene = makeScenario()
%MAKESCENARIO Return the mixed circular/polygonal scenario S2 of the design.
%   Bounds have rows [xmin xmax; ymin ymax], distances are in metres.
%   Polygon vertices are ordered; the closing vertex is not repeated.
%   No planning, plotting or simulation is performed here.

    scene.bounds = [0, 10; 0, 8];
    scene.circles = [3, 2.5, 0.7; 6, 5.5, 0.8];
    scene.polygons = {[4.5, 1; 5.5, 1; 5.5, 4.5; 4.5, 4.5]};
    scene.q0 = [1; 1; 0];
    scene.qf = [9; 7; pi/2];
end
