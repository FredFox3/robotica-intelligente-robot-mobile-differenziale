function scene = makeScenario()
%MAKESCENARIO Define the workspace, obstacles and start/goal poses.
%   bounds=[xmin xmax; ymin ymax] in metres; polygon vertices are ordered.

    scene.bounds = [0, 10; 0, 8];
    scene.circles = [3, 2.5, 0.7; 6, 5.5, 0.8];
    scene.polygons = {[4.5, 1; 5.5, 1; 5.5, 4.5; 4.5, 4.5]};
    scene.q0 = [1; 1; 0];
    scene.qf = [9; 7; pi/2];
end
