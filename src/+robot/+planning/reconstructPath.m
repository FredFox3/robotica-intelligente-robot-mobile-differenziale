function path = reconstructPath(tree, lastNode, cfg)
%RECONSTRUCTPATH Recover tree edges and sample their motion primitives.
%   Times are nominal planning times, distinct from closed-loop timing.

    count = size(tree.Q, 1);
    if ~isscalar(lastNode) || lastNode < 1 || lastNode > count || lastNode ~= floor(lastNode)
        error('RobotProject:InvalidTree', 'Invalid terminal node.');
    end
    indices = zeros(count, 1);
    used = 0;
    node = lastNode;
    while node ~= 0
        used = used + 1;
        if used > count
            error('RobotProject:InvalidTree', 'Parent cycle detected.');
        end
        indices(used) = node;
        parent = tree.parent(node);
        if parent < 0 || parent >= node || parent ~= floor(parent)
            error('RobotProject:InvalidTree', 'Parents must precede their children.');
        end
        if node ~= 1 && parent == 0
            error('RobotProject:InvalidTree', 'Disconnected node.');
        end
        node = parent;
    end
    indices = flipud(indices(1:used));
    edges = indices(2:end);
    path.states = tree.Q(indices, :);
    path.controls = tree.Uin(edges, :);
    % Explicit column indexing preserves 0-by-1 shape for a root-only path.
    path.durations = tree.Tin(edges, 1);
    path.clearances = tree.clearance(edges, 1);

    edgeCount = numel(edges);
    steps = zeros(edgeCount, 1);
    for index = 1:edgeCount
        twist = robot.model.wheelsToBody(path.controls(index, :)', cfg.robot);
        steps(index) = max([1, ceil(abs(twist(1))*path.durations(index)/cfg.planning.sampleDistance), ...
            ceil(abs(twist(2))*path.durations(index)/cfg.planning.sampleAngle)]);
    end
    path.samples = zeros(1+sum(steps), 3);
    path.time = zeros(1+sum(steps), 1);
    path.segment = zeros(1+sum(steps), 1);
    path.samples(1, :) = path.states(1, :);
    row = 1;
    startTime = 0;
    for index = 1:edgeCount
        duration = path.durations(index);
        for sample = 1:steps(index)
            elapsed = duration * (sample/steps(index));
            pose = robot.model.propagateConstant(path.states(index, :)', ...
                path.controls(index, :)', elapsed, cfg.robot);
            row = row + 1;
            path.samples(row, :) = pose';
            path.time(row) = startTime + elapsed;
            path.segment(row) = index;
        end
        startTime = startTime + duration;
    end
end
