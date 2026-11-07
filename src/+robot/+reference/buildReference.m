function reference = buildReference(plan, cfg)
%BUILDREFERENCE Validate and assemble the ordered RRT reference.
%   Samples include in-place rotations; cfg geometry and limits must be validated.

    cfg = robot.control.validateConfig(cfg);
    if ~isstruct(plan) || ~isscalar(plan) || ~isfield(plan, 'status') || ...
            ~strcmp(plan.status, 'success') || ~isfield(plan, 'path') || ...
            ~isstruct(plan.path) || ~isscalar(plan.path)
        error('RobotProject:InvalidPlan', 'A successful RRT plan is required.');
    end
    p = plan.path;
    names = {'states', 'controls', 'durations', 'samples', 'time', 'segment'};
    for index = 1:numel(names)
        if ~isfield(p, names{index}) || ~isfloat(p.(names{index})) || ...
                ~isreal(p.(names{index})) || issparse(p.(names{index})) || ...
                any(~isfinite(p.(names{index})(:)))
            error('RobotProject:InvalidPlan', 'Missing or invalid path array.');
        end
        p.(names{index}) = double(p.(names{index}));
    end
    m = size(p.controls, 1);
    k = size(p.samples, 1);
    % Keep the no-motion duration array as a 0-by-1 column.
    if m == 0 && isempty(p.durations)
        p.durations = zeros(0, 1);
    end
    if ~isequal(size(p.controls), [m, 2]) || ...
            ~isequal(size(p.states), [m+1, 3]) || ...
            ~isequal(size(p.durations), [m, 1]) || any(p.durations <= 0) || ...
            k < 1 || ~isequal(size(p.samples), [k, 3]) || ...
            ~isequal(size(p.time), [k, 1]) || ~isequal(size(p.segment), [k, 1]) || ...
            p.time(1) ~= 0 || p.segment(1) ~= 0 || any(diff(p.time) <= 0) || ...
            any(diff(p.segment) < 0) || any(p.segment ~= floor(p.segment)) || ...
            any(p.segment(2:end) < 1 | p.segment(2:end) > m) || ...
            (m == 0 && k ~= 1)
        error('RobotProject:InvalidPlan', 'Inconsistent path dimensions or ordering.');
    end
    positionTolerance = cfg.planning.goalPositionTolerance;
    angleTolerance = cfg.planning.goalAngleTolerance;
    if ~samePose(p.samples(1, :)', p.states(1, :)')
        error('RobotProject:InvalidPlan', 'Initial sample differs from the root.');
    end
    reference = struct('poses', p.samples, 'segment', p.segment, ...
        'nominalTime', p.time, 'states', p.states, 'controls', p.controls, ...
        'durations', p.durations, 'goal', p.states(end, :)', ...
        'rotation', false(m, 1), 'chordError', zeros(m, 1));
    startTime = 0;
    for edge = 1:m
        q0 = p.states(edge, :)';
        u = p.controls(edge, :)';
        twist = robot.model.wheelsToBody(u, cfg.robot);
        endpoint = robot.model.propagateConstant(q0, u, p.durations(edge), cfg.robot);
        if ~samePose(endpoint, p.states(edge+1, :)') || ...
                max(abs(u)) > cfg.limits.wheel+1e-12 || twist(1) < -1e-12 || ...
                twist(1) > cfg.limits.v+1e-12 || abs(twist(2)) > cfg.limits.omega+1e-12
            error('RobotProject:InvalidPlan', 'Invalid primitive endpoint or velocity limits.');
        end
        rows = find(p.segment == edge);
        if isempty(rows)
            error('RobotProject:InvalidPlan', 'A primitive has no reference samples.');
        end
        elapsed = p.time(rows)-startTime;
        timeTolerance = 1e-10*max(1, startTime+p.durations(edge));
        if any(elapsed <= 0) || any(elapsed > p.durations(edge)+timeTolerance) || ...
                abs(elapsed(end)-p.durations(edge)) > timeTolerance
            error('RobotProject:InvalidPlan', 'Sample time does not match its primitive.');
        end
        intervals = diff([0; elapsed]);
        if any(abs(twist(2))*intervals > cfg.planning.sampleAngle+1e-10) || ...
                any(abs(twist(1))*intervals > cfg.planning.sampleDistance+1e-10)
            error('RobotProject:InvalidPlan', 'Reference sampling is too coarse.');
        end
        for sample = 1:numel(rows)
            pose = robot.model.propagateConstant(q0, u, elapsed(sample), cfg.robot);
            if ~samePose(pose, p.samples(rows(sample), :)')
                error('RobotProject:InvalidPlan', 'A sample is not on its motion primitive.');
            end
        end
        reference.rotation(edge) = twist(1) == 0;
        % Hausdorff bound between circular arc and its sampled chords.
        reference.chordError(edge) = abs(twist(1)*twist(2))*max(intervals)^2/8 + positionTolerance;
        startTime = startTime + p.durations(edge);
    end
    if abs(p.time(end)-startTime) > 1e-10*max(1, startTime)
        error('RobotProject:InvalidPlan', 'Nominal duration mismatch.');
    end

    function same = samePose(a, b)
        same = norm(a(1:2)-b(1:2)) <= positionTolerance && ...
            abs(robot.model.normalizeAngle(a(3)-b(3))) <= angleTolerance;
    end
end
