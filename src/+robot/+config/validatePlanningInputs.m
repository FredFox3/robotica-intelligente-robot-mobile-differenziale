function [cfg, scene] = validatePlanningInputs(cfg, scene)
%VALIDATEPLANNINGINPUTS Validate robot settings and obstacle geometry.
%   Pose collisions are checked separately by planRRT.

    if ~isstruct(cfg) || ~isscalar(cfg) || ...
            ~all(isfield(cfg, {'robot', 'limits', 'geometry', 'planning'}))
        error('RobotProject:InvalidPlanningConfig', 'Incomplete planning configuration.');
    end
    cfg.robot = robot.model.validateParameters(cfg.robot);
    requireScalar(cfg.robot, 'bodyRadius', 0, Inf, true, false);
    if cfg.robot.bodyRadius < hypot(cfg.robot.d/2, cfg.robot.r)
        error('RobotProject:InvalidPlanningConfig', ...
            'The enclosing disc must contain at least the ideal wheel geometry.');
    end
    requireScalar(cfg.limits, 'wheel', 0, Inf, true, false);
    requireScalar(cfg.limits, 'v', 0, Inf, true, false);
    requireScalar(cfg.limits, 'omega', 0, Inf, true, false);
    requireScalar(cfg.geometry, 'margin', 0, Inf, false, false);
    requireScalar(cfg.geometry, 'tolerance', 0, Inf, true, false);
    requireScalar(cfg.geometry, 'maxSpatialStep', 0, Inf, true, false);
    requireScalar(cfg.geometry, 'maxDepth', 0, 20, false, true);
    p = cfg.planning;
    requireScalar(p, 'seed', 0, 2^32-1, false, true);
    requireScalar(p, 'maxIterations', 0, 1e6, false, true);
    requireScalar(p, 'maxSampleAttempts', 1, 1e6, false, true);
    requireScalar(p, 'numCandidates', 3, 1000, false, true);
    requireScalar(p, 'goalBias', 0, 1, false, false);
    names = {'headingWeight', 'connectDistance', 'duplicateDistance', ...
        'duplicateAngle', 'goalPositionTolerance', 'goalAngleTolerance', ...
        'sampleDistance', 'sampleAngle'};
    for index = 1:numel(names)
        requireScalar(p, names{index}, 0, Inf, true, false);
    end
    if ~isfield(p, 'durationRange') || ~finiteArray(p.durationRange) || ...
            ~isequal(size(p.durationRange), [1, 2]) || ...
            p.durationRange(1) <= 0 || p.durationRange(2) < p.durationRange(1)
        error('RobotProject:InvalidPlanningConfig', ...
            'durationRange must be [positive minimum, maximum >= minimum].');
    end
    groups = {'limits', 'geometry', 'planning'};
    for groupIndex = 1:numel(groups)
        group = groups{groupIndex};
        fields = fieldnames(cfg.(group));
        for fieldIndex = 1:numel(fields)
            field = fields{fieldIndex};
            if isfloat(cfg.(group).(field))
                cfg.(group).(field) = double(cfg.(group).(field));
            end
        end
    end
    cfg.robot.bodyRadius = double(cfg.robot.bodyRadius);
    if ~isstruct(scene) || ~isscalar(scene) || ...
            ~all(isfield(scene, {'bounds', 'circles', 'polygons', 'q0', 'qf'}))
        error('RobotProject:InvalidScene', 'Incomplete scene.');
    end
    if ~finiteArray(scene.bounds) || ~isequal(size(scene.bounds), [2, 2]) || ...
            any(scene.bounds(:, 2) <= scene.bounds(:, 1))
        error('RobotProject:InvalidScene', 'Bounds must be [xmin xmax; ymin ymax].');
    end
    if ~finiteArray(scene.circles) || ~ismatrix(scene.circles) || ...
            size(scene.circles, 2) ~= 3 || any(scene.circles(:, 3) <= 0)
        error('RobotProject:InvalidScene', 'Circles must be Nc-by-3 with positive radii.');
    end
    if ~iscell(scene.polygons)
        error('RobotProject:InvalidScene', 'Polygons must be a cell array.');
    end
    scene.bounds = double(scene.bounds);
    scene.circles = double(scene.circles);
    robot.internal.ensureFinite(diff(scene.bounds, 1, 2));
    tol = cfg.geometry.tolerance;
    for index = 1:numel(scene.polygons)
        vertices = scene.polygons{index};
        if ~finiteArray(vertices) || ~ismatrix(vertices) || ...
                size(vertices, 2) ~= 2 || size(vertices, 1) < 3
            error('RobotProject:InvalidScene', 'Each polygon needs at least three XY vertices.');
        end
        vertices = double(vertices);
        n = size(vertices, 1);
        next = [2:n, 1];
        edges = vertices(next, :) - vertices;
        lengths = hypot(edges(:, 1), edges(:, 2));
        % Translate first to reduce cancellation in the area formula.
        local = vertices - vertices(1, :);
        area2 = sum(local(:, 1).*local(next, 2) - local(:, 2).*local(next, 1));
        robot.internal.ensureFinite([lengths; area2]);
        if any(lengths <= tol) || abs(area2) <= tol * sum(lengths)
            error('RobotProject:InvalidScene', 'Polygon is degenerate or repeats its closing vertex.');
        end
        for a = 1:n
            b = mod(a, n) + 1;
            c = mod(b, n) + 1;
            incoming = vertices(b, :) - vertices(a, :);
            outgoing = vertices(c, :) - vertices(b, :);
            if abs(cross2(incoming, outgoing)) <= tol * max(lengths(a), lengths(b)) ...
                    && dot(incoming, outgoing) < 0
                error('RobotProject:InvalidScene', 'Adjacent polygon edges overlap.');
            end
            for other = a+1:n
                if other == b || (a == 1 && other == n)
                    continue;
                end
                otherNext = mod(other, n) + 1;
                if segmentsMeet(vertices(a, :), vertices(b, :), ...
                        vertices(other, :), vertices(otherNext, :), tol)
                    error('RobotProject:InvalidScene', 'Polygon is not simple.');
                end
            end
        end
        scene.polygons{index} = vertices;
    end
    scene.q0 = robot.internal.validateColumn(scene.q0, 3, 'scene.q0');
    scene.qf = robot.internal.validateColumn(scene.qf, 3, 'scene.qf');
    scene.q0(3) = robot.model.normalizeAngle(scene.q0(3));
    scene.qf(3) = robot.model.normalizeAngle(scene.qf(3));
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    robot.internal.ensureFinite(radius);
    if any(diff(scene.bounds, 1, 2) <= 2 * radius)
        error('RobotProject:InvalidScene', 'Workspace is too small for the robot and margin.');
    end
end

function requireScalar(s, name, lower, upper, strictLower, integer)
    if ~isstruct(s) || ~isscalar(s) || ~isfield(s, name)
        error('RobotProject:InvalidPlanningConfig', 'Missing configuration field %s.', name);
    end
    value = s.(name);
    if ~finiteArray(value) || ~isscalar(value) || value < lower || ...
            value > upper || (strictLower && value == lower) || ...
            (integer && value ~= floor(value))
        error('RobotProject:InvalidPlanningConfig', 'Invalid configuration field %s.', name);
    end
end

function yes = finiteArray(value)
    yes = isfloat(value) && isreal(value) && ~issparse(value) && ...
        all(isfinite(value(:)));
end

function value = cross2(a, b)
    value = robot.internal.ensureFinite(a(1)*b(2) - a(2)*b(1));
end

function yes = segmentsMeet(a, b, c, d, tol)
    if any(max(a, b) + tol < min(c, d)) || any(max(c, d) + tol < min(a, b))
        yes = false;
        return;
    end
    ab = b-a;
    cd = d-c;
    side1 = cross2(ab, c-a) / norm(ab);
    side2 = cross2(ab, d-a) / norm(ab);
    side3 = cross2(cd, a-c) / norm(cd);
    side4 = cross2(cd, b-c) / norm(cd);
    separated = (side1 > tol && side2 > tol) || (side1 < -tol && side2 < -tol) || ...
        (side3 > tol && side4 > tol) || (side3 < -tol && side4 < -tol);
    yes = ~separated;
end
