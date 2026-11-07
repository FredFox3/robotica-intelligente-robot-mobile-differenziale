function plan = planRRT(scene, cfg)
%PLANRRT Build a differential-drive RRT using constant wheel commands.
%   Exhausting the search budget does not prove that no path exists.

    plan = struct('status', 'invalid_input', 'message', '', ...
        'path', [], 'tree', [], 'iterations', 0, 'seed', []);
    try
        [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    catch exception
        inputErrors = {'RobotProject:InvalidPlanningConfig', 'RobotProject:InvalidScene', ...
            'RobotProject:InvalidParameters', 'RobotProject:InvalidVector'};
        if any(strcmp(exception.identifier, inputErrors))
            plan.message = exception.message;
            return;
        end
        rethrow(exception);
    end
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    rootClearance = robot.geometry.clearance(scene.q0(1:2), scene, radius);
    goalClearance = robot.geometry.clearance(scene.qf(1:2), scene, radius);
    if min(rootClearance, goalClearance) <= cfg.geometry.tolerance
        plan.message = 'Initial or final configuration is in collision (including footprint/margin).';
        return;
    end
    originalRng = rng;
    restoreRng = onCleanup(@() rng(originalRng)); %#ok<NASGU>
    rng(cfg.planning.seed, 'twister');
    plan.seed = cfg.planning.seed;
    capacity = cfg.planning.maxIterations + 4;
    tree.Q = zeros(capacity, 3);
    tree.parent = zeros(capacity, 1);
    tree.Uin = zeros(capacity, 2);
    tree.Tin = zeros(capacity, 1);
    tree.clearance = zeros(capacity, 1);
    tree.cost = zeros(capacity, 1); % Accumulated nominal time, not optimized.
    tree.Q(1, :) = scene.q0';
    tree.clearance(1) = rootClearance;
    nodes = 1;
    terminal = tryGoal(1);
    for iteration = 1:cfg.planning.maxIterations
        if terminal ~= 0
            break;
        end
        plan.iterations = iteration;
        [target, found] = sampleTarget();
        if ~found
            continue;
        end
        [~, near] = min(distances(tree.Q(1:nodes, :), target));
        qNear = tree.Q(near, :)';
        bestDistance = distances(qNear', target);
        best = [];
        for candidate = 1:cfg.planning.numCandidates
            [u, found] = sampleCommand(candidate);
            if ~found
                continue;
            end
            range = cfg.planning.durationRange;
            duration = range(1) + (range(2)-range(1))*rand;
            [valid, endpoint, bound] = robot.geometry.checkPrimitive( ...
                qNear, u, duration, scene, cfg);
            if ~valid
                continue;
            end
            positionDifferences = tree.Q(1:nodes, 1:2) - endpoint(1:2)';
            angleDifferences = robot.model.normalizeAngle(tree.Q(1:nodes, 3)-endpoint(3));
            duplicates = hypot(positionDifferences(:, 1), positionDifferences(:, 2)) ...
                <= cfg.planning.duplicateDistance & ...
                abs(angleDifferences) <= cfg.planning.duplicateAngle;
            if any(duplicates)
                continue;
            end
            distance = distances(endpoint', target);
            % Strict improvement: deterministic first-candidate tie breaking.
            if distance < bestDistance - 1e-12
                bestDistance = distance;
                best = struct('q', endpoint, 'u', u, 'duration', duration, 'bound', bound);
            end
        end
        if isempty(best)
            continue;
        end
        nodes = nodes + 1;
        tree.Q(nodes, :) = best.q';
        tree.parent(nodes) = near;
        tree.Uin(nodes, :) = best.u';
        tree.Tin(nodes) = best.duration;
        tree.clearance(nodes) = best.bound;
        tree.cost(nodes) = tree.cost(near) + best.duration;
        terminal = tryGoal(nodes);
    end
    fields = fieldnames(tree);
    for index = 1:numel(fields)
        tree.(fields{index}) = tree.(fields{index})(1:nodes, :);
    end
    plan.tree = tree;
    if terminal ~= 0
        plan.path = robot.planning.reconstructPath(tree, terminal, cfg);
        plan.status = 'success';
        plan.message = 'A differential path and final pose connection were certified.';
    else
        plan.status = 'budget_exhausted';
        plan.message = 'No certified path found within the iteration budget; infeasibility is not proved.';
    end

    function terminalNode = tryGoal(node)
        terminalNode = 0;
        q = tree.Q(node, :)';
        if norm(q(1:2)-scene.qf(1:2)) > cfg.planning.connectDistance
            return;
        end
        connection = robot.planning.connectGoal(q, scene, cfg);
        if ~connection.success
            return;
        end
        terminalNode = node;
        for edge = 1:numel(connection.durations)
            nodes = nodes + 1;
            tree.Q(nodes, :) = connection.states(edge+1, :);
            tree.parent(nodes) = terminalNode;
            tree.Uin(nodes, :) = connection.controls(edge, :);
            tree.Tin(nodes) = connection.durations(edge);
            tree.clearance(nodes) = connection.clearances(edge);
            tree.cost(nodes) = tree.cost(terminalNode) + connection.durations(edge);
            terminalNode = nodes;
        end
    end

    function [target, found] = sampleTarget()
        target = scene.qf;
        found = true;
        if rand < cfg.planning.goalBias
            return;
        end
        found = false;
        for attempt = 1:cfg.planning.maxSampleAttempts
            xy = scene.bounds(:, 1) + diff(scene.bounds, 1, 2).*rand(2, 1);
            if robot.geometry.clearance(xy, scene, radius) > cfg.geometry.tolerance
                target = [xy; -pi + 2*pi*rand];
                found = true;
                return;
            end
        end
    end

    function [u, found] = sampleCommand(candidate)
        found = true;
        if candidate == 1
            twist = [cfg.limits.v; 0];
        elseif candidate == 2
            twist = [0; cfg.limits.omega];
        elseif candidate == 3
            twist = [0; -cfg.limits.omega];
        else
            found = false;
            for attempt = 1:cfg.planning.maxSampleAttempts
                raw = cfg.limits.wheel*(2*rand(2, 1)-1);
                if sum(raw) >= 0 && any(raw ~= 0)
                    twist = robot.model.wheelsToBody(raw, cfg.robot);
                    found = true;
                    break;
                end
            end
            if ~found
                u = zeros(2, 1);
                return;
            end
        end
        u = robot.planning.boundedCommand(twist, cfg);
    end

    function result = distances(poses, target)
        dx = poses(:, 1)-target(1);
        dy = poses(:, 2)-target(2);
        heading = cfg.planning.headingWeight * ...
            robot.model.normalizeAngle(poses(:, 3)-target(3));
        result = robot.internal.ensureFinite(hypot(hypot(dx, dy), heading));
    end
end
