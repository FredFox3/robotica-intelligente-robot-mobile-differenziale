function connection = connectGoal(q, scene, cfg)
%CONNECTGOAL Try a validated rotate/translate/rotate differential connection.
%   Inputs SCENE/CFG must have passed validatePlanningInputs. A rejected
%   connection contains no partially accepted path. No state is snapped.

    q = robot.internal.validateColumn(q, 3, 'q');
    connection = struct('success', false, 'states', q', ...
        'controls', zeros(0, 2), 'durations', zeros(0, 1), ...
        'clearances', zeros(0, 1));
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    if min(robot.geometry.clearance(q(1:2), scene, radius), ...
            robot.geometry.clearance(scene.qf(1:2), scene, radius)) <= cfg.geometry.tolerance
        reject();
        return;
    end
    positionDelta = scene.qf(1:2) - q(1:2);
    distance = norm(positionDelta);
    if distance > cfg.planning.goalPositionTolerance
        targetHeading = atan2(positionDelta(2), positionDelta(1));
        if ~rotateTo(targetHeading)
            reject();
            return;
        end
        [u, applied] = robot.planning.boundedCommand([cfg.limits.v; 0], cfg);
        if ~appendPrimitive(u, distance/applied(1))
            reject();
            return;
        end
    end
    if ~rotateTo(scene.qf(3))
        reject();
        return;
    end
    connection.success = norm(q(1:2)-scene.qf(1:2)) <= cfg.planning.goalPositionTolerance && ...
        abs(robot.model.normalizeAngle(q(3)-scene.qf(3))) <= cfg.planning.goalAngleTolerance;
    if ~connection.success
        reject();
    end

    function ok = rotateTo(heading)
        delta = robot.model.normalizeAngle(heading-q(3));
        ok = true;
        if abs(delta) <= cfg.planning.goalAngleTolerance
            return;
        end
        [command, actual] = robot.planning.boundedCommand( ...
            [0; sign(delta)*cfg.limits.omega], cfg);
        ok = appendPrimitive(command, abs(delta)/abs(actual(2)));
    end

    function ok = appendPrimitive(command, duration)
        [ok, endpoint, bound] = robot.geometry.checkPrimitive(q, command, duration, scene, cfg);
        if ok
            connection.controls(end+1, :) = command';
            connection.durations(end+1, 1) = duration;
            connection.clearances(end+1, 1) = bound;
            connection.states(end+1, :) = endpoint';
            q = endpoint;
        end
    end

    function reject()
        connection.success = false;
        connection.states = zeros(0, 3);
        connection.controls = zeros(0, 2);
        connection.durations = zeros(0, 1);
        connection.clearances = zeros(0, 1);
    end
end
