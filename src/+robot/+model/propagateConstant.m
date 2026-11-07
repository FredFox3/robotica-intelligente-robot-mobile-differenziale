function qNext = propagateConstant(q, u, duration, params)
%PROPAGATECONSTANT Compute the exact pose for constant wheel speeds.
%   The midpoint/sinc formula stays stable as the yaw rate approaches zero.

    q = robot.internal.validateColumn(q, 3, 'q');
    u = robot.internal.validateColumn(u, 2, 'u');
    params = robot.model.validateParameters(params);
    duration = robot.internal.validateDuration(duration);
    q(3) = robot.model.normalizeAngle(q(3));
    if duration == 0
        qNext = q;
        return;
    end

    twist = robot.model.wheelsToBody(u, params);
    increments = robot.internal.ensureFinite(duration * twist);
    halfTurn = increments(2) / 2;
    if abs(halfTurn) < 1e-4
        % sinc_0(z) = sin(z)/z, with its continuous limit at z = 0.
        sincFactor = 1 - halfTurn^2 / 6 + halfTurn^4 / 120;
    else
        sincFactor = sin(halfTurn) / halfTurn;
    end
    midAngle = q(3) + halfTurn;
    displacement = increments(1) * sincFactor;
    qNext = robot.internal.ensureFinite(q + ...
        [displacement * cos(midAngle); ...
         displacement * sin(midAngle); increments(2)]);
    qNext(3) = robot.model.normalizeAngle(qNext(3));
end
