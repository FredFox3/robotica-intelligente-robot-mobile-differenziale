function qNext = propagateConstant(q, u, duration, params)
%PROPAGATECONSTANT Propagate a constant wheel command analytically.
%   Q is a 3-by-1 pose, U = [omega_R; omega_L], DURATION [s] >= 0.
%   Return the endpoint with theta in [-pi, pi); do not create a path.
%   The midpoint/sinc formula avoids dividing by a near-zero yaw rate.
%   This endpoint alone does not certify collision freedom of a motion.

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
