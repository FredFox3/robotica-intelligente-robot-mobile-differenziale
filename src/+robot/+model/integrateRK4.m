function qNext = integrateRK4(q, u, dt, params)
%INTEGRATERK4 Advance one classical fourth-order Runge-Kutta step.
%   U is held constant throughout DT [s]. DT must be nonnegative.
%   Input and output poses are 3-by-1 [m; m; rad]. Output theta is wrapped.
%   A zero step returns the same position and normalized input angle.
%   Intermediate stage angles are not wrapped: RK4 uses continuous stages.
%   No path planning, control, collision checking or simulation loop occurs.

    q = robot.internal.validateColumn(q, 3, 'q');
    u = robot.internal.validateColumn(u, 2, 'u');
    params = robot.model.validateParameters(params);
    dt = robot.internal.validateDuration(dt);
    q(3) = robot.model.normalizeAngle(q(3));
    if dt == 0
        qNext = q;
        return;
    end

    k1 = robot.model.derivative(q, u, params);
    stage = robot.internal.ensureFinite(q + (dt / 2) * k1);
    k2 = robot.model.derivative(stage, u, params);
    stage = robot.internal.ensureFinite(q + (dt / 2) * k2);
    k3 = robot.model.derivative(stage, u, params);
    stage = robot.internal.ensureFinite(q + dt * k3);
    k4 = robot.model.derivative(stage, u, params);

    qNext = robot.internal.ensureFinite(q + ...
        (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4));
    qNext(3) = robot.model.normalizeAngle(qNext(3));
end
