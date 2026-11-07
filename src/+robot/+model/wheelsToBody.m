function twist = wheelsToBody(u, params)
%WHEELSTOBODY Convert wheel angular velocities to planar body velocities.
%   U = [omega_R; omega_L] [rad/s], PARAMS.R [m], PARAMS.D [m].
%   TWIST = [v; omega] [m/s; rad/s]. Positive omega is counterclockwise.
%   This conversion neither clips nor saturates the input velocities.

    u = robot.internal.validateColumn(u, 2, 'u');
    params = robot.model.validateParameters(params);
    twist = [(params.r / 2) * (u(1) + u(2)); ...
             (params.r / params.d) * (u(1) - u(2))];
    twist = robot.internal.ensureFinite(twist);
end
