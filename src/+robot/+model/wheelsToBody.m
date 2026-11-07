function twist = wheelsToBody(u, params)
%WHEELSTOBODY Convert right/left wheel speeds to body velocities.
%   u=[omega_R;omega_L] [rad/s]; twist=[v;omega] [m/s;rad/s].

    u = robot.internal.validateColumn(u, 2, 'u');
    params = robot.model.validateParameters(params);
    twist = [(params.r / 2) * (u(1) + u(2)); ...
             (params.r / params.d) * (u(1) - u(2))];
    twist = robot.internal.ensureFinite(twist);
end
