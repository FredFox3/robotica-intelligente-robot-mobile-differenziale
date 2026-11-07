function u = bodyToWheels(twist, params)
%BODYTOWHEELS Convert body velocities to right/left wheel speeds.
%   twist=[v;omega] [m/s;rad/s]; u=[omega_R;omega_L] [rad/s].

    twist = robot.internal.validateColumn(twist, 2, 'twist');
    params = robot.model.validateParameters(params);
    halfTurnVelocity = (params.d / 2) * twist(2);
    u = [(twist(1) + halfTurnVelocity) / params.r; ...
         (twist(1) - halfTurnVelocity) / params.r];
    u = robot.internal.ensureFinite(u);
end
