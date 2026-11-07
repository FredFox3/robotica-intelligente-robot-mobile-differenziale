function u = bodyToWheels(twist, params)
%BODYTOWHEELS Convert body velocities to right/left wheel angular velocities.
%   TWIST = [v; omega] [m/s; rad/s]; U = [omega_R; omega_L] [rad/s].
%   PARAMS.R is wheel radius and PARAMS.D is full wheel separation [m].
%   Saturation and actuator limits are applied by robot.planning.boundedCommand.

    twist = robot.internal.validateColumn(twist, 2, 'twist');
    params = robot.model.validateParameters(params);
    halfTurnVelocity = (params.d / 2) * twist(2);
    u = [(twist(1) + halfTurnVelocity) / params.r; ...
         (twist(1) - halfTurnVelocity) / params.r];
    u = robot.internal.ensureFinite(u);
end
