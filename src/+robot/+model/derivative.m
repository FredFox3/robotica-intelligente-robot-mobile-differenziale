function dq = derivative(q, u, params)
%DERIVATIVE Evaluate the ideal no-slip differential-drive kinematics.
%   Q = [x; y; theta] [m; m; rad], U = [omega_R; omega_L] [rad/s].
%   DQ = [v*cos(theta); v*sin(theta); omega], a 3-by-1 column.
%   The state is referenced to the midpoint of the wheel axle.

    q = robot.internal.validateColumn(q, 3, 'q');
    twist = robot.model.wheelsToBody(u, params);
    dq = [twist(1) * cos(q(3)); ...
          twist(1) * sin(q(3)); ...
          twist(2)];
    dq = robot.internal.ensureFinite(dq);
end
