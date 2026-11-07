function dq = derivative(q, u, params)
%DERIVATIVE Compute the differential-drive state derivative.
%   q=[x;y;theta] at the axle midpoint; u=[omega_R;omega_L] (SI units).

    q = robot.internal.validateColumn(q, 3, 'q');
    twist = robot.model.wheelsToBody(u, params);
    dq = [twist(1) * cos(q(3)); ...
          twist(1) * sin(q(3)); ...
          twist(2)];
    dq = robot.internal.ensureFinite(dq);
end
