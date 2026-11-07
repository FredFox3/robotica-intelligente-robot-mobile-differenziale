function [u, appliedTwist] = boundedCommand(twist, cfg)
%BOUNDEDCOMMAND Scale wheel and body speeds together to preserve curvature.
%   cfg must pass validatePlanningInputs.

    twist = robot.internal.validateColumn(twist, 2, 'twist');
    u = robot.model.bodyToWheels(twist, cfg.robot);
    scale = max([1, max(abs(u))/cfg.limits.wheel, ...
        abs(twist(1))/cfg.limits.v, abs(twist(2))/cfg.limits.omega]);
    u = u / scale;
    appliedTwist = robot.model.wheelsToBody(u, cfg.robot);
end
