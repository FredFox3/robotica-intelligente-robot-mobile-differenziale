function [u, appliedTwist] = boundedCommand(twist, cfg)
%BOUNDEDCOMMAND Apply one common scale to satisfy wheel and body speed limits.
%   CFG must have passed validatePlanningInputs. Preserves curvature.
%   Forward-only planning is enforced separately by candidate generation.

    twist = robot.internal.validateColumn(twist, 2, 'twist');
    u = robot.model.bodyToWheels(twist, cfg.robot);
    scale = max([1, max(abs(u))/cfg.limits.wheel, ...
        abs(twist(1))/cfg.limits.v, abs(twist(2))/cfg.limits.omega]);
    u = u / scale;
    appliedTwist = robot.model.wheelsToBody(u, cfg.robot);
end
