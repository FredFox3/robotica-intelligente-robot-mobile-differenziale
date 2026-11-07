function [valid, qEnd, lowerBound, reason] = checkPrimitive(q, u, duration, scene, cfg)
%CHECKPRIMITIVE Certify an entire constant-command differential-drive motion.
%   SCENE and CFG must first pass validatePlanningInputs. Direct calls must
%   respect this precondition. Returns a conservative clearance lower bound.
%   Ambiguous intervals at the depth limit are rejected, never accepted.

    q = robot.internal.validateColumn(q, 3, 'q');
    u = robot.internal.validateColumn(u, 2, 'u');
    duration = robot.internal.validateDuration(duration);
    twist = robot.model.wheelsToBody(u, cfg.robot);
    valid = false;
    qEnd = q;
    lowerBound = -Inf;
    reason = 'command_limits';
    velocityTolerance = 1e-12;
    if max(abs(u)) > cfg.limits.wheel + velocityTolerance || ...
            twist(1) < -velocityTolerance || twist(1) > cfg.limits.v + velocityTolerance || ...
            abs(twist(2)) > cfg.limits.omega + velocityTolerance
        return;
    end
    qEnd = robot.model.propagateConstant(q, u, duration, cfg.robot);
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    c0 = robot.geometry.clearance(q(1:2), scene, radius);
    c1 = robot.geometry.clearance(qEnd(1:2), scene, radius);
    reason = 'uncertified';
    [valid, lowerBound] = checkInterval(0, duration, c0, c1, 0);
    if valid
        reason = 'free';
    end

    function [free, bound] = checkInterval(a, b, ca, cb, depth)
        free = false;
        bound = -Inf;
        tol = cfg.geometry.tolerance;
        if min(ca, cb) <= tol
            reason = 'collision';
            return;
        end
        lengthBound = abs(twist(1)) * (b-a);
        if lengthBound <= cfg.geometry.maxSpatialStep && min(ca, cb) > lengthBound/2 + tol
            bound = min(ca, cb) - lengthBound/2;
            free = true;
            return;
        end
        if depth >= cfg.geometry.maxDepth
            return;
        end
        mid = a + (b-a)/2;
        if mid == a || mid == b
            return;
        end
        pose = robot.model.propagateConstant(q, u, mid, cfg.robot);
        cm = robot.geometry.clearance(pose(1:2), scene, radius);
        [leftFree, leftBound] = checkInterval(a, mid, ca, cm, depth+1);
        if ~leftFree
            return;
        end
        [rightFree, rightBound] = checkInterval(mid, b, cm, cb, depth+1);
        free = rightFree;
        if free
            bound = min(leftBound, rightBound);
        end
    end
end
