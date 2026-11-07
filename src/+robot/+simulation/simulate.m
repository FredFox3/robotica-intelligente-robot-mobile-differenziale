function log = simulate(scene, plan, cfg)
%SIMULATE Run the ordered waypoint controller with guarded numerical RK4.
%   No automatic planning, plots, animation or file writing.
%   Inputs remain unchanged.
%   Samples of the executed numerical trajectory are in LOG.Q; wheel/body
%   commands apply on [time(k),time(k+1)). REFERENCE is the planned trajectory.

    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    cfg = robot.control.validateConfig(cfg);
    reference = robot.reference.buildReference(plan, cfg);
    positionTol = cfg.planning.goalPositionTolerance;
    angleTol = cfg.planning.goalAngleTolerance;
    if norm(reference.states(1, 1:2)'-scene.q0(1:2)) > positionTol || ...
            abs(robot.model.normalizeAngle(reference.states(1, 3)-scene.q0(3))) > angleTol || ...
            norm(reference.goal(1:2)-scene.qf(1:2)) > positionTol || ...
            abs(robot.model.normalizeAngle(reference.goal(3)-scene.qf(3))) > angleTol
        error('RobotProject:InvalidSimulationInput', 'Plan poses do not match this scene.');
    end
    reference.goal = scene.qf; % Judge success against the actual assigned goal.
    safeRadius = cfg.robot.bodyRadius + cfg.geometry.margin;
    if min(robot.geometry.clearance(scene.q0(1:2), scene, safeRadius), ...
            robot.geometry.clearance(scene.qf(1:2), scene, safeRadius)) <= cfg.geometry.tolerance
        error('RobotProject:InvalidSimulationInput', 'Initial or final pose is occupied.');
    end

    capacity = min(cfg.simulation.maxSteps, ceil(cfg.simulation.maxTime/cfg.integration.dt));
    log.configuration = cfg;
    log.scene = scene;
    log.reference = reference;
    log.time = zeros(capacity+1, 1);
    log.q = zeros(capacity+1, 3);
    log.q(1, :) = scene.q0';
    log.u = zeros(capacity, 2);
    log.twist = zeros(capacity, 2);
    log.clearanceBound = zeros(capacity, 1);
    log.integrationErrorBound = zeros(capacity, 1);
    log.rho = zeros(capacity+1, 1);
    log.alpha = zeros(capacity+1, 1);
    log.orientationError = zeros(capacity+1, 1);
    log.trackingError = zeros(capacity+1, 1);
    log.trackingApproximationBound = zeros(capacity+1, 1);
    log.referenceIndex = zeros(capacity+1, 1);
    log.mode = cell(capacity+1, 1);
    log.status = 'step_limit';
    log.message = 'Maximum number of accepted simulation steps reached.';
    state = [];
    q = scene.q0;
    for row = 1:capacity+1
        [u, state, info] = robot.control.step(q, reference, state, cfg);
        [tracking, approximation] = robot.reference.trackingError(q, reference, state.index);
        log.rho(row) = info.rho;
        log.alpha(row) = info.alpha;
        log.orientationError(row) = info.orientationError;
        log.trackingError(row) = tracking;
        log.trackingApproximationBound(row) = approximation;
        log.referenceIndex(row) = state.index;
        log.mode{row} = info.mode;
        if tracking+approximation > cfg.simulation.maxTrackingError
            log.status = 'tracking_blocked';
            log.message = 'Deviation from the active primitive exceeds the allowed tube.';
            break;
        end
        if info.done
            log.status = 'goal_reached';
            log.message = 'Position and orientation goal tolerances reached.';
            break;
        end
        if log.time(row) >= cfg.simulation.maxTime - 16*eps(max(1, cfg.simulation.maxTime))
            log.status = 'timeout';
            log.message = 'Maximum simulated time reached.';
            break;
        end
        if row > capacity
            break;
        end
        nextTime = min(row*cfg.integration.dt, cfg.simulation.maxTime);
        dt = nextTime-log.time(row);
        if dt <= 0
            log.status = 'numerical_failure';
            log.message = 'Time no longer advances at floating-point resolution.';
            break;
        end
        try
            actual = info.appliedTwist;
            % RK4 is Simpson quadrature for position with constant wheel input.
            % Each coordinate has error <= |v|*|omega|^4*dt^5/2880.
            errorBound = robot.internal.ensureFinite(sqrt(2)*abs(actual(1))* ...
                abs(actual(2))^4*dt^5/2880 + cfg.simulation.positionRoundoff);
            guardConfig = cfg;
            guardConfig.geometry.margin = cfg.geometry.margin + errorBound;
            [free, exactEndpoint, clearanceBound, reason] = robot.geometry.checkPrimitive( ...
                q, u, dt, scene, guardConfig);
            if ~free
                log.status = 'tracking_blocked';
                log.message = ['Command rejected before integration: ', reason];
                break;
            end
            candidate = robot.model.integrateRK4(q, u, dt, cfg.robot);
            if norm(candidate(1:2)-exactEndpoint(1:2)) > errorBound || ...
                    abs(robot.model.normalizeAngle(candidate(3)-exactEndpoint(3))) > ...
                    cfg.simulation.angleRoundoff
                log.status = 'numerical_failure';
                log.message = 'RK4 endpoint exceeds its analytical comparison tolerance.';
                break;
            end
            if robot.geometry.clearance(candidate(1:2), scene, safeRadius) <= cfg.geometry.tolerance
                log.status = 'tracking_blocked';
                log.message = 'Numerical endpoint is not free; step was not accepted.';
                break;
            end
        catch exception
            if ~strcmp(exception.identifier, 'RobotProject:NumericalFailure')
                rethrow(exception);
            end
            log.status = 'numerical_failure';
            log.message = exception.message;
            break;
        end
        log.u(row, :) = u';
        log.twist(row, :) = actual';
        log.clearanceBound(row) = clearanceBound;
        log.integrationErrorBound(row) = errorBound;
        q = candidate;
        log.q(row+1, :) = q';
        log.time(row+1) = nextTime;
    end

    stateFields = {'q', 'time', 'rho', 'alpha', 'orientationError', ...
        'trackingError', 'trackingApproximationBound', 'referenceIndex', 'mode'};
    for index = 1:numel(stateFields)
        name = stateFields{index};
        log.(name) = log.(name)(1:row, :);
    end
    intervalFields = {'u', 'twist', 'clearanceBound', 'integrationErrorBound'};
    for index = 1:numel(intervalFields)
        name = intervalFields{index};
        log.(name) = log.(name)(1:row-1, :);
    end
    log.finalState = log.q(end, :)';
    log.finalPositionError = norm(log.finalState(1:2)-scene.qf(1:2));
    log.finalAngleError = abs(robot.model.normalizeAngle(log.finalState(3)-scene.qf(3)));
    log.goalReached = strcmp(log.status, 'goal_reached');
end
