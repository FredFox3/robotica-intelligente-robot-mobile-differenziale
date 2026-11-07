function log = validateLog(log)
%VALIDATELOG Check actual simulation data before analysis or visualization.
%   Reject empty/misaligned/nonfinite arrays. Do not generate replacement data.
%   N poses correspond to N-1 held wheel commands; N=1 is a valid no-motion log.

    required = {'configuration', 'scene', 'reference', 'q', 'time', 'u', 'twist', ...
        'rho', 'alpha', 'orientationError', 'trackingError', 'trackingApproximationBound', ...
        'referenceIndex', 'mode', 'clearanceBound', 'integrationErrorBound', ...
        'status', 'message', 'goalReached', 'finalState', 'finalPositionError', 'finalAngleError'};
    if ~isstruct(log) || ~isscalar(log) || ~all(isfield(log, required))
        error('RobotProject:InvalidLog', 'An actual complete simulation log is required.');
    end
    [log.configuration, log.scene] = robot.config.validatePlanningInputs(log.configuration, log.scene);
    log.configuration = robot.control.validateConfig(log.configuration);
    cfg = log.configuration;
    n = size(log.q, 1);
    if n < 1
        error('RobotProject:InvalidLog', 'The executed trajectory must contain at least one pose.');
    end
    log.q = array('q', n, 3);
    log.time = array('time', n, 1);
    log.u = array('u', n-1, 2);
    log.twist = array('twist', n-1, 2);
    stateFields = {'rho', 'alpha', 'orientationError', 'trackingError', ...
        'trackingApproximationBound', 'referenceIndex'};
    for index = 1:numel(stateFields)
        name = stateFields{index};
        log.(name) = array(name, n, 1);
    end
    log.clearanceBound = array('clearanceBound', n-1, 1);
    log.integrationErrorBound = array('integrationErrorBound', n-1, 1);
    log.finalState = array('finalState', 3, 1);
    log.finalPositionError = array('finalPositionError', 1, 1);
    log.finalAngleError = array('finalAngleError', 1, 1);
    if log.time(1) ~= 0 || any(diff(log.time) <= 0) || ...
            any(log.rho < 0) || any(log.trackingError < 0) || ...
            any(log.trackingApproximationBound < 0) || any(log.integrationErrorBound < 0) || ...
            log.finalPositionError < 0 || log.finalAngleError < 0 || ...
            ~iscell(log.mode) || ~isequal(size(log.mode), [n, 1]) || ...
            ~all(cellfun(@(value) ischar(value) && isrow(value) && ~isempty(value), log.mode))
        error('RobotProject:InvalidLog', 'Invalid times, error magnitudes or mode history.');
    end
    statuses = {'goal_reached', 'tracking_blocked', 'timeout', 'step_limit', 'numerical_failure'};
    if ~ischar(log.status) || ~isrow(log.status) || ~any(strcmp(log.status, statuses)) || ...
            ~ischar(log.message) || ~isrow(log.message) || ...
            ~islogical(log.goalReached) || ~isscalar(log.goalReached) || ...
            log.goalReached ~= strcmp(log.status, 'goal_reached')
        error('RobotProject:InvalidLog', 'Invalid termination metadata.');
    end

    r = log.reference;
    refFields = {'poses', 'segment', 'nominalTime', 'states', 'controls', 'durations', 'goal'};
    if ~isstruct(r) || ~isscalar(r) || ~all(isfield(r, refFields))
        error('RobotProject:InvalidLog', 'Incomplete planned reference.');
    end
    path = struct('states', r.states, 'controls', r.controls, 'durations', r.durations, ...
        'samples', r.poses, 'time', r.nominalTime, 'segment', r.segment);
    reference = robot.reference.buildReference(struct('status', 'success', 'path', path), cfg);
    goal = robot.internal.validateColumn(r.goal, 3, 'reference.goal');
    if norm(goal(1:2)-log.scene.qf(1:2)) > cfg.planning.goalPositionTolerance || ...
            abs(robot.model.normalizeAngle(goal(3)-log.scene.qf(3))) > cfg.planning.goalAngleTolerance || ...
            norm(reference.goal(1:2)-goal(1:2)) > cfg.planning.goalPositionTolerance || ...
            abs(robot.model.normalizeAngle(reference.goal(3)-goal(3))) > cfg.planning.goalAngleTolerance
        error('RobotProject:InvalidLog', 'Reference and scene goals do not agree.');
    end
    reference.goal = goal;
    log.reference = reference;
    if any(log.referenceIndex ~= floor(log.referenceIndex)) || ...
            any(diff(log.referenceIndex) < 0) || any(log.referenceIndex < 2) || ...
            any(log.referenceIndex > size(reference.poses, 1)+1)
        error('RobotProject:InvalidLog', 'Invalid reference progress history.');
    end
    positionError = norm(log.q(end, 1:2)'-goal(1:2));
    angleError = abs(robot.model.normalizeAngle(log.q(end, 3)-goal(3)));
    if norm(log.q(1, 1:2)'-log.scene.q0(1:2)) > cfg.planning.goalPositionTolerance || ...
            abs(robot.model.normalizeAngle(log.q(1, 3)-log.scene.q0(3))) > cfg.planning.goalAngleTolerance || ...
            norm(reference.states(1, 1:2)'-log.scene.q0(1:2)) > cfg.planning.goalPositionTolerance || ...
            abs(robot.model.normalizeAngle(reference.states(1, 3)-log.scene.q0(3))) > cfg.planning.goalAngleTolerance || ...
            any(abs(log.finalState-log.q(end, :)') > 1e-10) || ...
            abs(log.finalPositionError-positionError) > 1e-10 || ...
            abs(log.finalAngleError-angleError) > 1e-10 || ...
            (log.goalReached && (positionError > cfg.control.goalPositionTolerance || ...
                angleError > cfg.control.goalAngleTolerance))
        error('RobotProject:InvalidLog', 'Initial/final states or reported goal errors are inconsistent.');
    end
    % Check the logged twist against the actual ordered wheel commands.
    for index = 1:n-1
        twist = robot.model.wheelsToBody(log.u(index, :)', cfg.robot);
        if any(abs(twist-log.twist(index, :)') > 1e-10) || ...
                max(abs(log.u(index, :))) > cfg.limits.wheel+1e-12 || ...
                twist(1) < -1e-12 || twist(1) > cfg.limits.v+1e-12 || ...
                abs(twist(2)) > cfg.limits.omega+1e-12
            error('RobotProject:InvalidLog', 'Logged wheel/body commands or limits are inconsistent.');
        end
    end

    function value = array(name, rows, columns)
        value = log.(name);
        if ~isfloat(value) || ~isreal(value) || issparse(value) || ...
                ~isequal(size(value), [rows, columns]) || any(~isfinite(value(:)))
            error('RobotProject:InvalidLog', 'Invalid or misaligned array %s.', name);
        end
        value = double(value);
    end
end
