function [u, state, info] = step(q, reference, state, cfg)
%STEP Compute wheel commands for ordered waypoint tracking.
%   Advance uses v=kRho*rho and omega=kAlpha*alpha, with coupled speed limits.

    q = robot.internal.validateColumn(q, 3, 'q');
    count = size(reference.poses, 1);
    if isempty(state)
        state = struct('index', 2, 'mode', 'align');
    end
    if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, {'index', 'mode'})) || ...
            ~isfloat(state.index) || ~isreal(state.index) || ...
            ~isscalar(state.index) || ~isfinite(state.index) || ...
            state.index ~= floor(state.index) || state.index < 2 || state.index > count+1 || ...
            ~ischar(state.mode) || ~any(strcmp(state.mode, ...
                {'align', 'advance', 'rotate', 'final_align', 'stop'}))
        error('RobotProject:InvalidControllerState', 'Invalid controller state.');
    end
    info = struct('rho', 0, 'alpha', 0, 'orientationError', 0, ...
        'mode', '', 'done', false, 'requestedTwist', zeros(2, 1), ...
        'appliedTwist', zeros(2, 1));
    while state.index <= count
        target = reference.poses(state.index, :)';
        info.rho = norm(target(1:2)-q(1:2));
        edge = reference.segment(state.index);
        if info.rho > cfg.control.waypointTolerance
            desired = pointCommand(target(1:2));
            finish();
            return;
        end
        if reference.rotation(edge)
            info.orientationError = robot.model.normalizeAngle(target(3)-q(3));
            if abs(info.orientationError) > cfg.control.rotationTolerance
                state.mode = 'rotate';
                desired = [0; cfg.control.kTheta*info.orientationError];
                finish();
                return;
            end
        end
        state.index = state.index + 1;
    end

    info.rho = norm(reference.goal(1:2)-q(1:2));
    info.alpha = 0;
    info.orientationError = robot.model.normalizeAngle(reference.goal(3)-q(3));
    if info.rho > cfg.control.goalPositionTolerance
        desired = pointCommand(reference.goal(1:2));
    elseif abs(info.orientationError) > cfg.control.goalAngleTolerance
        state.mode = 'final_align';
        desired = [0; cfg.control.kTheta*info.orientationError];
    else
        state.mode = 'stop';
        info.done = true;
        desired = [0; 0];
    end
    finish();

    function twistRequest = pointCommand(position)
        delta = position-q(1:2);
        info.rho = norm(delta);
        % rho is positive here, so the waypoint bearing is defined.
        info.alpha = robot.model.normalizeAngle(atan2(delta(2), delta(1))-q(3));
        info.orientationError = 0;
        if strcmp(state.mode, 'advance')
            aligning = abs(info.alpha) > cfg.control.alignEnter;
        else
            aligning = abs(info.alpha) > cfg.control.alignExit;
        end
        state.mode = 'advance';
        speed = cfg.control.kRho*info.rho;
        if aligning
            state.mode = 'align';
            speed = 0;
        end
        twistRequest = [speed; cfg.control.kAlpha*info.alpha];
    end

    function finish()
        [u, info.appliedTwist] = robot.planning.boundedCommand(desired, cfg);
        info.requestedTwist = desired;
        info.mode = state.mode;
    end
end
