function ax = animate(log, playbackSpeed)
%ANIMATE Show robot position and heading along the simulated trajectory.
%   Intermediate frames use partial RK4 steps with the recorded wheel commands.

    log = robot.analysis.validateLog(log);
    if nargin < 2
        playbackSpeed = 10;
    end
    if ~isfloat(playbackSpeed) || ~isscalar(playbackSpeed) || ~isreal(playbackSpeed) || ...
            ~isfinite(playbackSpeed) || playbackSpeed <= 0
        error('RobotProject:InvalidPlaybackSpeed', 'Playback speed must be a positive finite scalar.');
    end
    playbackSpeed = double(playbackSpeed);
    frameStep = playbackSpeed/20;
    if frameStep == 0 || log.time(end)/frameStep > 1e6
        error('RobotProject:InvalidPlaybackSpeed', 'Playback speed would require too many frames.');
    end
    times = (0:frameStep:log.time(end))';
    if times(end) < log.time(end)
        times(end+1, 1) = log.time(end);
    end
    cfg = log.configuration;
    displayPlan = struct('status', 'success', 'path', struct('samples', log.reference.poses));
    ax = robot.visualization.plotPlan(log.scene, displayPlan, cfg);
    fig = ancestor(ax, 'figure');
    set(fig, 'Name', 'Animazione del moto RK4');
    trail = animatedline(ax, 'Color', [0.85, 0.1, 0.1], ...
        'LineWidth', 1.2, 'DisplayName', 'Moto eseguito progressivo', ...
        'MaximumNumPoints', size(log.q, 1)+numel(times));
    angles = linspace(0, 2*pi, 80);
    q = log.q(1, :)';
    body = plot(ax, q(1)+cfg.robot.bodyRadius*cos(angles), ...
        q(2)+cfg.robot.bodyRadius*sin(angles), 'k-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    axle = plot(ax, q(1), q(2), 'k-', 'HandleVisibility', 'off');
    rightWheel = plot(ax, q(1), q(2), 'r-', 'LineWidth', 3, 'HandleVisibility', 'off');
    leftWheel = plot(ax, q(1), q(2), 'b-', 'LineWidth', 3, 'HandleVisibility', 'off');
    heading = quiver(ax, q(1), q(2), cfg.robot.bodyRadius, 0, 0, ...
        'k', 'HandleVisibility', 'off');
    legend(ax, 'show', 'Location', 'best');
    k = 1;
    lastAdded = 0;
    n = size(log.q, 1);
    for frame = 1:numel(times)
        if ~isgraphics(ax) || ~isgraphics(body) || ~isgraphics(trail)
            break;
        end
        while k < n && log.time(k+1) <= times(frame)
            k = k+1;
        end
        q = log.q(k, :)';
        elapsed = times(frame)-log.time(k);
        if k < n && elapsed > 0
            q = robot.model.integrateRK4(q, log.u(k, :)', elapsed, cfg.robot);
        end
        if k > lastAdded
            addpoints(trail, log.q(lastAdded+1:k, 1), log.q(lastAdded+1:k, 2));
            lastAdded = k;
        end
        if elapsed > 0
            addpoints(trail, q(1), q(2));
        end
        forward = [cos(q(3)); sin(q(3))];
        lateral = [-sin(q(3)); cos(q(3))];
        rightCentre = q(1:2)-(cfg.robot.d/2)*lateral;
        leftCentre = q(1:2)+(cfg.robot.d/2)*lateral;
        set(body, 'XData', q(1)+cfg.robot.bodyRadius*cos(angles), ...
            'YData', q(2)+cfg.robot.bodyRadius*sin(angles));
        set(axle, 'XData', [rightCentre(1), leftCentre(1)], 'YData', [rightCentre(2), leftCentre(2)]);
        set(rightWheel, 'XData', rightCentre(1)+cfg.robot.r*[-1, 1]*forward(1), ...
            'YData', rightCentre(2)+cfg.robot.r*[-1, 1]*forward(2));
        set(leftWheel, 'XData', leftCentre(1)+cfg.robot.r*[-1, 1]*forward(1), ...
            'YData', leftCentre(2)+cfg.robot.r*[-1, 1]*forward(2));
        set(heading, 'XData', q(1), 'YData', q(2), ...
            'UData', cfg.robot.bodyRadius*forward(1), 'VData', cfg.robot.bodyRadius*forward(2));
        title(ax, sprintf('Moto RK4: t = %.2f s - %s', times(frame), log.status), 'Interpreter', 'none');
        drawnow;
        if frame < numel(times)
            pause((times(frame+1)-times(frame))/playbackSpeed);
        end
    end
end
