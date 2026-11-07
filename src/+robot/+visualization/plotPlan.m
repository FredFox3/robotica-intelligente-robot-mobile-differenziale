function ax = plotPlan(scene, plan, cfg)
%PLOTPLAN Draw nominal obstacles, disc footprint and the planned trajectory.
%   No executed/controlled trajectory is implied. On failure show the scene
%   and status only. The figure is created only on an explicit call.

    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    fig = figure('Name', 'Differential-drive path planning');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    rectangle('Parent', ax, 'Position', [scene.bounds(1, 1), scene.bounds(2, 1), ...
        diff(scene.bounds(1, :)), diff(scene.bounds(2, :))], ...
        'EdgeColor', [0.15, 0.15, 0.15], 'HandleVisibility', 'off');
    angles = linspace(0, 2*pi, 100);
    for index = 1:size(scene.circles, 1)
        c = scene.circles(index, :);
        fill(ax, c(1)+c(3)*cos(angles), c(2)+c(3)*sin(angles), ...
            [0.4, 0.4, 0.4], 'HandleVisibility', 'off');
    end
    for index = 1:numel(scene.polygons)
        vertices = scene.polygons{index};
        fill(ax, vertices(:, 1), vertices(:, 2), [0.4, 0.4, 0.4], ...
            'HandleVisibility', 'off');
    end
    if strcmp(plan.status, 'success')
        plot(ax, plan.path.samples(:, 1), plan.path.samples(:, 2), ...
            'b-', 'LineWidth', 1.5, 'DisplayName', 'Planned path');
    end
    plot(ax, scene.q0(1), scene.q0(2), 'go', 'DisplayName', 'Start');
    plot(ax, scene.qf(1), scene.qf(2), 'rx', 'DisplayName', 'Goal');
    poses = [scene.q0, scene.qf];
    for index = 1:2
        q = poses(:, index);
        plot(ax, q(1)+cfg.robot.bodyRadius*cos(angles), ...
            q(2)+cfg.robot.bodyRadius*sin(angles), 'k-', 'HandleVisibility', 'off');
        radius = cfg.robot.bodyRadius + cfg.geometry.margin;
        plot(ax, q(1)+radius*cos(angles), q(2)+radius*sin(angles), ...
            'k:', 'HandleVisibility', 'off');
        quiver(ax, q(1), q(2), radius*cos(q(3)), radius*sin(q(3)), 0, ...
            'k', 'HandleVisibility', 'off');
    end
    axis(ax, 'equal');
    xlim(ax, scene.bounds(1, :));
    ylim(ax, scene.bounds(2, :));
    grid(ax, 'on');
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');
    title(ax, ['Planning: ', plan.status], 'Interpreter', 'none');
    legend(ax, 'show', 'Location', 'best');
end
