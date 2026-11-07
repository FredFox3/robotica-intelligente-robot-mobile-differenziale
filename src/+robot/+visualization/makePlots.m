function figures = makePlots(log, outputFolder)
%MAKEPLOTS Plot trajectories, tracking error and wheel speeds.
%   outputFolder exports PNG at 200 dpi, replacing files with the same names.

    log = robot.analysis.validateLog(log);
    if nargin < 2
        outputFolder = '';
    end
    if ~ischar(outputFolder) || (~isempty(outputFolder) && ~isrow(outputFolder))
        error('RobotProject:InvalidOutputFolder', 'Output folder must be a character vector.');
    end
    cfg = log.configuration;
    displayPlan = struct('status', 'success', 'path', struct('samples', log.reference.poses));
    ax = robot.visualization.plotPlan(log.scene, displayPlan, cfg);
    figures.trajectory = ancestor(ax, 'figure');
    set(figures.trajectory, 'Name', 'Traiettorie pianificata ed eseguita', 'Color', 'w');
    if size(log.q, 1) == 1
        plot(ax, log.q(1, 1), log.q(1, 2), 'ro', 'DisplayName', 'Eseguita: posa singola');
    else
        plot(ax, log.q(:, 1), log.q(:, 2), 'r-', 'LineWidth', 1.2, ...
            'DisplayName', 'Eseguita: campioni RK4');
    end
    plot(ax, log.q(end, 1), log.q(end, 2), 'rs', 'DisplayName', 'Posa finale eseguita');
    quiver(ax, log.q(end, 1), log.q(end, 2), cfg.robot.bodyRadius*cos(log.q(end, 3)), ...
        cfg.robot.bodyRadius*sin(log.q(end, 3)), 0, 'r', 'HandleVisibility', 'off');
    title(ax, ['Traiettorie - ', log.status], 'Interpreter', 'none');
    legend(ax, 'show', 'Location', 'best');

    figures.tracking = figure('Name', 'Errore di tracking', 'Color', 'w');
    ax = axes('Parent', figures.tracking);
    hold(ax, 'on');
    style = '-';
    if numel(log.time) == 1
        style = 'o';
    end
    plot(ax, log.time, log.trackingError, ['b', style], 'DisplayName', 'Distanza dalla polilinea attiva');
    plot(ax, log.time, log.trackingError+log.trackingApproximationBound, ...
        ['r', style], 'DisplayName', 'Limite superiore rispetto alla primitiva');
    thresholdStyle = 'k:';
    if numel(log.time) == 1
        thresholdStyle = 'kx';
    end
    plot(ax, log.time, cfg.simulation.maxTrackingError*ones(size(log.time)), ...
        thresholdStyle, 'DisplayName', 'Soglia di arresto');
    xlabel(ax, 't [s]');
    ylabel(ax, 'Errore di tracking [m]');
    title(ax, ['Tracking - ', log.status], 'Interpreter', 'none');
    grid(ax, 'on');
    legend(ax, 'show', 'Location', 'best');

    figures.wheels = figure('Name', 'Velocita angolari delle ruote', 'Color', 'w');
    ax = axes('Parent', figures.wheels);
    hold(ax, 'on');
    if isempty(log.u)
        text(ax, 0.5, 0.5, 'Durata nulla: nessun comando applicato', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    else
        % Extend the last of N-1 commands to the final time for the stair plot.
        stairs(ax, log.time, [log.u(:, 1); log.u(end, 1)], 'r-', ...
            'DisplayName', 'Ruota destra: omega_R');
        stairs(ax, log.time, [log.u(:, 2); log.u(end, 2)], 'b--', ...
            'DisplayName', 'Ruota sinistra: omega_L');
        plot(ax, log.time([1, end]), cfg.limits.wheel*[1; 1], 'k:', 'DisplayName', 'Limiti ruota');
        plot(ax, log.time([1, end]), -cfg.limits.wheel*[1; 1], 'k:', 'HandleVisibility', 'off');
        legend(ax, 'show', 'Location', 'best');
    end
    xlabel(ax, 't [s]');
    ylabel(ax, 'Velocita angolare [rad/s]');
    title(ax, ['Comandi effettivi delle ruote - ', log.status], 'Interpreter', 'none');
    grid(ax, 'on');

    if ~isempty(outputFolder)
        if ~isfolder(outputFolder)
            [created, message] = mkdir(outputFolder);
            if ~created
                error('RobotProject:OutputWriteFailed', '%s', message);
            end
        end
        handles = {figures.trajectory, figures.tracking, figures.wheels};
        names = {'traiettorie.png', 'errore_tracking.png', 'velocita_ruote.png'};
        for index = 1:numel(handles)
            if ~isgraphics(handles{index}, 'figure')
                error('RobotProject:FigureClosed', 'A figure was closed before PNG export.');
            end
            print(handles{index}, fullfile(outputFolder, names{index}), '-dpng', '-r200');
        end
    end
end
