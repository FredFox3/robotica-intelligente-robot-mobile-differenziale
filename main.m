function project = main(action)
%MAIN Set up the project or run planning, simulation and plots.
%   main() returns settings; main('run') runs the simulation and animation.

    if nargin == 0
        action = 'setup';
    end
    if ~ischar(action) || ~isrow(action) || ~any(strcmp(action, {'setup', 'run'}))
        error('RobotProject:InvalidMainAction', 'Action must be ''setup'' or ''run''.');
    end

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(fullfile(projectRoot, 'src'));

    project = struct( ...
        'name', 'PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE', ...
        'executionEnvironment', 'MATLAB desktop / MATLAB Online', ...
        'simulationExecuted', false);
    project.config = robot.config.defaultConfig();
    if strcmp(action, 'setup')
        return;
    end
    project.scene = robot.config.makeScenario();
    fprintf('Pianificazione RRT...\n');
    project.plan = robot.planning.planRRT(project.scene, project.config);
    if ~strcmp(project.plan.status, 'success')
        warning('RobotProject:PlanningFailed', 'Pianificazione: %s. %s', ...
            project.plan.status, project.plan.message);
        return;
    end
    fprintf('Simulazione del moto controllato...\n');
    project.log = robot.simulation.simulate(project.scene, project.plan, project.config);
    project.simulationExecuted = true;
    project.summary = robot.analysis.computeMetrics(project.log);
    disp(project.summary);
    project.outputFolder = fullfile(projectRoot, 'results');
    if ~isfolder(project.outputFolder)
        [created, message] = mkdir(project.outputFolder);
        if ~created
            error('RobotProject:OutputWriteFailed', '%s', message);
        end
    end
    % Save numerical data before graphics so rendering errors do not lose it.
    save(fullfile(project.outputFolder, 'risultati.mat'), 'project');
    project.figures = robot.visualization.makePlots(project.log, project.outputFolder);
    project.animationAxes = robot.visualization.animate(project.log);
end
