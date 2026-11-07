function project = main(action)
%MAIN Prepare settings or explicitly run the complete project pipeline.
%   PROJECT = MAIN() or MAIN('setup') adds SRC and returns settings only.
%   PROJECT = MAIN('run') plans, simulates, summarizes, saves three PNGs in
%   RESULTS and animates the actual motion. No tests are run automatically.
%   Execution is deferred to the final MATLAB Online verification phase.

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
        'status', 'visualization_analysis_implemented', ...
        'executionValidation', 'pending_matlab_online', ...
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
    project.executionValidation = 'program_executed_tests_not_run';
    project.summary = robot.analysis.computeMetrics(project.log);
    disp(project.summary);
    project.outputFolder = fullfile(projectRoot, 'results');
    project.figures = robot.visualization.makePlots(project.log, project.outputFolder);
    project.animationAxes = robot.visualization.animate(project.log);
end
