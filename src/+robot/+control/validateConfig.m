function cfg = validateConfig(cfg)
%VALIDATECONFIG Validate controller gains and simulation settings.
%   Geometry and speed limits must pass validatePlanningInputs separately.

    if ~isstruct(cfg) || ~isscalar(cfg) || ...
            ~all(isfield(cfg, {'control', 'simulation', 'integration'}))
        error('RobotProject:InvalidControlConfig', 'Incomplete control configuration.');
    end
    groups = {'control', 'simulation', 'integration'};
    fields = {{'kRho', 'kAlpha', 'kTheta', 'alignEnter', 'alignExit', ...
        'waypointTolerance', 'rotationTolerance', 'goalPositionTolerance', ...
        'goalAngleTolerance'}, ...
        {'maxTime', 'maxSteps', 'maxTrackingError', 'positionRoundoff', 'angleRoundoff'}, {'dt'}};
    for group = 1:numel(groups)
        settings = cfg.(groups{group});
        if ~isstruct(settings) || ~isscalar(settings)
            error('RobotProject:InvalidControlConfig', 'Settings must be scalar structs.');
        end
        for index = 1:numel(fields{group})
            name = fields{group}{index};
            if ~isfield(settings, name)
                error('RobotProject:InvalidControlConfig', 'Missing setting %s.', name);
            end
            value = settings.(name);
            if ~isfloat(value) || ~isreal(value) || issparse(value) || ...
                    ~isscalar(value) || ~isfinite(value) || value <= 0
                error('RobotProject:InvalidControlConfig', 'Setting %s must be positive and finite.', name);
            end
            cfg.(groups{group}).(name) = double(value);
        end
    end
    c = cfg.control;
    if c.kAlpha <= c.kRho || c.alignExit >= c.alignEnter || c.alignEnter >= pi/2 || ...
            c.goalAngleTolerance >= pi || c.rotationTolerance > c.goalAngleTolerance || ...
            c.waypointTolerance > c.goalPositionTolerance || ...
            cfg.simulation.maxSteps ~= floor(cfg.simulation.maxSteps) || ...
            cfg.simulation.maxSteps > 1e6
        error('RobotProject:InvalidControlConfig', ...
            'Inconsistent gains, hysteresis, tolerances or step budget.');
    end
end
