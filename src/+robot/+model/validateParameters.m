function params = validateParameters(params)
%VALIDATEPARAMETERS Validate geometry and return R and D as doubles.
%   PARAMS must be a scalar struct with positive, finite, real, full,
%   floating-point scalar fields R and D, in metres. Other fields are kept.

    if ~isstruct(params) || ~isscalar(params) || ...
            ~isfield(params, 'r') || ~isfield(params, 'd')
        error('RobotProject:InvalidParameters', ...
            'Geometry must be a scalar struct containing r and d.');
    end

    names = {'r', 'd'};
    for index = 1:numel(names)
        value = params.(names{index});
        if ~isfloat(value) || ~isreal(value) || issparse(value) || ...
                ~isscalar(value) || ~isfinite(value) || value <= 0
            error('RobotProject:InvalidParameters', ...
                'Geometry field %s must be a positive finite floating-point scalar.', ...
                names{index});
        end
        params.(names{index}) = double(value);
    end
end
