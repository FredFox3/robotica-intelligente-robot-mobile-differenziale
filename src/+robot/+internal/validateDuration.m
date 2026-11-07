function duration = validateDuration(duration)
%VALIDATEDURATION Accept a finite nonnegative floating-point scalar [s].

    if ~isfloat(duration) || ~isreal(duration) || issparse(duration) || ...
            ~isscalar(duration) || ~isfinite(duration) || duration < 0
        error('RobotProject:InvalidDuration', ...
            'Duration must be a finite nonnegative floating-point scalar.');
    end
    duration = double(duration);
end
