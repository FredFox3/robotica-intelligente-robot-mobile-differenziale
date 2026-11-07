function value = ensureFinite(value)
%ENSUREFINITE Reject nonfinite numerical results.

    if any(~isfinite(value(:)))
        error('RobotProject:NumericalFailure', ...
            'A kinematic calculation produced a nonfinite value.');
    end
end
