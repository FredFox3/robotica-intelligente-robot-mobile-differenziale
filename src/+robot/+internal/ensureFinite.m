function value = ensureFinite(value)
%ENSUREFINITE Reject numerical overflow instead of returning invalid data.

    if any(~isfinite(value(:)))
        error('RobotProject:NumericalFailure', ...
            'A kinematic calculation produced a nonfinite value.');
    end
end
