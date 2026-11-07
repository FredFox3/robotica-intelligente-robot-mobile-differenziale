function wrapped = normalizeAngle(angle)
%NORMALIZEANGLE Wrap angles to [-pi, pi), mapping +pi to -pi.

    if ~isfloat(angle) || ~isreal(angle) || issparse(angle) || ...
            isempty(angle) || any(~isfinite(angle(:)))
        error('RobotProject:InvalidAngle', ...
            'Angles must be a nonempty real finite full floating-point array.');
    end
    angle = double(angle);
    wrapped = atan2(sin(angle), cos(angle));
    wrapped(wrapped == pi) = -pi;
end
