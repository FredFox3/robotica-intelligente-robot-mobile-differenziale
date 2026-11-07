function wrapped = normalizeAngle(angle)
%NORMALIZEANGLE Map finite real angles [rad] into [-pi, pi).
%   Accept a nonempty full floating-point scalar or array; preserve size.
%   Output is double. The +pi endpoint is represented by -pi.
%   No Mapping Toolbox function is required.

    if ~isfloat(angle) || ~isreal(angle) || issparse(angle) || ...
            isempty(angle) || any(~isfinite(angle(:)))
        error('RobotProject:InvalidAngle', ...
            'Angles must be a nonempty real finite full floating-point array.');
    end
    angle = double(angle);
    wrapped = atan2(sin(angle), cos(angle));
    wrapped(wrapped == pi) = -pi;
end
