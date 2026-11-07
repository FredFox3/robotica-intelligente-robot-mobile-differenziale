function [distance, approximationBound] = trackingError(q, reference, index)
%TRACKINGERROR Distance to the active primitive's sampled polyline [m].
%   Distance to the true arc differs by at most APPROXIMATIONBOUND [m].
%   Uses only the active primitive, avoiding ambiguity at nearby path branches.
%   REFERENCE comes from buildReference; INDEX comes from control.step.

    q = robot.internal.validateColumn(q, 3, 'q');
    count = size(reference.poses, 1);
    if ~isfloat(index) || ~isreal(index) || ~isscalar(index) || ~isfinite(index) || ...
            index ~= floor(index) || index < 2 || index > count+1
        error('RobotProject:InvalidControllerState', 'Invalid reference index.');
    end
    if isempty(reference.controls)
        distance = norm(q(1:2)-reference.goal(1:2));
        approximationBound = 0;
        return;
    end
    edge = reference.segment(min(index, count));
    vertices = [reference.states(edge, 1:2); reference.poses(reference.segment == edge, 1:2)];
    distance = Inf;
    p = q(1:2);
    for segment = 1:size(vertices, 1)-1
        a = vertices(segment, :)';
        direction = vertices(segment+1, :)' - a;
        lengthSquared = robot.internal.ensureFinite(dot(direction, direction));
        fraction = 0;
        if lengthSquared > 0
            projection = robot.internal.ensureFinite(dot(p-a, direction)/lengthSquared);
            fraction = max(0, min(1, projection));
        end
        candidateDistance = robot.internal.ensureFinite(norm(p-(a+fraction*direction)));
        distance = min(distance, candidateDistance);
    end
    distance = robot.internal.ensureFinite(distance);
    approximationBound = reference.chordError(edge);
end
