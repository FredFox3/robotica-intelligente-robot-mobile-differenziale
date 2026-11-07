function value = clearance(p, scene, safeRadius)
%CLEARANCE Compute signed clearance from walls and obstacles [m].
%   safeRadius is body radius plus margin; scene must be validated.
%   Positive clearance is free space; zero is contact.

    p = robot.internal.validateColumn(p, 2, 'p');
    if ~isfloat(safeRadius) || ~isscalar(safeRadius) || ~isreal(safeRadius) || ...
            ~isfinite(safeRadius) || safeRadius < 0
        error('RobotProject:InvalidParameters', 'Disc radius must be finite and nonnegative.');
    end
    value = min([p(1)-scene.bounds(1, 1), scene.bounds(1, 2)-p(1), ...
        p(2)-scene.bounds(2, 1), scene.bounds(2, 2)-p(2)]);
    for index = 1:size(scene.circles, 1)
        circle = scene.circles(index, :);
        distance = robot.internal.ensureFinite( ...
            hypot(p(1)-circle(1), p(2)-circle(2)) - circle(3));
        value = min(value, distance);
    end
    for index = 1:numel(scene.polygons)
        vertices = scene.polygons{index};
        distance = Inf;
        for edge = 1:size(vertices, 1)
            a = vertices(edge, :)';
            b = vertices(mod(edge, size(vertices, 1))+1, :)';
            direction = b-a;
            projection = robot.internal.ensureFinite( ...
                dot(p-a, direction) / dot(direction, direction));
            fraction = max(0, min(1, projection));
            edgeDistance = robot.internal.ensureFinite(norm(p - (a + fraction*direction)));
            distance = min(distance, edgeDistance);
        end
        [inside, onBoundary] = inpolygon(p(1), p(2), vertices(:, 1), vertices(:, 2));
        if inside || onBoundary
            distance = -distance;
        end
        value = min(value, distance);
    end
    value = robot.internal.ensureFinite(value - safeRadius);
end
