function tests = test_kinematics
%TEST_KINEMATICS Analytical, numerical and invalid-input kinematics tests.
%   Prepared for later execution in MATLAB Online; no recorded result yet.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    testCase.TestData.originalPath = path;
    addpath(fullfile(projectRoot, 'src'));
    cfg = robot.config.defaultConfig();
    testCase.TestData.params = cfg.robot;
end

function teardownOnce(testCase)
    path(testCase.TestData.originalPath);
end

function testKnownWheelConversions(testCase)
    params = testCase.TestData.params;
    verifyEqual(testCase, robot.model.wheelsToBody([4; 4], params), ...
        [0.2; 0], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.wheelsToBody([3; -3], params), ...
        [0; 1], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.wheelsToBody([6; 2], params), ...
        [0.2; 2/3], 'AbsTol', 1e-12);
end

function testKnownInverseConversions(testCase)
    params = testCase.TestData.params;
    verifyEqual(testCase, robot.model.bodyToWheels([0.2; 0], params), ...
        [4; 4], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.bodyToWheels([0; 1], params), ...
        [3; -3], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.bodyToWheels([0.2; 2/3], params), ...
        [6; 2], 'AbsTol', 1e-12);
end

function testConversionRoundTrips(testCase)
    params = testCase.TestData.params;
    commands = [0, 4, 3, 6, -7, 1; 0, 4, -3, 2, -2, -5];
    for index = 1:size(commands, 2)
        u = commands(:, index);
        recovered = robot.model.bodyToWheels( ...
            robot.model.wheelsToBody(u, params), params);
        verifyEqual(testCase, recovered, u, 'AbsTol', 1e-12);
    end
end

function testCustomGeometry(testCase)
    params = struct('r', 0.1, 'd', 0.5);
    verifyEqual(testCase, robot.model.wheelsToBody([5; 1], params), ...
        [0.3; 0.8], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.bodyToWheels([0.3; 0.8], params), ...
        [5; 1], 'AbsTol', 1e-12);
end

function testDerivativeAndWheelOrder(testCase)
    params = testCase.TestData.params;
    dq = robot.model.derivative([1; 2; pi/2], [4; 4], params);
    verifySize(testCase, dq, [3, 1]);
    verifyEqual(testCase, dq, [0; 0.2; 0], 'AbsTol', 1e-12);
    dq = robot.model.derivative([0; 0; 0], [0; 6], params);
    verifyEqual(testCase, dq, [0.15; 0; -1], 'AbsTol', 1e-12);
end

function testNonholonomicConstraint(testCase)
    params = testCase.TestData.params;
    headings = [-pi, -0.7, 0, 0.4, pi/2, pi];
    for theta = headings
        dq = robot.model.derivative([0; 0; theta], [6; 2], params);
        residual = -sin(theta) * dq(1) + cos(theta) * dq(2);
        verifyEqual(testCase, residual, 0, 'AbsTol', 1e-12);
    end
    % Independent negative example: lateral motion at theta = 0 is invalid.
    lateralVelocity = [0; 1; 0];
    verifyEqual(testCase, [0, 1, 0] * lateralVelocity, 1);
end

function testStraightLineIntegration(testCase)
    params = testCase.TestData.params;
    q0 = [1; 2; pi/6];
    expected = [1 + 0.4*cos(pi/6); 2 + 0.4*sin(pi/6); pi/6];
    q = integrateRepeated(q0, [4; 4], 2, 100, params);
    verifyEqual(testCase, q, expected, 'AbsTol', 1e-11);
    verifyEqual(testCase, robot.model.propagateConstant(q0, [4; 4], 2, params), ...
        expected, 'AbsTol', 1e-12);
end

function testRotationOnSpotIntegration(testCase)
    params = testCase.TestData.params;
    q0 = [1; 2; 0];
    expected = [1; 2; pi/2];
    q = integrateRepeated(q0, [3; -3], pi/2, 100, params);
    verifyEqual(testCase, q, expected, 'AbsTol', 1e-11);
    verifyEqual(testCase, robot.model.propagateConstant(q0, [3; -3], pi/2, params), ...
        expected, 'AbsTol', 1e-12);
end

function testCurvedIntegration(testCase)
    params = testCase.TestData.params;
    % v=0.2, omega=2/3, radius=0.3; quarter circle from (0,0,0).
    duration = 3*pi/4;
    q = integrateRepeated([0; 0; 0], [6; 2], duration, 120, params);
    verifyEqual(testCase, q, [0.3; 0.3; pi/2], 'AbsTol', 1e-9);
    verifyEqual(testCase, robot.model.propagateConstant( ...
        [0; 0; 0], [6; 2], duration, params), ...
        [0.3; 0.3; pi/2], 'AbsTol', 1e-12);
end

function testClockwiseAndBackwardArcs(testCase)
    params = testCase.TestData.params;
    duration = 3*pi/4;
    verifyEqual(testCase, robot.model.propagateConstant( ...
        [0; 0; 0], [2; 6], duration, params), ...
        [0.3; -0.3; -pi/2], 'AbsTol', 1e-12);
    q = integrateRepeated([0; 0; 0], [-6; -2], duration, 120, params);
    verifyEqual(testCase, q, [-0.3; 0.3; -pi/2], 'AbsTol', 1e-9);
end

function testRK4Convergence(testCase)
    params = testCase.TestData.params;
    expected = [0.3; 0.3; pi/2];
    coarse = integrateRepeated([0; 0; 0], [6; 2], 3*pi/4, 4, params);
    fine = integrateRepeated([0; 0; 0], [6; 2], 3*pi/4, 8, params);
    coarseError = norm(coarse(1:2) - expected(1:2));
    fineError = norm(fine(1:2) - expected(1:2));
    verifyGreaterThan(testCase, coarseError, 1e-12);
    verifyLessThan(testCase, fineError, coarseError / 10);
end

function testTranslatedRotatedArc(testCase)
    params = testCase.TestData.params;
    % Rotate the known quarter circle by pi/2 and translate its origin.
    q0 = [1; -2; pi/2];
    qExact = robot.model.propagateConstant(q0, [6; 2], 3*pi/4, params);
    qNumeric = integrateRepeated(q0, [6; 2], 3*pi/4, 120, params);
    verifyEqual(testCase, qExact(1:2), [0.7; -1.7], 'AbsTol', 1e-12);
    verifyEqual(testCase, qNumeric(1:2), [0.7; -1.7], 'AbsTol', 1e-9);
    % Compare orientation modulo a turn, independently of the side of pi.
    verifyEqual(testCase, robot.model.normalizeAngle(qExact(3)-pi), 0, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.normalizeAngle(qNumeric(3)-pi), 0, ...
        'AbsTol', 1e-11);
end

function testZeroCommandAndDuration(testCase)
    params = testCase.TestData.params;
    q = [1; -2; 0.7];
    methods = {@robot.model.integrateRK4, @robot.model.propagateConstant};
    for index = 1:numel(methods)
        method = methods{index};
        verifyEqual(testCase, method(q, [0; 0], 1, params), q, ...
            'AbsTol', 1e-12);
        verifyEqual(testCase, method(q, [4; 1], 0, params), q, ...
            'AbsTol', 1e-12);
        wrapped = method([1; -2; 2*pi], [4; 1], 0, params);
        verifyEqual(testCase, wrapped, [1; -2; 0], 'AbsTol', 1e-12);
    end
end

function testAngleNormalization(testCase)
    input = [-pi, pi, 0; -4*pi, 0.7, 7.2];
    actual = robot.model.normalizeAngle(input);
    verifySize(testCase, actual, size(input));
    verifyGreaterThanOrEqual(testCase, actual, -pi);
    verifyLessThan(testCase, actual, pi);
    verifyEqual(testCase, actual(1, 1:2), [-pi, -pi]);
    verifyEqual(testCase, sin(actual), sin(input), 'AbsTol', 1e-12);
    verifyEqual(testCase, cos(actual), cos(input), 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.normalizeAngle(1e-20), 1e-20, ...
        'AbsTol', 1e-30);
end

function testIntegrationAcrossAngleCut(testCase)
    params = testCase.TestData.params;
    q = integrateRepeated([2; 3; pi-0.01], [3; -3], 0.02, 2, params);
    verifyEqual(testCase, q, [2; 3; -pi+0.01], 'AbsTol', 1e-12);
end

function testNearStraightAnalyticalLimit(testCase)
    params = testCase.TestData.params;
    actual = robot.model.propagateConstant([0; 0; 0], [4+1e-10; 4], 1, params);
    verifyEqual(testCase, actual, [0.2; 0; 0], 'AbsTol', 1e-9);
end

function testSingleInputsProduceDoubleColumns(testCase)
    params = struct('r', single(0.05), 'd', single(0.30));
    twist = robot.model.wheelsToBody(single([4; 4]), params);
    verifyClass(testCase, twist, 'double');
    verifySize(testCase, twist, [2, 1]);
    q = robot.model.integrateRK4(single([0; 0; 0]), single([4; 4]), ...
        single(0.02), params);
    verifyClass(testCase, q, 'double');
    verifySize(testCase, q, [3, 1]);
end

function testInvalidParameters(testCase)
    badValues = {0, -0.05, NaN, Inf, 1i, [0.05, 0.1], [], ...
        '0.05', int32(1), true, sparse(0.05)};
    fields = {'r', 'd'};
    for fieldIndex = 1:numel(fields)
        for index = 1:numel(badValues)
            params = testCase.TestData.params;
            params.(fields{fieldIndex}) = badValues{index};
            verifyError(testCase, @() robot.model.wheelsToBody([1; 1], params), ...
                'RobotProject:InvalidParameters');
        end
    end
    invalidStructs = {[], struct(), struct('r', 0.05), ...
        repmat(struct('r', 0.05, 'd', 0.3), 1, 2)};
    for index = 1:numel(invalidStructs)
        verifyError(testCase, @() robot.model.validateParameters(invalidStructs{index}), ...
            'RobotProject:InvalidParameters');
    end
end

function testInvalidVectors(testCase)
    params = testCase.TestData.params;
    badCommands = {[1, 2], [1; 2; 3], [NaN; 0], [Inf; 0], ...
        [1i; 0], [], int32([1; 2]), sparse([1; 2])};
    for index = 1:numel(badCommands)
        value = badCommands{index};
        verifyError(testCase, @() robot.model.wheelsToBody(value, params), ...
            'RobotProject:InvalidVector');
        verifyError(testCase, @() robot.model.bodyToWheels(value, params), ...
            'RobotProject:InvalidVector');
    end
    badStates = {[0, 0, 0], [0; 0], [0; Inf; 0], [0; 0; NaN], [0; 0; 1i]};
    for index = 1:numel(badStates)
        value = badStates{index};
        verifyError(testCase, @() robot.model.derivative(value, [1; 1], params), ...
            'RobotProject:InvalidVector');
    end
end

function testInvalidDurations(testCase)
    params = testCase.TestData.params;
    methods = {@robot.model.integrateRK4, @robot.model.propagateConstant};
    badDurations = {-1, NaN, Inf, [0.1, 0.2], 1i, [], int32(1)};
    for methodIndex = 1:numel(methods)
        method = methods{methodIndex};
        for index = 1:numel(badDurations)
            verifyError(testCase, @() method([0; 0; 0], [1; 1], ...
                badDurations{index}, params), 'RobotProject:InvalidDuration');
        end
    end
end

function testInvalidAngles(testCase)
    values = {NaN, Inf, 1i, [], 'pi', sparse(1), int32(1)};
    for index = 1:numel(values)
        verifyError(testCase, @() robot.model.normalizeAngle(values{index}), ...
            'RobotProject:InvalidAngle');
    end
end

function testZeroDurationStillValidatesInput(testCase)
    params = testCase.TestData.params;
    verifyError(testCase, @() robot.model.integrateRK4([0; 0; 0], ...
        [Inf; 0], 0, params), 'RobotProject:InvalidVector');
    params.d = 0;
    verifyError(testCase, @() robot.model.propagateConstant([0; 0; 0], ...
        [0; 0], 0, params), 'RobotProject:InvalidParameters');
end

function testNumericalOverflowIsReported(testCase)
    params = struct('r', 1, 'd', 1);
    verifyError(testCase, @() robot.model.wheelsToBody([realmax; realmax], params), ...
        'RobotProject:NumericalFailure');
    params.r = realmin;
    verifyError(testCase, @() robot.model.bodyToWheels([10; 0], params), ...
        'RobotProject:NumericalFailure');
    verifyError(testCase, @() robot.model.propagateConstant([0; 0; 0], ...
        [1; -1], realmax, struct('r', 1, 'd', 1)), ...
        'RobotProject:NumericalFailure');
end

function q = integrateRepeated(q, u, duration, steps, params)
% Test-only loop: a prescribed command, not a path planner or controller.
    dt = duration / steps;
    for index = 1:steps
        q = robot.model.integrateRK4(q, u, dt, params);
    end
end
