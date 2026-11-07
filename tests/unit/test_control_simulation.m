function tests = test_control_simulation
%TEST_CONTROL_SIMULATION Numerical checks of control and simulated motion.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    testCase.TestData.originalPath = path;
    addpath(fullfile(root, 'src'));
end

function teardownOnce(testCase)
    path(testCase.TestData.originalPath);
end

function testNominalLawAndInverseConversion(testCase)
    [cfg, scene, plan] = straightPlan();
    reference = robot.reference.buildReference(plan, cfg);
    q = reference.poses(2, :)' - [0.03; 0; 0];
    [u, state, info] = robot.control.step(q, reference, [], cfg);
    verifyEqual(testCase, state.mode, 'advance');
    verifyEqual(testCase, info.rho, 0.03, 'AbsTol', 1e-12);
    verifyEqual(testCase, info.alpha, 0, 'AbsTol', 1e-12);
    verifyEqual(testCase, info.requestedTwist, [0.024; 0], 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.model.wheelsToBody(u, cfg.robot), ...
        info.appliedTwist, 'AbsTol', 1e-12);
    verifyEqual(testCase, u, [0.48; 0.48], 'AbsTol', 1e-12);
    verifyEqual(testCase, reference.goal, scene.qf, 'AbsTol', 1e-8);
end

function testBehindTargetAndAlignmentHysteresis(testCase)
    [cfg, ~, plan] = straightPlan();
    reference = robot.reference.buildReference(plan, cfg);
    q = reference.poses(2, :)' - [0.03; 0; 0];
    q(3) = pi;
    [u, state, info] = robot.control.step(q, reference, [], cfg);
    verifyEqual(testCase, state.mode, 'align');
    verifyEqual(testCase, info.appliedTwist(1), 0, 'AbsTol', 1e-12);
    verifyEqual(testCase, abs(info.alpha), pi, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, max(abs(u)), cfg.limits.wheel+1e-12);
    q(3) = -0.2;
    [~, advancing] = robot.control.step(q, reference, struct('index', 2, 'mode', 'advance'), cfg);
    [~, aligning] = robot.control.step(q, reference, struct('index', 2, 'mode', 'align'), cfg);
    verifyEqual(testCase, advancing.mode, 'advance');
    verifyEqual(testCase, aligning.mode, 'align');
end

function testRotationAndZeroDistance(testCase)
    [cfg, scene, ~] = straightPlan();
    scene.qf = scene.q0 + [0; 0; pi/2];
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    reference = robot.reference.buildReference(plan, cfg);
    [u, state, info] = robot.control.step(scene.q0, reference, [], cfg);
    verifyEqual(testCase, state.mode, 'rotate');
    verifyEqual(testCase, info.rho, 0, 'AbsTol', 1e-12);
    verifyEqual(testCase, info.alpha, 0);
    verifyEqual(testCase, info.appliedTwist(1), 0, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, info.appliedTwist(2), 0);
    verifyEqual(testCase, u(1), -u(2), 'AbsTol', 1e-12);
end

function testFinalOrientationCannotBeIgnored(testCase)
    [cfg, scene, plan] = straightPlan();
    reference = robot.reference.buildReference(plan, cfg);
    state = struct('index', size(reference.poses, 1)+1, 'mode', 'advance');
    [~, state, info] = robot.control.step(scene.qf-[0; 0; pi/2], reference, state, cfg);
    verifyEqual(testCase, state.mode, 'final_align');
    verifyFalse(testCase, info.done);
    verifyEqual(testCase, info.appliedTwist(1), 0);
    [u, ~, info] = robot.control.step(scene.qf, reference, state, cfg);
    verifyTrue(testCase, info.done);
    verifyEqual(testCase, u, [0; 0]);
end

function testReferencePreservesPrimitivesAndRejectsForgery(testCase)
    [cfg, ~, plan] = straightPlan();
    reference = robot.reference.buildReference(plan, cfg);
    verifyEqual(testCase, reference.poses, plan.path.samples);
    verifyEqual(testCase, reference.controls, plan.path.controls);
    plan.path.samples(2, 2) = plan.path.samples(2, 2)+0.1;
    verifyError(testCase, @() robot.reference.buildReference(plan, cfg), 'RobotProject:InvalidPlan');
end

function testTrackingUsesActivePrimitive(testCase)
    [cfg, ~, plan] = straightPlan();
    reference = robot.reference.buildReference(plan, cfg);
    q = reference.poses(2, :)' + [0; 0.02; 0];
    [distance, bound] = robot.reference.trackingError(q, reference, 2);
    verifyEqual(testCase, distance, 0.02, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, bound, cfg.planning.goalPositionTolerance+1e-12);
end

function testClosedLoopGoalAndLoggedMotion(testCase)
    [cfg, scene, ~] = straightPlan();
    scene.qf(3) = pi/2;
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyEqual(testCase, log.status, 'goal_reached');
    verifyTrue(testCase, log.goalReached);
    verifyLessThanOrEqual(testCase, log.finalPositionError, cfg.control.goalPositionTolerance);
    verifyLessThanOrEqual(testCase, log.finalAngleError, cfg.control.goalAngleTolerance);
    verifyMotion(testCase, log, scene, cfg);
end

function testAlreadyAtGoalHasNoIntervals(testCase)
    [cfg, scene, ~] = straightPlan();
    scene.qf = scene.q0;
    plan = robot.planning.planRRT(scene, cfg);
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyTrue(testCase, log.goalReached);
    verifySize(testCase, log.q, [1, 3]);
    verifySize(testCase, log.u, [0, 2]);
    verifyEqual(testCase, log.time, 0);
end

function testCurvedControlledPrefixAndRK4Bound(testCase)
    [cfg, scene, ~] = straightPlan();
    scene.qf = [2.65; 2.65; pi/2];
    tree = struct('Q', [scene.q0'; scene.qf'], 'parent', [0; 1], ...
        'Uin', [0, 0; 6, 5], 'Tin', [0; 3*pi], 'clearance', [0.7; 0.7]);
    plan = struct('status', 'success', ...
        'path', robot.planning.reconstructPath(tree, 2, cfg));
    cfg.simulation.maxSteps = 3;
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyEqual(testCase, log.status, 'step_limit');
    verifySize(testCase, log.u, [3, 2]);
    verifyTrue(testCase, any(abs(log.twist(:, 1)) > 0 & abs(log.twist(:, 2)) > 0));
    verifyMotion(testCase, log, scene, cfg);
end

function testUnsafeCommandIsNotExecuted(testCase)
    [cfg, scene, plan] = straightPlan();
    % Deliberately invalidate the nominal corridor AFTER planning. Both poses
    % remain free; the executed motion must stop before the new obstruction.
    scene.circles = [1.4, 1, 0.02];
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyEqual(testCase, log.status, 'tracking_blocked');
    verifyFalse(testCase, log.goalReached);
    verifyGreaterThan(testCase, size(log.u, 1), 0);
    verifyMotion(testCase, log, scene, cfg);
end

function testTimeAndStepLimits(testCase)
    [cfg, scene, plan] = straightPlan();
    cfg.simulation.maxTime = 0.03;
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyEqual(testCase, log.status, 'timeout');
    verifyEqual(testCase, log.time, [0; 0.02; 0.03], 'AbsTol', 1e-12);
    cfg.simulation.maxTime = 180;
    cfg.simulation.maxSteps = 1;
    log = robot.simulation.simulate(scene, plan, cfg);
    verifyEqual(testCase, log.status, 'step_limit');
    verifySize(testCase, log.q, [2, 3]);
end

function testInvalidControlInputs(testCase)
    [cfg, scene, plan] = straightPlan();
    cfg.control.kRho = 0;
    verifyError(testCase, @() robot.simulation.simulate(scene, plan, cfg), ...
        'RobotProject:InvalidControlConfig');
    cfg = robot.config.defaultConfig();
    cfg.integration.dt = 0;
    verifyError(testCase, @() robot.control.validateConfig(cfg), 'RobotProject:InvalidControlConfig');
    cfg = robot.config.defaultConfig();
    scene.q0(1) = scene.q0(1)+0.1;
    verifyError(testCase, @() robot.simulation.simulate(scene, plan, cfg), ...
        'RobotProject:InvalidSimulationInput');
end

function verifyMotion(testCase, log, scene, cfg)
    count = size(log.q, 1);
    verifyEqual(testCase, size(log.u, 1), count-1);
    verifyEqual(testCase, size(log.twist, 1), count-1);
    verifyGreaterThan(testCase, diff(log.time), 0);
    verifyLessThanOrEqual(testCase, max(abs(log.u(:))), cfg.limits.wheel+1e-12);
    verifyGreaterThan(testCase, log.clearanceBound, cfg.geometry.tolerance);
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    for index = 1:count
        q = log.q(index, :)';
        verifyGreaterThan(testCase, robot.geometry.clearance(q(1:2), scene, radius), cfg.geometry.tolerance);
        if index == count
            continue;
        end
        u = log.u(index, :)';
        twist = robot.model.wheelsToBody(u, cfg.robot);
        verifyEqual(testCase, twist, log.twist(index, :)', 'AbsTol', 1e-12);
        verifyGreaterThanOrEqual(testCase, twist(1), -1e-12);
        verifyLessThanOrEqual(testCase, twist(1), cfg.limits.v+1e-12);
        verifyLessThanOrEqual(testCase, abs(twist(2)), cfg.limits.omega+1e-12);
        dq = robot.model.derivative(q, u, cfg.robot);
        verifyEqual(testCase, [-sin(q(3)), cos(q(3)), 0]*dq, 0, 'AbsTol', 1e-12);
        dt = log.time(index+1)-log.time(index);
        exact = robot.model.propagateConstant(q, u, dt, cfg.robot);
        verifyLessThanOrEqual(testCase, norm(exact(1:2)-log.q(index+1, 1:2)'), ...
            log.integrationErrorBound(index));
        guard = cfg;
        guard.geometry.margin = guard.geometry.margin+log.integrationErrorBound(index);
        verifyTrue(testCase, robot.geometry.checkPrimitive(q, u, dt, scene, guard));
    end
end

function [cfg, scene, plan] = straightPlan()
    cfg = robot.config.defaultConfig();
    cfg.planning.maxIterations = 0;
    scene = struct('bounds', [0, 10; 0, 8], 'circles', zeros(0, 3), ...
        'polygons', {{}}, 'q0', [1; 1; 0], 'qf', [1.8; 1; 0]);
    plan = robot.planning.planRRT(scene, cfg);
end
