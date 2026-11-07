function tests = test_planning
%TEST_PLANNING Essential numerical planning tests.
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

function testSignedClearanceAndFootprint(testCase)
    [cfg, scene] = emptyScene();
    scene.circles = [3, 3, 0.5];
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    verifyEqual(testCase, robot.geometry.clearance([4; 3], scene, radius), 0.25, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.geometry.clearance([3; 3], scene, radius), -0.75, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.geometry.clearance([0.25; 4], scene, radius), 0, 'AbsTol', 1e-12);
    scene.circles = zeros(0, 3);
    scene.polygons = {[4, 2; 6, 2; 6, 4; 4, 4]};
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    verifyEqual(testCase, robot.geometry.clearance([3; 3], scene, radius), 0.75, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.geometry.clearance([5; 3], scene, radius), -1.25, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.geometry.clearance([3.75; 3], scene, radius), 0, 'AbsTol', 1e-12);
    scene.polygons = {[4, 2; 6, 2; 6, 4; 5, 4; 5, 3; 4, 3]};
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    verifyEqual(testCase, robot.geometry.clearance([4.5; 3.5], scene, radius), 0.25, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.geometry.clearance([5.5; 3.5], scene, radius), -0.75, 'AbsTol', 1e-12);
end

function testCollisionBetweenFreeEndpoints(testCase)
    [cfg, scene] = emptyScene();
    scene.circles = [2, 1, 0.01];
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    verifyGreaterThan(testCase, robot.geometry.clearance([1; 1], scene, radius), 0);
    verifyGreaterThan(testCase, robot.geometry.clearance([3; 1], scene, radius), 0);
    valid = robot.geometry.checkPrimitive([1; 1; 0], [4; 4], 10, scene, cfg);
    verifyFalse(testCase, valid);
end

function testCurvedPrimitiveIsNotItsChord(testCase)
    [cfg, scene] = emptyScene();
    q0 = [1; 1; 0];
    % [6;5] gives v=0.275, omega=1/6, radius=1.65 m.
    % Obstacle at chord midpoint; the actual quarter circle stays clear.
    scene.circles = [1.825, 1.825, 0.02];
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    [valid, endpoint, bound] = robot.geometry.checkPrimitive(q0, [6; 5], 3*pi, scene, cfg);
    verifyTrue(testCase, valid);
    verifyGreaterThan(testCase, bound, cfg.geometry.tolerance);
    verifyEqual(testCase, endpoint, [2.65; 2.65; pi/2], 'AbsTol', 1e-12);
    % Now place the obstacle on the true arc, still away from both ends.
    scene.circles = [1+1.65/sqrt(2), 1+1.65*(1-1/sqrt(2)), 0.02];
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    verifyFalse(testCase, robot.geometry.checkPrimitive(q0, [6; 5], 3*pi, scene, cfg));
end

function testUncertifiedAndOutOfLimitPrimitivesRejected(testCase)
    [cfg, scene] = emptyScene();
    cfg.geometry.maxDepth = 0;
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    [valid, ~, ~, reason] = robot.geometry.checkPrimitive([1; 1; 0], [4; 4], 1, scene, cfg);
    verifyFalse(testCase, valid);
    verifyEqual(testCase, reason, 'uncertified');
    verifyFalse(testCase, robot.geometry.checkPrimitive([1; 1; 0], [20; 20], 1, scene, cfg));
    verifyFalse(testCase, robot.geometry.checkPrimitive([1; 1; 0], [-4; -4], 1, scene, cfg));
end

function testCoupledWheelBounds(testCase)
    [cfg, scene] = emptyScene();
    [cfg, ~] = robot.config.validatePlanningInputs(cfg, scene);
    [u, applied] = robot.planning.boundedCommand([0.3; 1.5], cfg);
    verifyLessThanOrEqual(testCase, max(abs(u)), cfg.limits.wheel+1e-12);
    verifyEqual(testCase, applied(2)/applied(1), 5, 'AbsTol', 1e-12);
    verifyEqual(testCase, u(1), 10, 'AbsTol', 1e-12);
end

function testRRTExpansionAndGoalReconstruction(testCase)
    [cfg, scene] = emptyScene();
    scene.qf = [2.2; 1; pi/2];
    % Ignore goal angle in this numerical fixture only by making its weight
    % small: the deterministic forward candidate must improve proximity.
    cfg.planning.headingWeight = 0.01;
    cfg.planning.goalBias = 1;
    cfg.planning.numCandidates = 3;
    cfg.planning.durationRange = [0.8, 0.8];
    cfg.planning.maxIterations = 2;
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    verifyGreaterThan(testCase, plan.iterations, 0);
    verifyPlan(testCase, plan, scene, cfg);
    % The connector's final pure rotation must remain in the sampled path.
    xySteps = diff(plan.path.samples(:, 1:2), 1, 1);
    angleSteps = robot.model.normalizeAngle(diff(plan.path.samples(:, 3)));
    verifyTrue(testCase, any(hypot(xySteps(:, 1), xySteps(:, 2)) < 1e-12 & abs(angleSteps) > 0.01));
end

function testCoincidentPosesAndPureRotation(testCase)
    [cfg, scene] = emptyScene();
    cfg.planning.maxIterations = 0;
    scene.qf = scene.q0;
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    verifySize(testCase, plan.path.samples, [1, 3]);
    verifyEmpty(testCase, plan.path.controls);
    verifySize(testCase, plan.path.durations, [0, 1]);
    verifySize(testCase, plan.path.clearances, [0, 1]);
    scene.qf(3) = pi/2;
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    verifyPlan(testCase, plan, scene, cfg);
    verifyEqual(testCase, plan.path.samples(:, 1:2), ...
        repmat(scene.q0(1:2)', size(plan.path.samples, 1), 1), 'AbsTol', 1e-12);
end

function testBarrierHasNoFalseConnection(testCase)
    [cfg, scene] = emptyScene();
    scene.q0 = [4; 4; 0];
    scene.qf = [6; 4; 0];
    scene.polygons = {[4.5, 0; 5.5, 0; 5.5, 8; 4.5, 8]};
    cfg.planning.connectDistance = 3;
    cfg.planning.maxIterations = 0;
    plan = robot.planning.planRRT(scene, cfg);
    verifyEqual(testCase, plan.status, 'budget_exhausted');
    verifyEmpty(testCase, plan.path);
end

function testSmallBearingWithTightPositionTolerance(testCase)
    [cfg, scene] = emptyScene();
    scene.qf = [1.5; 1+1e-6; 0];
    cfg.planning.goalPositionTolerance = 1e-10;
    cfg.planning.goalAngleTolerance = 1e-4;
    cfg.planning.maxIterations = 0;
    % Bearing is about 2e-6 rad: below final-angle tolerance, but cannot be
    % skipped before a half-metre translation with 1e-10 m position tolerance.
    plan = robot.planning.planRRT(scene, cfg);
    assertEqual(testCase, plan.status, 'success');
    verifyLessThanOrEqual(testCase, norm(plan.path.states(end, 1:2)'-scene.qf(1:2)), ...
        cfg.planning.goalPositionTolerance);
    verifyLessThanOrEqual(testCase, abs(robot.model.normalizeAngle( ...
        plan.path.states(end, 3)-scene.qf(3))), cfg.planning.goalAngleTolerance);
end

function testInvalidScenesAndConfigurations(testCase)
    [cfg, scene] = emptyScene();
    scene.q0(1) = 0.1; % Centre is inside bounds, but the disc is not.
    plan = robot.planning.planRRT(scene, cfg);
    verifyEqual(testCase, plan.status, 'invalid_input');
    [cfg, scene] = emptyScene();
    scene.polygons = {[3, 2; 6, 5; 3, 5; 5, 2]}; % Self-intersection.
    plan = robot.planning.planRRT(scene, cfg);
    verifyEqual(testCase, plan.status, 'invalid_input');
    [cfg, scene] = emptyScene();
    cfg.robot.bodyRadius = 0.01;
    plan = robot.planning.planRRT(scene, cfg);
    verifyEqual(testCase, plan.status, 'invalid_input');
    cfg = robot.config.defaultConfig();
    cfg.planning.durationRange = [1, 0.1];
    plan = robot.planning.planRRT(scene, cfg);
    verifyEqual(testCase, plan.status, 'invalid_input');
end

function verifyPlan(testCase, plan, scene, cfg)
    [cfg, scene] = robot.config.validatePlanningInputs(cfg, scene);
    path = plan.path;
    verifyEqual(testCase, path.states(1, :)', scene.q0, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, norm(path.states(end, 1:2)'-scene.qf(1:2)), ...
        cfg.planning.goalPositionTolerance);
    verifyLessThanOrEqual(testCase, abs(robot.model.normalizeAngle( ...
        path.states(end, 3)-scene.qf(3))), cfg.planning.goalAngleTolerance);
    verifyEqual(testCase, size(path.states, 1), size(path.controls, 1)+1);
    verifyGreaterThan(testCase, diff(path.time), 0);
    verifyEqual(testCase, path.time(end), sum(path.durations), 'AbsTol', 1e-12);
    for edge = 1:size(path.controls, 1)
        u = path.controls(edge, :)';
        q = path.states(edge, :)';
        replay = robot.model.propagateConstant(q, u, path.durations(edge), cfg.robot);
        verifyEqual(testCase, replay, path.states(edge+1, :)', 'AbsTol', 1e-12);
        [valid, ~, bound] = robot.geometry.checkPrimitive(q, u, path.durations(edge), scene, cfg);
        verifyTrue(testCase, valid);
        verifyGreaterThan(testCase, bound, cfg.geometry.tolerance);
        dq = robot.model.derivative(q, u, cfg.robot);
        verifyEqual(testCase, [-sin(q(3)), cos(q(3)), 0]*dq, 0, 'AbsTol', 1e-12);
        verifyLessThanOrEqual(testCase, max(abs(u)), cfg.limits.wheel+1e-12);
    end
    radius = cfg.robot.bodyRadius + cfg.geometry.margin;
    for sample = 1:size(path.samples, 1)
        verifyGreaterThan(testCase, robot.geometry.clearance( ...
            path.samples(sample, 1:2)', scene, radius), cfg.geometry.tolerance);
    end
end

function [cfg, scene] = emptyScene()
    cfg = robot.config.defaultConfig();
    scene = struct('bounds', [0, 10; 0, 8], 'circles', zeros(0, 3), ...
        'polygons', {{}}, 'q0', [1; 1; 0], 'qf', [9; 7; 0]);
end
