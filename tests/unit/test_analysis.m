function tests = test_analysis
%TEST_ANALYSIS Numerical checks using actual planner/simulator outputs only.
%   Prepared for final MATLAB Online execution; not executed in development.
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

function testStationarySummaryHasNoEmptyStatistics(testCase)
    log = actualLog(true, false);
    summary = robot.analysis.computeMetrics(log);
    verifyEqual(testCase, summary.duration_s, 0);
    verifyEqual(testCase, summary.appliedIntervalCount, 0);
    verifyEqual(testCase, summary.trackingRMS_m, 0);
    verifyEqual(testCase, summary.finalPositionError_m, 0);
    verifyEqual(testCase, summary.finalOrientationError_rad, 0);
    verifyTrue(testCase, summary.goalReached);
    verifyFalse(testCase, summary.collisionDetectedAtRecordedPoses);
    verifyTrue(testCase, summary.motionCertifiedInSimulationLog);
end

function testStraightTrackingAndGoalSummary(testCase)
    log = actualLog(false, false);
    summary = robot.analysis.computeMetrics(log);
    verifyTrue(testCase, summary.goalReached);
    verifyGreaterThan(testCase, summary.duration_s, 0);
    verifyEqual(testCase, summary.trackingRMS_m, 0, 'AbsTol', 1e-12);
    verifyEqual(testCase, summary.trackingMaximum_m, 0, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, summary.finalPositionError_m, ...
        log.configuration.control.goalPositionTolerance);
    verifyEqual(testCase, summary.appliedIntervalCount, numel(log.time)-1);
end

function testRejectedCommandIsNotACollision(testCase)
    log = actualLog(false, true);
    summary = robot.analysis.computeMetrics(log);
    verifyFalse(testCase, summary.goalReached);
    verifyTrue(testCase, summary.commandOrTrackingBlocked);
    verifyFalse(testCase, summary.collisionDetectedAtRecordedPoses);
    verifyEqual(testCase, summary.collisionPoseCount, 0);
    verifyTrue(testCase, summary.motionCertifiedInSimulationLog);
end

function testMisalignedAndInconsistentLogsAreRejected(testCase)
    original = actualLog(false, false);
    log = original;
    log.trackingError = log.trackingError(1:end-1);
    verifyError(testCase, @() robot.analysis.validateLog(log), 'RobotProject:InvalidLog');
    log = original;
    log.time(2) = log.time(1);
    verifyError(testCase, @() robot.analysis.validateLog(log), 'RobotProject:InvalidLog');
    log = original;
    log.u(1, 1) = NaN;
    verifyError(testCase, @() robot.analysis.validateLog(log), 'RobotProject:InvalidLog');
    log = original;
    log.finalPositionError = log.finalPositionError+0.1;
    verifyError(testCase, @() robot.analysis.validateLog(log), 'RobotProject:InvalidLog');
end

function log = actualLog(stationary, obstruct)
    cfg = robot.config.defaultConfig();
    cfg.planning.maxIterations = 0;
    scene = struct('bounds', [0, 10; 0, 8], 'circles', zeros(0, 3), ...
        'polygons', {{}}, 'q0', [1; 1; 0], 'qf', [1.8; 1; 0]);
    if stationary
        scene.qf = scene.q0;
    end
    plan = robot.planning.planRRT(scene, cfg);
    if obstruct
        scene.circles = [1.4, 1, 0.02];
    end
    log = robot.simulation.simulate(scene, plan, cfg);
end
