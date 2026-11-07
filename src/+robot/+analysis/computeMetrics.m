function summary = computeMetrics(log)
%COMPUTEMETRICS Compute tracking, goal and collision metrics from the log.
%   Mean and RMS tracking errors use trapezoidal time weights.

    log = robot.analysis.validateLog(log);
    cfg = log.configuration;
    n = size(log.q, 1);
    duration = log.time(end)-log.time(1);
    tracking = log.trackingError;
    if duration > 0
        meanTracking = trapz(log.time, tracking)/duration;
        rmsTracking = sqrt(trapz(log.time, tracking.^2)/duration);
    else
        meanTracking = tracking(1);
        rmsTracking = tracking(1);
    end
    physical = zeros(n, 1);
    safe = zeros(n, 1);
    for index = 1:n
        p = log.q(index, 1:2)';
        physical(index) = robot.geometry.clearance(p, log.scene, cfg.robot.bodyRadius);
        safe(index) = robot.geometry.clearance(p, log.scene, ...
            cfg.robot.bodyRadius+cfg.geometry.margin);
    end
    collisionSamples = physical <= cfg.geometry.tolerance;
    marginViolations = safe <= cfg.geometry.tolerance;
    summary.finalPositionError_m = log.finalPositionError;
    summary.finalOrientationError_rad = log.finalAngleError;
    summary.trackingMean_m = robot.internal.ensureFinite(meanTracking);
    summary.trackingRMS_m = robot.internal.ensureFinite(rmsTracking);
    summary.trackingMaximum_m = max(tracking);
    summary.trackingUpperMaximum_m = max(tracking+log.trackingApproximationBound);
    summary.duration_s = duration;
    summary.poseCount = n;
    summary.appliedIntervalCount = n-1;
    summary.goalReached = log.goalReached;
    summary.status = log.status;
    summary.terminationMessage = log.message;
    summary.collisionDetectedAtRecordedPoses = any(collisionSamples);
    summary.collisionPoseCount = sum(collisionSamples);
    summary.safetyMarginViolationAtRecordedPoses = any(marginViolations);
    summary.minimumRecordedSafetyClearance_m = min(safe);
    summary.uncertifiedIntervalCount = sum(log.clearanceBound <= cfg.geometry.tolerance);
    summary.motionCertifiedInSimulationLog = ~any(marginViolations) && ...
        summary.uncertifiedIntervalCount == 0;
    % A blocked command was not applied: do not count it as an actual collision.
    summary.commandOrTrackingBlocked = strcmp(log.status, 'tracking_blocked');
end
