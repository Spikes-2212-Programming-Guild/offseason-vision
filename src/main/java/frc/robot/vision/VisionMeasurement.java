package frc.robot.vision;

import limelight.networktables.PoseEstimate;

public record VisionMeasurement(PoseEstimate poseEstimate, StandardDeviations standardDeviations) {
}
