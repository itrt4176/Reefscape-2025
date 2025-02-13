// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator photonPoseEstimator;

  private final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private final static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /** Creates a new Vision. */
  public Vision() {
    
    //Name the camera
    camera = new PhotonCamera(VisionConstants.kCamName1);
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout,poseStrategy,VisionConstants.kRobotToCam);
  }

  public Optional<PhotonPipelineResult> getResult() {
    var results = camera.getAllUnreadResults();
    PhotonPipelineResult result = null;
    if (!results.isEmpty()) {
        result = results.get(results.size() - 1);
    }
    return Optional.of(result);
  }

  public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPipelineResult result) {
    return photonPoseEstimator.update(result);
  }

  public Optional<PhotonTrackedTarget> getTarget(PhotonPipelineResult result) {
    return Optional.of(result.getBestTarget());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<PhotonPipelineResult> optResult = getResult();
    EstimatedRobotPose estimatedPose = null;

    if (optResult.isPresent()) {
      PhotonPipelineResult result = optResult.get();
      Optional<EstimatedRobotPose> optEstimatedPose = photonPoseEstimator.update(result);
      if (optEstimatedPose.isPresent()) {
        estimatedPose = optEstimatedPose.get();
      }
    }

    

  }
}
