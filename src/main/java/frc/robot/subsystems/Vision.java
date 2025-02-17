// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
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

public class Vision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator photonPoseEstimator;

  private List<PhotonPipelineResult> latestResults;

  private final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private final static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /** Creates a new Vision. */
  public Vision() {
    
    //Name the camera
    camera = new PhotonCamera(VisionConstants.kCamName1);

    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      poseStrategy,
      VisionConstants.kRobotToCam);

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  /**
   * 
   * @return A list of poses from the latest results
   */
  public ArrayList<EstimatedRobotPose> getEstimatedPoses() {
    ArrayList<EstimatedRobotPose> list = new ArrayList<EstimatedRobotPose>(20);
    for(PhotonPipelineResult result : latestResults) {
      //Add matrices
      Optional<EstimatedRobotPose> optPose = photonPoseEstimator.update(result);
      if (optPose.isPresent()) {
        list.add(optPose.get());
      }
    }
    return list;
  }

  //Returns the best target of the latest result (if it exists)
  public Optional<PhotonTrackedTarget> getTarget() {
    Optional<PhotonTrackedTarget> optTarget = null;
    if (latestResults.size() > 0) {
      optTarget = Optional.of(latestResults.get(latestResults.size() - 1).getBestTarget());
    }
    return optTarget;
  }

  /**
   * Sets the reference pose for use in the fallback strategy.
   * @param pose Pose2d
   */
  public void setReferencePose(Pose2d pose) {
    photonPoseEstimator.setReferencePose(pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latestResults = camera.getAllUnreadResults();
  }
}
