// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator photonPoseEstimator;

  //Is this the 2025 field?
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


  /** Creates a new Vision. */
  public Vision() {
    
    //Name the camera this
    camera = new PhotonCamera("camera");
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,Constants.VisionConstants.ROBOT_TO_CAM);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
