// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera m_camera = new PhotonCamera("camera");
  PhotonTrackedTarget m_target;
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    m_target = m_camera.getLatestResult().getBestTarget();
  }

  public double getYawDegrees() {
    return m_target.getYaw();
  }

  public double getPitchDegrees() {
    return m_target.getPitch();
  }

  public double getYawRadians() {
    return Units.degreesToRadians(m_target.getYaw());
  }

  public double getPitchRadians() {
    return Units.degreesToRadians(m_target.getPitch());
  }

  public Pose3d getTargetPose() {
    return new Pose3d();
  }

  public Pose3d getCameraPose() {
    Transform3d cameraToTarget = m_target.getBestCameraToTarget();
    return getTargetPose().transformBy(cameraToTarget.inverse());
  }

  public Pose3d getRobotPose() {
    return getCameraPose().transformBy(CAMERA_TO_ROBOT);
  }

  public Pose3d getTurretPose() {
    return getCameraPose().transformBy(CAMERA_TO_TURRET);
  }

  /** Calculate the number of rotations required to look at the cube node */
  public double calculateCube() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = getTargetPose().transformBy(TAG_TO_CUBE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }

  /** Calculate the number of rotations required to look at left node */
  public double calculateLeftNode() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = getTargetPose().transformBy(TAG_TO_LEFT_CONE_NODE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }

  /** Calculate the number of rotations required to look at the right node */
  public double calculateRightNode() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = getTargetPose().transformBy(TAG_TO_RIGHT_CONE_NODE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }
}
