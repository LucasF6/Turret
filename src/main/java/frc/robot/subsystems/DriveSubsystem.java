// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(1);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public WPI_Pigeon2 getGyro() {
    return m_gyro;
  }

  public double getYawDegrees() {
    return m_gyro.getYaw();
  }

  public double getYawRadians() {
    return Units.degreesToRadians(m_gyro.getYaw());
  }

  public double getPitchDegrees() {
    return m_gyro.getPitch();
  }

  public double getPitchRadians() {
    return Units.degreesToRadians(m_gyro.getPitch());
  }
}
