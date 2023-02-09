// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystemAlt2 extends SubsystemBase {
  CANSparkMax m_turretMotor = new CANSparkMax(1, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_turretMotor.getEncoder();
  ProfiledPIDController m_turretPID = new ProfiledPIDController(
    P, I, D,
    new TrapezoidProfile.Constraints(MAX_VELOCITY * GEAR_RATIO, MAX_ACCELERATION * GEAR_RATIO)
  );

  /** Creates a new TurretSubsystemAlt2. */
  public TurretSubsystemAlt2() {
    m_turretMotor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    double measurement = m_encoder.getPosition();
    double output = m_turretPID.calculate(measurement);

    /**
    SmartDashboard.putNumber("Turret Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Turret Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Turret Setpoint", m_turretPID.getGoal().position);
    SmartDashboard.putNumber("ProfiledPIDController output", output);
    */

    if (giveNormalOutput(measurement, output > 0)) {
      // Give the turret the output from the profiled PID if within soft limits or moving towards them
      m_turretMotor.setVoltage(output);
    } else if (giveScaledOutput(measurement, output > 0)) {
      // Give the turret scaled down output if outside soft limits and moving towards hard limits
      m_turretMotor.setVoltage(BOUNDARY_SCALE * output);
    } else {
      // Bring the turret back to 0 if it went outside hard limits
      // Possibly change later
      m_turretMotor.setVoltage(0);
      m_turretPID.setGoal(0);
    }
  }

  public void reset() {
    m_encoder.setPosition(0);
    m_turretPID.setGoal(0);
  }

  public void setSetpoint(double setpoint) {
    m_turretPID.setGoal(setpoint);
  }

  public double getSetpoint() {
    return m_turretPID.getGoal().position;
  }

  private boolean giveNormalOutput(double measurement, boolean movingRight) {
    return 
      (SOFT_MIN_ANGLE <= measurement && measurement <= SOFT_MAX_ANGLE)
      || (HARD_MIN_ANGLE <= measurement && measurement < SOFT_MIN_ANGLE && movingRight)
      || (HARD_MAX_ANGLE >= measurement && measurement > HARD_MAX_ANGLE && !movingRight);
  }

  private boolean giveScaledOutput(double measurement, boolean movingRight) {
    return
      (HARD_MIN_ANGLE <= measurement && measurement < SOFT_MIN_ANGLE && !movingRight)
      || (HARD_MAX_ANGLE >= measurement && measurement > SOFT_MAX_ANGLE && movingRight);
  }
}
