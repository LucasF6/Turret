// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import static frc.robot.RobotContainer.m_joystick;
import static frc.robot.Constants.Turret.*;

public class DefaultTurret extends CommandBase {
  TurretSubsystem m_turretSubsystem;

  /** Creates a new DefaultTurret. */
  public DefaultTurret(TurretSubsystem subsystem) {
    m_turretSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = m_turretSubsystem.getSetpoint() + MANUAL_ROTATE_SCALE * m_joystick.getTwist();
    if (HARD_MIN_ANGLE < setpoint && setpoint < HARD_MAX_ANGLE) {
      m_turretSubsystem.setSetpoint(setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
