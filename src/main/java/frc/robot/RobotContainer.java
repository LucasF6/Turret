// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultTurret;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  public static Joystick m_joystick = new Joystick(0);

  DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  Command m_defaultTurret = new DefaultTurret(m_turretSubsystem);

  public RobotContainer() {
    m_turretSubsystem.setDefaultCommand(m_defaultTurret);
    configureBindings();
  }

  private void configureBindings() {
    // Turn left 90 degrees
    new JoystickButton(m_joystick, 3)
      .onTrue(new InstantCommand(() -> m_turretSubsystem.setSetpoint(-0.25)));

    // Turn right 90 degrees
    new JoystickButton(m_joystick, 4)
      .onTrue(new InstantCommand(() -> m_turretSubsystem.setSetpoint(0.25)));

    // Turn towards cube node
    new JoystickButton(m_joystick, 2)
      .onTrue(new InstantCommand(
        () -> m_turretSubsystem.setSetpoint(m_visionSubsystem.calculateCube())));

    // Turn towards left cone node
    new JoystickButton(m_joystick, 5)
      .onTrue(new InstantCommand(
        () -> m_turretSubsystem.setSetpoint(m_visionSubsystem.calculateLeftNode())));

    // Turn towards right cone node
    new JoystickButton(m_joystick, 6)
      .onTrue(new InstantCommand(
        () -> m_turretSubsystem.setSetpoint(m_visionSubsystem.calculateRightNode())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
