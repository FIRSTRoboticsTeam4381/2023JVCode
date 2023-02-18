// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  Swerve swerveDrive;

  /** Creates a new Balance. */
  public Balance(Swerve mainDrive) {
    swerveDrive = mainDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) mainDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotPitch = swerveDrive.gyro.getPitch(); // (Previous )For Forward to Back
    double robotRoll = swerveDrive.gyro.getRoll(); // Sideways Balancing

    if (robotPitch > 12.5) {
      swerveDrive.drive(new Translation2d(-0.6,0), 0, false, true);
    } else if (robotPitch < -12.5) {
      swerveDrive.drive(new Translation2d(0.6,0), 0, false, true);
    } else if (robotPitch > 8.25) {
      swerveDrive.drive(new Translation2d(-0.55,0), 0, false, true);
    } else if (robotPitch < -8.25) {
      swerveDrive.drive(new Translation2d(0.55,0), 0, false, true);
    } else {
      swerveDrive.drive(new Translation2d(0,0), 0, false, true);
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
