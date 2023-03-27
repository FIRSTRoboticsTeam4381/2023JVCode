// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

public class UpRamp extends CommandBase {
  /** Creates a new UpRamp. */
  Swerve swerveDrive;

  boolean direction;
  double time;
  boolean retry;
  /** Creates a new Balance. */
  public UpRamp(Swerve mainDrive) {
    swerveDrive = mainDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) mainDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 1+Timer.getFPGATimestamp();
    direction = false;
    swerveDrive.drive(new Translation2d(-4.5,0), 0, false, true);
    retry = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > time) {
      direction = !direction;

      if (direction) {
        time = 1+Timer.getFPGATimestamp();

        swerveDrive.drive(new Translation2d(1,0), 0, false, true);

        retry = true;
      } else {
        time = 1+Timer.getFPGATimestamp();
        swerveDrive.drive(new Translation2d(-4.5,0), 0, false, true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double robotPitch = swerveDrive.gyro.getPitch(); // (Previous )For Forward to Back

    return robotPitch >= 10;
  }

  public ConditionalCommand overAndBack(){
    return new ConditionalCommand(
      new WaitCommand(0),
      new SequentialCommandGroup(
      new RunCommand(() -> swerveDrive.drive(new Translation2d(-1.5,0), 0, false, true), swerveDrive).until(() -> swerveDrive.gyro.getPitch() < 1),
      new RunCommand(() -> swerveDrive.drive(new Translation2d(-1.5,0), 0, false, true), swerveDrive).until(() -> swerveDrive.gyro.getPitch() < -8),
      new RunCommand(() -> swerveDrive.drive(new Translation2d(-1.5,0), 0, false, true), swerveDrive).until(() -> swerveDrive.gyro.getPitch() > -1),
      new WaitCommand(.5),
      new RunCommand(() -> swerveDrive.drive(new Translation2d(3.5,0), 0, false, true), swerveDrive).until(() -> swerveDrive.gyro.getPitch() < -8),
      new WaitCommand(0.25)
      ),
    () -> retry); 
  }
}
