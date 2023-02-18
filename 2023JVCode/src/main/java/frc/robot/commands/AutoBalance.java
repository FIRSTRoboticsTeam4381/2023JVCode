// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBallence. */
  Swerve swerveDrive;
  public AutoBalance(Swerve mainDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = mainDrive;
    
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(swerveDrive.gyro.getPitch() > 4){
      swerveDrive.drive(new Translation2d(0,0), 0, true, isFinished());
    }
    if(swerveDrive.gyro.getPitch() < -4){
      swerveDrive.drive(new Translation2d(0,0), 0, true, isFinished());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
