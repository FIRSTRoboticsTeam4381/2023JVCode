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
      swerveDrive.drive(new Translation2d(-0.56381557247154503043246556639484,-0.20521208599540123982645976880936), 0, false, true);//-0.6,
    } else if (robotPitch < -12.5) {
      swerveDrive.drive(new Translation2d(0.56381557247154503043246556639484,0.20521208599540123982645976880936), 0, false, true);//0.6
    } else if (robotPitch > 8) {
      swerveDrive.drive(new Translation2d(-0.46984631039295419202705463866237, -0.17101007166283436652204980734113), 0, false, true);//-0.5
    } else if (robotPitch < -8) {
      swerveDrive.drive(new Translation2d(0.46984631039295419202705463866237,0.17101007166283436652204980734113), 0, false, true);//0.5
    } else if (robotPitch > 3.5) {
      swerveDrive.drive(new Translation2d(-0.2819077862357725152162327831974,-0.10260604299770061991322988440468), 0, false, true);//-0.3
    } else if (robotPitch < -3.5) {
      swerveDrive.drive(new Translation2d(0.28190778623577251521623278319742,0.102606042997700619913229884404680), 0, false, true);//0.3
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
