// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class Medic extends CommandBase {
  /** Creates a new medic. */
  Swerve swerve;
  Orchestra o;

  public Medic(Swerve s) {
    swerve = s;

    addRequirements(swerve);

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Collect talons
    ArrayList<TalonFX> talons = new ArrayList<TalonFX>();
    for(SwerveModule m : swerve.mSwerveMods)
    {
      talons.add(m.mAngleMotor);
      talons.add(m.mDriveMotor);
    }
    o = new Orchestra(talons, "medic.chrp");

    o.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    o.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
