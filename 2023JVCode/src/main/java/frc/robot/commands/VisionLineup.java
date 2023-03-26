// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.LogOrDash;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;

public class VisionLineup extends CommandBase {
  /** Creates a new VisionLineup. */
  Limelight limelight;
  Swerve swerve;
  LEDs leds;
  int pipeline;
  PIDController x;
  PIDController y;
  boolean invertY;
  public VisionLineup(Swerve s, Limelight lime, LEDs led, int p, boolean i)  {
    // Use addRequirements() here to declare subsystem dependencies
    limelight = lime;
    swerve = s;
    leds = led;
    pipeline = p;
    invertY = i;

    x = new PIDController(0.1, 0, 0);
    y = new PIDController(0.3, 0, 0);
    x.setTolerance(0.7);
    y.setTolerance(0.7);
    x.setSetpoint(0);
    y.setSetpoint(0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.pipeline(pipeline);

    limelight.snapshot(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.hasTarget()){
      limelight.snapshot(0);
        double limeX = x.calculate(limelight.getX());
        double limeY = y.calculate(limelight.getY());
        if(invertY){
          limeY = limeY * -1;
        }
        LogOrDash.logNumber("limeSpeeds/x", limeX);
        LogOrDash.logNumber("limeSpeeds/y", limeY);
        leds.setColors(0, 1-(Math.abs(limeY)/20), 1-(Math.abs(limeX)/20));
        swerve.drive(new Translation2d(limeY,limeX), 0, false, true);
    }else{
      swerve.drive(new Translation2d(0,0), 0, false, true);
      leds.setColors(1, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0, false, true);
    limelight.pipeline(0);
    leds.setColors(0, 1, 0);
    limelight.snapshot(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.hasTarget() && x.atSetpoint() && y.atSetpoint();
  }
}
