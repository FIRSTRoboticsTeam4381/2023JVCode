// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Wrapper for Limelight network tables
public class Limelight extends SubsystemBase {

  boolean driverMode = false;

  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command toggleCameraMode()
  {
    return new InstantCommand( () -> {
      driverMode = !driverMode;

      if(driverMode)
      {
        // Set to driver mode
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
      }
      else
      {
        // Set to vision processing mode
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
      }

    }, this);
  }
  public double getX(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  public double getY(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  public boolean hasTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }
  public void pipeline(int p){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(p);
  }

  public void snapshot(int p){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(p);
  }
}
