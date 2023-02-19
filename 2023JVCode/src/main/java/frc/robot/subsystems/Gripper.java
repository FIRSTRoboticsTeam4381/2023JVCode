// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private TalonSRX Gripper;
  public boolean gripperOn;

  // public void GripperPower ( double gripPosition ) { 
  //   Gripper.set(ControlMode.Position, gripPosition);
  // }

  public Command coneGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 1300);
        
      } else {
        Gripper.set(ControlMode.Position, -100);
        
      }
    });
  }

  public Command cubeGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 1300);
        
      } else {
        Gripper.set(ControlMode.Disabled, 0);
        
      }
    });
  }

  /** Creates a new Gripper. */
  public Gripper() {
    Gripper = new TalonSRX(3);

    TalonSRXConfiguration gripperConfig = new TalonSRXConfiguration();
    gripperConfig.slot0.kP = 1;
    gripperConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    gripperConfig.clearPositionOnLimitR = false;
    gripperConfig.continuousCurrentLimit = 5;
    gripperConfig.peakCurrentLimit = 10;
    gripperConfig.peakCurrentDuration = 10;
    gripperConfig.peakOutputReverse = -1;
    Gripper.configAllSettings(gripperConfig);

    Gripper.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    SmartDashboard.putNumber("Gripper: ", Gripper.getSelectedSensorPosition());
    SmartDashboard.putNumber("isGripperForward", Gripper.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("isGripperReverse", Gripper.isRevLimitSwitchClosed());

  }
  public void ControledGrab(boolean open){
    if (open){
      Gripper.set(ControlMode.PercentOutput, 0.3);
      } else {
      Gripper.set(ControlMode.PercentOutput, 0);
    }

  }
  public void ControledClose(boolean close){
    if (close){
      Gripper.set(ControlMode.PercentOutput, -0.3);
      } else {
      Gripper.set(ControlMode.PercentOutput, 0);
    }

  }
}