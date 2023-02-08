// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  public Command toggleGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 10000);
      } else {
        Gripper.set(ControlMode.Position, 0);
      }
    });
  }

  /** Creates a new Gripper. */
  public Gripper() {
    Gripper = new TalonSRX(3);

    TalonSRXConfiguration gripperConfig = new TalonSRXConfiguration();
    gripperConfig.slot0.kP = 1;
    gripperConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    Gripper.configAllSettings(gripperConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    SmartDashboard.putNumber("Gripper: ", Gripper.getSelectedSensorPosition());
  }
}
