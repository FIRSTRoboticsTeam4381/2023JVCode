// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private TalonFX Gripper;
  public boolean gripperOn;

  // public void GripperPower ( double gripPosition ) { 
  //   Gripper.set(ControlMode.Position, gripPosition);
  // }

  public Command toggleGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 100);
      } else {
        Gripper.set(ControlMode.Position, 0);
      }
    });
  }

  /** Creates a new Gripper. */
  public Gripper() {
    Gripper = new TalonFX(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
