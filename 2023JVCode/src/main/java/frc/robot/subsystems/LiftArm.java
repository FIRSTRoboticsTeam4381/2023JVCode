// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftArm extends SubsystemBase {
  private CANSparkMax pivot;
  private CANSparkMax winch;

  public Command winchPosition ( double winchDirection ) {
    return new StartEndCommand(() -> {
      winch.set(winchDirection);
    }, () -> {
      winch.set(0);
    });
  }

  public Command pivotPosition ( double pivotDirection ) {
    return new StartEndCommand(() -> {
      pivot.set(pivotDirection);
    }, () -> {
      pivot.set(0);
    });
  }

  /** Creates a new LiftArm. */
  public LiftArm() {
    pivot = new CANSparkMax(5, MotorType.kBrushless);
    winch = new CANSparkMax(4, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot:", pivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Winch", winch.getEncoder().getPosition());
  }
}
