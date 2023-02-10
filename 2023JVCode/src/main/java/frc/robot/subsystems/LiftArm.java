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
  private CANSparkMax wristPivot;
  private CANSparkMax armWinch;

  public Command armPivotPosition ( double winchDirection ) {
    return new StartEndCommand(() -> {
      armWinch.set(winchDirection);
    }, () -> {
      armWinch.set(0);
    });
  }

  public Command wristPivotPosition ( double pivotDirection ) {
    return new StartEndCommand(() -> {
      wristPivot.set(pivotDirection);
    }, () -> {
      wristPivot.set(0);
    });
  }

  /** Creates a new LiftArm. */
  public LiftArm() {
    wristPivot = new CANSparkMax(5, MotorType.kBrushless);
    armWinch = new CANSparkMax(4, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot:", wristPivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Winch", armWinch.getEncoder().getPosition());
  }
}
