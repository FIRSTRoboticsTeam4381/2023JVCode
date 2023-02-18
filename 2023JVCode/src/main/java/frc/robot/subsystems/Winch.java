// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
  private CANSparkMax armWinch;

  public Command JoystickWinch ( Supplier <Double> joystickPower ) {
    return new RunCommand(() -> {
      double power = joystickPower.get();
      armWinch.set(power);
    }, this);
  }

  public Command armPivotPosition ( double winchDirection ) {
    return new StartEndCommand(() -> {
      armWinch.set(winchDirection);
    }, () -> {
      armWinch.set(0);
    });
  }

  /** Creates a new LiftArm. */
  public Winch() {
    armWinch = new CANSparkMax(4, MotorType.kBrushless);

    armWinch.enableVoltageCompensation(12);
    armWinch.setSmartCurrentLimit(20);
    armWinch.setSoftLimit(SoftLimitDirection.kForward, 325);
    armWinch.setSoftLimit(SoftLimitDirection.kReverse, 0);
    armWinch.enableSoftLimit(SoftLimitDirection.kForward, true);
    armWinch.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch", armWinch.getEncoder().getPosition());
  }
}