// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private CANSparkMax wristPivot;

  public Command JoystickWrist ( Supplier <Double> wristIn, Supplier <Double> wristOut) {
    return new RunCommand(() -> {
      double power = (wristIn.get()*-1 + wristOut.get()) / 2; // Fast math does things
      wristPivot.set(power);
    }, this);
  }

  /* old thing */

  // public Command wristPivotPosition ( double pivotDirection ) {
  //   return new StartEndCommand(() -> {
  //     wristPivot.set(pivotDirection);
  //   }, () -> {
  //     wristPivot.set(0);
  //   });
  // }

  /** Creates a new Wrist. */
  public Wrist() {
    wristPivot = new CANSparkMax(5, MotorType.kBrushless);

    wristPivot.enableVoltageCompensation(12);
    wristPivot.setSmartCurrentLimit(20);
    wristPivot.setSoftLimit(SoftLimitDirection.kForward, 50);
    wristPivot.setSoftLimit(SoftLimitDirection.kReverse, 0);
    wristPivot.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristPivot.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Pivot:", wristPivot.getEncoder().getPosition());
  }
}
