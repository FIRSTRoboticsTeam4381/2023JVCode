// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkMaxPosition;

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

    wristPivot.getPIDController().setP(1, 1);
    wristPivot.getPIDController().setI(0,1);
    wristPivot.getPIDController().setD(0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist/position", wristPivot.getEncoder().getPosition());
    SmartDashboard.putNumber("wrist/velocity", wristPivot.getEncoder().getVelocity());
    SmartDashboard.putNumber("wrist/setspeed", wristPivot.get());
    SmartDashboard.putNumber("wrist/appliedoutput", wristPivot.getAppliedOutput());
    SmartDashboard.putNumber("wrist/temperature", wristPivot.getMotorTemperature());
    SmartDashboard.putNumber("wrist/outputcurrent", wristPivot.getOutputCurrent());

    SmartDashboard.putBoolean("wrist/reverselimit", wristPivot.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
  }

  public SparkMaxPosition goToPosition (double pos, double err) {
    return new SparkMaxPosition(wristPivot, pos, 1, err, this);
  }
}
