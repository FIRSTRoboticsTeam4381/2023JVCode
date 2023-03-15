// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogOrDash;
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
    wristPivot.setSoftLimit(SoftLimitDirection.kForward, 0.4f);
    wristPivot.setSoftLimit(SoftLimitDirection.kReverse, 0.05f);
    wristPivot.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristPivot.enableSoftLimit(SoftLimitDirection.kReverse, true);

    wristPivot.getPIDController().setFeedbackDevice(wristPivot.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle));

    wristPivot.getPIDController().setP(12, 1);
    wristPivot.getPIDController().setI(0, 1);
    wristPivot.getPIDController().setD(0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //putNumber("wrist/position", wristPivot.getEncoder().getPosition());
    LogOrDash.logNumber("wrist/velocity", wristPivot.getEncoder().getVelocity());
    LogOrDash.logNumber("wrist/setspeed", wristPivot.get());
    LogOrDash.logNumber("wrist/appliedoutput", wristPivot.getAppliedOutput());
    LogOrDash.logNumber("wrist/temperature", wristPivot.getMotorTemperature());
    LogOrDash.logNumber("wrist/outputcurrent", wristPivot.getOutputCurrent());

    LogOrDash.logBoolean("wrist/reverselimit", wristPivot.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());

    LogOrDash.logNumber("wrist/absoluteencoder", wristPivot.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition());
    //SmartDashboard.putNumber("wrist/alternateencoder", wristPivot.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192).getPosition());

    LogOrDash.logNumber("wrist/faults", wristPivot.getFaults());
  }

  public SparkMaxPosition goToPosition (double pos, double err) {
    return new SparkMaxPosition(wristPivot, pos, 1, err, this, wristPivot.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)::getPosition);
  }
}
