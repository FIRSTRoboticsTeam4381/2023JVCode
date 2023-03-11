// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkMaxPosition;
import frc.robot.commands.TalonSRXPosition;

public class Winch extends SubsystemBase {
  private TalonSRX armWinch;
  private TalonSRX armWinch2;

  public Command JoystickWinch ( Supplier <Double> joystickPower ) {
    return new RunCommand(() -> {
      double power = joystickPower.get();
      armWinch.set(TalonSRXControlMode.PercentOutput, power);
    }, this);
  }



  /** Creates a new LiftArm. */
  public Winch() {
    armWinch = new TalonSRX(4);
    armWinch2 = new TalonSRX(8);
    
    armWinch2.follow(armWinch);
    armWinch.setInverted(InvertType.InvertMotorOutput);
    armWinch2.setInverted(InvertType.FollowMaster);
    TalonSRXConfiguration winchConfig = new TalonSRXConfiguration();
    
    //armWinch.enableVoltageCompensation(12); //Can't figure out voltage comp for talon
    winchConfig.continuousCurrentLimit = 20;
    armWinch2.configAllSettings(winchConfig);

    winchConfig.forwardSoftLimitEnable = true;
    winchConfig.reverseSoftLimitEnable = true;
    winchConfig.forwardSoftLimitThreshold = 200000;
    winchConfig.reverseSoftLimitThreshold = 0;

    winchConfig.slot0.kP = 0.4;
    winchConfig.slot0.kI = 0;
    winchConfig.slot0.kD = 0;
    armWinch.configAllSettings(winchConfig);

    armWinch.enableVoltageCompensation(true);
    armWinch2.enableVoltageCompensation(true);
    armWinch.setNeutralMode(NeutralMode.Brake);
    armWinch2.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("winch/m1/position", armWinch.getSelectedSensorPosition());
    SmartDashboard.putNumber("winch/m1/velocity", armWinch.getSelectedSensorVelocity());
    SmartDashboard.putNumber("winch/m1/setspeed", armWinch.getMotorOutputPercent());
    SmartDashboard.putNumber("winch/m1/temperature", armWinch.getTemperature());
    SmartDashboard.putNumber("winch/m1/statorcurrent", armWinch.getStatorCurrent());
    SmartDashboard.putNumber("winch/m1/supplycurrent", armWinch.getSupplyCurrent()); 
    SmartDashboard.putNumber("winch/m1/looperror",armWinch.getClosedLoopError());
  //  SmartDashboard.putNumber("winch/m1/target",armWinch.getClosedLoopTarget());
    SmartDashboard.putNumber("winch/m1/derivative",armWinch.getErrorDerivative());
    SmartDashboard.putNumber("winch/m1/iaccum",armWinch.getIntegralAccumulator());
    SmartDashboard.putNumber("winch/m2/position", armWinch2.getSelectedSensorPosition());
    SmartDashboard.putNumber("winch/m2/velocity", armWinch2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("winch/m2/setspeed", armWinch2.getMotorOutputPercent());
    SmartDashboard.putNumber("winch/m2/temperature", armWinch2.getTemperature());
    SmartDashboard.putNumber("winch/m2/statorcurrent", armWinch2.getStatorCurrent());
    SmartDashboard.putNumber("winch/m2/supplycurrent", armWinch2.getSupplyCurrent());
    
  }

  public TalonSRXPosition goToPosition( double pos, double err) {
    return new TalonSRXPosition(armWinch, pos, 1, err, this);
  }
}
