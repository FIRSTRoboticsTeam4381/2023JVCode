// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogOrDash;
import frc.robot.RobotContainer;
import frc.robot.commands.SparkMaxPosition;
import frc.robot.commands.TalonSRXPosition;

public class Winch extends SubsystemBase {
  private TalonSRX armWinch;
  private TalonSRX armWinch2;

  public Command JoystickWinch ( Supplier <Double> joystickPower, Supplier <Boolean> buttonStatus ) {
    return new RunCommand(() -> {
      double power = joystickPower.get() * 6773;
      if(Math.abs(power) < 0.025 * 6773) {
        power = 0;
      }

      if(buttonStatus.get() == false) {
        power = power/4;
      }
      armWinch.set(TalonSRXControlMode.Velocity, power);
      
      //armWinch.set(TalonSRXControlMode.PercentOutput, power);
    }, this);

  }

// 482 RPM
// 8096 ticks/revolution
// Need target change per 100ms
// 6476
// 5.33 before
// 4300
// 99830 rpm
// 5.88 rev/100ms
// 52773

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
    winchConfig.forwardSoftLimitThreshold = 28570;
    winchConfig.reverseSoftLimitThreshold = 0;

    winchConfig.closedloopRamp = 0.05;

    winchConfig.slot0.kP = 0.4;
    winchConfig.slot0.kI = 0;
    winchConfig.slot0.kD = 0;

    winchConfig.slot1.kP = 0.6;
    winchConfig.slot1.kI = 0.0015;
    winchConfig.slot1.kD = 0;
    winchConfig.slot1.integralZone = 500;
    winchConfig.slot1.maxIntegralAccumulator = 10000000;
    
    armWinch.configAllSettings(winchConfig);

    armWinch.enableVoltageCompensation(true);
    armWinch2.enableVoltageCompensation(true);
    armWinch.setNeutralMode(NeutralMode.Brake);
    armWinch2.setNeutralMode(NeutralMode.Brake);

    armWinch.selectProfileSlot(1, 0);

    armWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LogOrDash.logNumber("winch/m1/position", armWinch.getSelectedSensorPosition());
    LogOrDash.logNumber("winch/m1/velocity", armWinch.getSelectedSensorVelocity());
    LogOrDash.logNumber("winch/m1/setspeed", armWinch.getMotorOutputPercent());
    LogOrDash.logNumber("winch/m1/temperature", armWinch.getTemperature());
    LogOrDash.logNumber("winch/m1/statorcurrent", armWinch.getStatorCurrent());
    LogOrDash.logNumber("winch/m1/supplycurrent", armWinch.getSupplyCurrent()); 
    LogOrDash.logNumber("winch/m1/looperror",armWinch.getClosedLoopError());
  //  SmartDashboard.putNumber("winch/m1/target",armWinch.getClosedLoopTarget());
    LogOrDash.logNumber("winch/m1/derivative",armWinch.getErrorDerivative());
    LogOrDash.logNumber("winch/m1/iaccum",armWinch.getIntegralAccumulator());
    LogOrDash.logNumber("winch/m2/position", armWinch2.getSelectedSensorPosition());
    LogOrDash.logNumber("winch/m2/velocity", armWinch2.getSelectedSensorVelocity());
    LogOrDash.logNumber("winch/m2/setspeed", armWinch2.getMotorOutputPercent());
    LogOrDash.logNumber("winch/m2/temperature", armWinch2.getTemperature());
    LogOrDash.logNumber("winch/m2/statorcurrent", armWinch2.getStatorCurrent());
    LogOrDash.logNumber("winch/m2/supplycurrent", armWinch2.getSupplyCurrent());
    
    // Fault detection
    Faults f = new Faults();
    armWinch.getFaults(f);

    if(f.hasAnyFault())
    {
      LogOrDash.logString("winch/m1/faults", f.toString());
    }

    armWinch2.getFaults(f);
    if(f.hasAnyFault())
    {
        LogOrDash.logString("winch/m2/faults", f.toString());
    }

    // Reboot detection
    if(armWinch.hasResetOccurred())
    {
        DriverStation.reportError("ALERT: Winch motor 1 has crashed!", false);
    }
    if(armWinch2.hasResetOccurred())
    {
        DriverStation.reportError("ALERT: Winch motor 2 has crashed!", false);
    }
  }

  public TalonSRXPosition goToPosition( double pos, double err) {
    return new TalonSRXPosition(armWinch, pos, 1, err, this);
  }

  public FunctionalCommand WinchResetOverride()
  {
    return new FunctionalCommand(() -> {
      RobotContainer.leds.setColors(1, 0, 0);
      armWinch.configReverseSoftLimitEnable(false);
      armWinch.set(ControlMode.PercentOutput, -0.1);
      
    }, 
    () -> {},
    interruped -> {
      RobotContainer.leds.setColors(0, 1, 0);
      armWinch.set(ControlMode.PercentOutput, 0);
      armWinch.setSelectedSensorPosition(0);
      armWinch.configReverseSoftLimitEnable(true);
    }, () -> {return false;}, this);
  }
}
