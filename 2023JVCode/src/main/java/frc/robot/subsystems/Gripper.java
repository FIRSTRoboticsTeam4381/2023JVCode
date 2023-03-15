// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogOrDash;

public class Gripper extends SubsystemBase {
  private TalonSRX Gripper;
  public boolean gripperOn;

  // public void GripperPower ( double gripPosition ) { 
  //   Gripper.set(ControlMode.Position, gripPosition);
  // }

  public Command coneGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 1600);
        
      } else {
        Gripper.set(ControlMode.Position, -100);
        
      }
    });
  }

  public Command cubeGripper () {
    return new InstantCommand(() -> {
      gripperOn = !gripperOn;

      if (gripperOn) {
        Gripper.set(ControlMode.Position, 1600);
        
      } else {
        Gripper.set(ControlMode.Disabled, 0);
        
      }
    });
  }

  /** Creates a new Gripper. */
  public Gripper() {
    Gripper = new TalonSRX(3);

    TalonSRXConfiguration gripperConfig = new TalonSRXConfiguration();
    gripperConfig.slot0.kP = 1;
    gripperConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    gripperConfig.clearPositionOnLimitR = false;
    gripperConfig.continuousCurrentLimit = 5;
    gripperConfig.peakCurrentLimit = 10;
    gripperConfig.peakCurrentDuration = 10;
    gripperConfig.peakOutputReverse = -1;
    Gripper.configAllSettings(gripperConfig);

    Gripper.setNeutralMode(NeutralMode.Coast);
    //Gripper.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    LogOrDash.logNumber("gripper/position", Gripper.getSelectedSensorPosition());
    LogOrDash.logBoolean("gripper/reverselimit", Gripper.isRevLimitSwitchClosed()==1);
    LogOrDash.logNumber("gripper/looperror",Gripper.getClosedLoopError());
    //SmartDashboard.putNumber("gripper/target",Gripper.getClosedLoopTarget());
    LogOrDash.logNumber("gripper/derivative",Gripper.getErrorDerivative());
    LogOrDash.logNumber("gripper/iaccum",Gripper.getIntegralAccumulator());
    LogOrDash.logNumber("gripper/outpercent",Gripper.getMotorOutputPercent());
    LogOrDash.logNumber("gripper/outvoltage",Gripper.getMotorOutputVoltage());
    LogOrDash.logNumber("gripper/velocity",Gripper.getSelectedSensorVelocity());
    LogOrDash.logNumber("gripper/statorcurrent",Gripper.getStatorCurrent());
    LogOrDash.logNumber("gripper/supplycurrent",Gripper.getSupplyCurrent());
    LogOrDash.logNumber("gripper/controllertemp",Gripper.getTemperature());

    // Fault detection
    Faults f = new Faults();
    Gripper.getFaults(f);

    if(f.hasAnyFault())
    {
      LogOrDash.logString("gripper/faults", f.toString());
    }


    // Check for reboot
    if(Gripper.hasResetOccurred())
    {
        DriverStation.reportError("ALERT: Gripper motor has crashed!", false);
    }
  }
  public void ControledGrab(boolean open){
    if (open){
      Gripper.set(ControlMode.PercentOutput, 0.3);
      } else {
      Gripper.set(ControlMode.PercentOutput, 0);
    }

  }
  public void ControledClose(boolean close){
    if (close){
      Gripper.set(ControlMode.PercentOutput, -0.3);
      } else {
      Gripper.set(ControlMode.PercentOutput, 0);
    }

  }
}