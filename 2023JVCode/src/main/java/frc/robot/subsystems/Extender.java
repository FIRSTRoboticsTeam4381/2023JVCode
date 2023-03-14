// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkMaxPosition;

public class Extender extends SubsystemBase {
  /** Creates a new Extender. */
  CANSparkMax Extender1; 
  CANSparkMax Extender2;
  DigitalInput digitalExtenderInputTop;
  DigitalInput digitalExtenderInputBottom;
  boolean extenderDeadzone;

  public Command JoystickElevator ( Supplier <Double> joystickPower ) {
    return new RunCommand(() -> {
      Double power = joystickPower.get() * -5600;
      // Deadzone
      if (Math.abs(power) < 0.05 * 5600) {
        Extender1.getPIDController().setReference(0, ControlType.kVelocity, 0);
        SmartDashboard.putNumber("extender/m1/velocitysetpoint", 0);
        extenderDeadzone = true;
      } else {
        extenderDeadzone = false;
        if (!digitalExtenderInputTop.get() && power > 0) {
          Extender1.set(0);
        } else if (!digitalExtenderInputBottom.get() && power < 0) {
          Extender1.set(0);
        } else {
          Extender1.getPIDController().setReference(power, ControlType.kVelocity, 0);
          SmartDashboard.putNumber("extender/m1/velocitysetpoint", power);
        }
      }
    }, this);
  }

  public Extender() {
    Extender1 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    Extender2 = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
    digitalExtenderInputTop = new DigitalInput(1);
    digitalExtenderInputBottom = new DigitalInput(0);

    Extender2.follow(Extender1, true);
    
    Extender1.enableVoltageCompensation(12);
    Extender2.enableVoltageCompensation(12);
    Extender1.setSmartCurrentLimit(20);
    Extender2.setSmartCurrentLimit(20);
    Extender1.setSoftLimit(SoftLimitDirection.kForward, 50);
    Extender1.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Extender2.setSoftLimit(SoftLimitDirection.kForward, 50);
    Extender2.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Extender1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Extender1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Extender2.enableSoftLimit(SoftLimitDirection.kForward, true);
    Extender2.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Extender1.getPIDController().setP(0.00018, 0);
    Extender1.getPIDController().setI(0.000001,0);
    Extender1.getPIDController().setD(0.0, 0);
    Extender1.getPIDController().setIAccum(0);
    Extender1.getPIDController().setIMaxAccum(0.05, 0);
    Extender1.getPIDController().setIZone(100, 0);

    Extender1.getPIDController().setP(0.2, 1);
    Extender1.getPIDController().setI(0,1);
    Extender1.getPIDController().setD(0, 1);

    Extender1.setClosedLoopRampRate(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("extender/m1/position",Extender1.getEncoder().getPosition());
    SmartDashboard.putNumber("extender/m1/setspeed", Extender1.get());
    SmartDashboard.putNumber("extender/m1/velocity", Extender1.getEncoder().getVelocity());
    SmartDashboard.putNumber("extender/m1/appliedoutput", Extender1.getAppliedOutput());
    SmartDashboard.putNumber("extender/m1/temperature", Extender1.getMotorTemperature());
    SmartDashboard.putNumber("extender/m1/outputcurrent", Extender1.getOutputCurrent());
    SmartDashboard.putNumber("extender/m1/iaccum", Extender1.getPIDController().getIAccum());

    SmartDashboard.putNumber("extender/m2/position",Extender2.getEncoder().getPosition());
    SmartDashboard.putNumber("extender/m2/setspeed", Extender2.get());
    SmartDashboard.putNumber("extender/m2/velocity", Extender2.getEncoder().getVelocity());
    SmartDashboard.putNumber("extender/m2/appliedoutput", Extender2.getAppliedOutput());
    SmartDashboard.putNumber("extender/m2/temperature", Extender2.getMotorTemperature());
    SmartDashboard.putNumber("extender/m2/outputcurrent", Extender2.getOutputCurrent());
    
    SmartDashboard.putBoolean("extender/top", digitalExtenderInputTop.get());
    SmartDashboard.putBoolean("extender/bottom", digitalExtenderInputBottom.get());
    //SmartDashboard.putBoolean("Extender Deadzone", extenderDeadzone);

    SmartDashboard.putNumber("extender/m1/faults", Extender1.getFaults());
    SmartDashboard.putNumber("extender/m2/faults", Extender2.getFaults());

    

    if (Extender1.get() > 0 && digitalExtenderInputTop.get() == false || Extender1.get() < 0 && digitalExtenderInputBottom.get() == false) {
      Extender1.set(0.0);
    }
  }

  public void ExtendOut(Boolean ButtonHeldOut){
    if (ButtonHeldOut && digitalExtenderInputTop.get()){
      Extender1.set(0.3);
      
    }else{
      Extender1.set(0);
      
    }
  }



  public void ExtendIn(Boolean ButtonHeldIn){
    if (ButtonHeldIn && digitalExtenderInputBottom.get() == false){
      Extender1.set(-0.3);
      
    }else{
      Extender1.set(0);
      
    }
  }
  public SparkMaxPosition goToPosition(double pos, double err) {
    return new SparkMaxPosition(Extender1, pos, 1, err, this);
  }
 }
