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

public class Extender extends SubsystemBase {
  /** Creates a new Extender. */
  CANSparkMax Extender1; 
  CANSparkMax Extender2;
  DigitalInput digitalExtenderInputTop;
  DigitalInput digitalExtenderInputBottom;
  
  public Extender() {
    Extender1 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    Extender2 = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
    digitalExtenderInputTop = new DigitalInput(1);
    digitalExtenderInputBottom = new DigitalInput(0);

Extender1.setInverted(false);

    Extender2.follow(Extender1, true);

    Extender1.enableVoltageCompensation(12);
    Extender1.setSmartCurrentLimit(20);

    Extender2.enableVoltageCompensation(12);
    Extender2.setSmartCurrentLimit(20);

    
    // SOFT LIMITS ON FOLLOWERS ARE CAUSING PROBLEMS! USE MANUALLY CODED LIMITS!!!
    // Ignore above if using closed loop control, closed loop needs the soft limits

    Extender1.setSoftLimit(SoftLimitDirection.kForward, 50);
    Extender1.enableSoftLimit(SoftLimitDirection.kForward, true);


    Extender2.setSoftLimit(SoftLimitDirection.kForward, 50);
    Extender2.enableSoftLimit(SoftLimitDirection.kForward, true);

    
    Extender1.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Extender1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Extender2.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Extender2.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // PID stuff, primarily to hold in place against gravity
    Extender1.getPIDController().setP(0.00018);
    Extender1.getPIDController().setI(0.000001);
    Extender1.getPIDController().setD(0.0);

    Extender1.getPIDController().setIAccum(0);
    Extender1.getPIDController().setIMaxAccum(0.05, 0);
    Extender1.getPIDController().setIZone(100);

    Extender1.setClosedLoopRampRate(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender: ",Extender1.getEncoder().getPosition());
    SmartDashboard.putNumber("Extender 2: ", Extender2.getEncoder().getPosition());
    SmartDashboard.putBoolean("ExtenderTop", digitalExtenderInputTop.get());
    SmartDashboard.putBoolean("ExtenderBottom", digitalExtenderInputBottom.get());

    // PID tuning
    SmartDashboard.putNumber("Extender Integrator", Extender1.getPIDController().getIAccum());
    SmartDashboard.putNumber("Extender Speed", Extender1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Extender Output", Extender1.getAppliedOutput());

    //if (Extender1.get() > 0 && digitalExtenderInputTop.get() == false || Extender1.get() < 0 && digitalExtenderInputBottom.get() == false) {
    //  Extender1.set(0.0);
    //}

    
  }


  public void ExtendOut(Boolean ButtonHeldOut){
    if (ButtonHeldOut && digitalExtenderInputTop.get()){
      Extender1.set(0.3);
      
    }else{
      Extender1.set(0);
      
    }
  }



  public void ExtendIn(Boolean ButtonHeldIn){
    if (ButtonHeldIn && digitalExtenderInputBottom.get()){
      Extender1.set(-0.3);
      
    }else{
      Extender1.set(0);
      
    }
  }

    public Command extenderJoystick(Supplier<Double> p)
    { 
      return new RunCommand(() -> {
        double pow = p.get();
       
        // Deadzone
        if(Math.abs(pow)<0.05)
        {
          pow = 0;
        }
        
        if(pow < 0 && !digitalExtenderInputTop.get() || pow > 0 && !digitalExtenderInputBottom.get() || pow < 0 && Extender1.getEncoder().getPosition() > 50)
          Extender1.set(0);
        else
          Extender1.getPIDController().setReference(-pow*5600, ControlType.kVelocity);
          //Extender1.set(pow);
          //Extender1.set(Extender1.getEncoder().getPosition() > 30 ? Math.min(-pow, -0.2) : -pow);
      }, this);
    }
  }
