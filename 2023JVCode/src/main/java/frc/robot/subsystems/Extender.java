// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
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

    Extender2.follow(Extender1, true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender: ",Extender1.getEncoder().getPosition());
    SmartDashboard.putBoolean("ExtenderTop", digitalExtenderInputTop.get());
    SmartDashboard.putBoolean("ExtenderBottom", digitalExtenderInputBottom.get());

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
  }
