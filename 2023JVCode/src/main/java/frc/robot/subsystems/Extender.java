// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  /** Creates a new Extender. */
  CANSparkMax Extender1; 
  CANSparkMax Extender2;
  
  public Extender() {
    Extender1 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    Extender2 = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);

    Extender2.follow(Extender1, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void ExtendOut(Boolean ButtonHeldOut){
    if (ButtonHeldOut){
      Extender1.set(0.3);
    }else{
      Extender1.set(0);
    }
    
  }
  public void ExtendIn(Boolean ButtonHeldIn){
    if (ButtonHeldIn){
      Extender1.set(-0.3);
    }else{
      Extender1.set(0);
    }
  }
}
