package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private double desiredAngle;
    private double lastSpeed;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "DriveTrain");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "DriveTrain");
        configAngleMotor();
        
        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "DriveTrain");
        configDriveMotor();

        mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 2);
        mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 2);

        mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 40);
        mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 40);

        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 2);
        

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            lastSpeed = percentOutput;
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            lastSpeed = velocity;
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); 
        desiredAngle = angle;
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition((int)absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public Double getTemp(int motor){
        return (motor == 1)?mDriveMotor.getTemperature():mAngleMotor.getTemperature();
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public double getDesiredSpeed(){
        return lastSpeed;
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getPosition(){
        double distance = Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModulePosition(distance, angle);
    }


    public void sendTelemetry() {
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getCanCoder().getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/integratedposition", getState().angle.getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond);    
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/temperature", getTemp(1));
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/temperature", getTemp(2));
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/setpoint", desiredAngle);
        LogOrDash.logNumber("Swerve/m" + moduleNumber + "/drive/setpoint", lastSpeed);
        
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/statorcurrent", mAngleMotor.getStatorCurrent());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/supplycurrent", mAngleMotor.getSupplyCurrent());

        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/statorcurrent", mDriveMotor.getStatorCurrent());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/supplycurrent", mDriveMotor.getSupplyCurrent());

        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/outputvoltage", mDriveMotor.getMotorOutputVoltage());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/outputvoltage", mAngleMotor.getMotorOutputVoltage());
    
        // Check for faults
        Faults f = new Faults();
        mDriveMotor.getFaults(f);

        if(f.hasAnyFault())
        {
            LogOrDash.logString("swerve/m" + moduleNumber + "/drive/faults", f.toString());
        }

        mAngleMotor.getFaults(f);
        if(f.hasAnyFault())
        {
            LogOrDash.logString("swerve/m" + moduleNumber + "/angle/faults", f.toString());
        }

        // Check for reboots
        if(mDriveMotor.hasResetOccurred())
        {
            DriverStation.reportError("ALERT: Drive Motor "+moduleNumber+" has crashed!", false);
        }

        if(mAngleMotor.hasResetOccurred())
        {
            DriverStation.reportError("ALERT: Angle Motor "+moduleNumber+" has crashed!", false);
        }

        if(angleEncoder.hasResetOccurred())
        {
            DriverStation.reportError("ALERT: CANCoder "+moduleNumber+" has crashed!", false);
        }

        
    }
}