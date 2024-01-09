package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkFlex mAngleMotor;
    private CANSparkFlex mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;

        double positionConversionFactor = Constants.Swerve.wheelCircumference * Constants.Swerve.driveGearRatio;
        double velocityConversionFactor = positionConversionFactor / 60.0;

        mDriveMotor.getEncoder().setPositionConversionFactor(positionConversionFactor);
        mDriveMotor.getEncoder().setVelocityConversionFactor(velocityConversionFactor);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput); // TODO: Made assumption that percent output is between -1 and 1. 
        }
        else {
            mDriveMotor.setVoltage(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed * Constants.Swerve.maxVoltage);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        double voltage = angle.getDegrees() / (360.0 / (Constants.Swerve.angleGearRatio * Constants.Swerve.encoderTicksPerRotation)) * Constants.Swerve.maxVoltage;
        mAngleMotor.setVoltage(voltage);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        double angle = mAngleMotor.getEncoder().getPosition() * (360.0 / (Constants.Swerve.angleGearRatio * Constants.Swerve.encoderTicksPerRotation));
        return Rotation2d.fromDegrees(angle);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = (getCanCoder().getDegrees() - angleOffset.getDegrees()) / (360.0 / (Constants.Swerve.angleGearRatio * 2048.0));
        mAngleMotor.getEncoder().setPosition(absolutePosition);
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

    public SwerveModuleState getState(){
        double motorRPM = mDriveMotor.getEncoder().getVelocity() * (600.0 / Constants.Swerve.encoderTicksPerRotation);        
        double wheelRPM = motorRPM / Constants.Swerve.driveGearRatio;
        double wheelMPS = (wheelRPM * Constants.Swerve.wheelCircumference) / 60;
        return new SwerveModuleState(
            wheelMPS, 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        double pos = mDriveMotor.getEncoder().getPosition()  * (Constants.Swerve.wheelCircumference / 
                        (Constants.Swerve.driveGearRatio * Constants.Swerve.encoderTicksPerRotation));
        return new SwerveModulePosition(
            pos, 
            getAngle()
        );
    }
}