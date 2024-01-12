package frc.robot.VortexSwerveCode.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.VortexSwerveCode.Constants;
import frc.robot.VortexSwerveCode.lib.math.Conversions;
import frc.robot.VortexSwerveCode.lib.util.CTREModuleState;
import frc.robot.VortexSwerveCode.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class SwerveModule {
    public int _moduleNumber;
    private Rotation2d _angleOffset;
    private Rotation2d _lastAngle;

    private CANSparkFlex _angleMotor;
    private CANSparkFlex _driveMotor;
    private CANcoder _angleEncoder;

    private SparkPIDController _rotatePID;

    // Only use this for debugging
    private SwerveModuleState _debugModuleState;

    SimpleMotorFeedforward _feedforward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this._moduleNumber = moduleNumber;
        this._angleOffset = moduleConstants._angleOffset;

        /* Angle Encoder Config */
        _angleEncoder = new CANcoder(moduleConstants._cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        _angleMotor = new CANSparkFlex(moduleConstants._angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        _driveMotor = new CANSparkFlex(moduleConstants._driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        configDriveMotor();

        _lastAngle = getCurrentState().angle;

        _rotatePID = _angleMotor.getPIDController();

        _debugModuleState = new SwerveModuleState();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        _debugModuleState = desiredState;
        desiredState = CTREModuleState.optimize(desiredState, getCurrentState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        _driveMotor.set(desiredState.speedMetersPerSecond
                / Constants.Swerve.maxSpeed);
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond)
            <= (Constants.Swerve.maxSpeed * 0.01)) ? _lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        _rotatePID.setReference(Conversions.degreesToVortex(angle.getDegrees(), Constants.Swerve.angleGearRatio), CANSparkBase.ControlType.kPosition);
        _lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.vortexToDegrees(
            _angleMotor.getEncoder().getPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(_angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public Rotation2d getCanCoderWithOffset() {
        return Rotation2d.fromDegrees(getCanCoder().getDegrees() - _angleOffset.getDegrees());
    }

    public void resetToAbsolute() {
        _angleMotor.getEncoder().setPosition(getCanCoder().getDegrees() - _angleOffset.getDegrees());
    }

    private void configAngleEncoder() {
        // This is the new way to reset to the default config
        _angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.SensorDirection = (Constants.Swerve.canCoderInvert)
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

        _angleEncoder.getConfigurator().apply(config);
    }

    private void configAngleMotor(){
        _angleMotor.restoreFactoryDefaults();
        // _angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        // _angleMotor.setSecondaryCurrentLimit(Constants.Swerve.anglePeakCurrentDuration);
        _angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        _angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        
        _angleMotor.getEncoder().setPositionConversionFactor((1/Constants.Swerve.angleGearRatio) * 360);

        resetToAbsolute();

        SparkPIDController anglePIDController = _angleMotor.getPIDController();
        anglePIDController.setP(Constants.Swerve.angleKP);
        anglePIDController.setI(Constants.Swerve.angleKI);
        anglePIDController.setD(Constants.Swerve.angleKD);
        anglePIDController.setFF(Constants.Swerve.angleKF);

        _angleMotor.getEncoder().setPositionConversionFactor(360 * Constants.Swerve.driveGearRatio * Math.PI);
        _angleMotor.getEncoder().setVelocityConversionFactor((360 * Constants.Swerve.driveGearRatio * Math.PI / 60));

        _angleMotor.getPIDController().setOutputRange(-0.25, 0.25);
    }

    private void configDriveMotor(){        
        _driveMotor.restoreFactoryDefaults();
        // _driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        // _driveMotor.setSecondaryCurrentLimit(Constants.Swerve.drivePeakCurrentDuration);
        _driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        _driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        // _driveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        // _driveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        // _driveMotor.getEncoder().setVelocityConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference / 60);
        // _driveMotor.getEncoder().setPositionConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference);
        // _driveMotor.getEncoder().setPosition(0);

        // _driveMotor.getPIDController().setP(Constants.Swerve.driveKP);
        // _driveMotor.getPIDController().setI(Constants.Swerve.driveKI);
        // _driveMotor.getPIDController().setD(Constants.Swerve.driveKD);
        // _driveMotor.getPIDController().setFF(Constants.Swerve.driveKF);

        // _driveMotor.getPIDController().setOutputRange(-0.5, 0.5);
    }

    public SwerveModuleState getCurrentState() {
        // double motorRPM = _driveMotor.getEncoder().getVelocity() * (600.0 / Constants.Swerve.encoderTicksPerRotation);
        // double wheelRPM = motorRPM / Constants.Swerve.driveGearRatio;
        // double wheelMPS = (wheelRPM * Constants.Swerve.wheelCircumference) / 60;
        return new SwerveModuleState(
            _driveMotor.get(),
            getAngle()
        );
    }

    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(
            _debugModuleState.speedMetersPerSecond / Constants.Swerve.maxSpeed,
            _debugModuleState.angle
        );
    }

    public SwerveModulePosition getPosition() {
        double pos = _driveMotor.getEncoder().getPosition()  * (Constants.Swerve.wheelCircumference
            / (Constants.Swerve.driveGearRatio * Constants.Swerve.encoderTicksPerRotation));
        return new SwerveModulePosition(
            pos,
            getAngle()
        );
    }
}
