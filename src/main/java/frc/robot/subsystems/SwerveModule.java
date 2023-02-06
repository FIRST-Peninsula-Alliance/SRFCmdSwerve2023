package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.Constants.Swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    //private CANCoder angleEncoder;
    private AnalogInput angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        System.out.println("Initializing Module number "+this.moduleNumber);
        System.out.println("Angle offset "+moduleConstants.angleOffset);
        System.out.println("EncoderId "+moduleConstants.absEncoderID);
        System.out.println("Angle Motor ID "+moduleConstants.angleMotorID);
        System.out.println("Drive Motor ID "+moduleConstants.driveMotorID);
    
        /* Angle Encoder Config */
        //was: angleEncoder = new CANCoder(moduleConstants.absEncoderID);
        //was: configAngleEncoder();
        angleEncoder = new AnalogInput(moduleConstants.absEncoderID);


        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        System.out.println("Configuring Angle Motor");
        System.out.println("Motor invert "+Constants.Swerve.driveMotorInvert);
        System.out.println("Neutral mode "+Constants.Swerve.driveNeutralMode);
        System.out.println("Sensor Position "+mAngleMotor.getSelectedSensorPosition());
        System.out.println("Angle gear ratio "+Constants.Swerve.angleGearRatio);  
        System.out.println("Angle Peak Current "+Constants.Swerve.anglePeakCurrentLimit);  
  
        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        System.out.println("Configuring Drive Motor");
        System.out.println("Motor invert "+Constants.Swerve.driveMotorInvert);
        System.out.println("Neutral mode "+Constants.Swerve.driveNeutralMode);
        System.out.println("Sensor Position "+mAngleMotor.getSelectedSensorPosition());

        lastAngle = getState().angle;
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
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; 
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    // This gets the angle from the TalonFX integrated encoder, returned as a Rotation2d
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public double getRawAbsEncoderDeg() {
        // This method returns the absolute (uncorrected) analog encoder reading
        // in degrees with a range of -179.9999... to 180.
        double angleDeg = angleEncoder.getVoltage() * Constants.Swerve.ANALOG_ENCODER_DEG_PER_VOLT;
        return angleDeg;             //(angleDeg > 180.0) ? (angleDeg - 360.0) : angleDeg;
    }

    public double getCorrectedAbsEncoderDeg() {
        // This method returns the absolute corrected analog encoder reading
        // in degrees with a range of (-180, 180 ].
        double angleDeg = getRawAbsEncoderDeg() - angleOffset.getDegrees();
        return angleDeg;              //(angleDeg < -180.0) ? (angleDeg + 360.0) : angleDeg;
    }

    public double getAvgRawAbsEncoderDeg() {
        // This method returns the absolute (uncorrected) analog encoder reading,
        // averaged for greater stability, in the range -179.9999... to 180.
        double angleDeg = angleEncoder.getAverageVoltage() * Constants.Swerve.ANALOG_ENCODER_DEG_PER_VOLT;
        return angleDeg;              //(angleDeg > 180.0) ? (angleDeg - 360.0) : angleDeg;
    }

    public void resetToAbsolute() {
        double absAngleOffsetDeg = getAvgRawAbsEncoderDeg() - angleOffset.getDegrees();
        double absolutePosition = Conversions.degreesToFalcon(absAngleOffsetDeg, Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);

        double readBackPos = this.getAngle().getDegrees();
        SmartDashboard.putString("Start "+this.moduleNumber, 
                                 " AbsEncoderOffset = "+String.format("%.1f", absAngleOffsetDeg)+
                                 " with Readback error of "+(readBackPos - absAngleOffsetDeg));
        lastAngle = Rotation2d.fromDegrees(absAngleOffsetDeg);
    }
/*
    // Not needed when The Thrifty bot Analog encoder is used
    private void configAngleEncoder(){        
        //angleEncoder.configFactoryDefault();
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }
*/
    private void configAngleMotor() {
        ErrorCode e_code;
        mAngleMotor.configFactoryDefault();     // could check error code here
        for (int i = 0; i<5; ++i) {
            e_code = mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
            if (e_code != ErrorCode.OK ) {
                System.out.println("Module "+this.moduleNumber+" failed to initialize "+e_code.toString());
            } else {
                break;
            }
        }
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){  
        ErrorCode e_code;      
        mDriveMotor.configFactoryDefault();
        for (int i = 0; i<5; ++i) {
            e_code = mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
            if (e_code != ErrorCode.OK ) {
                System.out.println("Module "+this.moduleNumber+" failed to initialize "+e_code.toString());
            } else {
                break;
            }
        }
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), 
                                    Constants.Swerve.wheelCircumference, 
                                    Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}