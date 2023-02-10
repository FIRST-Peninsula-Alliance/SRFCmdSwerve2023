package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
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
    public int m_moduleNum;
    private Rotation2d m_angleOffset2d;
    private Rotation2d m_lastAngle2d;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    //private CANCoder m_absAngleEncoder;
    private AnalogInput m_absAngleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.m_moduleNum = moduleNumber;
        this.m_angleOffset2d = moduleConstants.angleOffset2d;

        System.out.println("Initializing Module number "+this.m_moduleNum);
        System.out.println("Angle offset "+moduleConstants.angleOffset2d);
        System.out.println("EncoderId "+moduleConstants.absAngleEncoderID);
        System.out.println("Angle Motor ID "+moduleConstants.angleMotorID);
        System.out.println("Drive Motor ID "+moduleConstants.driveMotorID);
    
        /* Angle Encoder Config */
        //was: m_absAngleEncoder = new CANCoder(moduleConstants.absAngleEncoderID);
        //was: configAbsAngleEncoder();
        m_absAngleEncoder = new AnalogInput(moduleConstants.absAngleEncoderID);


        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        System.out.println("Configuring Angle Motor");
        System.out.println("Motor invert "+Constants.Swerve.driveMotorInvert);
        System.out.println("Neutral mode "+Constants.Swerve.driveNeutralMode);
        System.out.println("Sensor Position "+m_angleMotor.getSelectedSensorPosition());
        System.out.println("Angle gear ratio "+Constants.Swerve.angleGearRatio);  
        System.out.println("Angle Peak Current "+Constants.Swerve.anglePeakCurrentLimit);
        System.out.println("Angle kP "+Constants.Swerve.angleKP);  
        System.out.println("Angle kI "+Constants.Swerve.angleKI);  
        System.out.println("Angle kD "+Constants.Swerve.angleKD);  

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        System.out.println("Configuring Drive Motor");
        System.out.println("Motor invert "+Constants.Swerve.driveMotorInvert);
        System.out.println("Neutral mode "+Constants.Swerve.driveNeutralMode);
        System.out.println("Sensor Position "+m_driveMotor.getSelectedSensorPosition());

        m_lastAngle2d = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle2d = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? m_lastAngle2d : desiredState.angle; 
        
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle2d.getDegrees(), Constants.Swerve.angleGearRatio));
        m_lastAngle2d = angle2d;
    }

    // This method is intended for use with test commands. It takes an angle in 
    // degrees, and sets the module to that angle without changing m_lastAngle2d 
    // (so the module should return to m_lastAngle2d after whatever command is triggering
    // this method ends).
    public void rotateToAngle( double angleDeg ) {
        double targetTics = Conversions.degreesToFalcon(angleDeg, 
                                        Constants.Swerve.angleGearRatio);
        m_angleMotor.set(ControlMode.Position, targetTics);
    }

    // This gets the angle from the TalonFX integrated encoder, returned as a Rotation2d
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public double getRawAbsEncoderDeg() {
        // This method returns the absolute (uncorrected) analog encoder reading
        // in degrees with a range of -179.9999... to 180.
        double angleDeg = m_absAngleEncoder.getVoltage() * Constants.Swerve.ANALOG_ENCODER_DEG_PER_VOLT;
        return angleDeg;             //(angleDeg > 180.0) ? (angleDeg - 360.0) : angleDeg;
    }

    public double getCorrectedAbsEncoderDeg() {
        // This method returns the absolute corrected analog encoder reading
        // in degrees with a range of (-180, 180 ].
        double angleDeg = getRawAbsEncoderDeg() - m_angleOffset2d.getDegrees();
        return angleDeg;              //(angleDeg < -180.0) ? (angleDeg + 360.0) : angleDeg;
    }

    public double getAvgRawAbsEncoderDeg() {
        // This method returns the absolute (uncorrected) analog encoder reading,
        // averaged for greater stability, in the range -179.9999... to 180.
        double angleDeg = m_absAngleEncoder.getAverageVoltage() * Constants.Swerve.ANALOG_ENCODER_DEG_PER_VOLT;
        return angleDeg;              //(angleDeg > 180.0) ? (angleDeg - 360.0) : angleDeg;
    }

    public void resetToAbsolute() {
        double absAngleOffsetDeg = getAvgRawAbsEncoderDeg() - m_angleOffset2d.getDegrees();
        double absolutePosition = Conversions.degreesToFalcon(absAngleOffsetDeg, Constants.Swerve.angleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);

        double readBackPos = this.getAngle().getDegrees();
        SmartDashboard.putString("Start "+this.m_moduleNum, 
                                 " AbsEncoderOffset = "+String.format("%.1f", absAngleOffsetDeg)+
                                 " with Readback error of "+String.format("%.1f", (readBackPos - absAngleOffsetDeg)));
        m_lastAngle2d = Rotation2d.fromDegrees(absAngleOffsetDeg);
    }

/*
    // Not needed when The Thrifty bot Analog encoder is used
    private void configAbsAngleEncoder(){        
        //m_absAngleEncoder.configFactoryDefault();
        //m_absAngleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }
*/
    private void configAngleMotor() {
        ErrorCode e_code;
        m_angleMotor.configFactoryDefault();     // could check error code here
        for (int i = 0; i<5; ++i) {
            e_code = m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
            if (e_code != ErrorCode.OK ) {
                System.out.println("Module "+this.m_moduleNum+" failed to initialize "+e_code.toString());
            } else {
                break;
            }
        }
        m_angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){  
        ErrorCode e_code;      
        m_driveMotor.configFactoryDefault();
        for (int i = 0; i<5; ++i) {
            e_code = m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
            if (e_code != ErrorCode.OK ) {
                System.out.println("Module "+this.m_moduleNum+" failed to initialize "+e_code.toString());
            } else {
                break;
            }
        }
        m_driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), 
                                    Constants.Swerve.wheelCircumference, 
                                    Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}