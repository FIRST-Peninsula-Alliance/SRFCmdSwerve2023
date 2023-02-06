package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
//Uncomment the following two lines if Navx USB interface is wanted,
//import com.kauailabs.navx.frc.AHRS.SerialDataType;
//import edu.wpi.first.wpilibj.SerialPort.Port;

//import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    //public Pigeon2 m_gyro;
    public AHRS m_gyro;

    public Swerve() {
        //m_gyro = new Pigeon2(Constants.Swerve.pigeonID);
        //m_gyro.configFactoryDefault();

        // uncomment USB init line and comment out the MXP init line if USB desired
        m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
        //m_gyro = new AHRS(Port.kUSB, SerialDataType.kProcessedData, (byte) 200); // NavX connected over USB
        
        // was zeroGyro();
        // This happensautomatically upon navx object creation
        // m_gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);   // was 1.0, which really only applied to canCoders, not our Analog absolute encoder.
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean print, long _count) {
        String outstr;
        // Live stream input control values every ~1 second, or whenever the caller (a Command)
        // sets the "print" boolean to true
        if (print || (_count%50 < .01)) {
            outstr = " X = "+String.format("%.1f", translation.getX())+
                     " Y = "+String.format("%.1f", translation.getY())+
                     " Rot = "+String.format("%.1f", rotation);
            if ( print ) {
                SmartDashboard.putString("Test loop "+_count, outstr); 
            } else {
                SmartDashboard.putString("D in", outstr);
            }
        }    

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            if (print || (_count%50 < .01)) {
                outstr = " Speed "+String.format("%.1f", swerveModuleStates[mod.moduleNumber].speedMetersPerSecond)+
                         " Rot "+String.format("%.1f", swerveModuleStates[mod.moduleNumber].angle.getDegrees());
                if (print) {
                    SmartDashboard.putString("D out "+_count+" Mod "+mod.moduleNumber, outstr);
                } else {
                    SmartDashboard.putString("D out "+mod.moduleNumber, outstr);
                }

            } else if (_count%50 < .0198) {             // arbitrary epsilon
 
                            
            }
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        //WAS: m_gyro.setYaw(0);      // Works for Pidgeon
         m_gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        Double heading = (Constants.Swerve.invertGyro) ? (360.0 - m_gyro.getYaw()) : m_gyro.getYaw();
        return  Rotation2d.fromDegrees(heading);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        Rotation2d heading = getYaw();
        swerveOdometry.update(heading, getModulePositions());  
        SmartDashboard.putNumber("GyroDeg ", heading.getDegrees());
    
        for(SwerveModule mod : mSwerveMods){
            // was: SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod"+mod.moduleNumber+" Abs deg ", mod.getCorrectedAbsEncoderDeg());
            SmartDashboard.putNumber("Mod"+mod.moduleNumber+" Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod"+mod.moduleNumber+" Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}