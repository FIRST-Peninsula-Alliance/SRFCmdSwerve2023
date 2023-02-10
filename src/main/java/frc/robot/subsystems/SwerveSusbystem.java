package frc.robot.subsystems;

import frc.lib.util.CTREModuleState;
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

public class SwerveSusbystem extends SubsystemBase {
    private SwerveDriveOdometry m_swerveOdometry;
    private SwerveModule[] m_swerveMods;
    //public Pigeon2 m_gyro;
    private AHRS m_gyro;
    private double m_maxOutput;
    private boolean m_isFieldOriented;
    
    public SwerveSusbystem() {
        //m_gyro = new Pigeon2(Constants.Swerve.pigeonID);
        //m_gyro.configFactoryDefault();

        // uncomment USB init line and comment out the MXP init line if USB desired
        m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
        //m_gyro = new AHRS(Port.kUSB, SerialDataType.kProcessedData, (byte) 200); // NavX connected over USB
        
        // was zeroGyro();
        // This happensautomatically upon navx object creation
        // m_gyro.reset();

        m_swerveMods = new SwerveModule[] {
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

        m_swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void setMaxOutput( double maxOutputFactor ) {
        m_maxOutput = maxOutputFactor;
    }

    public double getMaxOutput() {
        return m_maxOutput;
    }

    public void setFieldOriented( boolean isFieldOriented ) {
        m_isFieldOriented = isFieldOriented;
    }

    public boolean isFieldOriented() {
        return m_isFieldOriented;
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
/*
        String outstr;

        // Test code
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
*/
        // Allow driver to "slow down" robot response speed for fine adjustments on the field
        // by setting a class variable to a percentage reduction, range will be .2 to 1.
        translation = translation.times( m_maxOutput );

        // FIXME: set persistent rotate speed reduction after testing.
        rotation = rotation * .6 * m_maxOutput;

        SwerveModuleState[] swerveModuleStates =
                            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                m_isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(
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

        for(SwerveModule mod : m_swerveMods){
/*
            if (print) {
                outstr = " Speed "+String.format("%.1f", swerveModuleStates[mod.m_moduleNum].speedMetersPerSecond)+
                         " Rot "+String.format("%.1f", swerveModuleStates[mod.m_moduleNum].angle.getDegrees())+
                         " IPS "+String.format("%.1f", mod.getPosition().angle.getDegrees())+
                         " Abs "+String.format("%.1f", mod.getCorrectedAbsEncoderDeg());
                SmartDashboard.putString("D out "+_count+" Mod "+mod.m_moduleNum, outstr);
            }
*/
            /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
            swerveModuleStates[mod.m_moduleNum] = 
                    CTREModuleState.optimize(swerveModuleStates[mod.m_moduleNum],
                                             mod.getState().angle); 
/*
            if (print || (_count%50 < .01)) {
                outstr = " Speed "+String.format("%.1f", swerveModuleStates[mod.m_moduleNum].speedMetersPerSecond)+
                         " Rot "+String.format("%.1f", swerveModuleStates[mod.m_moduleNum].angle.getDegrees())+
                         " IPS "+String.format("%.1f", mod.getPosition().angle.getDegrees())+
                         " Abs "+String.format("%.1f", mod.getCorrectedAbsEncoderDeg());
                if (print) {
                    SmartDashboard.putString("Opt "+_count+" Mod "+mod.m_moduleNum, outstr);
                } else {
                    SmartDashboard.putString("D out "+mod.m_moduleNum, outstr);
                }
            }
*/
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNum], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
       
        for(SwerveModule mod : m_swerveMods){
            CTREModuleState.optimize(desiredStates[mod.m_moduleNum],
                                     mod.getState().angle);  
            mod.setDesiredState(desiredStates[mod.m_moduleNum], false);
        }
    }    

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_moduleNum] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.m_moduleNum] = mod.getPosition();
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
        for(SwerveModule mod : m_swerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }

    @Override
    public void periodic(){
        Rotation2d heading = getYaw();
        m_swerveOdometry.update(heading, getModulePositions());  
        SmartDashboard.putNumber("GyroDeg ", heading.getDegrees());
    
        for(SwerveModule mod : m_swerveMods){
            // was: SmartDashboard.putNumber("Mod " + mod.m_moduleNum + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod"+mod.m_moduleNum+" Abs deg ", mod.getCorrectedAbsEncoderDeg());
            SmartDashboard.putNumber("Mod"+mod.m_moduleNum+" Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod"+mod.m_moduleNum+" Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    // This is a test routine, designed to rotate all Steering motors
    // to a specified heading, typically 0 but able to support any angle.
    public void rotateAllModulesToAngle( double angleDeg ) {
        for(SwerveModule mod : m_swerveMods) {
            mod.rotateToAngle( angleDeg );
        }
    }
}