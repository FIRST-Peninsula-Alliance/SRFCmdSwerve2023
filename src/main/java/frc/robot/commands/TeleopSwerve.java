package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSusbystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private SwerveSusbystem m_swerveSubsystem;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;

    public TeleopSwerve(SwerveSusbystem swerveSubsystem, 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup) {
        this.m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.m_translationSup = translationSup;
        this.m_strafeSup = strafeSup;
        this.m_rotationSup = rotationSup;
    }

    private double smoothJoyStick( double joyStickValue ) {
        joyStickValue = (MathUtil.applyDeadband(joyStickValue, Constants.stickDeadband));
        return ((joyStickValue * joyStickValue) * Math.signum(joyStickValue));
     }


    @Override
    public void execute() {
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;    
        
        translationVal   = -smoothJoyStick(m_translationSup.getAsDouble());
        strafeVal        = -smoothJoyStick(m_strafeSup.getAsDouble());
        rotationVal      = -smoothJoyStick(m_rotationSup.getAsDouble());
  
        m_swerveSubsystem.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                                rotationVal * Constants.Swerve.maxAngularVelocity, 
                                true);
    }
}