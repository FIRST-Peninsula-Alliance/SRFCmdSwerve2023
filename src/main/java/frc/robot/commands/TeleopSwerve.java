package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private long count = 0;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    private double smoothJoyStick( double joyStickValue ) {
        joyStickValue = (joyStickValue * joyStickValue) * Math.signum(joyStickValue);
        return (MathUtil.applyDeadband(joyStickValue, Constants.stickDeadband));
    }


    @Override
    public void execute() {
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;    
/*
 * Temporary test routine, designed to move modules to 4 different states, each collectively 
 * consitent with constant simulated joystick levels, chageable by editing the assignments below.
 * Afer a 140 loops of each state (~3 seconds, long enough to observe actual module steering
 * and drive behavior) the Swerve.drive method will send the desired states for comparison on
 * the SmartDashboard, in order to see if they match expected States, and if wheel behavior 
 * also matches the desired states being sent to the modules.
 * We are using SDS Mk4-L2 modules, driven by Falcon 500's for both drive and steering, with The Thrifty
 * Bot analog absolute encoders instead of canCoders. We are seeing two significant anomalous symptoms:
 * 1. All modules mostly behave correctly at startup, and respond to joystick inputs correctly when 
 * driving with just fore aft translation, and strafing. Sometimes at startup, and after using rotation,
 * the two rear modules might lack, or lose, respectively, absolute calibration. When that happens, they 
 * often remain in sync with each other, but with mirror image angles, Other than the consistency of 
 * losing absolute angle calibration.
 * Have tried shifting absolute encoder and initial calibration module angles into -PI to PI range.  Also 
 * tried without, using raw angles with corrections, which can exceed even a -360, 360 range.  
 * Hovever, with joystick rotation input (either CCW or CW) the wheels form this pattern:
 *   \     \
 *   
 * 
 *   \     \
 * To restart this test, press the left Xbox bumper button.  Normally that button is used to switch 
 * in Robot centric mode, but for these tests Robot Centric has been set to be the default, and we
 * don't even want to test Field Oriented mode until the strange module behavior is fixed.
 * What is seen is that the back left and back right modules act as if they lose the angle calibration,
 * but often they steer as mirror images of each other. or both back modules will be at ~90 degrees (+/-
 * 15 degrees or so) to their desired positions.
 * Using TheThriftyBot analog absolute encoders. Have checked magnets in SDS Mk4 modules, all are secure. 
 * Have swapped input ports on RoboRio, same behavior.
 * Have output both absolute sensor angles and 
 */
        if (robotCentricSup.getAsBoolean() == true) {
            count = 0;
        }
        if (count<10000) {
            if (count < 150) {
                translationVal              = 0.0;
                strafeVal                   = 0.25;
                rotationVal=0;
                count++;
            } else if (count < 300) {
                translationVal              = -0.25;
                strafeVal                   = 0.0;
                rotationVal=0;
                count++;
            } else if (count < 450) {
                translationVal              = 0.0;
                strafeVal                   = -0.25;
                rotationVal=0;
                count++;
            } else if (count < 600) {
                translationVal              = 0.25;
                strafeVal                   = 0.0;
                rotationVal =0;
                count++;
            } else {
                count = 10100;
            } 
        } else {
            /* from here on just Get Values, Deadband */
            translationVal   = -smoothJoyStick(translationSup.getAsDouble());
            strafeVal        = -smoothJoyStick(strafeSup.getAsDouble());
            rotationVal      = -smoothJoyStick(rotationSup.getAsDouble());
            count++;
        }

        /* Drive -  added two temporary arguments to get Swerve.drive to print out 
         * useful info on known stateds wihtout adding much to Network Table traffic.
        */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                                rotationVal * Constants.Swerve.maxAngularVelocity, 
                                false,              // robotCentricSup.getAsBoolean(),  // negate for default field oriented!   
                                true,
                                (count<10000) ? (count+10)%150==0 : false,
                                count );
    }
}