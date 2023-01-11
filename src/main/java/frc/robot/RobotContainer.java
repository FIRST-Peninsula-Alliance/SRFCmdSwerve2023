// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.SRFSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  double gyroTargetAngle, gyroRange;      // not sure if these are needed, or even used...

  // The robot's subsystems and commands are defined here...
  private final SRFSwerveDrivetrain m_drivetrain = new SRFSwerveDrivetrain();
 
  private final CommandXboxController m_controller =
      new CommandXboxController(OIConstants.kGAME_CONTROLLER_PORT);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void init() {        // facilitates resetting encoders and Gyro from 
                              // robot.java when needed for autonomousInit() and other
                              // state init methods, without running a new constructor
    m_drivetrain.initEncodersAndGyro();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Default drive command is swerve ".drive". This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getDefaultDriveCommand());

    // Schedule `DriveDistanceCommand` when the Xbox controller's B button is held,
    // cancelling on release (in case a panic stop is needed)
    m_controller.b()
        .whileTrue(new DriveDistanceCommand( m_drivetrain,
                                    6.0,
                                    0.5,
                                    0.0,
                                    0.0));
/*
 * The following will limit robot speed as long as the right bumper is held.
 * On release it will return to max possible speed.
 * However, it was written for arcadeDrive (differential). Need to figure out 
 * if setMaxOutput() is even supported for swerve. For now, a stub method has 
 * been added to SRWSwerveDrive class which just sends a string to the dashboard
 * on each key press and release (based on Arg value).
 */
    m_controller.rightBumper()
      .whileTrue(Commands.runOnce(() -> m_drivetrain.setMaxOutput(0.5)))
      .onFalse(Commands.runOnce(() -> m_drivetrain.setMaxOutput(1)));
                            
    /* Stabilize robot to drive straight with gyro when left bumper is held
       This was written for an arcade drive. Need to figure out how to pathfind
       and use wpilib kinematics to correct cource. Leaving this as a placeholder for now
       
       m_controller.leftBumper()
                .whenHeld(
                    new PIDCommand(
                        new PIDController( DriveConstants.kDRIVE_STABIL_P,
                                           DriveConstants.kDRIVE_STABIL_I,
                                           DriveConstants.kDRIVE_STABIL_D ),
                        // Close the loop on the turn rate
                        m_drivetrain::getTurnRate,
                        // Setpoint is 0
                        0,
                        // Pipe the output to the turning controls
                        output -> 
                        m_drivetrain.arcadeDrive(-m_controller.getRawAxis(OIConstants.kDRIVE_AXIS),
                                                  output),
                        // Require the robot drive
                        m_drivetrain );
    */                        
                                
    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Score & Exit", 
                               new DriveDistanceCommand(m_drivetrain,
                                                        6.0,
                                                        0.5,
                                                        0.0,
                                                        0.0));
    //m_chooser.addOption("Auto Do Nothing", null);
    //m_chooser.addOption("Auto Exit Community", new AutoExitCommunity(m_drivetrain));
    //m_chooser.addOption("Auto Exit & Charge", new AutoScoreExitChargeUp(m_drivetrain));
    //m_chooser.addOption("Auto Score Exit & Charge", new AutoScoreExitChargeUp(m_drivetrain));
    //m_chooser.addOption("Auto Multi Score", new AutoScoreMultiple(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getDefaultDriveCommand() {
    /*
     * Note: standard wpilib HID joystick input classes automatically
     * apply deadbanding and squaring. Slew rate limiting can also be had
     * via wpilib. Need minus sign on Y axis source beacuse pushing stick 
     * forward yields negative values.
     * 
     */
    return new DefaultDriveCommand( m_drivetrain,
                                    () -> m_controller.getRawAxis(OIConstants.kDRIVE_X_AXIS),
                                    () -> -m_controller.getRawAxis(OIConstants.kDRIVE_Y_AXIS),
                                    () -> m_controller.getRawAxis(OIConstants.kROTATE_AXIS));
  }
}
