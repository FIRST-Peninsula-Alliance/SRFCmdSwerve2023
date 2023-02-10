package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController m_xbox1 = new CommandXboxController(0);
/*
    WAS:
    private final Joystick m_xbox1 = new Joystick(0);

    / * Drive Controls * /
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    / * Driver Buttons * /
    private final JoystickButton zeroGyro = new JoystickButton(m_xbox1, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(m_xbox1, XboxController.Button.kLeftBumper.value);
*/  // end WAS

    /* Subsystems */
    private final SwerveSusbystem m_swerveSubsystem = new SwerveSusbystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

/*  WAS:
        m_swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                m_swerveSubsystem, 
                () -> -m_xbox1.getRawAxis(translationAxis), 
                () -> -m_xbox1.getRawAxis(strafeAxis), 
                () -> -m_xbox1.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
*/
        m_swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                m_swerveSubsystem,                            // Joystick centric outputs:
                () -> m_xbox1.getLeftY(),             // translate: - fore / + laft
                () -> m_xbox1.getLeftX(),             // strafe: - left / + right
                () -> m_xbox1.getRightX()            // rotate: - left / + right
                                                     // Since WPILib wants the exact
                                                     // opposite of all of them,
                                                     // need minus signs - will apply
                                                     // in TeleopSwerve                                        
             )
        ); 

        // How to pass a button supplier to a command:
        //  () -> m_xbox1.getHID().getLeftBumper()   // e.g. left bumper cancels Field 
                                                    // oriented when held, makes control "robot centric"
                                                    //problem is this makes button binding less "visible"
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // was: zeroGyro.onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
    	m_xbox1.back().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
        m_xbox1.start().onTrue(new InstantCommand(() -> m_swerveSubsystem.resetModulesToAbsolute()));
        
        //m_xbox1.a() 
        //m_xbox1.b() 
       // m_xbox1.x().onTrue(new InstantCommand(rotatemodulewheels(m_swerveSubsystem, 0)));
        m_xbox1.y().onTrue(new RotateModulesToAngleCommand(m_swerveSubsystem, 0));

        //m_xbox1.povUp() 
        //m_xbox1.povLeft() 
        //m_xbox1.povDown() 
        //m_xbox1.povRight() 
        //m_xbox1.leftBumper().OnTrue(new InstantCommand(field oriented toggle));
        //m_xbox1.rightBumper().OnTrue(new InstantCommand(toggle goSlow)) 
        //m_xbox1.leftTrigger() 
        //m_xbox1.rightTrigger()
        //m_xbox1.leftStick() 
        //m_xbox1.rightStick() 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(m_swerveSubsystem);
    }
}
