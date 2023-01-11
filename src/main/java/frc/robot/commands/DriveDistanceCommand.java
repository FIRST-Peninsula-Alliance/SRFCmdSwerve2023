// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SRFSwerveDrivetrain;

public class DriveDistanceCommand extends CommandBase {
  private final SRFSwerveDrivetrain m_drivetrain;
  private final double m_distance;
  private final double m_driveSpeed;
  private final double m_rotateSpeed;
  private final double m_xSpeed;
  private final double m_ySpeed;
  private final boolean m_prevFieldOrientedState;
   
  /** Creates a new DriveDistanceCommand. */
  public DriveDistanceCommand( SRFSwerveDrivetrain drivetrain,
                               double distance,     // always positive
                               double driveSpeed,   // 0 - 1
                               double direction,    // field oriented vector angle
                               double rotateSpeed   // Rotation is normally 0 for
                                                    // such commands, but why not allow 
                                                    // twirling during move?
                              ) {
      double angleRad;
      
      m_drivetrain = drivetrain;
      m_distance = distance;
      m_driveSpeed = driveSpeed;
      // convert field oriented direction into X, Y speed components
      // after which no longer need to save direction.
      // However, do save the field oriented state,
      // then set field oriented to false, at least until command 
      // terminates (no need for field oriented conversion here, as the
      // X, Y vector is already field referenced!)
      angleRad = (direction/180) * Math.PI;
      m_xSpeed = m_driveSpeed * Math.cos(angleRad);
      m_ySpeed = m_driveSpeed * Math.sin(angleRad);
      m_prevFieldOrientedState = m_drivetrain.isFieldOriented();
      m_drivetrain.setFieldOriented(false);
      m_rotateSpeed = rotateSpeed;
      m_drivetrain.resetDistanceTraveled();
      addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // init encoders and / or gyro here as needed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive( m_xSpeed, m_ySpeed, m_rotateSpeed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop motors?
    m_drivetrain.setFieldOriented(m_prevFieldOrientedState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drivetrain.getDistanceTraveled() > m_distance);
  }
}
