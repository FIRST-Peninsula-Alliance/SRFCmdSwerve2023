// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSusbystem;

public class RotateModulesToAngleCommand extends CommandBase {
  double m_targetAngleDeg;
  SwerveSusbystem m_drive;
  long m_startTime;

  /** Creates a new RotateToAngle. */
  public RotateModulesToAngleCommand( SwerveSusbystem drivetrain, double angleDeg ) {
    m_drive = drivetrain;
    m_targetAngleDeg = angleDeg;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.rotateAllModulesToAngle( m_targetAngleDeg );
    // no need to write to drivePID once per loop}
    // but command is not reliably terminating. Add a 2 scond timeout.
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we could query the angles looking for them having
    // reached the target, but since this is a test routine
    // for potentially buggy systems, instead 
    // we'll just end the command after 2 seconds.
    return ((System.currentTimeMillis() - m_startTime) > 2000);
  }
}
