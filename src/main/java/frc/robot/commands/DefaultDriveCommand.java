// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SRFSwerveDrivetrain;

public class DefaultDriveCommand extends CommandBase {

  private final SRFSwerveDrivetrain m_drivetrain;
  private final Supplier<Double> m_xSpeedSupplier;
  private final Supplier<Double> m_ySpeedSupplier;
  private final Supplier<Double> m_rotateSpeedSupplier;

  /**
   * Creates a new DefaultDriveCommand. This command will drive your robot 
   * according to the speed supplier lambdas. This command does not terminate
   * on its own - but of course is interruptable.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xSpeedSupplier Lambda supplier of forward / backward speed
   * @param ySpeedSupplier Lambda supplier of left / right speed
   * @param rotateSpeedSupplier Lambda supplier of rotational speed
   */

  public DefaultDriveCommand( SRFSwerveDrivetrain drivetrain,
                              Supplier<Double> xSpeedSupplier,
                              Supplier<Double> ySpeedSupplier,
                              Supplier<Double> rotateSpeedSupplier) { 
    m_drivetrain = drivetrain;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_rotateSpeedSupplier = rotateSpeedSupplier;
    addRequirements(drivetrain);   // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // do encoder and gyro initialization here as needed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive( m_xSpeedSupplier.get(), 
                        m_ySpeedSupplier.get(), 
                        m_rotateSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop motors?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
