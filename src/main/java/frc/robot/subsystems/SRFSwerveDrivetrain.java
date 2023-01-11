// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
//import javax.lang.model.util.ElementScanner6;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRFSwerveDrivetrain extends SubsystemBase {
  private double wheelBase, trackWidth, radius;
  private double[] wheelSpeed = new double[4];
  private double[] wheelAngle = new double[4];

  //Driving initialization
  private TalonFX frontLeft  = new TalonFX(12);
  private TalonFX frontRight = new TalonFX(14);
  private TalonFX rearLeft   = new TalonFX(18);
  private TalonFX rearRight  = new TalonFX(16);

  private TalonFX frontLeftRot  = new TalonFX(11);
  private TalonFX frontRightRot = new TalonFX(13);
  private TalonFX rearLeftRot   = new TalonFX(17);
  private TalonFX rearRightRot  = new TalonFX(15);

/*
  private SRFSwerveModule frontLeftModule;
  private SRFSwerveModule frontRightModule;
  private SRFSwerveModule rearLeftModule;
  private SRFSwerveModule rearRightModule;
*/

  //PIDVALUES - move to Constants!
  private final double  drivekP = 0.55, 
                        drivekI = 0, 
                        drivekD = 0;

  // wheel pod magnet alignment static offsets - move to Constants!
  private final double  flOffset = 4.37, 
                        frOffSet = 1.854,
                        rlOffSet = 2.567,
                        rrOffSet = 1.361;

  private SRFSwerveModule flModule = new SRFSwerveModule(0,11,12, drivekP, drivekI, drivekD, flOffset);
  private SRFSwerveModule frModule = new SRFSwerveModule(1,13,14, drivekP, drivekI, drivekD, frOffSet);
  private SRFSwerveModule rlModule = new SRFSwerveModule(2,17,18, drivekP, drivekI, drivekD, rlOffSet);
  private SRFSwerveModule rrModule = new SRFSwerveModule(3,15,16, drivekP, drivekI, drivekD, rrOffSet);

  private final double m_wheelBase = 25.0;
  private final double m_trackwidth = 21.0;
  private final double m_radius = 32.65;

  AHRS navx = new AHRS(SPI.Port.kOnboardCS0);

  private double m_gyroStartAngle;
  private double m_estimatedDistance;
  private boolean m_isFieldOriented;
  
  public SRFSwerveDrivetrain() {
      m_isFieldOriented = true;           // default is field oriented
      m_estimatedDistance = 0;

      initEncodersAndGyro();
  }

  public void initEncodersAndGyro() {
    m_gyroStartAngle = getNavxAngle();
    
    flModule.setZeroOffset(flOffset);
    frModule.setZeroOffset(frOffSet);
    rlModule.setZeroOffset(rlOffSet);
    rrModule.setZeroOffset(rrOffSet); 

    flModule.setIntEncToZero();
    frModule.setIntEncToZero();
    rlModule.setIntEncToZero();
    rrModule.setIntEncToZero();
  }

  /*
  Changes the X,Y, values inputed (Like from a controller joystick) so they are oriented 
  to forward being the opposite end of the field from you as opposed to the front of the robot
  */
	public double[] convertToFieldOriented(double X, double Y, double gyroAngleDeg) {
        double newY, newX, angleRad, temp;
        
        angleRad = (gyroAngleDeg/180) * Math.PI;              
        //SmartDashboard.putNumber("angle at Convert", angleRad);

        temp =  Y * Math.cos(angleRad) + X * Math.sin(angleRad);
        newX = -Y * Math.sin(angleRad) + X * Math.cos(angleRad);
        newY = temp;
        //SmartDashboard.putNumber("newX", newX);
        //SmartDashboard.putNumber("newY", newY);

        return new double[] {newX,newY};
    }

    public void drive(double X, double Y, double W) {
        double A, B, C, D;
        //double greatestValue = -1;
        double greatestValue;
       
        if (isFieldOriented()) {
          double [] vettedXY = new double[2];
          vettedXY = convertToFieldOriented(X, Y, getNavxAngle());
          X = vettedXY[0];
          Y = vettedXY[1];
        }

        A = X - W * (wheelBase/radius);  
        B = X + W * (wheelBase/radius);
        C = Y - W * (trackWidth/radius);
        D = Y + W * (trackWidth/radius);

        // wheel speed given as value in range [0, 1] in the direction of the wheel angle
        wheelSpeed[0] = Math.sqrt(Math.pow(B,2) + Math.pow(C,2));
        wheelSpeed[1] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelSpeed[2] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelSpeed[3] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));
            
        //if no controller input keep current angle(Math inside would change) but change speed to zero(Math above)
        if (X != 0 || Y != 0 || W != 0) {
            /*
            Finds the largest wheel speed greater then 1 and converts all speeds so they are proportional
            to the largest being 1. It does this since the range of the motor is [-1,1].
            Note: the values from the equation above for the wheel speed are only positive so only need to check for speed > 1
            */
            // find the largest wheelspeed value
            greatestValue = wheelSpeed[0]; 
            for (int wheel = 1; wheel < 4; wheel++) {
                if (wheelSpeed[wheel] > greatestValue) {
                    greatestValue = wheelSpeed[wheel];
                }
            }
            // scale all the wheelspeed values to be <= 1
            if (greatestValue > 1) {
                for (int wheel = 0; wheel < 4; wheel++) {
                    wheelSpeed[wheel] = wheelSpeed[wheel]/greatestValue;
                }
            }

            // The angles are in the range [-1, +1], measured + clockwise, with zero being 
            // the straight ahead position.
            wheelAngle[0] = Math.atan2(B,C) / Math.PI;       // RF
            wheelAngle[1] = Math.atan2(B,D) / Math.PI;       // LF
            wheelAngle[2] = Math.atan2(A,D) / Math.PI;       // LR
            wheelAngle[3] = Math.atan2(A,C) / Math.PI;       // RR
            SmartDashboard.putNumber("B ", B);
            SmartDashboard.putNumber("D ", D); 
            SmartDashboard.putNumber("LF angle ", Math.atan2(B,D)* (180 /Math.PI)); 
        }   
        
        frModule.set(wheelAngle[0], wheelSpeed[0]);       // is this correct? Code elsewhere always lists FL first!
        flModule.set(wheelAngle[1], wheelSpeed[1]);
        rlModule.set(wheelAngle[2], wheelSpeed[2]);
        rrModule.set(wheelAngle[3], wheelSpeed[3]);
    }
 
    public void setSpeedZero() {
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;
    }

    public void displaySmartDashboard(boolean showAngle, 
                                      boolean showSpeed, 
                                      boolean showRotEncoder, 
                                      boolean showPIDTarget) {
        if (showAngle) {
            //Angle in degrees
            SmartDashboard.putNumber("Front Left Angle", getFrontLeftAngle());
            SmartDashboard.putNumber("Front Right Angle", getFrontRightAngle());
            SmartDashboard.putNumber("Rear Left Angle", getRearLeftAngle());
            SmartDashboard.putNumber("Rear Right Angle",getRearRightAngle());
        }
        if (showSpeed) {
            SmartDashboard.putNumber("Front Left Speed", getFrontLeftSpeed());
            SmartDashboard.putNumber("Front Right Speed", getFrontRightSpeed());
            SmartDashboard.putNumber("Rear Left Speed", getRearLeftSpeed());
            SmartDashboard.putNumber("Rear Right Speed", getRearRightSpeed());
        }
        if (showRotEncoder) {
            SmartDashboard.putNumber("Front Left Sensor", flModule.getSensorValue());
            SmartDashboard.putNumber("Front Right Sensor", frModule.getSensorValue());
            SmartDashboard.putNumber("Rear Left Sensor", rlModule.getSensorValue());
            SmartDashboard.putNumber("Rear Right Sensor", rrModule.getSensorValue());
        }
        if (showPIDTarget) {
            SmartDashboard.putNumber("Front Left PID Target", flModule.getPIDTarget());
            SmartDashboard.putNumber("Front Right PID Target", frModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Left PID Target", rlModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Right PID Target", rrModule.getPIDTarget());
        }
    }

    public double[] getWheelAngles() {
        return new double[] {wheelAngle[0], wheelAngle[1], wheelAngle[2], wheelAngle[3]};
    }

    public double getGyroStartAngle() {
        return m_gyroStartAngle;
    }

    public double getNavxAngle() {
      double navxAngle;
      
      navxAngle = navx.getAngle() % 360;
      if (navxAngle < 0) {
        navxAngle += 360;
      }
      return navxAngle;
    }

    public double getFrontLeftSpeed() {
        return wheelSpeed[1];
    }

    public double getFrontRightSpeed() {
        return wheelSpeed[0];
    }

    public double getRearLeftSpeed() {
        return wheelSpeed[2];
    }

    public double getRearRightSpeed() {
        return wheelSpeed[3];
    }

    public double getFrontLeftAngle() {
        return wheelAngle[1];
    }

    public double getFrontRightAngle() {
        return wheelAngle[0];
    }

    public double getRearLeftAngle() {
        return wheelAngle[2];
    }

    public double getRearRightAngle() {
        return wheelAngle[3];
    }

    public void setMaxOutput(double speed) {
      if (speed < .5) {
        SmartDashboard.putString("SlowMo", "Slow Mo Activated");
      } else {
        SmartDashboard.putString("SlowMo", "Slow Mo Cancelled");    
      }
    }
    
    public boolean isFieldOriented() {
      return m_isFieldOriented;
    }

    public void setFieldOriented( boolean fieldOriented ) {
      m_isFieldOriented = fieldOriented;
    }

    public double getDistanceTraveled() {
      return m_estimatedDistance;   // for now, time based velocity
                                    // assessment / converion is used,
                                    // (see periodic below) based on 
                                    // assumed .5 test speed setting.
                                    // This is very inaccurate, of course,
                                    // but does allow testing of 
                                    // DriveDistanceCommand before
                                    // pose2D can be obtained (i.e. after
                                    // implementing use of wpilib's 
                                    // extensive kinematic routines.
    }

    public void resetDistanceTraveled() {
      m_estimatedDistance = 0;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run.
      // Other than reporting values to a dashboard, no need to 
      // call drive() here, as that is done via a Command (either 
      // default via joystick controller (i.e. during tele-op), 
      // or a specific Command which provides either simple speed, 
      // direction, and rotation required, or some version of path 
      // following with pre-configured values.
      
      // For now, increment "distance traveled" everytime through.
      // Assume periodic is caled every 20 ms, and an average speed of
      // .5 (for testing) * max speed of 16 ft/sec, less allowance for
      // accel / decel, so 7 ft/sec average.
      // So: 8 ft/sec * .02 sec = .14 ft per 20 ms
      m_estimatedDistance += .14;
    }
}
