// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This version of the file is for SRF Swerve drive using the 
// TTB Analog Abs Position sensors 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRFSwerveModule {    
    private TalonFX m_rotationMotor;
    private TalonFX m_speedMotor;
    private AnalogInput m_encoder;
    
    //CANPIDController speedPID;
    
    double  speedP = 5e-5, 
            speedI = 1e-6, 
            speedD = 0;
    double zeroOffset=0;
    final int countsPerRev = 2048;
    
    double Vcc = 4.82;  //Measured from 5V pins of analog input port of RoboRio
    double countsPerVolt = 26214/Vcc;

    private double m_PIDTarget;

    private int m_encoderID;

    public SRFSwerveModule( int encoderID,
                            int rotID, 
                            int driveID, 
                            double P, 
                            double I,
                            double D, 
                            double offset) {
        m_encoderID = encoderID;
        m_encoder = new AnalogInput(m_encoderID);
        m_rotationMotor = new TalonFX(rotID);
        m_rotationMotor.setNeutralMode(NeutralMode.Brake);
        m_rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_rotationMotor.setInverted(true);
       
        m_rotationMotor.config_kP(0, P, 0);
        m_rotationMotor.config_kI(0, I, 0);
        m_rotationMotor.config_kD(0, D, 0);
    
        m_speedMotor = new TalonFX(driveID);
        m_speedMotor.setNeutralMode(NeutralMode.Brake);
        m_speedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_speedMotor.setInverted(false);

        m_speedMotor.config_kP(0,speedP,0);
        m_speedMotor.config_kI(0,speedI,0);
        m_speedMotor.config_kD(0,speedD,0);
        m_speedMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true,
                                    30,
                                    33.5,
                                    0.2));
        m_speedMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true,
                                    30,
                                    33.5, 
                                    0.2));       
        
        /*
        // On startup, computes the offset of the initial wheel pod position from the static zero offset 
        double offsetDifference = offset-m_m_encoder.getVoltage();
        SmartDashboard.putNumber("current pos"+m_m_encoderID, m_m_encoder.getVoltage());
        
        // minimize the amount of change
        if(Math.abs(offsetDifference)>=(Vcc/2))
        {
            if(offsetDifference>0)
            {
            offsetDifference= offsetDifference - Vcc;
            }
            else if(offsetDifference<=0)
            {
                offsetDifference= Vcc - Math.abs(offsetDifference);
            } 
        }
        SmartDashboard.putNumber("offsetDifference"+m_m_encoderID, offsetDifference);
        //SmartDashboard.putNumber("current"+m_m_encoderID, m_m_encoder.getVoltage());
        //SmartDashboard.putNumber("offsetDifference"+m_m_encoderID, offsetDifference);
        zeroOffset=offsetDifference;

        */

        //zeroOffset=Math.abs(offset-m_m_encoder.getVoltage());
        //m_rotationMotor.set(ControlMode.Position, m_rotationMotor.getSelectedSensorPosition() + Math.abs((m_m_encoder.getVoltage()-zeroOffset)*26214.4));
    }
    
    public void set(double angle, double speed) {
        //SmartDashboard.putNumber(("m_encoder"+m_encoder.getChannel()), m_encoder.getVoltage());
        //SmartDashboard.updateValues();
        //SmartDashboard.putNumber("angle", angle);
        //SmartDashboard.putNumber(("m_encoder"+m_encoder),m_m_rotationMotor.getSelectedSensorPosition());
        
        double currentAngle;
        double distanceBetween;
        int sign = 1;
        double gearratio=12.8;
        double PodCntsRev = countsPerRev*gearratio;  // for Falcon 500 internal m_encoder with SDS modules 12.8:1 ratio
        double countsPerVolt = PodCntsRev / 4.82;  // = 5438.6
        
        //SmartDashboard.putNumber("currentangle1", currentAngle%26214);
        currentAngle = m_rotationMotor.getSelectedSensorPosition();
        currentAngle %= PodCntsRev;
        if (currentAngle < 0) {
            currentAngle += PodCntsRev;
        }
        // Put current angle in range [0, PodCntsRev]
        SmartDashboard.putNumber("current counts "+m_encoderID, currentAngle);
        //SmartDashboard.putNumber("% counts/Rev", currentAngle);
        // angle is passed in as degrees [-1, 1]
        angle = (angle + 1) * (PodCntsRev/2);  // converts input angle from [-1 to +1] to [0, PodCntsRev]
        SmartDashboard.putNumber("target counts "+m_encoderID, angle);
        angle -= (zeroOffset*countsPerVolt);
        //angle += (zeroOffset*countsPerVolt);
        SmartDashboard.putNumber("target angle w offset "+m_encoderID, angle);

        distanceBetween = angle - currentAngle;
        //SmartDashboard.putNumber("FirstDistBetween", distanceBetween);
        // ensure distancBetween is in range [0, PodCntsRev]
        if (distanceBetween < 0)
            distanceBetween += PodCntsRev;
        //SmartDashboard.putNumber("Init DistBetween", distanceBetween);
        
        if (distanceBetween > (PodCntsRev - distanceBetween)) {
            distanceBetween = PodCntsRev - distanceBetween;
            sign *= -1;
        }

        if (distanceBetween > (PodCntsRev/4)) {      // if the change in angle will be > 90 deg, can reduce by rotating the other direction and reversing wheel speed
            distanceBetween = (PodCntsRev/2) - distanceBetween;
            sign *= -1;
            speed *= -1;
        }
        SmartDashboard.putNumber("distBetween "+m_encoderID, distanceBetween);

        //SmartDashboard.putNumber("speed", speed);
        // keep wheel angle the same unless change will be > ~ 1%
        if (Math.abs(distanceBetween) > (PodCntsRev/100)) {    
            m_rotationMotor.set(ControlMode.Position, (m_rotationMotor.getSelectedSensorPosition() + distanceBetween* sign ));  // should it be -
        }
        SmartDashboard.putNumber("m_encoderCount"+m_encoderID, m_encoder.getVoltage());
        //m_rotationMotor.setSelectedSensorPosition(distanceBetween * sign );

        //speedPID.setReference(speed, ControlType.kDutyCycle);
        //SmartDashboard.putNumber("sign", sign);
        //SmartDashboard.putNumber("Value", m_rotationMotor.getSelectedSensorPosition() + distanceBetween * sign);
        m_speedMotor.set(ControlMode.PercentOutput, speed);
        
        //SmartDashboard.putNumber("Distance Between", distanceBetween);
        //SmartDashboard.putNumber("MotorPosition",m_rotationMotor.getSelectedSensorPosition());
        m_PIDTarget = m_rotationMotor.getSelectedSensorPosition() + distanceBetween;
    }

    public double setZeroOffset(double offset) {
        // On startup, computes the offset of the initial wheel pod position from the static zero offset 
        double offsetDifference = offset-m_encoder.getVoltage();
        SmartDashboard.putNumber("initial pos V "+m_encoderID, m_encoder.getVoltage());
        
        // minimize the amount of change
        if (Math.abs(offsetDifference)>=(Vcc/2)) {
            if (offsetDifference>0) {
                offsetDifference= offsetDifference - Vcc;
            } else if(offsetDifference<=0) {
                offsetDifference= Vcc - Math.abs(offsetDifference);
            } 
        }
        SmartDashboard.putNumber("offsetDifference V "+m_encoderID, offsetDifference);
        //SmartDashboard.putNumber("current"+m_encoderID, m_encoder.getVoltage());
        //SmartDashboard.putNumber("offsetDifference"+m_encoderID, offsetDifference);
        zeroOffset = offsetDifference;
        return zeroOffset;
    }

    public void setIntEncToZero() {
        m_rotationMotor.setSelectedSensorPosition(0);
        SmartDashboard.putNumber("initial zero "+m_encoderID, m_rotationMotor.getSelectedSensorPosition());
    }

    public double getPIDTarget() {
        return m_PIDTarget;
    }
    
    public double getSensorValue() {
        return m_rotationMotor.getSelectedSensorPosition();
    }

    public double getMotorPosition() {
        return m_speedMotor.getSelectedSensorPosition();
    }

    public double getEncoderVoltage() {
        return m_encoder.getVoltage();
    }
}
