// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLEFT_MOTOR_PORT = 0;
    public static final int kRIGHT_MOTOR_PORT = 1;
    
    public static final int kLEFT_ENCODER_PORT_A = 4;
    public static final int kLEFT_ENCODER_PORT_B = 5;
    public static final int kRIGHT_ENCODER_PORT_A = 6;
    public static final int kRIGHT_ENCODER_PORT_B = 7;
    public static final boolean kLEFT_ENCODER_REVERSED = false;
    public static final boolean kRIGHT_ENCODER_REVERSED = true;

    // Romi encoders are directly mounted on the motor shafts, 
    // CPR = 12. Gear ration = 120:1, for WheelCPR = 1440.
    public static final double kCOUNTS_PER_REV = 1440.0;
    public static final double kWHEEL_DIA_INCH = 2.75591; // 70 mm
    public static final double kENCODER_INCHES_PER_ROMI_PULSE = 
          (Math.PI * kWHEEL_DIA_INCH) / kCOUNTS_PER_REV; // Units: inches

    public static final boolean kGYRO_REVERSED = false;

    public static final double kDRIVE_STABIL_P = 1;
    public static final double kDRIVE_STABIL_I = 0.05;
    public static final double kDRIVE_STABIL_D = 0;

    public static final double kDRIVE_DIST_P = 1;
    public static final double kDRIVE_DIST_I = 0.005;
    public static final double kDRIVE_DIST_D = 0;

    public static final double kDIST_TOLERANCE = 1.1; // 1.1 inches is close enough
    public static final double kMAX_TRAVEL_RATE_IN_PER_S = 60; // inches per second
    public static final double kTRAVEL_RATE_TOLERANCE_IN_PER_S = 10; // inches per second
    public static final double kMAX_TRAVEL_ACCEL_IN_PER_S_SQ = 300; // inches per second squared

    public static final double kTURN_P = 1;
    public static final double kTURN_I = 0;
    public static final double kTURN_D = 0;

    public static final double kTURN_TOLERANCE_DEG = 1.5;
    public static final double kMAX_TURN_RATE_DEG_PER_S = 100;
    public static final double kTURN_RATE_TOLERANCE_DEG_PER_S = 10; // degrees per second
    public static final double kMAX_TURN_ACCEL_DEG_PER_S_SQ = 300;
  }

  public static final class OIConstants {
  /*
      Enum values for controller axis inputs
      Represents an axis on an XboxController:
      public enum Axis {
        kLeftX(0),
        kLeftY(1),
        kLeftTrigger(2),
        kRightTrigger(3),
        kRightX(4),
        kRightY(5);
      }
  */
    public static final int kXBOX_LEFT_STICK_LR_AXIS = 0;
    public static final int kXBOX_LEFT_STICK_FB_AXIS = 1;
    public static final int kXBOX_LEFT_TRIGGER_AXIS = 2;
    public static final int kXBOX_RIGHT_TIGGER_AXIS = 3;
    public static final int kXBOX_RIGHT_STICK_LR_AXIS = 4;
    public static final int kXBOX_RIGHT_STICK_FB_AXIS = 5;

    public static final int kDRIVE_Y_AXIS = kXBOX_LEFT_STICK_FB_AXIS;
    public static final int kDRIVE_X_AXIS = kXBOX_LEFT_STICK_LR_AXIS;
    public static final int kROTATE_AXIS = kXBOX_RIGHT_STICK_LR_AXIS;

    /*
      Enum values for controller BUTTONS. 
      Use is m_XboxController.Button.EnumSymbolic. A specific example is:
        m_Xbox.Button.kA (for the A button)
      
      This enum list is already defined in XboxController class, so use as is:
      
      public enum Button {
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kLeftBumper(5),
        kRightBumper(6),
        kBack(7),
        kStart(8),
        kLeftStick(9),
        kRightStick(10);
      }

      To Make use of the RUMBLE feedback function, use the following:
        m_Xbox.setRumble(RumbleType.kLeftRumble, amplitudeValue);
        m_Xbox.setRumble(RumbleType.kRightRumble, amplitudeValue);
      where
        Left runble is weaker, higher frequency runble
        Right rumble is a stronger, lower frequency runble
        and amplitudeValue can range from 0 (off) to 1.0 (max amplitude), for the specified side
*/
    public static final int kXBOX_A_BUTTON = 0;
    public static final int kXBOX_B_BUTTON = 1;
    public static final int kXBOX_X_BUTTON = 2;
    public static final int kXBOX_Y_BUTTON = 3;
    public static final int kXBOX_RIGHT_BUMPER_BUTTON = 4;
    public static final int kXBOX_LEFT_BUMPER_BUTTON = 5;
    public static final int kXBOX_BACK_BUTTON = 6;
    public static final int kXBOX_START_BUTTON = 7;
    public static final int kXBOX_LEFT_STICK_BUTTON = 12;
    public static final int kXBOX_RIGHT_STICK_BUTTON = 11;


    public static final int kGAME_CONTROLLER_PORT = 0;
    public static final int kDRIVE_JOYSTICK_AXIS = 1;
    public static final int kROTATE_JOYSTICK_AXIS = 4;
    public static final double kSLEW_RATE_LIMIT = 10;   /* For use with wpi filter class
                                                         edu.wpi.first.math.filter.SlewRateLimiter
                                                         the lower the value, the slower the response
                                                         the higher the value, the faster the response */
  }
  public static final int kDISABLED = 0;
  public static final int kAUTO = 1;
  public static final int kTELEOP = 2;
  public static final int kTEST = 3;
}