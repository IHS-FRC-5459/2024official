package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.LinearInterpolator;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Climber {
        public static final int climberMotorID = 24;
        public static final double climberDownPower = -0.75;
        public static final double climberUpPower = 1;
    }

    public static final class LimeLight {
        public static final double kP_rotate = 0.025;
        public static final double kD_rotate = -0.0014;

        public static final String llTableName = "limelight";
        public static final String targetInVisionKey = "tv";
        public static final String targetInAngleKey = "tx";
        //public static final String targetPoseCameraSpaceKey = "targetpose_cameraspace";
        public static final String targetPoseRobotSpaceKey = "targetpose_robotspace";

        public static final double[] cubicFit = {0.603155,-8.36751,40.1452,-26.7}; 
//old:0.191216 x^3 - 4.39133 x^2 + 27.9382 x - 14.6854
//0.603155 x^3 - 8.36751 x^2 + 40.1452 x - 26.4888
        public static final double[] shotRange = {1.1,4.6}; // in meters, closest location for shots, furthest location for shots.
    }


    public static final class Arm {
        public static final double subwooferAngle = 15;

        public static final double restingAngle = -6.2;
        public static final double ampAngle = 90;
        public static final double voltageDeadband = 0.1;

        public static final double[][] shotAngles = 
        {{1.33,15},{2,26},{2.5,31},{3.02,34.4},{3.5,36.25}, {4,37}, {4.5,38.1}, {5,39.1}};
        public static final LinearInterpolator shooterLinInt = new LinearInterpolator(shotAngles);
    }

    public static final class EndEffector {
        public static final int flywheelMotor1 = 41;
        public static final int flywheelMotor2 = 43;
        public static final int intakeMotor = 40;

        public static final double waitTime = 4;


        public static final double restingVelocity =  200 * 0.016666666666667;

        public static final double subwooferShotRPS = 3600 * 0.016666666666667;
        public static final double speakerShotRPS = 3600 * 0.016666666666667;
        public static final double ampShotRPS = 500 * 0.016666666666667;
        public static final double neutralRPS = 200 * 0.016666666666667;
        public static final double intakingFlywheelRPS = -200 * 0.016666666666667;



        public static final double intakingPower = 0.75;
        public static final double passthroughPower = 0.4;
        public static final double recenterPower = -0.13;



    }

    public static final class Swerve {

        public static final int centerWaitTime = 1; 

        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.217); 
        public static final double wheelBase = Units.inchesToMeters(18.217); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10771; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.14012);
        public static final double driveKV = (2.2247);
        public static final double driveKA = (0.25784);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(360-12.128);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(88.417);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(164.619);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(360-69.697);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 5.5;
        public static final double kPYController = 2;
        public static final double kPThetaController = 0.9;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
