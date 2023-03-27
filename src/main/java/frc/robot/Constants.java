package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 20;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(25); //TODO: This must be tuned to specific robot
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
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 28; //25
        public static final int anglePeakCurrentLimit = 50;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 38; //35
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 8.0;//10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.53);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(258.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(212.60);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.72);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final double speedMod = 0.5;
        public static final double balanceSpeedMod = .47;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 4;
        public static final double kPYController = 4;
        public static final double kPXandYControllers = 4;
        public static final double kPThetaController = 4;

        public static final double maxPlatformPositivePitch = 6.6;
        public static final double maxPlatformNegativePitch = -8.6;

        public static final double desiredBalanceAngle = -1.6;
        public static final double balanceP = .5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ExtenderConstants {
        
        public static final int extenderMotorID = 13;

        private final static double gearRatio = 22; // 4:1 cartridge + 4:1 cartrdige + 1.375 
        private final static double winchDia_in = 1.79; // For PWF Arm
        private final static double winchDia_m = Units.inchesToMeters(winchDia_in);
        private final static double winchCircumference_m = winchDia_m * Math.PI;
        private final static double winchCircumference_in = winchDia_in * Math.PI;

        public final static double extenderMetersToNeoRotationsFactor = winchCircumference_m / gearRatio;
        public final static double extenderMetersVelocityToNeoRotationsFactor = extenderMetersToNeoRotationsFactor / 60;

        public final static double extenderInchesToNeoRotationsFactor = winchCircumference_in / gearRatio;
        public final static double extenderInchesVelocityToNeoRotationsFactor = extenderInchesToNeoRotationsFactor / 60;
    }
    
    public static final class ElevatorConstants {

        public static final int elevatorMotorID = 14;

        private final static double gearRatio = 49.5; // 4:1 cartridge + 3:1 cartrdige + 3:1 Cartridge + 1.375
        private final static double winchDia_in = 1.79; // For PWF Arm
        private final static double winchDia_m = Units.inchesToMeters(winchDia_in);
        private final static double winchCircumference_m = winchDia_m * Math.PI;
        private final static double winchCircumference_in = winchDia_in * Math.PI;
        
        public final static double elevatorMetersToNeoRotationsFactor = winchCircumference_m / gearRatio;
        public final static double elevatorMetersVelocityToNeoRotationsFactor = elevatorMetersToNeoRotationsFactor / 60;
        
        public final static double elevatorInchesToNeoRotationsFactor = winchCircumference_in / gearRatio;
        public final static double elevatorInchesVelocityToNeoRotationsFactor = elevatorInchesToNeoRotationsFactor / 60;

        //FeedForward Gains
        public static final double elevatorKs = 0;
        public static final double elevatorKg = 0;
        public static final double elevatorKv = 0;
        public static final double elevatorKa = 0;

        //PID Controller
        public static final double elevatorKp = 0;
        public static final double elevatorKi = 0;
        public static final double elevatorKd = 0;

        //Constraints
        public static final double elevatorMaxVel = 0;
        public static final double elevatorMaxAccel = 0;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 15;
    }

    public static final class WristConstants {        
        public static final int wristMotorID = 16;
    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

}