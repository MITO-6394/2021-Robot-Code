/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 10;

	// Velocity constants

	// Multiply this constant with the target RPM to get the velocity input of
	// TalonFX in units/100ms.
	public static final double falconVelocityConstant = 4 * 2048 / 600.0;
	// Multiply this constant with the target RPM to get the velocity input of
	// TalonSRX in units/100ms.
	public static final double talonVelocityConstant = 4 * 1024 / 600.0;

	// Position constatns

	public static final double falconPositionConstant = 2048;
	public static final double talonPositionConstant = 4096;

	// rotation
	public static final double kTurnToleranceDeg = 1;
	public static final double kTurnRateToleranceDegPerS = 10;

	// target speed
	public static final double drivetrainTargetRPM = 500;
	public static final double conveyerOutput = 0;
	public static final double spinnerOutput = 0;
	public static final double intakeMotorOutput = -0.35;

	public static final double wheelRadius = 0.152;
	public static final double targetDriveSpeed = 5; // meter per second
	public static final double encoderCPR = 2048; // Counts Per Revolution of the encoder

	// motor safety
	public static final int kCurrentLimit = 8;
	public static final int kCurrentLimitDuration = 10;

	public static final class Shooter {

		public static int flywheelTargetSpeed = 650;
		public static final double rotateTargetSpeed = 100 * Constants.falconVelocityConstant;
		public static final int flywheelSpeedTolerance = 100;
		public static final double aimConstant = Constants.talonVelocityConstant * 1600;
		public static final double elevationConstant = Constants.talonVelocityConstant * 3000;
		public static final double minElevateCommand = 0.001;

	}

	// shooter
	// shooter angle
	public static final int shooterMaxAngle = 0;
	public static final int shooterMinAngle = 0;

	// shooter calculation constants
	public static final double taToDistanceRatio = 0;
	public static final double regressionCoeff = 0;
	public static final double largerShooterAngleDis = 0;

	// flywheel whole cycle (in raw sensor units)
	public static final double flywheelOneCircle = 4096;

	// shooter & sensor gap
	public static final double shooterWaitSecond = 1;

	// Ball loader speed (in percent output)
	public static final double loadBallTalonOutput = -0.35;

	// limelight
	public static final double kpAim = -0.02;
	public static final double minAimCommand = 0.0;
	public static final double kTxTolerance = 0.1;
	public static final double kTyTolerance = 0;
	public static final double kpDistance = -0.1;

	// spinner(postion control)
	public static final int robotToSensorDis = 0;

	public static final class Drivetrain {

		public static final double accelerationLimit = 0.05;
		public static final double ks = 0.635;
		public static final double kv = 1.24;
		public static final double ka = 0.689;
		public static final double kTrackwidth = 0.654;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);
		public static final double kMaxSpeed = 1.5;
		public static final double kMaxAcceleration = 1;
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
		public static final double kPDriveVel = 20.6 * 0.2;

	}

	public static final class Climber {

		public static final double stretchTargetPos = 0.1;
		public static final double telescopeTargetPos = 0.1;
		public static final double telescopeTargetSpeed = 400;

	}

	public static final class Drum {

		// The percentage output of the drum's rotation when intaking balls
		public static final double intakeOutput = 0.5;

		// The percentage output of the drum's rotation when loading balls
		public static final double loadOutput = 0.6;

		public static final double regularOutput = 0.5;

	}

	public static final class Controller {
		public static final double axisZeroCorrectionRange = 0.15;
		public static final double triggerPressedThreshold = 0.75;
	}

	// Joystick

	// Intake
	public static final int bBallIntake = 0;
	public static final int bIntakeBasket = 0;
	public static final int bUpperBallIntake = 0;

	// Spinner
	public static final int bRotationControl = 0;
	public static final int bPositionControl = 0;
	public static final int bSpinnerSolenoid = 0;

	// Shooter
	public static final int bShoot = 0;

	// Drivetrain
	public static final int forwardAxis = 1;
	public static final int rotationAxis = 4;
	public static final double rotationCoeff = 0.35;
	public static final double joystickYErrorTolerance = 0.15; // The tolerance of joystick(left) error in the Y-axis.
																// While the axis is within this value, correct it to 0.

	public static final double aimRate = 1.0;
	// Test
	public static final int MotorSpeed = 3;

}
