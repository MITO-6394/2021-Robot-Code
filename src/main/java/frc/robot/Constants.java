/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

	// Multiply this constant with the target RPM to get the velocity input of
	// TalonFX in units/100ms.
	public static final double falconVelocityConstant = 4 * 2048 / 600.0;
	// Multiply this constant with the target RPM to get the velocity input of
	// TalonSRX in units/100ms.
	public static final double talonVelocityConstant = 4 * 1024 / 600.0;

	public static final double falconPositionConstant = 2048;
	public static final double talonPositionConstant = 4096;

	public static final class Utility {

		public static final int PIDLoopIdx = 0;
		public static final int timeoutMs = 10;

		// Motor safety
		public static final int currentLimit = 8;
		public static final int currentLimitDuration = 10;

	}

	public static final class Shooter {

		public static int flywheelTargetSpeed = 650;
		public static final double rotateTargetSpeed = 65 * falconVelocityConstant;
		public static final int flywheelSpeedTolerance = 100;

		public static final double aimConstant = talonVelocityConstant * 1600;

		// The target elevation in degrees/100
		public static final double targetElevation = 47.0 / 100;
		public static final double elevationConstant = talonVelocityConstant * 3000;
		public static final double elevateKp = -0.8;
		public static final double elevateMinCommand = 0.001;
		public static final double elevationTolerance = 0.01;
		public static final double loadBallTalonOutput = -0.5;

	}

	public static final class Limelight {

		public static final double aimKp = -0.02;
		public static final double minAimCommand = 0.0;
		public static final double txTolerance = 1.0;

	}

	public static final class Drivetrain {

		public static final double drivetrainTargetRPM = 500;
		public static final double rotationCoeff = 0.25;

	}

	public static final class Intake {

		public static final double intakeMotorOutput = -0.35;

	}

	public static final class Climber {

		// Number of rotations of the encoder needed to stretch out the climber
		public static final int stretchRotations = 30;

	}

	public static final class Drum {

		// The percentage output of the drum's rotation when intaking balls
		public static final double intakeOutput = 0.35;

		// The percentage output of the drum's rotation when loading balls
		public static final double shootOutput = 0.82;

		public static final double regularOutput = 0.35;

	}

	public static final class Controller {

		public static final double axisZeroCorrectionRange = 0.15;
		public static final double triggerPressedThreshold = 0.75;

	}

}
