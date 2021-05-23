/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 10;

	//velocity constant
	public static final double falconVelocityConstant = 2048.0/600.0;
	public static final double velocityConstants = 4096.0/600.0;

	//rotation
	public static final double kTurnToleranceDeg = 1;
	public static final double kTurnRateToleranceDegPerS = 10;

	//target speed
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
		
		public static final int flywheelTargetSpeed = 4000;

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
	public static final double kpAim = -0.1;
	public static final double minAimCommend = 0.05;
	public static final double kTxTorrance = 0;
	public static final double kTyTorrance = 0;
	public static final double kpDistance = -0.1;

	// spinner(postion control)
    public static final int robotToSensorDis = 0;
    
    public static final class Climber {

		public static final double stretchOutput = 0.3;
		public static final double telescopeOutput = 0.5;

	}

    public static final class Drum {

        // The percentage output of the drum's rotation when intaking balls
        public static final double intakeOutput = 1.0;

        // The percentage output of the drum's rotation when loading balls
        public static final double loadOutput = 1.0;

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
		public static final double rotationCoeff = 0.5;
		public static final double joystickYErrorTolerance = 0.15; // The tolerance of joystick(left) error in the Y-axis. While the axis is within this value, correct it to 0.

		public static final double aimRate = 1.0;
		// Test
		public static final int MotorSpeed = 3;




}
