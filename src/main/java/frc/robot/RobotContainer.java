/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.LeaveInitialLine;
import frc.robot.commands.ShootBall;
import frc.robot.commands.StretchClimber;
import frc.robot.commands.TelescopeClimber;
// import frc.robot.commands.accurateAutoAim;
// import frc.robot.commands.conveyBall;
import frc.robot.commands.Aim;
// import frc.robot.commands.intakeBall;
// import frc.robot.commands.positionControl;
// import frc.robot.commands.rotationControl;
// import frc.robot.commands.shooterPrep;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private Joystick moveStick = new Joystick(Pin.moveStickPort);
    // private final UsbCamera camera = new UsbCamera("Camera", 1);

    private final Compressor compressor = new Compressor();
    private final XboxController controller = new XboxController(Pin.Controller.moveStick);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Drum drum = new Drum();
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight limelight = new Limelight();

    private final Aim aim = new Aim(limelight, shooter);
    private final IntakeBall intakeBall = new IntakeBall(intake, drum);
    private final ShootBall shootBall = new ShootBall(shooter, drum);
    private final StretchClimber stretchClimber = new StretchClimber(climber);
    private final TelescopeClimber telescopeClimber = new TelescopeClimber(climber);
    private final LeaveInitialLine leaveInitialLine = new LeaveInitialLine(limelight, drivetrain, shooter);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        CameraServer.getInstance().startAutomaticCapture(0);
        // CameraServer.getInstance().addSwitchedCamera("font");

        drivetrain.setDefaultCommand(new RunCommand(
                () -> drivetrain.velocityDrive(axisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.forward)),
                        axisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.rotation))),
                drivetrain));

        // limelight.setDefaultCommand(defaultCommand);
        shooter.setDefaultCommand(new RunCommand(shooter::defaultState, shooter));
        drum.setDefaultCommand(new RunCommand(() -> drum.rotate(Constants.Drum.regularOutput), drum));
        // intake.setDefaultCommand(new RunCommand(() ->
        // intake.launch(controller.getTriggerAxis(Hand.kLeft)), intake));
        // drum.setDefaultCommand(new RunCommand(() ->
        // drum.rotate(Constants.Drum.intakeOutput,
        // controller.getTriggerAxis(Hand.kLeft),
        // controller.getTriggerAxis(Hand.kRight)), drum));
        // shooter.setDefaultCommand(new RunCommand(() ->
        // shooter.shoot(controller.getTriggerAxis(Hand.kRight)), shooter));
        // intake.setDefaultCommand(new IntakeBall(intake, drum,
        // controller.getTriggerAxis(Hand.kLeft)));
        // m_shooter.setDefaultCommand(
        // new RunCommand(() ->
        // m_shooter.TestMotor(moveStick.getRawAxis(Constants.MotorSpeed)), m_shooter));

        // mShooter
        // .setDefaultCommand(new RunCommand(() ->
        // mShooter.shootBall(Constants.shooterRegularRunningSpeed), mShooter));

        // mLimelight.setDefaultCommand(new dummyAutoAim());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // new JoystickButton(controller,
        // Pin.Controller.Button.autoAim).whileActiveContinuous(dummyAutoAim);

        // Shift gearbox
        new JoystickButton(controller, Pin.Controller.Button.shiftGearbox)
                .whenActive(new InstantCommand(drivetrain::shiftGearbox, drivetrain));

        // Deploy intake system
        new JoystickButton(controller, Pin.Controller.Button.deployIntake)
                .whenActive(new InstantCommand(intake::deploy, intake));

        // Hold intake system
        // new JoystickButton(controller, Pin.Controller.Button.holdIntake)
        //         .whenActive(new InstantCommand(intake::hold, intake));

        // Intake balls
        new TriggerPressed(controller, Hand.kRight).whileActiveOnce(intakeBall);

        // Shoot balls
        new TriggerPressed(controller, Hand.kLeft).whileActiveContinuous(shootBall, false);

        // Rotate shooter leftward
        new JoystickButton(controller, Pin.Controller.Button.rotateShooterLeftward)
                .whileHeld(new InstantCommand(() -> shooter.rotate(Constants.Shooter.rotateTargetSpeed), shooter));

        // Rotate shooter rightward
        new JoystickButton(controller, Pin.Controller.Button.rotateShooterRightward)
                .whileHeld(new InstantCommand(() -> shooter.rotate(-Constants.Shooter.rotateTargetSpeed), shooter));

        // Release climber
        new JoystickButton(controller, Pin.Controller.Button.releaseClimber)
                .whenActive(new InstantCommand(climber::release, climber));

        // Stretch climber
        new JoystickButton(controller, Pin.Controller.Button.stretchClimber).whenActive(stretchClimber);

        // Telescope climber
        new JoystickButton(controller, Pin.Controller.Button.telescopeClimber).whenActive(telescopeClimber);

        // // Increase elevation
        // new JoystickButton(controller, Pin.Controller.Button.stretchClimber)
        // .whileHeld(new InstantCommand(() -> shooter.elevate(200), shooter));

        // // Decrease
        // new JoystickButton(controller, Pin.Controller.Button.telescopeClimber)
        // .whileHeld(new InstantCommand(() -> shooter.elevate(-200), shooter));

        // Aim
        // new PerpetualCommand(aim);
        new JoystickButton(controller, Pin.Controller.Button.aim).whenHeld(aim);


        // Down the shooter
        // new TriggerPressed(controller, Hand.kLeft).when(new InstantCommand(() ->
        // shooter.elevateToTargetElevation(11)));

        // // Test solenoid
        // new JoystickButton(controller, XboxController.Button.kY.value)
        // .whenActive(new InstantCommand(example::shiftSolenoid, example));

        // Spinner
        // new JoystickButton(functionStick, Constants.bPositionControl)
        // .whenPressed(new positionControl());
        // new JoystickButton(functionStick, Constants.bRotationControl)
        // .whenPressed(new rotationControl());
        // new JoystickButton(functionStick, Constants.bSpinnerSolenoid)
        // .whenPressed(new InstantCommand(mSpinner::spinnerState, mSpinner));

        // // Ball Intake
        // new JoystickButton(functionStick, Constants.bBallIntake)
        // .and(new JoystickButton(functionStick,Constants.bUpperBallIntake))
        // .whenActive(new intakeBall(1))
        // .whenInactive(new intakeBall(3));

        // new JoystickButton(functionStick, Constants.bBallIntake)
        // .whenActive(new intakeBall(2))
        // .whenInactive(new intakeBall(3));

        // new JoystickButton(functionStick, Constants.bIntakeBasket)
        // .whenPressed(new InstantCommand(mBallIntake::basketStateChange,
        // mBallIntake));

        // // shooter
        // new JoystickButton(moveStick, Constants.bShoot)
        // .whenPressed(new SequentialCommandGroup(new accurateAutoAim().alongWith(new
        // dummyAutoAim()),
        // new SequentialCommandGroup(new conveyBall(),
        // new WaitCommand(Constants.shooterWaitSecond))
        // .deadlineWith(new shooterPrep())));

        // drivetrain
        // new JoystickButton(moveStick, Constants.buttonShiftGear)
        // .whenPressed(new InstantCommand(m_drivetrain::shiftGear, m_drivetrain));
        // if (moveStick.getPOV() != 0) {
        // new POVButton(moveStick, moveStick.getPOV()).whenPressed(
        // new turnToAngle(Utility.targetAngleModify(moveStick.getPOV(),
        // mDrivetrain.getCurrentAngle()), mDrivetrain));
        // }

    }

    private double axisZeroCorrect(double axis) {
        if (axis <= Constants.Controller.axisZeroCorrectionRange
                && axis >= -Constants.Controller.axisZeroCorrectionRange) {
            return -0.01;
        }
        return axis;
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(leaveInitialLine, aim, shootBall);
    }

    // /**
    //  * Use this to pass the autonomous command to the main {@link Robot} class.
    //  *
    //  * @return the command to run in autonomous
    //  */
    // public Command getAutonomousCommand() {

    //     // Create a voltage constraint to ensure we don't accelerate too fast
    //     var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(Constants.Drivetrain.ks,
    //                                 Constants.Drivetrain.kv,
    //                                 Constants.Drivetrain.ka),
    //             Constants.Drivetrain.kDriveKinematics,
    //             10);

    //     // Create config for trajectory
    //     TrajectoryConfig config =
    //     new TrajectoryConfig(Constants.Drivetrain.kMaxSpeed,
    //                         Constants.Drivetrain.kMaxAcceleration)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(Constants.Drivetrain.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    //     // An example trajectory to follow.  All units in meters.
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(
    //             new Translation2d(1, 0),
    //             new Translation2d(2, 0)
    //         ),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         // Pass config
    //         config
    //     );

    //     RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose,
    //             new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
    //             new SimpleMotorFeedforward(Constants.Drivetrain.ks, Constants.Drivetrain.kv, Constants.Drivetrain.ka),
    //             Constants.Drivetrain.kDriveKinematics, drivetrain::getWheelSpeeds,
    //             new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
    //             new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
    //             // RamseteCommand passes volts to the callback
    //             drivetrain::tankDriveVolts, drivetrain);


    // //    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup(ramseteCommand, aim, shootBall);

    //     // Reset odometry to the starting pose of the trajectory.
    //     drivetrain.resetOdometry(trajectory.getInitialPose());

    //     // Run path following command, then stop at the end.
    //     return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
        
    // }

}
