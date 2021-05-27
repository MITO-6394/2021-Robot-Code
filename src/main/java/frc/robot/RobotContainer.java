/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.naming.LimitExceededException;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeBall;
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
import frc.robot.commands.turnToAngle;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Shooter;
import frc.robot.TriggerPressed;

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
    private final Compressor compressor = new Compressor();
    private final XboxController controller = new XboxController(Pin.Controller.moveStick);
    private final Joystick functionStick = new Joystick(Pin.Controller.functionStick);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Drum drum = new Drum();
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight limelight = new Limelight();
    private final ExampleSubsystem example = new ExampleSubsystem();

    private final Aim aim = new Aim(limelight, shooter);
//     private final IntakeBall intakeBall = new IntakeBall(intake, drum, controller.getTriggerAxis(Hand.kLeft));
    private final IntakeBall intakeBall = new IntakeBall(intake, drum);
    private final ShootBall shootBall = new ShootBall(shooter, drum);
    private final StretchClimber stretchClimber = new StretchClimber(climber);
    private final TelescopeClimber telescopeClimber = new TelescopeClimber(climber);
    // private Shooter mShooter = new Shooter();
    // private BallIntake mBallIntake = new BallIntake();
    // private Spinner mSpinner = new Spinner();

    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        
        drivetrain.setDefaultCommand(
            new RunCommand(() -> drivetrain.velocityDrive(axisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.forward)),
                    axisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.rotation))), drivetrain));
            
        // limelight.setDefaultCommand(defaultCommand);
        shooter.setDefaultCommand(new RunCommand(shooter::stop, shooter));
        // drum.setDefaultCommand(new RunCommand(() -> drum.stop(), drum));
        // intake.setDefaultCommand(new RunCommand(() -> intake.launch(controller.getTriggerAxis(Hand.kLeft)), intake));
        // drum.setDefaultCommand(new RunCommand(() -> drum.rotate(Constants.Drum.intakeOutput, controller.getTriggerAxis(Hand.kLeft), controller.getTriggerAxis(Hand.kRight)), drum));
        // shooter.setDefaultCommand(new RunCommand(() -> shooter.shoot(controller.getTriggerAxis(Hand.kRight)), shooter));
        //intake.setDefaultCommand(new IntakeBall(intake, drum, controller.getTriggerAxis(Hand.kLeft)));
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
        
        // new JoystickButton(controller, Pin.Controller.Button.autoAim).whileActiveContinuous(dummyAutoAim);

        // Shift gearbox
        new JoystickButton(controller, Pin.Controller.Button.shiftGearbox)
            .whenActive(new InstantCommand(drivetrain::shiftGearbox, drivetrain));

        // Deploy intake system
        new JoystickButton(controller, Pin.Controller.Button.deployIntake)
            .whenActive(new InstantCommand(intake::deploy, intake));

        // Hold intake system
        new JoystickButton(controller, Pin.Controller.Button.holdIntake)
            .whenActive(new InstantCommand(intake::hold, intake));

        // Intake balls
        new TriggerPressed(controller, Hand.kLeft).whileActiveOnce(intakeBall);

        // Shoot balls
        new TriggerPressed(controller, Hand.kRight).whileActiveOnce(shootBall);

        // Rotate shooter leftward
        new JoystickButton(controller, Pin.Controller.Button.rotateShooterLeftward)
            .whileHeld(new InstantCommand(() -> shooter.rotate(-Constants.Shooter.rotateTargetSpeed), shooter));

        // Rotate shooter rightward
        new JoystickButton(controller, Pin.Controller.Button.rotateShooterRightward)
            .whileHeld(new InstantCommand(() -> shooter.rotate(Constants.Shooter.rotateTargetSpeed), shooter));

        // Release climber
        new JoystickButton(controller, Pin.Controller.Button.releaseClimber)
            .whenActive(new InstantCommand(climber::release, climber));

        // Stretch climber
        new JoystickButton(controller, Pin.Controller.Button.stretchClimber)
            .whenActive(stretchClimber);

        // Telescope climber
        new JoystickButton(controller, Pin.Controller.Button.telescopeClimber)
            .whenActive(telescopeClimber);

        // Aim
        new PerpetualCommand(aim);


        // // Test solenoid
        // new JoystickButton(controller, XboxController.Button.kY.value)
        //         .whenActive(new InstantCommand(example::shiftSolenoid, example));

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
        if (axis <= Constants.Controller.axisZeroCorrectionRange && axis >= -Constants.Controller.axisZeroCorrectionRange) {
                return -0.01;
        }
        return axis;
    }

}
