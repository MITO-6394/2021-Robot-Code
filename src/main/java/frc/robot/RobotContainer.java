/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.LeaveInitialLine;
import frc.robot.commands.ShootBall;
import frc.robot.commands.Aim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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

    // Subsystems
    private final Compressor compressor = new Compressor();
    private final XboxController controller = new XboxController(Pin.Controller.moveStick);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Drum drum = new Drum();
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight limelight = new Limelight();

    // Commands
    private final Aim aim = new Aim(limelight, shooter);
    private final IntakeBall intakeBall = new IntakeBall(intake, drum);
    private final ShootBall shootBall = new ShootBall(shooter, drum);
    private final LeaveInitialLine leaveInitialLine = new LeaveInitialLine(limelight, drivetrain, shooter);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        CameraServer.getInstance().startAutomaticCapture(0);

        drivetrain.setDefaultCommand(new RunCommand(
                () -> drivetrain.velocityDrive(Utility.controllerAxisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.forward)),
                        Utility.controllerAxisZeroCorrect(controller.getRawAxis(Pin.Controller.Axis.rotation))),
                drivetrain));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Shift gearbox
        new JoystickButton(controller, Pin.Controller.Button.shiftGearbox)
                .whenActive(new InstantCommand(drivetrain::shiftGearbox, shooter));

        // Deploy intake system
        new JoystickButton(controller, Pin.Controller.Button.deployIntake)
                .whenActive(new InstantCommand(intake::deploy, intake));

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
        new JoystickButton(controller, Pin.Controller.Button.stretchClimber)
                .whenActive(new InstantCommand(climber::stretch, climber));

        // Telescope climber
        new JoystickButton(controller, Pin.Controller.Button.telescopeClimber)
                .whenActive(new InstantCommand(climber::telescope, climber));

        // Aim
        new JoystickButton(controller, Pin.Controller.Button.aim).whenPressed(aim);

    }



    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(leaveInitialLine, aim.withTimeout(2), shootBall.withTimeout(5));
    }

}
