/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class LeaveInitialLine extends CommandBase {

    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    double targetDis = 6;
    double distanceErr;
    double distanceErrTolerance = 0.3;

    public LeaveInitialLine(Limelight limelight, Drivetrain drivetrain, Shooter shooter) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        addRequirements(this.limelight, this.drivetrain, this.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double Kp = 0.21;
        distanceErr = targetDis - limelight.getEstimatedDistance(26.2);
        double distanceAdjustment = Kp * distanceErr;

        drivetrain.velocityDrive(distanceAdjustment, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.velocityDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(distanceErr) <= distanceErrTolerance) {
            return true;
        }
        return false;
    }

}
