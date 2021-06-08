/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Aim extends CommandBase {

    private Limelight limelight;
    private Shooter shooter;

    public Aim(Limelight limelight, Shooter shooter) {
        this.limelight = limelight;
        this.shooter = shooter;
        addRequirements(this.limelight);
        addRequirements(this.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double steeringAdjustment = 0;
        if (limelight.isTargetFound()) {
            SmartDashboard.putBoolean("Target found?", true);

            double headError = -limelight.getTx();

            if (limelight.getTx() > 1.0) {
                steeringAdjustment = Constants.Limelight.aimKp * headError - Constants.Limelight.minAimCommand;
            } else if (limelight.getTx() < 1.0) {
                steeringAdjustment = Constants.Limelight.aimKp * headError + Constants.Limelight.minAimCommand;
            }

            shooter.rotate(steeringAdjustment);

        } else {
            SmartDashboard.putBoolean("Target found?", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.rotate(0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(limelight.getTx()) < Constants.Limelight.txTolerance) {
            SmartDashboard.putBoolean("Aimed?", true);
            return true;
        }
        SmartDashboard.putBoolean("Aimed?", false);
        return false;
    }

}
