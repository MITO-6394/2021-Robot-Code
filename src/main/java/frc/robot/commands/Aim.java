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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter;

public class Aim extends CommandBase {

    private Limelight limelight;
    private Shooter shooter;

    double headError;
    double steeringAdjustment;

    public Aim(Limelight limelight, Shooter shooter) {
        this.limelight = limelight;
        this.shooter = shooter;
        addRequirements(this.limelight);
        addRequirements(this.shooter);
        // addRequirements(mShooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        steeringAdjustment = 0.0;
        if (limelight.isTargetFound()) {
            headError = -limelight.getTx();

            Double tx = limelight.getTx();
            Double ty = limelight.getTy();

            SmartDashboard.putNumber("tx", tx);
            SmartDashboard.putNumber("ty", ty);

            if (limelight.getTx() > 1.0) {
                steeringAdjustment = Constants.kpAim * headError - Constants.minAimCommend;
            } else if (limelight.getTx() < 1.0) {
                steeringAdjustment = Constants.kpAim * headError + Constants.minAimCommend;
            }

            System.out.println("steeringAdjustment:" + steeringAdjustment);

            // mShooter.shooterRotation(steeringAdjustment);
            // drivetrain.velocityDrive(0.0, -steeringAdjustment * Constants.aimRate);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(limelight.getTx()) < Constants.kTxTorrance;
    }

}
