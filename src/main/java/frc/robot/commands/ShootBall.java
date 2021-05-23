/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBall extends CommandBase {

    private final Shooter shooter;
    private final Drum drum;

    public ShootBall(Shooter shooter, Drum drum) {
        this.shooter = shooter;
        this.drum = drum;
        addRequirements(this.shooter, this.drum);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drum.rotate(Constants.Drum.loadOutput);
        shooter.shoot();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drum.stop();
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
