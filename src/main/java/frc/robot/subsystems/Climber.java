/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;

public class Climber extends SubsystemBase {

    private Solenoid holder = new Solenoid(Pin.Climber.Solenoid.holder);
    private WPI_TalonFX lTelescopeFalcon = new WPI_TalonFX(Pin.Climber.Motor.lTelescopeFalcon);
    private WPI_TalonFX rTelescopeFalcon = new WPI_TalonFX(Pin.Climber.Motor.rTelescopeFalcon);

    // Climber is allowed to be released if insurance is false.
    private boolean insurance = true;

    private boolean solenoidState = false;

    public Climber() {
        holder.set(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Release the climber.
     */
    public void release() {
        if (insurance) {
            solenoidState = !solenoidState;
            holder.set(solenoidState);
        }
        else {
            insurance = false;
        }
    }

    public void stretch() {
        lTelescopeFalcon.set(ControlMode.PercentOutput, -Constants.Climber.stretchOutput);
        rTelescopeFalcon.set(ControlMode.PercentOutput, Constants.Climber.stretchOutput);
    }

    public void telescope() {
        lTelescopeFalcon.set(ControlMode.PercentOutput, Constants.Climber.telescopeOutput);
        rTelescopeFalcon.set(ControlMode.PercentOutput, -Constants.Climber.telescopeOutput);
    }

    public void stop() {
        lTelescopeFalcon.set(ControlMode.PercentOutput, 0);
        rTelescopeFalcon.set(ControlMode.PercentOutput, 0);
    }

}
