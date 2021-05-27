/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Climber extends SubsystemBase {

    private Solenoid holder = new Solenoid(Pin.Climber.Solenoid.holder);
    private TalonSRX lTelescopeTalon = new TalonSRX(Pin.Climber.Motor.lTelescopeTalon);
    private TalonSRX rTelescopeTalon = new TalonSRX(Pin.Climber.Motor.rTelescopeTalon);

    // Climber is allowed to be released if insurance is false.
    private boolean insurance = true;

    private boolean solenoidState = false;

    public Climber() {
        holder.set(true);
        // Utility.configTalonSRXPID(lTelescopeTalon, 0.1097, 0.22, 0.0, 0, 0, 0);
        // Utility.configTalonSRXPID(rTelescopeTalon, 0.1097, 0.22, 0.0, 0, 0, 0);
        Utility.configTalonSRXPID(lTelescopeTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);
        Utility.configTalonSRXPID(rTelescopeTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);

        lTelescopeTalon.configPeakOutputForward(1.0);
        lTelescopeTalon.configPeakOutputReverse(-1.0);
        rTelescopeTalon.configPeakOutputForward(1.0);
        rTelescopeTalon.configPeakOutputReverse(-1.0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Release the climber.
     */
    public void release() {
        if (!insurance) {
            solenoidState = !solenoidState;
            holder.set(solenoidState);
        }
        else {
            insurance = false;
        }
    }

    public void stretch() {
        // lTelescopeTalon.set(ControlMode.PercentOutput, -Constants.Climber.stretchOutput);
        // rTelescopeTalon.set(ControlMode.PercentOutput, Constants.Climber.stretchOutput);
        // lTelescopeTalon.set(TalonSRXControlMode.Position, -Constants.Climber.stretchTargetPos * Constants.talonPositionConstant);
        lTelescopeTalon.set(TalonSRXControlMode.Position, 30*-4096);
        rTelescopeTalon.set(TalonSRXControlMode.Position, 30*4096);
        // rTelescopeTalon.set(TalonSRXControlMode.Position, Constants.Climber.stretchTargetPos * Constants.talonPositionConstant);
    }

    public void telescope() {
        // lTelescopeTalon.set(ControlMode.PercentOutput, Constants.Climber.telescopeOutput);
        // rTelescopeTalon.set(ControlMode.PercentOutput, -Constants.Climber.telescopeOutput);
        // lTelescopeTalon.set(TalonSRXControlMode.Position, Constants.Climber.telescopeTargetPos * Constants.talonPositionConstant);
        lTelescopeTalon.set(TalonSRXControlMode.Position, 0);
        rTelescopeTalon.set(TalonSRXControlMode.Position, 0);
        // rTelescopeTalon.set(TalonSRXControlMode.Position, -Constants.Climber.telescopeTargetPos * Constants.talonPositionConstant);
    }

    public void stop() {
        lTelescopeTalon.set(ControlMode.PercentOutput, 0);
        rTelescopeTalon.set(ControlMode.PercentOutput, 0);
    }

}
