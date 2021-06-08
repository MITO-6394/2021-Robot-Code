/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

        holder.set(false);
        rTelescopeTalon.follow(lTelescopeTalon);

        Utility.configTalonSRXPID(lTelescopeTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);
        Utility.configTalonSRXPID(rTelescopeTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);

        lTelescopeTalon.configPeakOutputForward(1.0);
        lTelescopeTalon.configPeakOutputReverse(-1.0);

        lTelescopeTalon.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public void periodic() {
    }

    /**
     * Release the climber.
     */
    public void release() {
        if (!insurance) {
            solenoidState = !solenoidState;
            holder.set(solenoidState);
        } else {
            insurance = false;
        }
    }

    public void stretch() {
        lTelescopeTalon.set(TalonSRXControlMode.Position,
                -Constants.Climber.stretchRotations * Constants.talonPositionConstant);
    }

    public void telescope() {
        lTelescopeTalon.set(TalonSRXControlMode.Position, 0);
    }

    public void stop() {
        lTelescopeTalon.set(ControlMode.PercentOutput, 0);
    }

}
