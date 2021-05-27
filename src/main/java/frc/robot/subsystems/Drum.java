/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pin;

public class Drum extends SubsystemBase {

    private TalonSRX rotateTalon = new TalonSRX(Pin.Drum.Motor.rotateTalon);

    public Drum() {
        rotateTalon.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        rotateTalon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Rotate the drum.
     * @param out The percentage output of the motor.
     */
    public void rotate(Double out) {
        rotateTalon.set(ControlMode.PercentOutput, out);
    }

    public void stop() {
        rotateTalon.set(ControlMode.PercentOutput, 0);
    }

}
