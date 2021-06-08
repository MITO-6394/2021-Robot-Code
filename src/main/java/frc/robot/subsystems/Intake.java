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
import frc.robot.Utility;

public class Intake extends SubsystemBase {

    private WPI_TalonFX intakeFalcon = new WPI_TalonFX(Pin.Intake.Motor.intakeFalcon);
    private Solenoid deploySolenoid = new Solenoid(Pin.Intake.Solenoid.deploy);

    private boolean deployState = true;

    public Intake() {
        Utility.TalonFXInit(intakeFalcon);
        deploySolenoid.set(deployState);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        intakeFalcon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Deploy the intake system, or fold the arms if it is already deployed.
     */
    public void deploy() {
        deployState = !deployState;
        deploySolenoid.set(deployState);
    }

    /**
     * Start to intake balls.
     */
    public void launch() {
        intakeFalcon.set(ControlMode.PercentOutput, -Constants.Intake.intakeMotorOutput);
    }

}
