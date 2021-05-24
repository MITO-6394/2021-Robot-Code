/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Intake extends SubsystemBase {

    private WPI_TalonFX intakeFalcon = new WPI_TalonFX(Pin.Intake.Motor.intakeFalcon);
    private DoubleSolenoid deploySolenoid = new DoubleSolenoid(Pin.Intake.Solenoid.deployForward, Pin.Intake.Solenoid.deployReverse);

    // Whether the intake system is deployed
    private boolean deployState = false;


    public Intake() {
        Utility.TalonFXInit(intakeFalcon);
        intakeFalcon.setInverted(false);
        deploySolenoid.set(Value.kReverse);
        // deploySolenoid.set(Value.kOff);
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
        deploySolenoid.set(deployState ? Value.kForward : Value.kReverse);
        deploySolenoid.set(Value.kOff);
        // deploySolenoid.toggle();
        // SmartDashboard.putBoolean("Intake deployed?", deployState);
    }

    /**
     * Start to intake balls.
     */
    public void launch() {
        intakeFalcon.set(ControlMode.PercentOutput, -Constants.intakeMotorOutput);
    }

    /**
     * Stop intaking.
     */
    public void stop() {
        intakeFalcon.set(ControlMode.PercentOutput, 0);
    }

    

}
