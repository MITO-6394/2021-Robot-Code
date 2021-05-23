/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    // public Solenoid solenoid = new Solenoid(6);
    // public DoubleSolenoid solenoid = new DoubleSolenoid(1, 2);
    private boolean state = false;

    /**
     * Creates a new ExampleSubsystem.
     */
    public ExampleSubsystem() {
        // solenoid.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void shiftSolenoid() {
        System.out.println("SHIFT SOLENOID!!!!!!!!!!!!!!!!!!!!");
        state = !state;
        // solenoid.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
}
