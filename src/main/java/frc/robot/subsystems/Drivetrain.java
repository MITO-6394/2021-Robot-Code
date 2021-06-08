/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonFX rFalconMaster = new WPI_TalonFX(Pin.Drivetrain.Motor.rFalconMaster);
    private final WPI_TalonFX rFalconSlave = new WPI_TalonFX(Pin.Drivetrain.Motor.rFalconSlave);

    private final WPI_TalonFX lFalconMaster = new WPI_TalonFX(Pin.Drivetrain.Motor.lFalconMaster);
    private final WPI_TalonFX lFalconSlave = new WPI_TalonFX(Pin.Drivetrain.Motor.lFalconSlave);

    public DoubleSolenoid shiftGearboxSolenoid = new DoubleSolenoid(Pin.Drivetrain.Solenoid.shiftGearboxForward,
            Pin.Drivetrain.Solenoid.shiftGearboxReverse);

    private Boolean gearboxState = false;

    public Drivetrain() {

        Utility.TalonFXInit(lFalconMaster);
        Utility.TalonFXInit(lFalconSlave);
        Utility.TalonFXInit(rFalconMaster);
        Utility.TalonFXInit(rFalconSlave);

        lFalconMaster.setNeutralMode(NeutralMode.Brake);
        lFalconSlave.setNeutralMode(NeutralMode.Brake);
        rFalconMaster.setNeutralMode(NeutralMode.Brake);
        rFalconSlave.setNeutralMode(NeutralMode.Brake);

        lFalconMaster.setInverted(true);
        lFalconSlave.setInverted(true);
        rFalconMaster.setInverted(false);
        rFalconSlave.setInverted(false);

        Utility.configTalonFXPID(lFalconMaster, 0.1097, 0.22, 0, 0, 1.2);
        Utility.configTalonFXPID(rFalconMaster, 0.1097, 0.22, 0, 0, 1.2);
        Utility.configTalonFXPID(lFalconSlave, 0.1097, 0.22, 0, 0, 1.2);
        Utility.configTalonFXPID(rFalconSlave, 0.1097, 0.22, 0, 0, 1.2);

    }

    @Override
    public void periodic() {
    }

    public void velocityDrive(double forward, double rotation) {

        double lSpeed = forward;
        double rSpeed = forward;

        double rotationGain;
        if (forward != 0) {
            rotationGain = Math.copySign(rotation * Constants.Drivetrain.rotationCoeff, forward);
        } else {
            rotationGain = rotation * Constants.Drivetrain.rotationCoeff;
        }

        lSpeed += (rotation >= 0 ? -rotationGain : rotationGain);
        rSpeed += (rotation >= 0 ? rotationGain : -rotationGain);

        lSpeed *= Constants.falconVelocityConstant * Constants.Drivetrain.drivetrainTargetRPM;
        rSpeed *= Constants.falconVelocityConstant * Constants.Drivetrain.drivetrainTargetRPM;

        lFalconMaster.set(TalonFXControlMode.Velocity, lSpeed);
        lFalconSlave.set(TalonFXControlMode.Velocity, lSpeed);
        rFalconMaster.set(TalonFXControlMode.Velocity, rSpeed);
        rFalconSlave.set(TalonFXControlMode.Velocity, rSpeed);

    }

    /**
     * Change to another gear ratio through solenoids
     */
    public void shiftGearbox() {
        gearboxState = !gearboxState;
        shiftGearboxSolenoid.set(gearboxState ? Value.kForward : Value.kReverse);
    }

    public Boolean getGearboxState() {
        return gearboxState;
    }

}
