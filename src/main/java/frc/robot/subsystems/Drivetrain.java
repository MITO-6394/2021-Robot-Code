/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonFX rFalconMaster = new WPI_TalonFX(Pin.Drivetrain.Motor.rFalconMaster);
    private final WPI_TalonFX rFalconSlave = new WPI_TalonFX(Pin.Drivetrain.Motor.rFalconSlave);
    // private SpeedControllerGroup rControllerGroup = new
    // SpeedControllerGroup(rFalconMaster, rFalconSlave);
    // private SpeedControllerGroup lControllerGroup = new
    // SpeedControllerGroup(lFalconMaster, lFalconSlave);

    private final WPI_TalonFX lFalconMaster = new WPI_TalonFX(Pin.Drivetrain.Motor.lFalconMaster);
    private final WPI_TalonFX lFalconSlave = new WPI_TalonFX(Pin.Drivetrain.Motor.lFalconSlave);
    // private final DifferentialDrive m_drive = new DifferentialDrive(lFalconMaster,
    // rFalconMaster);
    private final TalonFXSensorCollection rFalconMasterSensorCollection = rFalconMaster.getSensorCollection();
    private final TalonFXSensorCollection lFalconMasterSensorCollection = lFalconMaster.getSensorCollection();
    public DoubleSolenoid shiftGearboxSolenoid = new DoubleSolenoid(Pin.Drivetrain.Solenoid.shiftGearboxForward, Pin.Drivetrain.Solenoid.shiftGearboxReverse);

    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private Boolean gearboxState = false;
    // private int gearState = 0;

    // private final SpeedControllerGroup test = new SpeedControllerGroup(new
    // WPI_TalonFX(Pin.lFalconMaster),
    // new WPI_TalonFX(Pin.lFalconSlave));

    // private final DifferentialDrive m_drive = new DifferentialDrive(lFalconMaster,
    // rFalconMaster);

    public Drivetrain() {
        Utility.TalonFXInit(rFalconMaster);
        Utility.TalonFXInit(lFalconMaster);
        Utility.TalonFXInit(rFalconSlave);
        Utility.TalonFXInit(lFalconSlave);

        // Utility.TalonSRXInit(rFalconMaster);
        // Utility.TalonSRXInit(lFalconMaster);

        // rFalconSlave.follow(rFalconMaster);
        // lFalconSlave.follow(lFalconMaster);

        lFalconMaster.setInverted(true);
        lFalconSlave.setInverted(true);
        rFalconMaster.setInverted(false);
        rFalconSlave.setInverted(false);

        Utility.configTalonFXPID(lFalconMaster, 0.1097, 0.22, 0, 0, 0);
        Utility.configTalonFXPID(rFalconMaster, 0.1097, 0.22, 0, 0, 0);
        Utility.configTalonFXPID(lFalconSlave, 0.1097, 0.22, 0.0, 0.0, 0);
        Utility.configTalonFXPID(rFalconSlave, 0.1097, 0.22, 0.0, 0.0, 0);

        ahrs.reset();

    }

    // public void velocityDrive(double forward, final double rot) {
        // if (forward <= 0.15 && forward >= -0.15) {
        //     forward = 0;
        // }
        // double rSpeed, lSpeed;
        // if (forward <= 0) {
        //     lSpeed = (forward - 0.4 * (rot + forward * 0.7)) * Constants.drivetrainTargetRPM
        //             * Constants.velocityConstantsFalcon;
        //     rSpeed = (forward + 0.4 * (rot - forward * 0.7)) * Constants.drivetrainTargetRPM
        //             * Constants.velocityConstantsFalcon;
        // } else {
        //     lSpeed = (forward + 0.4 * (rot - forward * 0.7)) * Constants.drivetrainTargetRPM
        //             * Constants.velocityConstantsFalcon;
        //     rSpeed = (forward - 0.4 * (rot + forward * 0.7)) * Constants.drivetrainTargetRPM
        //             * Constants.velocityConstantsFalcon;
        // }

        // // double lSpeed = (forward + Math.abs(turn)) *
        // // Constants.velocityConstantsFalcon * 0.2;
        // // double rSpeed = (forward - Math.abs(turn)) *
        // // Constants.velocityConstantsFalcon * 0.2;
        // lFalconMaster.set(ControlMode.PercentOutput, lSpeed * 0.00036);
        // rFalconMaster.set(ControlMode.PercentOutput, rSpeed * 0.00036);

        // SmartDashboard.putNumber("Right motor velocity Low", -rFalconMasterSensorCollection.getIntegratedSensorVelocity()
        //         / 204.8 * 60 / 12.85 * Math.PI * (0.152 / 2) * (0.152 / 2));
        // SmartDashboard.putNumber("Right motor velocity High",
        //         -rFalconMasterSensorCollection.getIntegratedSensorVelocity() / 204.8 * 60 / 4.4 * Math.PI * (0.152 / 2)
        //                 * (0.152 / 2));
        // SmartDashboard.putNumber("Left motor velocity High", -lFalconMasterSensorCollection.getIntegratedSensorVelocity()
        //         / 204.8 * 60 / 4.4 * Math.PI * (0.152 / 2) * (0.152 / 2));
        // SmartDashboard.putNumber("Left motor velocity Low", -lFalconMasterSensorCollection.getIntegratedSensorVelocity()
        //         / 204.8 * 60 / 12.85 * Math.PI * (0.152 / 2) * (0.152 / 2));
        // // SmartDashboard.putNumber("output", (lSpeed+rSpeed)/2);
        // SmartDashboard.putNumber("forward", forward);
        // SmartDashboard.putNumber("rot", rot);
        // SmartDashboard.putBoolean("High speed mode?", gearboxState);

    // }

    public void velocityDrive(double forward, double rotation) {
        double lSpeed = forward;
        double rSpeed = forward;

        double rotationGain = Math.copySign(rotation * Constants.rotationCoeff, forward);

        lSpeed += (rotation >= 0 ? rotationGain : -rotationGain);
        rSpeed += (rotation >= 0 ? -rotationGain : rotationGain);

        lSpeed *= Constants.falconVelocityConstant * Constants.drivetrainTargetRPM;
        rSpeed *= Constants.falconVelocityConstant * Constants.drivetrainTargetRPM;

        lFalconMaster.set(TalonFXControlMode.Velocity, lSpeed);
        lFalconSlave.set(TalonFXControlMode.Velocity, lSpeed);
        rFalconMaster.set(TalonFXControlMode.Velocity, rSpeed);
        rFalconSlave.set(TalonFXControlMode.Velocity, rSpeed);

        SmartDashboard.putBoolean("High speed mode?", gearboxState);

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

    /**
     * rotate use pid to turn
     */
    public double getCurrentAngle() {
        return Math.IEEEremainder(ahrs.getAngle(), 360);
    }

}
