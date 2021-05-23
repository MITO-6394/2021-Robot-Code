
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Shooter extends SubsystemBase {

    private WPI_TalonFX lFlywheelFalcon = new WPI_TalonFX(Pin.Shooter.Motor.lFlywheelFalcon);
    private WPI_TalonFX rFlywheelFalcon = new WPI_TalonFX(Pin.Shooter.Motor.rFlywheelFalcon);
    private TalonFXSensorCollection testMotorSensorCollection = lFlywheelFalcon.getSensorCollection();

    private TalonSRX loadBallTalon = new TalonSRX(Pin.Shooter.Motor.loadBallTalon);
    private TalonSRX rotateTalon = new TalonSRX(Pin.Shooter.Motor.rotateTalon);
    private TalonSRX elevateTalon = new TalonSRX(Pin.Shooter.Motor.elevateTalon);

    private AnalogPotentiometer elevationSensor = new AnalogPotentiometer(Pin.Shooter.Sensor.elevation);

    public Shooter() {
        Utility.TalonFXInit(lFlywheelFalcon);
        Utility.TalonFXInit(rFlywheelFalcon);
        lFlywheelFalcon.setInverted(false);
        rFlywheelFalcon.setInverted(false);

        Utility.configTalonFXPID(lFlywheelFalcon, 0.1097, 0.22, 0, 0, 0);
        Utility.configTalonFXPID(rFlywheelFalcon, 0.1097, 0.22, 0, 0, 0);

        // lFlywheelFalcon.configVoltageCompSaturation(12); // "full output" will now scale to 11 Volts for all control
        //                                                      // modes when enabled.
        // rFlywheelFalcon.configVoltageCompSaturation(12);
        // lFlywheelFalcon.enableVoltageCompensation(true);
        // rFlywheelFalcon.enableVoltageCompensation(true);

    }

    public void TestMotor(double Speed) {
        Speed = (Speed - 1) / 2;
        lFlywheelFalcon.set(ControlMode.PercentOutput, Speed);
        SmartDashboard.putNumber("Shooter motor velocity",
                -testMotorSensorCollection.getIntegratedSensorVelocity() / 204.8 * 60);
        SmartDashboard.putNumber("Shooter Control Value", -Speed);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        lFlywheelFalcon.set(TalonFXControlMode.Velocity, 0.8*Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        rFlywheelFalcon.set(TalonFXControlMode.Velocity, -0.8*Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        
        SmartDashboard.putNumber("Shooter elevation", getElevation());
    }

    // // Accelerate the flywheel to the speed ready to shoot balls
    // public void shoot() {
    //     lFlywheelFalcon.set(ControlMode.Velocity, Constants.flywheelTargetSpeed);
    // }

    /**
     * Load balls into the shooter and shoot.
     */
    public void shoot() {
        // if (axis >= 0.75) {
            lFlywheelFalcon.set(TalonFXControlMode.Velocity, Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
            rFlywheelFalcon.set(TalonFXControlMode.Velocity, -Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
            loadBallTalon.set(ControlMode.PercentOutput, Constants.loadBallTalonOutput);
        // }
        // else {
        //     stop();
        // }
    }

    /**
     * Stop the ball loader.
     */
    public void stop() {
        loadBallTalon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Rotate the shooter.
     * @param velocity
     */
    public void rotate(double velocity) {
        rotateTalon.set(ControlMode.Velocity, velocity);
    }

    /**
     * Change the elevation of shooter.
     * @param elevation The target elevation, in degree.
     */
    public void elevate(double elevation) {
        elevateTalon.set(ControlMode.Position, 0);
    }

    public double getElevation() {
        return elevationSensor.get();
    }

}