
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class Shooter extends SubsystemBase {

    private WPI_TalonFX lFlywheelFalcon = new WPI_TalonFX(Pin.Shooter.Motor.lFlywheelFalcon);
    private WPI_TalonFX rFlywheelFalcon = new WPI_TalonFX(Pin.Shooter.Motor.rFlywheelFalcon);

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
        Utility.configTalonSRXPID(rotateTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);
        Utility.configTalonSRXPID(elevateTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);

        lFlywheelFalcon.configVoltageCompSaturation(12); // "full output" will now scale to 11 Volts for all control
                                                         // modes when enabled.
        rFlywheelFalcon.configVoltageCompSaturation(12);
        lFlywheelFalcon.enableVoltageCompensation(true);
        rFlywheelFalcon.enableVoltageCompensation(true);

        lFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        rFlywheelFalcon.setNeutralMode(NeutralMode.Coast);

        rotateTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                0);
        rotateTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                0);

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        loadBallTalon.set(ControlMode.PercentOutput, 0);
        rotateTalon.set(ControlMode.PercentOutput, 0);
        elevateTalon.set(ControlMode.PercentOutput, 0);
        lFlywheelFalcon.set(TalonFXControlMode.Velocity,
                0.7 * Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        rFlywheelFalcon.set(TalonFXControlMode.Velocity,
                -0.7 * Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        elevate(0.08);
    }

    /**
     * Load balls into the shooter and shoot.
     * 
     * @param targetElevation The target elevation to reach before shooting.
     */
    public boolean shoot(double targetElevation) {

        lFlywheelFalcon.set(TalonFXControlMode.Velocity,
                Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        rFlywheelFalcon.set(TalonFXControlMode.Velocity,
                -Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);

        elevate(targetElevation);

        if ((lFlywheelFalcon.getSelectedSensorVelocity() >= (Constants.Shooter.flywheelTargetSpeed
                + Constants.Shooter.flywheelSpeedTolerance) * Constants.falconVelocityConstant)
                && (Math.abs(getElevation() - targetElevation) <= Constants.Shooter.elevationTolerance)) {
            loadBallTalon.set(ControlMode.PercentOutput, Constants.Shooter.loadBallTalonOutput);
            return true;
        }
        return false;

    }

    /**
     * Rotate the shooter.
     * 
     * @param velocity
     */
    public void rotate(double velocity) {
        rotateTalon.set(TalonSRXControlMode.Velocity, velocity * Constants.Shooter.aimConstant);
    }

    public void elevate(double targetElevation) {
        double elevationError = getElevation() - targetElevation;
        double elevationAdjustment;
        if (elevationError > 0.05) {
            elevationAdjustment = Constants.Shooter.elevateKp * elevationError - Constants.Shooter.elevateMinCommand;
        } else {
            elevationAdjustment = Constants.Shooter.elevateKp * elevationError + Constants.Shooter.elevateMinCommand;
        }
        elevateTalon.set(TalonSRXControlMode.Velocity, elevationAdjustment * Constants.Shooter.elevationConstant);
    }

    public double getElevation() {
        return elevationSensor.get();
    }

}