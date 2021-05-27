
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
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
        // Utility.configTalonSRXPID(rotateTalon, 0.1097, 0.22, 0, 0, 0, 0);
        // Utility.configTalonSRXPID(rotateTalon, 0.0, 0.4, 0.0002, 40, 150, 0.5);
        Utility.configTalonSRXPID(elevateTalon , 0.0, 0.4, 0.0002, 40, 150, 0.5);


        lFlywheelFalcon.configVoltageCompSaturation(12); // "full output" will now scale to 11 Volts for all control
                                                             // modes when enabled.
        rFlywheelFalcon.configVoltageCompSaturation(12);
        lFlywheelFalcon.enableVoltageCompensation(true);
        rFlywheelFalcon.enableVoltageCompensation(true);

        lFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        rFlywheelFalcon.setNeutralMode(NeutralMode.Coast);

        // rotateTalon.overrideLimitSwitchesEnable(true);
        // rotateTalon.overrideLimitSwitchesEnable(false);
        rotateTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, rotateTalon.getDeviceID());
        rotateTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, rotateTalon.getDeviceID());

        // rotateTalon.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, rotateTalon.getDeviceID());
        // rotateTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, rotateTalon.getDeviceID());

        // rotateTalon.configLimitSwitchDisableNeutralOnLOS(limitSwitchDisableNeutralOnLOS, timeoutMs)

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter elevation", getElevation());
        SmartDashboard.putNumber("rotate v", rotateTalon.getMotorOutputVoltage());
        SmartDashboard.putNumber("limit switch", rotateTalon.isFwdLimitSwitchClosed());
        SmartDashboard.putNumber("rev limit switch", rotateTalon.isRevLimitSwitchClosed());
    }

    /**
     * Load balls into the shooter and shoot.
     */
    public void shoot() {
        lFlywheelFalcon.set(TalonFXControlMode.Velocity, Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        rFlywheelFalcon.set(TalonFXControlMode.Velocity, -Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        if (lFlywheelFalcon.getSelectedSensorVelocity() >= (Constants.Shooter.flywheelTargetSpeed - Constants.Shooter.flywheelSpeedTolerance) * Constants.falconVelocityConstant) {
            loadBallTalon.set(ControlMode.PercentOutput, Constants.loadBallTalonOutput);
        }
    }

    /**
     * Stop all triggering motors and return to default state.
     */
    public void stop() {
        loadBallTalon.set(ControlMode.PercentOutput, 0);
        rotateTalon.set(ControlMode.PercentOutput, 0);
        elevateTalon.set(ControlMode.PercentOutput, 0);
        // lFlywheelFalcon.set(TalonFXControlMode.Velocity, 0.4*Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
        // rFlywheelFalcon.set(TalonFXControlMode.Velocity, -0.4*Constants.Shooter.flywheelTargetSpeed * Constants.falconVelocityConstant);
    }

    /**
     * Rotate the shooter.
     * @param velocity
     */
    public void rotate(double velocity) {
        // rotateTalon.set(TalonSRXControlMode.Velocity, velocity*Constants.Shooter.aimConstant);
        if (velocity > 0) {
            rotateTalon.set(TalonSRXControlMode.PercentOutput, 0.2);
        }
        else if (velocity < 0) {
            rotateTalon.set(TalonSRXControlMode.PercentOutput, -0.2);
        }
        System.out.println("rotating");
    }

    /**
     * Change the elevation of shooter.
     * @param elevation The target elevation, in degree.
     */
    public void elevate(double targetElevation) {
        double elevationError = getElevation() - targetElevation;
        double kp = 0.1;
        double elevationAdjustment = kp * elevationError;
        elevateTalon.set(TalonSRXControlMode.Velocity, elevationAdjustment);
    }

    public double getElevation() {
        return elevationSensor.get();
    }

}