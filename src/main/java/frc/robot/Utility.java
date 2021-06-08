/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Utility {

    // TalonFX init
    public static void TalonFXInit(TalonFX _talon) {
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Utility.PIDLoopIdx,
                Constants.Utility.timeoutMs);
        _talon.setSensorPhase(true);

        _talon.configNominalOutputForward(0, Constants.Utility.timeoutMs);
        _talon.configNominalOutputReverse(0, Constants.Utility.timeoutMs);
        _talon.configPeakOutputForward(1.0, Constants.Utility.timeoutMs);
        _talon.configPeakOutputReverse(-1.0, Constants.Utility.timeoutMs);
    }

    // TalonSRX init
    public static void TalonSRXInit(TalonSRX _talon) {
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.Utility.PIDLoopIdx,
                Constants.Utility.timeoutMs);
        _talon.setSensorPhase(true);

        _talon.configNominalOutputForward(0, Constants.Utility.timeoutMs);
        _talon.configNominalOutputReverse(0, Constants.Utility.timeoutMs);
        _talon.configPeakOutputForward(1, Constants.Utility.timeoutMs);
        _talon.configPeakOutputReverse(-1, Constants.Utility.timeoutMs);
    }

    // PID setting
    public static void configTalonSRXPID(TalonSRX _talon, double kF, double kP, double kI, double kD, int kIzone,
            double kRamp) {
        _talon.config_kF(Constants.Utility.PIDLoopIdx, kF, Constants.Utility.timeoutMs);
        _talon.config_kP(Constants.Utility.PIDLoopIdx, kP, Constants.Utility.timeoutMs);
        _talon.config_kI(Constants.Utility.PIDLoopIdx, kI, Constants.Utility.timeoutMs);
        _talon.config_kD(Constants.Utility.PIDLoopIdx, kD, Constants.Utility.timeoutMs);
        _talon.config_IntegralZone(Constants.Utility.PIDLoopIdx, kIzone, Constants.Utility.timeoutMs);
        _talon.configClosedloopRamp(kRamp, Constants.Utility.timeoutMs);
    }

    public static void configTalonFXPID(TalonFX _talon, double kF, double kP, double kI, double kD, double kRamp) {
        _talon.config_kF(Constants.Utility.PIDLoopIdx, kF, Constants.Utility.timeoutMs);
        _talon.config_kP(Constants.Utility.PIDLoopIdx, kP, Constants.Utility.timeoutMs);
        _talon.config_kI(Constants.Utility.PIDLoopIdx, kI, Constants.Utility.timeoutMs);
        _talon.config_kD(Constants.Utility.PIDLoopIdx, kD, Constants.Utility.timeoutMs);
        _talon.configClosedloopRamp(kRamp, Constants.Utility.timeoutMs);
    }

    // Motor safety
    public static void configMotorSafety(TalonSRX _talon) {
        _talon.enableCurrentLimit(true);
        _talon.configContinuousCurrentLimit(Constants.Utility.currentLimit, Constants.Utility.timeoutMs);
        _talon.configPeakCurrentLimit((int) (Constants.Utility.currentLimit * 1.5), Constants.Utility.timeoutMs);
        _talon.configPeakCurrentDuration(Constants.Utility.currentLimitDuration, Constants.Utility.timeoutMs);
    }

    public static double controllerAxisZeroCorrect(double axis) {
        if (axis <= Constants.Controller.axisZeroCorrectionRange
                && axis >= -Constants.Controller.axisZeroCorrectionRange) {
            return 0;
        }
        return axis;
    }

}
