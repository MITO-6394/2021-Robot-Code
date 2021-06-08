/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean isTargetFound() {
        double tv = table.getEntry("tv").getDouble(0);
        if (tv < 1.0) {
            return false;
        } else {
            return true;
        }
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTa() {
        return table.getEntry("ta").getDouble(0);
    }

    public double getEstimatedDistance(double elevation) {
        // The height of the center of the Reflective Material on the POWER PORT (the
        // limelight's crosshair), in meters
        double crosshairHeight = 2.49 - 0.76 / 4;
        // The height of limelight camera on our robot, in meters
        double limelightHeight = 0.57;
        return (crosshairHeight - limelightHeight) / Math.tan(Math.toRadians(elevation + getTy()));
    }

}
