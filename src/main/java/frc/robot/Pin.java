/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Pin {

    public static final class Drivetrain {

        public static final class Motor {

            public static final int lFalconMaster = 1;
            public static final int lFalconSlave = 2;
            public static final int rFalconMaster = 3;
            public static final int rFalconSlave = 4;

        }

        public static final class Solenoid {

            public static final int shiftGearboxForward = 1;
            public static final int shiftGearboxReverse = 2;

        }
    }

    public static final class Intake {

        public static final class Motor {

            public static final int intakeFalcon = 13;

        }

        public static final class Solenoid {

            public static final int deploy = 3;

        }

    }

    public static final class Shooter {

        public static final class Motor {

            public static final int lFlywheelFalcon = 10;
            public static final int rFlywheelFalcon = 11;
            public static final int loadBallTalon = 8;
            public static final int rotateTalon = 9;
            public static final int elevateTalon = 12;

        }

        public static final class Solenoid {

        }

        public static final class Sensor {

            public static final int elevation = 0;

        }

    }

    public static final class Climber {

        public static final class Motor {

            public static final int lTelescopeTalon = 5;
            public static final int rTelescopeTalon = 6;

        }

        public static final class Solenoid {

            public static final int holder = 0;

        }

    }

    public static final class Drum {

        public static final class Motor {

            public static final int rotateTalon = 7;

        }

    }

    public static class Test {

        public static final class Motor {

            public static final int Master = 5;
            public static final int Slave = 6;

        }

        public static final class Solenoid {

        }

        public static final class Sensor {

        }

    }

    public static final class Controller {

        public static final int moveStick = 0;
        // public static final int functionStick = 1;

        public static final class Axis {

            public static final int forward = 1;
            public static final int rotation = 4;
            public static final int intake = 2;
            public static final int shoot = 3;

        }

        public static final class Button {

            public static final int shiftGearbox = XboxController.Button.kB.value;
            public static final int deployIntake = XboxController.Button.kY.value;
            public static final int aim = XboxController.Button.kX.value;
            public static final int intakeBall = XboxController.Button.kStickLeft.value;
            public static final int shootBall = XboxController.Button.kStickRight.value;
            public static final int releaseClimber = XboxController.Button.kA.value;
            public static final int stretchClimber = XboxController.Button.kStart.value;
            public static final int telescopeClimber = XboxController.Button.kBack.value;
            public static final int rotateShooterLeftward = XboxController.Button.kBumperLeft.value;
            public static final int rotateShooterRightward = XboxController.Button.kBumperRight.value;

        }

    }

}
