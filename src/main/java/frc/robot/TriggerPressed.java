package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A triggering event that is active when either trigger on xbox controller is pressed.
 */
public class TriggerPressed extends Trigger {

    private double axis;

    public TriggerPressed(XboxController controller, Hand hand) {
        this.axis = controller.getTriggerAxis(hand);
    }


    @Override
    public boolean get() {
        if (Math.abs(axis) >= Constants.Controller.triggerPressedThreshold) {
            return true;
        }
        return false;
    }
    
}
