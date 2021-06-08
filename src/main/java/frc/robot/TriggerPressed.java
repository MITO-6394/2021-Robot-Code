package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A triggering event that is active when either trigger on xbox controller is
 * pressed.
 */
public class TriggerPressed extends Trigger {

    private XboxController controller;
    private Hand hand;

    public TriggerPressed(XboxController controller, Hand hand) {
        this.controller = controller;
        this.hand = hand;
    }

    @Override
    public boolean get() {
        double axis = controller.getTriggerAxis(hand);
        // When the axis reading is larger than the given threshold, consider the
        // trigger as pressed
        if (Math.abs(axis) >= Constants.Controller.triggerPressedThreshold) {
            return true;
        }
        return false;
    }

}
