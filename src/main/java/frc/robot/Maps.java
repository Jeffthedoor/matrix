package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * The general Hash map for all paths. Has most calls needed for the paths to run.
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @param leds
     * @return
     */
    public static HashMap<String, Command> getPathMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        return eventMap;
    }
}
