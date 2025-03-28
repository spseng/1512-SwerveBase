package frc.robot.Utils;

import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.ReefDirection;

public class RobotState {
    private static RobotState instance = new RobotState();
    private CoralLevel scoringCoralLevel = CoralLevel.L1; 
    private ReefDirection scoringReefDirection = ReefDirection.LEFT;

    private RobotState() {}

    public static RobotState getInstance() {
        return instance;
    }

    public void setScoringCoralLevel(CoralLevel target) {
        scoringCoralLevel = target;
    }

    public CoralLevel getScoringCoralLevel() {
        return scoringCoralLevel;
    }

    public void setScoringReefDirection(ReefDirection target) {
        scoringReefDirection = target;
    }

    public ReefDirection getScoringReefDirection() {
        return scoringReefDirection;
    }
}
