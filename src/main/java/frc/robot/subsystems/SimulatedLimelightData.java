package frc.robot.subsystems;

public class SimulatedLimelightData {
    public double xAngleToTag; // Angle to the target along the X-axis
    public double robotAngleInTagSpace; // Angle of the robot in the AprilTag's space
    public double distanceToTag; // Distance to the target
    public boolean targetVisible; // Whether the target is visible

    public SimulatedLimelightData(double xAngleToTag, double robotAngleInTagSpace, double distanceToTag, boolean targetVisible) {
        this.xAngleToTag = xAngleToTag;
        this.robotAngleInTagSpace = robotAngleInTagSpace;
        this.distanceToTag = distanceToTag;
        this.targetVisible = targetVisible;
    }
}