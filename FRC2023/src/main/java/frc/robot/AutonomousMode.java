package frc.robot;

public class AutonomousMode implements IRobotMode {

    private IDrive drive;
    private double speed = 0.15;

    public AutonomousMode(IDrive drive) {
        this.drive = drive;
    }

    public void autoMove() {
        drive.driveDistance(52, -speed, 0, null);
        Debug.log("autoMove2");
    }

    @Override
    public void periodic() {

    }
}
