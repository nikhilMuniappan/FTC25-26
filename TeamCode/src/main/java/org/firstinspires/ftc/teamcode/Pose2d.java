package org.firstinspires.ftc.teamcode;

public class Pose2d {
    private final double x;
    private final double y;
    private final double heading; //radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    /**
     * Adds a delta pose to the current pose. Used for odometry integration.
     */
    public Pose2d plus(Pose2d delta) {
        return new Pose2d(
                this.x + delta.x,
                this.y + delta.y,
                this.heading + delta.heading
        );
    }
}
