package org.firstinspires.ftc.teamcode.movement;

public class Pose {
    public double x, y, h;

    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Pose plus(Pose other) {
        return new Pose(x + other.x, y + other.y, h + other.h);
    }

    public Pose minus(Pose other) {
        return new Pose(x - other.x, y - other.y, h - other.h);
    }

    public Pose times(double scalar) {
        return new Pose(x * scalar, y * scalar, h * scalar);
    }

    public double norm() {
        return Math.sqrt(x * x + y * y + h * h);
    }

    @Override
    public String toString() {
        return String.format("Pose(%.2f, %.2f, %.2f°)", x, y, Math.toDegrees(h));
    }
}