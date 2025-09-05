package org.firstinspires.ftc.teamcode.movement;

public class Twist {
    public double x, y, h;

    public Twist(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Twist mult(double scalar) {
        return new Twist(x * scalar, y * scalar, h * scalar);
    }

    public Pose exp() {
        // Convert twist to pose using exponential map
        // For small rotations, this is approximately just the twist values
        if (Math.abs(h) < 1e-6) {
            return new Pose(x, y, h);
        }

        // For larger rotations, use proper exponential map
        double s = Math.sin(h) / h;
        double c = (1.0 - Math.cos(h)) / h;

        double newX = s * x + c * y;
        double newY = -c * x + s * y;

        return new Pose(newX, newY, h);
    }

    @Override
    public String toString() {
        return String.format("Twist(%.2f, %.2f, %.2f°/s)", x, y, Math.toDegrees(h));
    }
}