package org.firstinspires.ftc.teamcode.fsm.movement;
public interface Localizer {
    Pose pos(double t);
    Twist vel(double t);
    void update(double t);
    void setPose(Pose p);
}