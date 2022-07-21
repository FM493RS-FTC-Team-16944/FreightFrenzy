package org.firstinspires.ftc.teamcode.models;

public class Twist2d {
    public double dx;
    public double dy;
    public double dtheta;


    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    @Override
    public String toString() {
        return String.format("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Twist2d) {
            return Math.abs(((Twist2d) obj).dx - dx) < 1E-9
                    && Math.abs(((Twist2d) obj).dy - dy) < 1E-9
                    && Math.abs(((Twist2d) obj).dtheta - dtheta) < 1E-9;
        }
        return false;
    }
}