package org.firstinspires.ftc.teamcode.models;

public class XyhVector {
    public double x;
    public double y;
    public double h;

    public XyhVector(double x, double y, double h) {
        this.y = y;
        this.x = x;
        this.h = h;
    }

    public XyhVector(XyhVector vector) {
        this.x = vector.x;
        this.y = vector.y;
        this.h = vector.h;
    }

    public XyhVector() {
        this.x = 0;
        this.y = 0;
        this.h = 0;
    }
}
