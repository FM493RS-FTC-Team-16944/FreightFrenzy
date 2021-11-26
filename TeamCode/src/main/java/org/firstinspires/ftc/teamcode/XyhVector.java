package org.firstinspires.ftc.teamcode;

public class XyhVector {
    public double x;
    public double y;
    public double h;
    XyhVector(double x, double y, double h){
        this.y = y;
        this.x = x;
        this.h = h;
    }
    XyhVector(XyhVector vector){
        this.x = vector.x;
        this.y = vector.y;
        this.h = vector.h;
    }
}
