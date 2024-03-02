package frc.robot.subsystems;

import java.util.List;

public class Barrier {
    private double x;
    private double y;
    private double width;
    private double height;

    public Barrier(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public boolean isPointNear(double x, double y) {
        return x >= this.x && x <= this.x + this.width && y >= this.y && y <= this.y + this.height;
    }
}