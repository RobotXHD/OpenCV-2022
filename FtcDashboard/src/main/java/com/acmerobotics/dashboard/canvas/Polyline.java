package com.acmerobotics.dashboard.canvas;

public class Polyline extends CanvasOp {
    private final double[] xPoints;
    private final double[] yPoints;

    public Polyline(double[] xPoints, double[] yPoints) {
        super(Type.POLYLINE);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
    }
}
