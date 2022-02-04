package com.acmerobotics.dashboard.canvas;

@SuppressWarnings({"checkstyle:MultipleVariableDeclarations", "checkstyle:EmptyLineSeparator"})
public class Spline extends CanvasOp {
    private final double ax;
    private final double bx;
    private final double cx;
    private final double dx;
    private final double ex;
    private final double fx;
    private final double ay;
    private final double by;
    private final double cy;
    private final double dy;
    private final double ey;
    private final double fy;

    public Spline(double ax, double bx, double cx, double dx, double ex, double fx,
                  double ay, double by, double cy, double dy, double ey, double fy) {
        super(Type.SPLINE);

        this.ax = ax;
        this.bx = bx;
        this.cx = cx;
        this.dx = dx;
        this.ex = ex;
        this.fx = fx;

        this.ay = ay;
        this.by = by;
        this.cy = cy;
        this.dy = dy;
        this.ey = ey;
        this.fy = fy;
    }
}
