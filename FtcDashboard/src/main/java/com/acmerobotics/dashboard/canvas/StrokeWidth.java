package com.acmerobotics.dashboard.canvas;

public class StrokeWidth extends CanvasOp {
    private final int width;

    public StrokeWidth(int width) {
        super(Type.STROKE_WIDTH);

        this.width = width;
    }
}
