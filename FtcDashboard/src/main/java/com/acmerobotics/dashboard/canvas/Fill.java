package com.acmerobotics.dashboard.canvas;

public class Fill extends CanvasOp {
    private final String color;

    public Fill(String color) {
        super(Type.FILL);

        this.color = color;
    }
}
