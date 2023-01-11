// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SRFSwerveBlock {
    private int signature; 
    private int xCenter;
    private int yCenter;
    private int width;
    private int height;

    public SRFSwerveBlock() {
        signature = -1;
        yCenter = -1;
        xCenter = -1;
        width = -1;
        height = -1;
    }

    public SRFSwerveBlock(int sig, int centerY, int centerX, int w, int h) {
        signature = sig;
        yCenter = centerY;
        xCenter = centerX;
        width = w;
        height = h;
    }

    public int getSignature() {
        return signature;
    }

    public int getY() {
        return yCenter;
    }

    public int getX() {
        return xCenter;
    }

    public int getHeight() {
        return height;
    }

    public int getWidth() {
        return width;
    }

    public void set(int sig, int centerY, int centerX, int w, int h) { 
        signature = sig;
        yCenter = centerY;
        xCenter = centerX;
        width = w;
        height = h;
    }
}
