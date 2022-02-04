package com.acmerobotics.dashboard;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Container for information about the active op mode and its state.
 */
public class RobotStatus {

    /**
     * Status of an op mode.
     */
    public enum OpModeStatus {
        INIT,
        RUNNING,
        STOPPED
    }

    private final boolean available;
    private final String activeOpMode;
    private final OpModeStatus activeOpModeStatus;
    private final String warningMessage;
    private final String errorMessage;

    /**
     * Creates a status object with the default values.
     */
    public RobotStatus() {
        this.available = false;
        this.activeOpMode = "";
        this.activeOpModeStatus = OpModeStatus.STOPPED;
        this.warningMessage = "";
        this.errorMessage = "";
    }

    /**
     * Creates a status object from the active op mode and it status.
     * @param activeOpMode active op mode
     * @param activeOpModeStatus active op mode status
     */
    public RobotStatus(String activeOpMode, OpModeStatus activeOpModeStatus) {
        this.available = true;
        this.activeOpMode = activeOpMode;
        this.activeOpModeStatus = activeOpModeStatus;
        this.warningMessage = RobotLog.getGlobalWarningMessage().message;
        this.errorMessage = RobotLog.getGlobalErrorMsg();
    }
}
