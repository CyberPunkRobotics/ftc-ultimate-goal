package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class GlobalCoordinateSystemTutorial implements Runnable {

    DcMotor leftEncoder, rightEncoder;
    DcMotor middleEncoder;

    boolean isRunning = true;

    double leftEncoderPosition, rightEncoderPosition, middleEncoderPosition;
    double changeInOrientation;
    double OLDLeftEncoderPosition, OLDRightEncoderPosition, OLDMiddleEncoderPosition;

    double globalX, globalY, robotOrientation;

    double encoderWheelDistance;
    double middleEncoderTickOffset;

    int sleepTime;

    File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    public GlobalCoordinateSystemTutorial(DcMotor leftEncoder, DcMotor rightEncoder, DcMotor middleEncoder, double TICKS_PER_INCH, int threadSleepDelay) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.middleEncoder = middleEncoder;
        sleepTime = threadSleepDelay;

//        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(sideWheelSeparationFile).trim()) * TICKS_PER_INCH;
        encoderWheelDistance = 31 * 2.54 * TICKS_PER_INCH;
//        middleEncoderTickOffset = Double.parseDouble(ReadWriteFile.readFile(middleTickOffsetFile).trim());
        middleEncoderTickOffset = 4 * 2.54;
    }

    public void updatePositionTest() {
        leftEncoderPosition = leftEncoder.getCurrentPosition();
        rightEncoderPosition = rightEncoder.getCurrentPosition();
        middleEncoderPosition = middleEncoder.getCurrentPosition();

        double leftChange = leftEncoderPosition - OLDLeftEncoderPosition;
        double rightChange = rightEncoderPosition - OLDRightEncoderPosition;
        double middleEncoderChange = middleEncoderPosition - OLDMiddleEncoderPosition;
        double middleChange = (leftChange + rightChange) / 2;//dm

        double theta = (rightChange - leftChange) / encoderWheelDistance;


        if (theta != 0) {
            globalX = globalX + ((middleChange/theta) * Math.tan(theta)) * Math.cos(robotOrientation + theta/2) + middleEncoderChange * Math.cos(robotOrientation + Math.PI/2 + theta/2);
            globalY = globalY + ((middleChange/theta) * Math.tan(theta)) * Math.sin(robotOrientation + theta/2) + middleEncoderChange * Math.sin(robotOrientation + Math.PI/2 + theta/2);
            robotOrientation += theta;
        }

        OLDLeftEncoderPosition = leftEncoderPosition;
        OLDRightEncoderPosition = rightEncoderPosition;
        OLDMiddleEncoderPosition = middleEncoderPosition;
    }

    public void positionUpdate() {
        leftEncoderPosition = leftEncoder.getCurrentPosition();
        rightEncoderPosition = rightEncoder.getCurrentPosition();

        double leftChange = leftEncoderPosition - OLDLeftEncoderPosition;
        double rightChange = rightEncoderPosition - OLDRightEncoderPosition;

        changeInOrientation = (leftChange - rightChange) / encoderWheelDistance;
        robotOrientation += changeInOrientation;

        middleEncoderPosition = middleEncoder.getCurrentPosition();
        double rawHorizontalChange = middleEncoderPosition - OLDMiddleEncoderPosition;
        double horizontalChange = rawHorizontalChange - (changeInOrientation * middleEncoderTickOffset);

        double sides = (rightChange + leftChange) / 2;
        double frontBack =  horizontalChange;

        globalX = sides * Math.sin(robotOrientation) + frontBack * Math.cos(robotOrientation);
        globalY = sides * Math.cos(robotOrientation) + frontBack * Math.sin(robotOrientation);

        OLDLeftEncoderPosition = leftEncoderPosition;
        OLDRightEncoderPosition = rightEncoderPosition;
        OLDMiddleEncoderPosition = middleEncoderPosition;
    }

    public double returnXCoordinate() {
        return globalX;
    }

    public double returnYCoordinate() {
        return globalY;
    }

    public double returnOrientation() {
        return Math.toDegrees(robotOrientation) % 360;
    }

    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            updatePositionTest();
        }

        try {
            Thread.sleep(sleepTime);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
