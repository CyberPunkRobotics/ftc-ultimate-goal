package org.firstinspires.ftc.teamcode.odometryNew;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMap;

public class RobotMovement {

    private RobotMap robot;
    private OdometryGlobalCoordinatePosition globalCoordinatePosition;

    public RobotMovement(RobotMap robot, OdometryGlobalCoordinatePosition globalCoordinatePosition) {
        this.robot = robot;
        this.globalCoordinatePosition = globalCoordinatePosition;
    }

    public void goToPosition(double x, double y, double prefferedAngle, double movementSpeed, double turnSpeed, int reverse) {

        double worldXPosition = globalCoordinatePosition.returnXCoordinate();
        double worldYPosition = globalCoordinatePosition.returnYCoordinate();
        double worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        while (distanceToTarget > 5 * movementSpeed) {
            worldXPosition = globalCoordinatePosition.returnXCoordinate();
            worldYPosition = globalCoordinatePosition.returnYCoordinate();
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            if (reverse == -1) {
                worldAngularOrientation -= Math.toRadians(180);
            }

            distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

            double relativeAngle = AngleWrap(absoluteAngleToTarget - worldAngularOrientation);
            double movementTurn = Range.clip(Range.clip(relativeAngle, -1, 1) * turnSpeed, -1, 1);

            if (distanceToTarget < 10) {
                movementTurn = 0;
            }

            double movementY = Range.clip(distanceToTarget, -1, 1) * movementSpeed * reverse;

            robot.teleOpDrive(movementY, movementTurn);


        }

        worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();
        while (Math.abs(AngleWrap(worldAngularOrientation - prefferedAngle)) > Math.toRadians(1)) {
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            double relativeAngleToPoint = AngleWrap(prefferedAngle - worldAngularOrientation);
            double movementTurn = Range.clip(relativeAngleToPoint, -1, 1) * turnSpeed;

            robot.teleOpDrive(0, movementTurn);
        }

    }

    private double AngleWrap(double angle) {

        while (angle < Math.PI) {
            angle += (2*Math.PI);
        }
        while (angle > Math.PI) {
            angle -= (2*Math.PI);
        }
        return angle;

    }

}
