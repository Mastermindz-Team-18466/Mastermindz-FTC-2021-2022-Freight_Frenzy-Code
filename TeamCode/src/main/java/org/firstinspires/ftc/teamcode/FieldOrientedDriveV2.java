package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class FieldOrientedDriveV2 {
    //The Stuff(variables)
    Hardware hardware = new Hardware();
    ElapsedTime runtime = new ElapsedTime();
    Vector vector;

    BNO055IMU imu;
    Orientation angles;

    double offset = 0;

    public FieldOrientedDriveV2() {
        hardware.init(hardware.hardwareMap);
    }

    public void move() {
        //gets angle from imu
        hardware.angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        //creates vector
        Vector vector = new Vector(15);
        vector.setCartesian(hardware.gamepad.left_stick_x, hardware.gamepad.left_stick_y);
        vector.rotateDegrees(hardware.angles.firstAngle - offset);


        if (hardware.gamepad.a) { // set the offset to the current angle when a is pressed (or any button you want) to make the current angle 0
            offset = hardware.angles.firstAngle;
        }

        double rx = hardware.gamepad.right_stick_x;

        double frontLeftPower = -vector.getY() + vector.getX() + rx;
        double backLeftPower = -vector.getY() - vector.getX() + rx;
        double frontRightPower = -vector.getY() - vector.getX() - rx;
        double backRightPower = -vector.getY() + vector.getX() - rx;

        if (Math.abs(frontLeftPower) > 1 ||
                Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 ||
                Math.abs(backRightPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        hardware.left_front_driver.setPower(-frontLeftPower);
        hardware.left_back_driver.setPower(-backLeftPower);
        hardware.right_front_driver.setPower(-frontRightPower);
        hardware.right_back_driver.setPower(-backRightPower);
    }

    public void finish() {
        hardware.left_front_driver.setPower(0);
        hardware.left_back_driver.setPower(0);
        hardware.right_front_driver.setPower(0);
        hardware.right_back_driver.setPower(0);
    }
}
