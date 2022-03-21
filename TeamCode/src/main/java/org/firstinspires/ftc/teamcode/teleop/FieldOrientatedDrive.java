package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Vector;


public class FieldOrientatedDrive {
    //The Stuff(variables)
    ElapsedTime runtime = new ElapsedTime();
    Vector vector;

    BNO055IMU imu;
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    Orientation angles;

    Gamepad gamepad;

    double offset = 0;

    public FieldOrientatedDrive(Gamepad gamepad, HardwareMap hardwareMap) {
        //Set Up The Hardware
        this.gamepad = gamepad;

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //left back motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //LETS GO!!!!!
        runtime.reset();
    }

    public void move() {
        //gets angle from imu
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        //creates vector
        Vector vector = new Vector(15);
        vector.setCartesian(gamepad.left_stick_x, gamepad.left_stick_y);
        vector.rotateDegrees(angles.firstAngle - offset);


        if (gamepad.a) { // set the offset to the current angle when a is pressed (or any button you want) to make the current angle 0
            offset = angles.firstAngle;
        }

        double rx = gamepad.right_stick_x;

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

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);
    }

    public void finish() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
