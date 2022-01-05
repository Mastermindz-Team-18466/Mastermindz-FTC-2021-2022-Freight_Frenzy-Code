package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Concept: MecanumTeleOp", group = "Test")
//@Disabled

public class FieldOrientedDriveV2 {
    //The Stuff(variables)
    ElapsedTime runtime = new ElapsedTime();
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    Gamepad gamepad1;
    Vector vector;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    double offset = 0;

    public FieldOrientedDriveV2(DcMotor blm, DcMotor flm, DcMotor brm, DcMotor frm, BNO055IMU i, Gamepad g) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        this.imu = i;
        this.backLeftMotor = blm;
        this.backRightMotor = brm;
        this.frontRightMotor = frm;
        this.frontLeftMotor = flm;
        this.gamepad1 = g;

        imu.initialize(parameters);

        //left back motor
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Motor
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void move(double power, double ticks, double targetAngle) {
        //gets angle from imu
        //creates vector
        //vector = new Vector(15);
        //vector.setCartesian(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //vector.rotateDegrees(angles.firstAngle - offset);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);


        if(gamepad1.a) { // set the offset to the current angle when a is pressed (or any button you want) to make the current angle 0
            offset = angles.firstAngle;
        }

        double leftPower;
        double rightPower;

        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);

        while(backLeftMotor.getCurrentPosition() < ticks && frontLeftMotor.getCurrentPosition() < ticks) {
            if (angles.firstAngle < targetAngle) {
                rightPower = power + 0.5;
                leftPower = power - 0.5;
            } else if (angles.firstAngle > targetAngle) {
                rightPower = power - 0.5;
                leftPower = power + 0.5;
            } else {
                rightPower = power;
                leftPower = power;
            }
        }

        finish();
    }

    public void finish() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
