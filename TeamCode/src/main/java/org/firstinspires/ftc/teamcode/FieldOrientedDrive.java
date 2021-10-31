package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Concept: FieldCentricMecanumTeleOp", group = "Test")
//@Disabled
public class FieldOrientedDrive extends LinearOpMode {
    //The Stuff(variables)
    ElapsedTime runtime = new ElapsedTime();
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontRightMotor;

    BNO055IMU imu;
    Orientation angles;

    double offset = 0;

    @Override
    public void runOpMode() {
        //Set Up The Hardware
        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //left back motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack_drive");
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack_drive");
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront_drive");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront_drive");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //LETS GO!!!!!
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //gets angle from imu
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            //creates vector
            Vector vector = new Vector();
            vector.setCartesian(gamepad1.left_stick_x, gamepad1.left_stick_y);
            vector.rotateDegrees(angles.firstAngle - offset);


            if(gamepad1.a) { // set the offset to the current angle when a is pressed (or any button you want) to make the current angle 0
                offset = angles.firstAngle;
            }

            double rx = gamepad1.right_stick_x;

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

    }
}