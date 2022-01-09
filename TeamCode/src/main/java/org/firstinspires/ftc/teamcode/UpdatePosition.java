package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class UpdatePosition extends LinearOpMode {
    Hardware hardware = new Hardware();
    static final double TICKS_PER_REV = 8192;
    static final double WHEEL_DIAMETER = 100 / 25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    double offset = 0;
    BNO055IMU imu;
    Orientation angles;

    GlobalCoordinateSystem positionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);

        resetEncoders();

        waitForStart();

        positionUpdate = new GlobalCoordinateSystem(TICKS_PER_INCH, 100);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            //creates vector
            Vector vector = new Vector(15);
            vector.setCartesian(gamepad1.left_stick_x, gamepad1.left_stick_y);
            vector.rotateDegrees(angles.firstAngle - offset);


            if (gamepad1.a) { // set the offset to the current angle when a is pressed (or any button you want) to make the current angle 0
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

            hardware.left_front_driver.setPower(-frontLeftPower);
            hardware.left_back_driver.setPower(-backLeftPower);
            hardware.right_front_driver.setPower(-frontRightPower);
            hardware.right_back_driver.setPower(-backRightPower);

            telemetry.addData("[X Position]", positionUpdate.returnXCoordinate() / TICKS_PER_INCH);
            telemetry.addData("[Y Position]", positionUpdate.returnYCoordinate() / TICKS_PER_INCH);
            telemetry.addData("[Orientation (360)]", positionUpdate.returnOrientation());
            telemetry.update();
        }

        positionUpdate.stop();
    }

    private void resetEncoders() {
        hardware.left_front_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.left_back_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right_front_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right_back_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.left_front_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.left_back_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right_front_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right_back_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
