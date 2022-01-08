package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptGamepadRumble;

@TeleOp(name = "Concept: Main,", group = "Concept")
//@Disabled
public class Main extends LinearOpMode {

    SlidesPID slides;
    FieldOrientedDriveV2 driver;
    Gamepad gamepad;

    @Override
    public void runOpMode() {
        driver = new FieldOrientedDriveV2();

        waitForStart();

        while (opModeIsActive()) {
            driver.move();

            slides.control();

            telemetry.addData("Vector X", driver.vector.getX());
            telemetry.addData("Vector Y", driver.vector.getY());
            telemetry.addData("Vector Direction", driver.vector.getDir());
            telemetry.update();

            driver.finish();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
