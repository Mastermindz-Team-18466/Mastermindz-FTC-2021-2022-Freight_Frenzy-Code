package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.SlidesPID;
import org.firstinspires.ftc.teamcode.auto.Trajectories;

import java.util.function.DoubleToIntFunction;

@TeleOp (name = "TeleOp", group = "Concept")
//@Disabled
public class TeleOpMode extends LinearOpMode {

    FieldOrientatedDrive driver;
    Trajectories trajectories;
    Intake intake;
    CarouselMechanism carouselMechanism;
    SlidesTeleOp slides;
    Outtake outtake;

    @Override
    public void runOpMode() {
        // intake = new Intake(gamepad1, 1);
        // carouselMechanism = new CarouselMechanism(gamepad1);
        slides = new SlidesTeleOp(gamepad1, hardwareMap);
        outtake = new Outtake(slides, new V4B(gamepad1), new Claw(gamepad1), gamepad1);


        waitForStart();


        while (opModeIsActive()) {
            outtake.control();

            /*
            Trajectories.Action[] actions = new Trajectories.Action[]{
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.moveForward(100),
                    () -> Trajectories.strafeLeft(50)
            };

            for (Trajectories.Action action : actions) {
                action.move();
            }
            */

            telemetry.update();

            driver.finish();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
