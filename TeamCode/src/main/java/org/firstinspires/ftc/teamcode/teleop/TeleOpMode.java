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
        outtake = new Outtake(slides, new V4B(gamepad1, hardwareMap), new Claw(gamepad1, hardwareMap), gamepad1);


        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.b) {
                outtake.control(Outtake.Position.BACK);
            } else if (gamepad1.dpad_down) {
                outtake.control(Outtake.Position.BOTTOM_FORWARD);
            } else if (gamepad1.dpad_up) {
                outtake.control(Outtake.Position.TOP_FORWARD);
            }

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
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
