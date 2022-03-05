package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: V4BTest", group = "Test")
public class V4BTest extends LinearOpMode {
    Servo rightV4B, leftV4B;

    @Override
    public void runOpMode() throws InterruptedException {
        rightV4B = hardwareMap.get(Servo.class, "rightV4B");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                rightV4B.setDirection(Servo.Direction.FORWARD);
                leftV4B.setDirection(Servo.Direction.REVERSE);
                rightV4B.setPosition(0);
                leftV4B.setPosition(0);
            } else if (gamepad1.a) {
                rightV4B.setDirection(Servo.Direction.FORWARD);
                leftV4B.setDirection(Servo.Direction.REVERSE);
                rightV4B.setPosition(1);
                leftV4B.setPosition(1);
            }
        }
    }
}
