package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;

@TeleOp(group = "EncodersTest")
public class EncodersTest extends LinearOpMode {
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("LeftEncoder", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
