package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Sliders")
public class SlidersPID extends LinearOpMode {

    DcMotor linear_slider;
    double x = 0;
    double goal = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        //Linear Sliders Init
        linear_slider = hardwareMap.get(DcMotor.class, "linear_slider");
        linear_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_slider.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
            linear_slider.setPower(control(x, goal));
            x += control(x, goal);

            if (x == goal) {
                reset();
            }
        }
    }

    public double control(double x, double goal) {
        double error = goal - x;
        double p_gain = 0.1;
        return error * p_gain;
    }

    public void reset() {
        linear_slider.setPower(0);
    }
}
