package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class V4B {
    HardwareMap hardwareMap;

    Servo rightV4B, leftV4B;
    Gamepad gamepad;

    public V4B(Gamepad gamepad) {
        rightV4B = hardwareMap.get(Servo.class, "rightV4B");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");

        this.gamepad = gamepad;
    }

    public void control() {
        if (gamepad.right_trigger == 0.75) {
            rightV4B.setDirection(Servo.Direction.FORWARD);
            leftV4B.setDirection(Servo.Direction.FORWARD);
            rightV4B.setPosition(0.5);
            leftV4B.setPosition(0.5);
        } else if (gamepad.left_trigger == 0.75) {
            rightV4B.setDirection(Servo.Direction.REVERSE);
            leftV4B.setDirection(Servo.Direction.REVERSE);
            rightV4B.setPosition(0.2);
            leftV4B.setPosition(0.2);
        }
    }
}
