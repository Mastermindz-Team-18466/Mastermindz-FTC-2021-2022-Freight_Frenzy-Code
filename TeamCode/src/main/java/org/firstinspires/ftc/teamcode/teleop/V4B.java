package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class V4B {
    Servo rightV4B, leftV4B;
    Gamepad gamepad;

    public enum State {
        OPEN,
        CLOSE,
        SHARED
    }

    public V4B(Gamepad gamepad, HardwareMap hardwareMap) {
        rightV4B = hardwareMap.get(Servo.class, "rightV4B");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");

        this.gamepad = gamepad;
    }

    public void control(State state) {
        if (state == State.CLOSE) {
            rightV4B.setDirection(Servo.Direction.FORWARD);
            leftV4B.setDirection(Servo.Direction.REVERSE);
            rightV4B.setPosition(0.025);
            leftV4B.setPosition(0.025);
        } else if (state == State.SHARED) {
            rightV4B.setDirection(Servo.Direction.FORWARD);
            leftV4B.setDirection(Servo.Direction.REVERSE);
            rightV4B.setPosition(1);
            leftV4B.setPosition(1);
        } else if (state == State.OPEN) {
            rightV4B.setDirection(Servo.Direction.FORWARD);
            leftV4B.setDirection(Servo.Direction.REVERSE);
            rightV4B.setPosition(0.67);
            leftV4B.setPosition(0.67);
        }
    }
}
