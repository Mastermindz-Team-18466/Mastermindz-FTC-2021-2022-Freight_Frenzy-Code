package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Hardware {
    public DcMotor left_front_driver, right_front_driver, left_back_driver, right_back_driver;
    public BNO055IMU imu;
    public OpenCvInternalCamera webcam;
    public DcMotor left_linear_slide, right_linear_slide;
    public DcMotor left_encoder, right_encoder, middle_encoder;
    public SampleMecanumDrive drivetrain;
    public DcMotor intake_motor;
    public double intakePower;
    Orientation angles;

    HardwareMap hardwareMap;

    public Hardware() {
    }

    public void init(HardwareMap hardwareMap) {
        //Drivetrain
        drivetrain = new SampleMecanumDrive(hardwareMap);

        //Left Front Driver
        left_front_driver = hardwareMap.get(DcMotor.class, "leftFront_driver");
        left_front_driver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_driver.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Front Driver
        right_front_driver = hardwareMap.get(DcMotor.class, "rightFront_driver");
        right_front_driver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_driver.setDirection(DcMotorSimple.Direction.FORWARD);

        //Left Back Driver
        left_back_driver = hardwareMap.get(DcMotor.class, "leftBack_driver");
        left_back_driver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back_driver.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Driver
        right_back_driver = hardwareMap.get(DcMotor.class, "rightBack_driver");
        right_back_driver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_driver.setDirection(DcMotorSimple.Direction.FORWARD);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //OpenCV Internal Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();

        //Left Linear Slide
        left_linear_slide = hardwareMap.get(DcMotor.class, "leftLinear_slide");
        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Linear Slide
        right_linear_slide = hardwareMap.get(DcMotor.class, "rightLinear_slide");
        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Left Encoder
        left_encoder = hardwareMap.get(DcMotor.class, "left_encoder");

        //Right Encoder
        right_encoder = hardwareMap.get(DcMotor.class, "right_encoder");

        //Middle Encoder
        middle_encoder = hardwareMap.get(DcMotor.class, "middle_encoder");

        //Intake
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
    }
}