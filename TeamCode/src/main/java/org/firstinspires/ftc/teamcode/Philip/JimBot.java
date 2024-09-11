package org.firstinspires.ftc.teamcode.Philip;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class JimBot extends LinearOpMode {

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor;

    Servo FLServo, BLServo, BRServo, FRServo;

    ElapsedTime turnTime = new ElapsedTime();

    IMU imu;


    @Override
    public void runOpMode() {

        initRobot(); // Does all the robot stuff

        waitForStart();
        while (opModeIsActive()) {
            double speed = gamepad1.right_trigger + (-gamepad1.left_trigger); // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angle = gamepad1.right_stick_x;
            if (speed != 0)
                move(gamepad1.left_stick_x, speed);
            else if (angle != 0)
                rotate(angle);
            else {
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }
            if (gamepad1.a == true)
                moveHome();

        }

    }


    public void initRobot() {

        //init imu and stuff stolen from SensorIMUOrthogonal in external samples
        imu = hardwareMap.get(IMU.class, "imu");
        LogoFacingDirection logoDirection = UP;
        UsbFacingDirection usbDirection = BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Maps the motor objects to the physical ports
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");

        // Sets the encoder mode
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);

        FLMotor.setDirection(FORWARD);
        BLMotor.setDirection(FORWARD);
        BRMotor.setDirection(REVERSE);
        FRMotor.setDirection(REVERSE);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(0.0, 1.0);
        BLServo.scaleRange(0.0, 1.0);
        BRServo.scaleRange(0.0, 1.0);
        FRServo.scaleRange(0.0, 1.0);

        double[] test = cartesianToPolar(1, 0);

    }


    // Converts cartesian coordinates to polar
    // 0 = r
    // 1 = theta
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta


        return arrayToReturn;
    }


    /*
    Converts polar coordinates to cartesian
     0 = x
     1 = y
    */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }

    //double oldH = 0;

    //oldH newH not used maybe later to have motors wait to turn wheel until Servo is in right position
    public void move(double heading, double power) {
        //double newH;
        /*
        /if(newH>=oldH-5 && newH <=oldH+5)
            oldH = newH;
         */

        heading = (heading + 1) / 2;

        FLServo.setPosition(heading);
        BLServo.setPosition(heading);
        BRServo.setPosition(heading);
        FRServo.setPosition(heading);

        //if(turnTime<)

        FLMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);
        //oldH = newH;
    }

    public void rotate(double angle) {

        //set wheels for rotation
        FLServo.setPosition(.25);
        BLServo.setPosition(.75);
        BRServo.setPosition(.25);
        FRServo.setPosition(.75);



        //turn motors to rotate robot
        FLMotor.setPower(angle);
        BLMotor.setPower(angle);
        BRMotor.setPower(angle);
        FRMotor.setPower(angle);

    }


    //uses imu
    public void moveHome() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawAngle = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw angle", yawAngle);
        rotate(yawAngle);
    }

    public void moveToSpecimen() {

    }

    public void moveToStore() {

    }

}
