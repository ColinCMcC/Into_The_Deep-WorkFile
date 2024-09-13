package org.firstinspires.ftc.teamcode;

// Static imports for ENUMs that are used a lot

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.JimBot_Coded_diff.turnDir.LEFT;
import static org.firstinspires.ftc.teamcode.JimBot_Coded_diff.turnDir.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// This file is testing out driving the robot using calculated wheel speeds for turning while driving
// For turning, 0° is straight forward, 90° is to the right, and -90° is to the left

@TeleOp(name = "Curved Steering OpMode")
public class JimBot_Coded_diff extends LinearOpMode {

    // Constants for robot geometry
    private static final double WHEELBASE = 13; // Wheelbase (distance between front and rear axles) in IN.
    private static final double TRACK_WIDTH = 14; // Track width (distance between left and right wheels) in IN.
    private static final double WHEEL_DIAMETER = 3; // Wheel diameter in IN.


    // Create motor instances
    DcMotor FLMotor, FRMotor, BLMotor, BRMotor;

    // Create servo instances
    Servo FLServo, BLServo, BRServo, FRServo;


    @Override
    public void runOpMode() {


        // Initialize hardware stuff
        initRobot();


        waitForStart(); // Waits for the Start button to be pressed
        while (opModeIsActive()) {

            // Gets the user inputs from the gamepad
            double drive = -gamepad1.left_stick_y; // Forward/backward movement
            double turn = gamepad1.right_stick_x;  // Left/right turn

            // Calculates the angular velocity and turning radius based on user inputs
            double turningRadius = calculateTurningRadius(turn); // Radius in IN.
            double angularVelocity = drive / turningRadius; // Calculate angular velocity (rad/s)

            // Calculates the speeds and angles for each motor
            double[] wheelSpeed = calculateWheelSpeed(angularVelocity, turningRadius);
            double[] wheelAngle = calculateWheelAngles(turningRadius);


            // If the gamepad is turning right, turn the robot right, otherwise turn left
            turnDir direction = LEFT;
            if (turn < 0) direction = RIGHT;

            moveIt(wheelSpeed, wheelAngle, direction);

        }
    }


    // Configures all the stuff for the robot
    public void initRobot() {

        // Maps where the motors are physically connected
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Reverse the right motors to account for their opposite orientation
        FLMotor.setDirection(FORWARD);
        FRMotor.setDirection(REVERSE);
        BLMotor.setDirection(FORWARD);
        BRMotor.setDirection(REVERSE);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(0.0, 1.0);
        FRServo.scaleRange(0.0, 1.0);
        BLServo.scaleRange(0.0, 1.0);
        BRServo.scaleRange(0.0, 1.0);

    }


    // Moves the robot
    // wheelSpeed[] = Four parameters - FLSpeed, FRSpeed, BLSpeed, BRSpeed
    // wheelAngle[] = Two parameters - Inner angle, Outer angle
    // turnDirection = Direction to turn the robot - LEFT or RIGHT
    public void moveIt(double[] wheelSpeed, double[] wheelAngle, turnDir dir) {

        if (dir == RIGHT) // Turning right
        {
            // Set power to motors based on calculated speeds
            FLMotor.setPower(wheelSpeed[0]); // Outside Front
            FRMotor.setPower(wheelSpeed[1]); // Inside Front
            BLMotor.setPower(wheelSpeed[2]); // Outside Rear
            BRMotor.setPower(wheelSpeed[3]); // Inside Rear

            // Sets the custom positions of the wheels
            FLServo.setPosition(wheelAngle[1]); // Outer
            FRServo.setPosition(wheelAngle[0]); // Inner
            BLServo.setPosition(-wheelAngle[1]); // Outer. Negative because the rear wheels need to turn the opposite direction
            BRServo.setPosition(-wheelAngle[0]); // Inner

        } else // Turning Left
        {
            // Set power to motors based on calculated speeds
            FLMotor.setPower(wheelSpeed[0]); // Inside Front
            FRMotor.setPower(wheelSpeed[1]); // Outside Front
            BLMotor.setPower(wheelSpeed[2]); // Inside Rear
            BRMotor.setPower(wheelSpeed[3]); // Outside Rear

            // Sets the custom positions of the wheels
            FLServo.setPosition(wheelAngle[0]); // Inner
            FRServo.setPosition(wheelAngle[1]); // Outer
            BLServo.setPosition(-wheelAngle[0]); // Inner. Negative because the rear wheels need to turn the opposite direction
            BRServo.setPosition(-wheelAngle[1]); // Outer.
        }
    }


    // Function to calculate wheel speeds for a given desired speed and turning radius
    // desiredSpeed = Desired speed of the robot
    private double[] calculateWheelSpeed(double desiredSpeed, double turningRadius) {

        // Inside and outside turning radii for front/rear wheels
        double R_inner = turningRadius - (TRACK_WIDTH / 2);
        double R_outer = turningRadius + (TRACK_WIDTH / 2);

        // double I_Modifier = ;

        // Calculate wheel speeds
        double S_IF = desiredSpeed * R_inner; // Inside Front
        double S_OF = desiredSpeed * R_outer; // Outside Front
        double S_IB = desiredSpeed * R_inner; // Inside Back
        double S_OB = desiredSpeed * R_outer; // Outside Back

        // If any wheel speeds are above 1.0, scales all the others down proportionally
        double maxSpeed = Math.max(Math.max(Math.abs(S_IF), Math.abs(S_OF)), Math.max(Math.abs(S_IB), Math.abs(S_OB)));
        if (maxSpeed > 1.0) {
            S_IF /= maxSpeed;
            S_OF /= maxSpeed;
            S_IB /= maxSpeed;
            S_OB /= maxSpeed;
        }

        return new double[]{S_IF, S_OF, S_IB, S_OB};
    }


    // Calculates the inner and outer wheel angles for a given radius
    public double[] calculateWheelAngles(double turningRadius) {

        // Takes the turning radius, and calculates the angle of a line tangent to it at the appropriate distance
        double inner = Math.atan(WHEELBASE / (turningRadius - (TRACK_WIDTH / 2)));
        double outer = Math.atan(WHEELBASE / (turningRadius + (TRACK_WIDTH / 2)));

        return new double[]{inner, outer};
    }


    // Calculates the turning radius based on the turn input
    private double calculateTurningRadius(double turn) {
        // Converts joystick input (-1 to 1) to turning radius

        double maxTurnRadius = 24; // Max turning radius in IN., might need to be changed

        if (Math.abs(turn) < 0.01) {
            return Double.POSITIVE_INFINITY; // Moving straight
        }

        // Smaller turn input means tighter radius
        return maxTurnRadius / Math.abs(turn);
    }


    // Used to store possible turning directions
    enum turnDir {
        LEFT, RIGHT
    }


}

