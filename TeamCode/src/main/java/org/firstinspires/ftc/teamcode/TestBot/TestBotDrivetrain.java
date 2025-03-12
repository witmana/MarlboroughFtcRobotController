package org.firstinspires.ftc.teamcode.TestBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.MotionProfile;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Pose2D;
import org.firstinspires.ftc.teamcode.Utility.SparkfunLocalizer;

import java.util.Locale;

@Config
public class TestBotDrivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //TODO: Declare OpMode member for the Odometry System
    // If using GoBilda Pinpoint computer then use PinPointLocalizer class
    // If using Sparkfun OTOS then use SparfunLocalizer class
    //public PinPointLocalizer localizer;
    public SparkfunLocalizer localizer;

    ElapsedTime time = new ElapsedTime();


    //drivetrain motors
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    //PID controllers for moving to target pose
    PIDController xController;
    PIDController yController;
    PIDController headingController;

    MotionProfile xProfile;
    MotionProfile yProfile;
    MotionProfile headingProfile;

    public boolean targetReached = false;
    Pose2D targetPose;

    //Static Variables
    //TODO Adjust drive constants based on auto performance

    public static double HEADING_KP = 0.015;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.05;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.01;//0.0003;
    public static double DRIVE_MAX_ACC = 2000;
    public static double DRIVE_MAX_VEL = 3500;
    public static double HEADING_MAX_ACC = 2000;
    public static double HEADING_MAX_VEL = 3500;
    public static double DRIVE_MAX_OUT = 0.8;

    /*
    public static double MAX_SPEED = .5;
    public static double MIN_SPEED = .1;
    public static double RAMP_UP_RATE = 0.01;
    public static double RAMP_DOWN_RATE = 0.01;
    public static double THRESHOLD = 1;
    public static double HEADING_MAX_SPEED = .5;
    public static double HEADING_MIN_SPEED = .1;
    public static double HEADING_RAMP_UP_RATE = 5;
    public static double HEADING_RAMP_DOWN_RATE = 5;
    public static double HEADING_THRESHOLD = 1;

     */

    public TestBotDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //Initialize PID controllers
        //xController = new RampingController(MAX_SPEED, MIN_SPEED, RAMP_UP_RATE, RAMP_DOWN_RATE, THRESHOLD);
        //yController = new RampingController(MAX_SPEED, MIN_SPEED, RAMP_UP_RATE, RAMP_DOWN_RATE, THRESHOLD);
        //headingController = new RampingController(MAX_SPEED, MIN_SPEED, RAMP_UP_RATE, RAMP_DOWN_RATE, THRESHOLD);
        xController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP,HEADING_KI,HEADING_KD,DRIVE_MAX_OUT);



        //TODO Change constructor based on localization system
        //localizer = new PinPointLocalizer(myOpMode);
        localizer = new SparkfunLocalizer(myOpMode);

        localizer.init();

        //TODO Set the offset of the localizer sensor
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(5, 0, 0);
        localizer.myOtos.setOffset(offset);

        xProfile = new MotionProfile(DRIVE_MAX_ACC,DRIVE_MAX_VEL, 0);
        yProfile = new MotionProfile(DRIVE_MAX_ACC,DRIVE_MAX_VEL, 0);
        headingProfile = new MotionProfile(HEADING_MAX_ACC,HEADING_MAX_VEL,0);

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        useEncoders();

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }


    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOp() {
        //update localizer (should display position to dashboard)
        localizer.update();

        //drive train
        double max;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        leftFrontPower = (drive + turn - strafe);
        rightFrontPower = (drive - turn + strafe);
        leftBackPower = (drive + turn + strafe);
        rightBackPower = (drive - turn - strafe);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        //Slow and Turbo Buttons
        //TODO Adjust factors affecting slow and turbo buttons
        //turbo button (full power)
        if (myOpMode.gamepad1.right_bumper) {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
        //slow button (fraction of full power)
        else if (myOpMode.gamepad1.left_bumper) {
            leftFrontDrive.setPower(leftFrontPower / 7);
            rightFrontDrive.setPower(rightFrontPower / 7);
            leftBackDrive.setPower(leftBackPower / 7);
            rightBackDrive.setPower(rightBackPower / 7);
        }
        //default power
        else {
            leftFrontDrive.setPower(leftFrontPower / 2);
            rightFrontDrive.setPower(rightFrontPower / 2);
            leftBackDrive.setPower(leftBackPower / 2);
            rightBackDrive.setPower(rightBackPower / 2);
        }
    }

    public void update(){
        localizer.update();
    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;\
    }

    public void driveToPose(double xTarget, double yTarget, double degreeTarget) {
        targetPose = new Pose2D(DistanceUnit.INCH, xTarget,yTarget,AngleUnit.DEGREES,degreeTarget);
        //check if drivetrain is still working towards target
        targetReached = xController.targetReached && yController.targetReached && headingController.targetReached;
        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xController.calculate(xTarget, localizer.getX());
        double yPower = yController.calculate(yTarget, localizer.getY());

        double wrappedAngleError = angleWrap(degreeTarget - localizer.getHeading());
        double tPower = headingController.calculate(wrappedAngleError);

        double radianHeading = Math.toRadians(localizer.getHeading());

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
        double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

        String data = String.format(Locale.US, "{tX: %.3f, tY: %.3f, tH: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));

        myOpMode.telemetry.addData("Target Position", data);
        myOpMode.telemetry.addData("XReached", xController.targetReached);
        myOpMode.telemetry.addData("YReached", yController.targetReached);
        myOpMode.telemetry.addData("HReached", headingController.targetReached);
        myOpMode.telemetry.addData("targetReached", targetReached);
    }

    //TODO Account for direction of travel...consider having Motion Profile class have a direction variable and multiply the output by direction
    public void profiledDriveToPose(double xTarget, double yTarget, double degreeTarget) {

        //check if the target pose is new
        if(targetPose.getX(DistanceUnit.INCH) != xTarget ||
            targetPose.getY(DistanceUnit.INCH) != yTarget ||
                targetPose.getHeading(AngleUnit.DEGREES) != degreeTarget){

            //if target is new, calculate motion profile time and reset timer, and store original distance
            targetPose = new Pose2D(DistanceUnit.INCH, xTarget,yTarget,AngleUnit.DEGREES,degreeTarget);
            xProfile = new MotionProfile(DRIVE_MAX_ACC,DRIVE_MAX_VEL,xTarget-localizer.getX());
            yProfile = new MotionProfile(DRIVE_MAX_ACC,DRIVE_MAX_VEL,yTarget-localizer.getY());
            headingProfile = new MotionProfile(HEADING_MAX_ACC,HEADING_MAX_VEL,angleWrap(degreeTarget - localizer.getHeading()));
        }


        //check if drivetrain is still working towards target
        targetReached = xProfile.profileComplete && yProfile.profileComplete && headingProfile.profileComplete;
        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xController.calculate(xProfile.returnInstantTarget(), localizer.getX());
        double yPower = yController.calculate(yProfile.returnInstantTarget(), localizer.getY());

        double wrappedAngleError = angleWrap(degreeTarget - localizer.getHeading());
        double tPower = headingController.calculate(wrappedAngleError);

        double radianHeading = Math.toRadians(localizer.getHeading());

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
        double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

        String data = String.format(Locale.US, "{tX: %.3f, tY: %.3f, tH: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));

        myOpMode.telemetry.addData("Target Position", data);
        myOpMode.telemetry.addData("XReached", xController.targetReached);
        myOpMode.telemetry.addData("YReached", yController.targetReached);
        myOpMode.telemetry.addData("HReached", headingController.targetReached);
        myOpMode.telemetry.addData("targetReached", targetReached);
    }

    public void stop(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }

        // keep in mind that the result is in degrees
        return degrees;
    }
}
