package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@TeleOp(name = "StuyPoofsFullSRC", group = "Bot")
//@Disabled
public class StuyPoofsSRCv1 extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx leftShooter = null;
    private DcMotorEx rightShooter = null;

    private DcMotorEx intake = null;
    private DcMotorEx indexer = null;
    
    private Servo leftPlowerServo = null;
    private Servo rightPlowerServo = null;
    // private Servo blockerServo = null;
    
    private IMU imu = null;

    // need to update
    public final int LAUNCHER_TARGET_VELOCITY = 1150; //needs to be higher 
    public final int LAUNCHER_MIN_VELOCITY = 1100;
    public final int LAUNCHER_COAST_VELOCITY = 800;
    public final int INDEXER_FEED_VELOCITY = 800;
    
    //current logic is for a position; not for power
    double plowerPosition;
    // double blockerPower;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;

    //ran at INIT, this is just setup
    @Override
    public void init() {

        //update with ports
        leftFrontDrive = hardwareMap.get(DcMotor.class, "ch1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "eh1");
        leftBackDrive = hardwareMap.get(DcMotor.class, "ch0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "eh0");

        leftShooter = hardwareMap.get(DcMotorEx.class, "ch2");
        rightShooter = hardwareMap.get(DcMotorEx.class, "eh3");

        intake = hardwareMap.get(DcMotorEx.class, "ch3");
        indexer = hardwareMap.get(DcMotorEx.class, "eh2");

        leftPlowerServo = hardwareMap.get(Servo.class, "ch5");
        rightPlowerServo = hardwareMap.get(Servo.class, "ehs0");
        // blockerServo = hardwareMap.get(CRServo.class, "");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(//UPDATE
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(parameters);

        //correct for inversing 
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(DcMotor.Direction.REVERSE);
        
        // blockerServo.setDirection(CRServo.Direction.FORWARD);

        //behaviors 
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        
        //PID config, don't know if necessary 
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // blockerPower = 0;//whatever number gets it to start upright
        // blockerServo.setPower(blockerPower);
        
        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit START
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits START
    @Override
    public void start() {
        intake.setVelocity(1000);

        // SERVOS
        rightPlowerServo.setPosition(-0.5);
        leftPlowerServo.setPosition(-0.5);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //these are field relative
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

/*        if(gamepad1.x){imu.resetYaw();}//click at start of a match to set what direction we're relative to 

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //these are translated to what the robot uses
        double roboX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double roboY = strafe * Math.sin(heading) + forward * Math.cos(heading);
        mecanumDrive(roboY * 0.5, roboX * 0.5, rotate * 0.5);*/
        mecanumDrive(forward, strafe, rotate);

        launch();
        // blockerCheck();
        // plowerCheck();
        telemetry.addData("leftposiiton", leftPlowerServo.getPosition());
        telemetry.addData("rightposition", rightPlowerServo.getPosition());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("shooterL", leftShooter.getVelocity());
        telemetry.addData("shooterR", rightShooter.getVelocity());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        //spin to target if held and indexer on, coast if not and indexer off
        if (gamepad1.right_bumper) {
            leftShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
            rightShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (leftShooter.getVelocity() > LAUNCHER_MIN_VELOCITY && rightShooter.getVelocity() > LAUNCHER_MIN_VELOCITY){
                indexer.setVelocity(INDEXER_FEED_VELOCITY);
            }
        } else {
            leftShooter.setVelocity(LAUNCHER_COAST_VELOCITY);
            rightShooter.setVelocity(LAUNCHER_COAST_VELOCITY);
            indexer.setVelocity(0);
        }

        //if button pressed & velocity is appropriate, stage balls 
        // if (gamepad1.right_bumper &&
        //     leftShooter.getVelocity() > LAUNCHER_MIN_VELOCITY &&
        //     rightShooter.getVelocity() > LAUNCHER_MIN_VELOCITY) {
        //     indexer.setVelocity(INDEXER_FEED_VELOCITY);
        // } else {
        //     indexer.setVelocity(0);
        // }
    }

    void plowerCheck(){
         if(gamepad1.left_bumper){
             plowerPosition *= -1;
         }

        // leftPlowerServo.setPower(plowerPower);
        // rightPlowerServo.setPower(plowerPower);
     }

    // void blockerCheck(){
    //     if(gamepad1.left_trigger){
    //         blockerPower *= -1;
    //     }
    //     blockerServo.setPower(blockerPower);
    // }


}
