package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class SkystoneCommon extends LinearOpMode {

    VuforiaLocalizer vuforia;

    //HardwareXplorer robot = new HardwareXplorer();
    HardwareGobilda robot = new HardwareGobilda();


    double offsetGyro = 0.0;
    double startHeading = 0.0;
    final double GYRO_THRESHOLD = 0.5;
    final double CORRECTION_MULTIPLIER = 0.02;
    final double ENC_PER_INCH = 31; // recalibrated for GoBilda 5202; AndyMark was 44
    double forwardSpeed = 0.15;
    double leftChange, rightChange;
    double changeNum;
    final double DRIVE_SPEED = 0.1; //was 0.7
    final double DS = 0.7;
    final double MAX_SPEED = 1;
    final double TURN_CORRECTION = 0.001;
    double turnSpeed = 0.5;
    double MIN_TURN_LIMIT = 0.1;
    double globalAngle, power = .30, correction;
    Orientation lastAngles = new Orientation();
    private static final String VUFORIA_KEY = "AYEB3rP/////AAABmVMCZTLJIE0TrkP2639XOkl5oPNywXyUOnq52N57nxQ2Q4KVO6xRk1CWWvTIbeZfVku0ISp4m3dPUpfORFAHlDqKOCdLUfRP78YbJexyDYJ+q2KQlap1/SH5sZi/llSpn5Y0b30k/VK/txgdo7TsyZBNZrcldc0KRiwo3NVeQwuVHjxFLJsU0P3MmwNDKZ5Fax3l1yglpGB5Ej2Vevu1gmVfGxxcnMaw0m89+olVLW/rKOB1mOjNC4nwOMsljk/5uY0OMBfkm14r6/HnvXk5bX/GLyBL2gPRdmIL5wtAn0JDjoa+RkhA930mTv0h4Mzh1gZ3PLwWf3Nnt3T5Qu7ffT/RX9zPJ7pKOlp9m4a0Hfs5";
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private final double ANGLE_THRESHOLD = 0.5;
    private String finalPos = "";
    //private Debugger d = new Debugger("Crater debug");
  @Override
  public void runOpMode() {
  }

    public void driveX(double forwardSpeed, double startHeading, double dist){
        robot.resetEncoders();
        dist = dist * ENC_PER_INCH;
        telemetry.addData("moving forward (enc val)...", dist);
        telemetry.update();

        double encVal = robot.fl.getCurrentPosition();
        telemetry.addData("enc val start: ", encVal);

        while(opModeIsActive() && robot.fl.getCurrentPosition() < dist) {
            double encVal = robot.fl.getCurrentPosition();
            telemetry.addData("enc val: ", encVal);
            telemetry.update();
            moveXForward (forwardSpeed, startHeading);
        }
        telemetry.update();
        robot.allStop();
    }

    public void driveXback(double backwardSpeed, double startHeading, double dist){
        backwardSpeed = Math.abs(backwardSpeed);
        robot.resetEncoders();
        //telemetry.addData("moving backward...", dist);
        //telemetry.update();
        dist = dist * ENC_PER_INCH;

        while(opModeIsActive()  && robot.fr.getCurrentPosition() < dist) {
            moveXBackward (backwardSpeed, startHeading);
            telemetry.addData("enc val: ", robot.fr.getCurrentPosition());
            telemetry.update();
        }
        robot.allStop();
    }

    public void strafeRight(double speed, double dist)
    {
        dist = dist * ENC_PER_INCH;
        telemetry.addData("StrafeRight: ", robot.fr.getCurrentPosition());

        while(opModeIsActive()  && robot.fr.getCurrentPosition() > -dist)
        {
            robot.driveLimitless(-speed, -speed, speed, speed);
            telemetry.addData("enc val: ", robot.fr.getCurrentPosition());
            telemetry.update();
        }
        robot.allStop();
        robot.resetEncoders();
    }

    public void strafeLeft(double speed, double dist)
    {
        dist = dist * ENC_PER_INCH;
        telemetry.addData("StrafeLeft: ", robot.fr.getCurrentPosition());

        while(opModeIsActive()  && robot.fr.getCurrentPosition() < dist)
        {
            robot.driveLimitless(speed, speed, -speed, -speed);
            telemetry.addData("enc val: ", robot.fr.getCurrentPosition());
            telemetry.update();
        }
        robot.allStop();
        robot.resetEncoders();
    }

    public void moveXForward(double forwardSpeed, double startHeading){
        leftChange = forwardSpeed;
        rightChange = forwardSpeed;
        if(robot.getXHeadingGyro() - offsetGyro - startHeading > GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getXHeadingGyro() - offsetGyro - startHeading);
            rightChange += CORRECTION_MULTIPLIER * changeNum;
            telemetry.addData("heading: ", robot.getXHeadingGyro());
            telemetry.addData("right adj: ", rightChange);
        }else if(robot.getXHeadingGyro() - offsetGyro - startHeading < -GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getXHeadingGyro() - offsetGyro - startHeading);
            leftChange += CORRECTION_MULTIPLIER * changeNum;
            telemetry.addData("heading: ", robot.getXHeadingGyro());
            telemetry.addData("left adj: ", leftChange);
        }else{
            //robot is driving within acceptable range
        }
        robot.driveLimitless(-leftChange, rightChange, -leftChange, rightChange);
        telemetry.update();
        //comDbg.debugMessage("MvFwd: left Change, right Change: " + Double.toString(leftChange) +", " +Double.toString(rightChange));
    }

    public void moveXBackward(double backwardSpeed, double startHeading){
        telemetry.addData("movingBack: ", startHeading);
        telemetry.update();
        leftChange = backwardSpeed;
        rightChange = backwardSpeed;
        if(robot.getXHeadingGyro() -offsetGyro - startHeading > GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getXHeadingGyro() -offsetGyro - startHeading);
            leftChange += CORRECTION_MULTIPLIER * changeNum;
        }else if(robot.getXHeadingGyro() -offsetGyro - startHeading < -GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getXHeadingGyro() -offsetGyro - startHeading);
            rightChange += CORRECTION_MULTIPLIER * changeNum;
        }else{
            //robot is driving within acceptable range
        }
        robot.driveLimitless(leftChange, -rightChange, leftChange, -rightChange);
        //telemetry.addData("heading: ", robot.getHeadingGyro());
        //telemetry.addData("left adj: ", leftChange);
        //telemetry.addData("right adj: ", rightChange);
        //telemetry.update();
    }


}
