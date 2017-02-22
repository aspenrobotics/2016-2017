package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name = "Auto_Test", group = "Auto")
@Disabled
public class Auto_Test extends OpMode {

    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    private OpticalDistanceSensor opticalSensor;
    private boolean readyToFire = true;
    private ModernRoboticsI2cCompassSensor compassSensor;
    private double initialHeading, angle;

    private enum RobotState{
        FirstTwoShots, ReadCompassSensor, TurnToWhiteLine, BeforeWhiteCross, AfterWhiteCross, CenterWhiteLine, OffLine, CenterWhiteLinePartTwo, BackUp,
        Forward, ColorSensorRead
    }
    private RobotState robotState;


    public void init(){
        //declaring motor vars
        robotState = RobotState.FirstTwoShots;
        leftMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor = hardwareMap.dcMotor.get("left_drive");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        rollerMotor = hardwareMap.dcMotor.get("roller_motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        topSensor = hardwareMap.colorSensor.get("top_sensor");
        opticalSensor = hardwareMap.opticalDistanceSensor.get("optical_sensor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        topSensor.enableLed(false);
        compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass_sensor");
        compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //setting mode
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        angle = 45;
    }
    public void stop(){

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void loop(){
        //Double shot
        startShot(this.time);
        //MoveToWhiteLine
        moveToWhiteLine(this.time);
        //Line Up With Thing
        centerToWhiteLine(this.time);
        //Meet with color stuff
        pushTheBeacon(this.time);
        




        //The code above works for now
        //Moving forward
        telemetry.addData("This is the time value:  " + this.time, this.time);
        telemetry.addData("This is the optical sensor reading: " + opticalSensor.getLightDetected(), opticalSensor);
        telemetry.addData("This is the color sensor reading: " + topSensor.blue() + " " + topSensor.red(), topSensor );
        /*telemetry.addData("This is the Heading: " + compassSensor.getDirection(), compassSensor.getDirection());
        telemetry.addData("This is the Initial Heading value: " + initialHeading, compassSensor);
        */

    }
    private void pushTheBeacon(double time){
        if(robotState == RobotState.BackUp && time > 14.7 && time < 15.4){
            rightMotor.setPower(-.5);
            leftMotor.setPower(.5);
        }
        if(time > 15.4 && robotState == RobotState.BackUp){
            robotState = RobotState.Forward;
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }
    private void centerToWhiteLine(double time){
        if(robotState == RobotState.CenterWhiteLine && time > 11.8 && time < 12){
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        }
        if((robotState == RobotState.CenterWhiteLine || robotState == RobotState.CenterWhiteLinePartTwo) && time > 12){
            rightMotor.setPower(-.5);
            leftMotor.setPower(.5);
            if(opticalSensor.getLightDetected() > .2){
                robotState = RobotState.CenterWhiteLinePartTwo;
            }
            if(opticalSensor.getLightDetected() < .2 && robotState == RobotState.CenterWhiteLinePartTwo){
                robotState = RobotState.OffLine;
            }
        }
        if(robotState == RobotState.OffLine && time > 14.5){
            leftMotor.setPower(.5);
            rightMotor.setPower(-.5);
        }
        if(robotState == RobotState.OffLine && time > 14.5 && time < 14.7){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            robotState = RobotState.BackUp;
        }
    }
    private void catapultArmFullPower() throws Exception{
        if(readyToFire) {
            armMotor.setPower(1);
            Thread.sleep(550);
            readyToFire = false;
        }
    }
    //Move to white line
    private void moveToWhiteLine(double time){
        if(opticalSensor.getLightDetected() > .20 && opticalSensor.getLightDetected() < .40 && robotState == RobotState.BeforeWhiteCross){
            robotState = RobotState.AfterWhiteCross;
        }

        if(time > 7 && time < 7.57) {
            leftMotor.setPower(1);
        }
        if((time > 7.57 && robotState == RobotState.BeforeWhiteCross) && time < 8.2){
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        }
        if(time > 8.2 && (robotState == RobotState.BeforeWhiteCross)){
            leftMotor.setPower(.3);
            rightMotor.setPower(.3);
        }
        if(time > 7.7 && robotState == RobotState.AfterWhiteCross){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            robotState = RobotState.CenterWhiteLine;
        }
    }
    //The Start of the Program Shoots Twice
    private void startShot(double time){
        if(time < 1.4 && time > .7){
            armMotor.setPower(-.3);
        }

        if(time > 1.4 && time < 2.1){
            armMotor.setPower(0);

        }
        if(time > 2.2 && readyToFire){
            try {
                catapultArmFullPower();
            } catch (Exception e) {
                e.printStackTrace();
            }
            armMotor.setPower(0);
        }
        //second shot
        if(time > 3.5 && time < 5.5){
            elevatorMotor.setPower(1);
        }
        if(time < 5.9 && time > 5.5){
            elevatorMotor.setPower(0);
            armMotor.setPower(-.3);
        }
        if((time > 6.1 && time < 6.2)){
            readyToFire = true;
            robotState = RobotState.BeforeWhiteCross;
        }
    }

}
