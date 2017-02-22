package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Disabled
@Autonomous(name = "Straight Forward", group = "Auto")
public class Auto_Regional_Red extends OpMode {

    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor;
    private ColorSensor topSensor;
    private OpticalDistanceSensor opticalSensor;
    private boolean readyToFire = true;
    private double initialHeading, angle;

    private enum RobotState{
        FirstTwoShots, ContinueForward, Turn45, RollUpRamp
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
        angle = 45;
    }
    public void stop(){

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void loop(){
        //Double shot
        startShot(this.time);
        //MoveForward
        continueForward(this.time);
        //Turn45
        /*turn45(this.time);
        //Backup
        RollUpRamp(this.time);
        */
        




        //The code above works for now
        //Moving forward
        telemetry.addData("This is the time value:  " + this.time, this.time);
        telemetry.addData("This is the optical sensor reading: " + opticalSensor.getLightDetected(), opticalSensor);
        telemetry.addData("This is the color sensor reading: " + topSensor.blue() + " " + topSensor.red(), topSensor);
        /*telemetry.addData("This is the Heading: " + compassSensor.getDirection(), compassSensor.getDirection());
        telemetry.addData("This is the Initial Heading value: " + initialHeading, compassSensor);
        */

    }
    private void RollUpRamp(double time){
        if(time > 13 && time < 18 && robotState == RobotState.RollUpRamp){
            rightMotor.setPower(-1);
            leftMotor.setPower(-1);
        }
        if(time > 18 && robotState == RobotState.RollUpRamp){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    private void continueForward(double time){
        if(time > 7 && (robotState == RobotState.ContinueForward && time < 8.8)){
            rightMotor.setPower(1);
            leftMotor.setPower(1);
        }
        if(time > 8.8 && robotState == RobotState.ContinueForward){
            rightMotor.setPower(0);
            leftMotor.setPower(0);
            robotState = RobotState.Turn45;
        }
    }
    private void turn45(double time){
        if(time > 8.8 && time < 10.2 && robotState == RobotState.Turn45){
            rightMotor.setPower(-1);
        }
        if(time > 10.2 && robotState == RobotState.Turn45){
            rightMotor.setPower(0);
            robotState = RobotState.RollUpRamp;
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
        if((time > 6.1 && time < 6.2) && robotState == RobotState.FirstTwoShots){
            readyToFire = true;
            robotState = RobotState.ContinueForward;
        }
    }

}
