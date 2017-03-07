package org.firstinspires.ftc.robotcontroller.internal.SuperRegionals;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by AHSRobotics on 3/6/2017.
 */

@TeleOp(name = "Test", group = "Tester")
public class ArmTest extends OpMode {
    private DcMotor leftMotor, rightMotor, armMotor, elevatorMotor, rollerMotor, liftMotor, liftMotorTwo;
    private ColorSensor topSensor;
    private Servo beaconHit;

    private enum CState {
        Normal, SlowSpeed
    }

    private enum ServoState {
        Inside, Outside
    }

    private CState cState;
    private ServoState servoState;
    @Override
    public void init() {
        // Save reference to Hardware map
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMaxSpeed(600);
        beaconHit = hardwareMap.servo.get("main_servo");

        // Define and initialize ALL installed servos.
    }

    @Override
    public void start() {
        armMotor.setPower(0);
        //bottomSensor.enableLed(false);

    }

    @Override
    public void stop() {
        armMotor.setPower(0);
    }

    @Override
    public void loop() {
        telemetry.addData("Servo Position is at: " + beaconHit.getPosition(), beaconHit);
        if(gamepad1.a){
            beaconHit.setPosition(0);
        }
        if (gamepad1.x){
            beaconHit.setPosition(1);
        }
        if(gamepad1.y){
            armMotor.setPower(1);

        }else{
            armMotor.setPower(0);
        }

    }
}
