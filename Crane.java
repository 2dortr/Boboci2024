package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;

public class Crane {
    //MOTOR DECLARATION
    public DcMotor motorCrane1, motorCrane2;
    public CRServo servoGrippy, servoSlide1, servoSlide2;
    public TouchSensor slideSensor;
    public AnalogInput slideEncoder;
    //PID VARIABLES
    public int craneTarget = 0;
    public static double craneP = 0.017, craneI = 0, craneD = 0.0005;
    double craneIntegralSum = 0;
    public static double craneF = 0.28;
    public static int craneOffset = 100;
    public double slideExtension = 0;
    public double slideEncoderLastPosition = 0;
    private final double threetwelvemotorticksindegree = 537.7 / 360;
    double craneLastError = 0;
    public int gripperDirection = 1;
    public int slidesDirection = 1;
    public double gripperPower = 1;
    public double slidesPower = 1;
    ElapsedTime craneTimer = new ElapsedTime();

    //MOTOR INIT
    public Crane(HardwareMap hardwareMap){
        motorCrane1 = hardwareMap.dcMotor.get("motorCrane1");
        motorCrane2 = hardwareMap.dcMotor.get("motorCrane2");

        servoGrippy = hardwareMap.crservo.get("servoGrippy");
        servoSlide1 = hardwareMap.crservo.get("servoSlide1");
        servoSlide2 = hardwareMap.crservo.get("servoSlide2");

        slideSensor = hardwareMap.touchSensor.get("slideSensor");

        slideEncoder = hardwareMap.analogInput.get("slideEncoder");

        motorCrane1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCrane1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCrane2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCrane2.setDirection(DcMotorSimple.Direction.REVERSE);
        servoGrippy.setDirection(CRServo.Direction.FORWARD);
        servoSlide1.setDirection(CRServo.Direction.FORWARD);
        servoSlide2.setDirection(CRServo.Direction.REVERSE);

    }
    //MOVE THE SLIDES FUNCTION
    public void setSlides(double slidesPower){
        if(slidesDirection == 1){
            servoSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
            servoSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
            if(slideExtension < 120 && slideSensor.isPressed()){
                servoSlide1.setPower(0);
                servoSlide2.setPower(0);
            }
            else {
                servoSlide1.setPower(slidesPower);
                servoSlide2.setPower(slidesPower);
            }
        }

        else if (slidesDirection == -1){
            servoSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            servoSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            if(slideExtension > 150 && slideSensor.isPressed()){
                servoSlide1.setPower(0);
                servoSlide2.setPower(0);
            }
            else {
                servoSlide1.setPower(slidesPower);
                servoSlide2.setPower(slidesPower);
            }
        }

    }

    //SET DIRECTION AND THE POWER OF THE GRIPPER
    public void setGripper(double gripperPower){
        if(gripperDirection == 1) {
            servoGrippy.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if(gripperDirection == -1) {
            servoGrippy.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        servoGrippy.setPower(gripperPower);
    }

    //PID FOR MOVING THE CRANE
    public double PIDControlCrane(double craneTarget){

        double cranePos = motorCrane1.getCurrentPosition();
        double craneError = craneTarget - cranePos;
        craneIntegralSum += craneError * craneTimer.seconds();
        double craneDerivative = (craneError - craneLastError) / craneTimer.seconds();
        craneLastError = craneError;

        craneTimer.reset();

        double craneOutput = (craneError * craneP) + (craneDerivative * craneD) + (craneIntegralSum * craneI);
        return craneOutput;
    }

    //RETURNS THE POWER CALCULATED BY THE PID
    public double cranePower(double craneTarget){
        double cranePos = motorCrane1.getCurrentPosition();
        double craneFF = Math.cos(Math.toRadians((cranePos - craneOffset) / threetwelvemotorticksindegree)) * craneF;
        double cranePower = PIDControlCrane(craneTarget) + craneFF;
        return cranePower;
    }



}
