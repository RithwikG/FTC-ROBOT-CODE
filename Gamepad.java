/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gamepad", group="Linear Opmode") //@Autonomous is the other common choice
//@Disabled
public class Gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    //motor power range: (-1 - 1)
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor launcher;
    DcMotor conveyor;
    DcMotor intake;
    DcMotor wobbleLifter;
    boolean increaseDown;
    boolean decreaseDown;
    boolean launcherIncreaseDown;
    boolean launcherDecreaseDown;
    boolean a1Down = false;
    Servo wobbleGrabber;
    Servo ringBlocker;
    double wobbleGrabberPosition = 1.0;
    double ringBlockerPosition = 1.0;
    BNO055IMU imu;
    Orientation angle;
    double resetPower = 0;
    double speedAdjust = 5;
    double launcherSpeedAdjust = 9;
    double lifterSpeedAdjust = 0.2;
    boolean dpadLeftDown1 = false;
    boolean dpadRightDown1 = false;
    boolean b2Down = false;



    public void turnMotor(DcMotor motor, double power){
        motor.setPower(power);
    }

    public void resetMotor(DcMotor motor){
        turnMotor(motor, 0);
    }
    public double[] rotatePoint(float x, float y, double angle){
        double[] newVals = {(x * Math.cos(angle)) - (y * Math.sin(angle)), (x * Math.sin(angle)) + (y * Math.cos(angle))};

        return newVals;
    }

    public ArrayList adjust_keys(boolean key_down, boolean key_up, double speed_adjust, boolean up_down, boolean down_down, double change, float min, float max){
        ArrayList vals = new ArrayList();
        if (key_down){
            if (!down_down) {
                if (speed_adjust > min && !down_down) {
                    speed_adjust -= change;
                    down_down = true;
                }
            }
        }
        else {
            down_down = false;
        }

        if (key_up){
            if (!up_down) {
                if (speed_adjust < max && !up_down) {
                    speed_adjust += change;
                    up_down = true;
                }
            }
        }
        else {
            up_down = false;
        }
        vals.add(up_down);
        vals.add(down_down);
        vals.add(speed_adjust);

        return vals;
    }

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        boolean autoAdjust = true;
        boolean aKeyDown = false;
        boolean outtake = false;
        boolean yKeyDown = false;
        double originalAngleX = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
        double originalAngleY = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        double originalAngleZ = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        wobbleGrabber = hardwareMap.servo.get("wobbleGrabber");
        ringBlocker = hardwareMap.servo.get("ringBlocker");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        launcher = hardwareMap.dcMotor.get("outtake");
        intake = hardwareMap.dcMotor.get("intake");
        wobbleLifter = hardwareMap.dcMotor.get("wobbleLifter");
        wobbleGrabber.setPosition(wobbleGrabberPosition);
        ringBlocker.setPosition(ringBlockerPosition);
        increaseDown = false;
        decreaseDown = false;
        launcherIncreaseDown = false;
        launcherDecreaseDown = false;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] allMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        DcMotor[] leftMotors = {frontLeftMotor, backLeftMotor};
        DcMotor[] rightMotors = {frontRightMotor, backRightMotor};

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
              angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
              double xValue = angle.firstAngle - originalAngleX;
              double yValue = angle.secondAngle - originalAngleY;
              double zValue = angle.thirdAngle - originalAngleZ;

              double[] driverSticks = {gamepad1.right_stick_x, gamepad1.right_stick_y};
              if (autoAdjust) {
                  driverSticks = rotatePoint(gamepad1.right_stick_x, gamepad1.right_stick_y, zValue);
              }
              double[] robotValues = {-gamepad2.right_stick_x, -gamepad2.right_stick_y};

              if (!outtake) {
                  robotValues[0] = gamepad2.right_stick_x;
                  robotValues[1] = gamepad2.right_stick_y;
              }

              double robotAngle = Math.atan2(robotValues[0], robotValues[1]);

              double turnAmount = zValue - robotAngle;

              if (Math.toDegrees(turnAmount) > 180){
                  turnAmount = -(180 - (turnAmount % 180));
              }
              else if (Math.toDegrees(turnAmount) < -180) {
                  turnAmount = -(turnAmount % 180);
              }

              double degreeAngle = Math.toDegrees(robotAngle);

              double degreeZ = Math.toDegrees(zValue);

              if (Math.abs(degreeAngle) != 180.0) {

                  degreeAngle = degreeAngle % 180;
              }
              if (Math.abs(degreeZ) != 180.0) {
                  degreeZ = degreeZ % 180;

              }

              if (Math.pow(gamepad2.right_stick_x, 2) + Math.pow(gamepad2.right_stick_y, 2) >= 1) {
                  if (turnAmount < 0) {
                      turnMotor(frontLeftMotor, 1 * (speedAdjust / 10));
                      turnMotor(frontRightMotor, -1 * (speedAdjust / 10));
                      turnMotor(backLeftMotor, 1 * (speedAdjust / 10));
                      turnMotor(backRightMotor, -1 * (speedAdjust / 10));
                  } else {
                      turnMotor(frontLeftMotor, -1 * (speedAdjust / 10));
                      turnMotor(frontRightMotor, 1 * (speedAdjust / 10));
                      turnMotor(backLeftMotor, -1 * (speedAdjust / 10));
                      turnMotor(backRightMotor, 1 * (speedAdjust / 10));

                  }
                  while (Math.round(degreeAngle) != Math.round(degreeZ) && opModeIsActive()) {
                      angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
                      zValue = angle.thirdAngle - originalAngleZ;
                      degreeZ = Math.toDegrees(zValue);
                      telemetry.update();
                  }
              }








//            turnMotor(backRightMotor, 0.5);
//            turnMotor(backLeftMotor, 0.5);
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.right_stick_y;
//            double turn  =  gamepad1.left_stick_x;
////            // BASIC
////
//            turnMotor(backLeftMotor, -drive);
//            turnMotor(backRightMotor, -drive);
//            turnMotor(frontLeftMotor, -drive);
//            turnMotor(frontRightMotor, -drive);
//
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//            // MECANUM
//            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
//            double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
//            double rightX = gamepad1.left_stick_x;
//            final double v1 = r * Math.cos(robotAngle) + rightX;
//            final double v2 = r * Math.sin(robotAngle) - rightX;
//            final double v3 = r * Math.sin(robotAngle) + rightX;
//            final double v4 = r * Math.cos(robotAngle) - rightX;


            if (gamepad1.a){
                if (!aKeyDown){
                    autoAdjust = ! autoAdjust;
                    aKeyDown = true;
                }
            }
            else{
                aKeyDown = false;
            }

            if (gamepad1.y){
                if (!yKeyDown){
                    outtake = ! outtake;
                    yKeyDown = true;
                }
            }
            else{
                yKeyDown = false;
            }
            if (gamepad1.dpad_up){
                if (speedAdjust < 10 && !increaseDown){
                    speedAdjust++;
                    increaseDown = true;
                }
            }
            else{
                increaseDown = false;
            }

            if (gamepad1.a){
                if (!a1Down) {
                    a1Down = true;
                    if (wobbleGrabberPosition == 1.0) {
                        wobbleGrabberPosition = 0.5;
                    } else {
                        wobbleGrabberPosition = 1.0;
                    }
                    wobbleGrabber.setPosition(wobbleGrabberPosition);
                }
            }
            else{
                a1Down = false;
            }

            if (gamepad2.b){
                if (!b2Down) {
                    b2Down = true;
                    if (ringBlockerPosition == 1.0) {
                        ringBlockerPosition = 0.5;
                    } else {
                        ringBlockerPosition = 1.0;
                    }
                    ringBlocker.setPosition(ringBlockerPosition);
                }
            }
            else{
                b2Down = false;
            }

            ArrayList values = adjust_keys(gamepad1.dpad_left, gamepad1.dpad_right, lifterSpeedAdjust, dpadLeftDown1, dpadRightDown1, 0.1, 0, 1);
            dpadLeftDown1 = (boolean) values.get(0);
            dpadRightDown1 = (boolean) values.get(1);
            lifterSpeedAdjust = (double) values.get(2);
            lifterSpeedAdjust = lifterSpeedAdjust;



            telemetry.addData("Lifter Speed Adjust", lifterSpeedAdjust);

            if (gamepad1.right_bumper){
               wobbleLifter.setPower(lifterSpeedAdjust);
            }
            else if(gamepad1.left_bumper){
                wobbleLifter.setPower(-(lifterSpeedAdjust));
            }
            else{
                wobbleLifter.setPower(0);
            }

            double finalSpeedAdjust = speedAdjust/10;
            double m1 = (driverSticks[1] + driverSticks[0] - gamepad1.left_stick_x) * (finalSpeedAdjust);
            double m2 = (driverSticks[1] - driverSticks[0] + gamepad1.left_stick_x) * (finalSpeedAdjust);
            double m3 = (driverSticks[1] - driverSticks[0] - gamepad1.left_stick_x) * (finalSpeedAdjust);
            double m4 = (driverSticks[1] + driverSticks[0] + gamepad1.left_stick_x) * (finalSpeedAdjust);

            turnMotor(frontLeftMotor, m1);
            turnMotor(frontRightMotor, m2);
            turnMotor(backLeftMotor, -m3);
            turnMotor(backRightMotor, -m4);

            if (gamepad2.right_bumper) {
                turnMotor(intake, 1);
            } else if (gamepad2.left_bumper) {
                turnMotor(intake, -1);
            }
            else {
                turnMotor(intake, 0);
            }
            turnMotor(conveyor, -gamepad2.right_trigger);
            turnMotor(conveyor, gamepad2.left_trigger);
            if (gamepad2.x) {
                turnMotor(launcher, 0.05 * launcherSpeedAdjust);
            } else if (gamepad2.y) {
                turnMotor(launcher, -0.05 * launcherSpeedAdjust);
            } else {
                turnMotor(launcher, 0);
            }
            if (gamepad2.dpad_up){
                if (launcherSpeedAdjust < 20 && !launcherIncreaseDown){
                    launcherSpeedAdjust++;
                    launcherIncreaseDown = true;
                }
            }
            else{
                launcherIncreaseDown = false;
            }
            if (gamepad2.dpad_down){
                if (launcherSpeedAdjust > 1 && !launcherDecreaseDown){
                    launcherSpeedAdjust--;
                    launcherDecreaseDown = true;
                }
            }
            else{
                launcherDecreaseDown = false;
            }

//            if (gamepad2.a){
//                if (!a2Down) {d
//                    if (wobbleGrabberPosition == 1.0) {
//                        wobbleGrabberPosition = 0.5;
//                    } else {
//                        wobbleGrabberPosition = 1.0;
//                    }
//                    a2Down = true;
//                }
//                wobbleGrabber.setPosition(wobbleGrabberPosition);
//            }
//            else{
//                a2Down = false;
//            }
//            telemetry.addData("Pos", wobbleGrabberPosition);



//            // Tank Mode uses one stick to control each wheel.
//            // - This requires no math, but it is hard to drive forward slowly and keep straight.
//            leftPower  = -gamepad1.left_stick_y ;
//            rightPower = -gamepad1.right_stick_y ;
//
//            // Send calculated power to wheels
////            leftDrive.setPower(leftPower);
////            rightDrive.setPower(rightPower);
//
//            // Show the
//            //\elapsed game time and wheel power.
            telemetry.addData("Speed", speedAdjust);
            telemetry.addData("Launcher Speed", launcherSpeedAdjust);
            telemetry.addData("Motor Speeds", m1 + " " + m2 + " " + m3 + " " + m4 + " ");
            telemetry.addData("X Rotation", xValue);
            telemetry.addData("Y Rotation", yValue);
            telemetry.addData("Z Rotation", zValue);
            telemetry.addData("Auto Adjust is ", autoAdjust);
            telemetry.addData("Pointing Towards Intake: ", outtake);
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

