/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Telop", group="Linear OpMode")
public class BasicTelop extends LinearOpMode {

    // Declare OpMode members for each motor
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor larmExtensionMotor = null;
    private DcMotor rarmExtensionMotor = null;
    private DcMotor langleMotor = null;
    private DcMotor rangleMotor = null;

    private Servo armPivot = null;
    private Servo rClaw = null;
    private Servo lClaw = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // will eentually have to name this system but not today. 
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        lExtensionMotor  = hardwareMap.get(DcMotor.class, "lem");
        rExtensionMotor  = hardwareMap.get(DcMotor.class, "rem");
        lAngleMotor  = hardwareMap.get(DcMotor.class, "lam");
        rAngleMotor  = hardwareMap.get(DcMotor.class, "ram");
        
        //vslidesMotor = hardwareMap.get(DcMotor.class, "vslides");
        elbow = hardware.get(Servo.class, "aP");
        lClaw = hardware.get(Servo.class, "lc");
        rClaw = hardware.get(Servo.class, "rc");

        //initialize motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rExtensionMotor.setDirection(DcMotor.Direction.FORWARD);
        lExtensionMotor.setDirection(DcMotor.Direction.REVERSE);
        rAngleMotor.setDirection(DcMotor.Direction.FORWARD);
        lAngleMotor.setDirection(DcMotor.Direction.FORWARD); // can't tell what reverse is basically.
        
        //initialize toggle servos (servos that go between angles at the press of a button)
        sleep(10);

        ToggleServo leftClawServo = new ToggleServo(lClaw, new int[]{15, 215}, Servo.Direction.FORWARD);
        ToggleServo rightClawServo = new ToggleServo(rClaw, new int[]{15, 215}, Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //previous button states
        boolean lb1Pressed = false;
        boolean rb1Pressed = false;
        boolean b1Pressed = false;
        boolean a1Pressed = false;
        boolean x1Pressed = false;
        boolean y1Pressed = false;
        boolean down1Pressed = false;
        boolean up1Pressed = false;
        boolean right1Pressed = false;
        boolean left1Pressed = false;

        boolean lb2Pressed = false;
        boolean rb2Pressed = false;
        boolean b2Pressed = false;
        boolean a2Pressed = false;
        boolean x2Pressed = false;
        boolean y2Pressed = false;
        boolean down2Pressed = false;
        boolean up2Pressed = false;
        boolean right2Pressed = false;
        boolean left2Pressed = false;

        //other variables used in teleop
        // gamepad 1 is for basically all possible arm and angle and grabbing, gamepad 2 is for all motion and driving. 
        boolean intake = false;
        double driveSensitivity = 1;
        int left_angle_dir = 1;
        int right_angle_dir = -1;
        boolean angle_brake = false;
        boolean extension_brake = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y*.95;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x*.95;
            double yaw     =  gamepad1.right_stick_x*.95;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if(gamepad1.left_bumper && gamepad1.right_bumper) driveSensitivity = 0.4;
            else driveSensitivity = 1;
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * driveSensitivity);
            rightFrontDrive.setPower(rightFrontPower * driveSensitivity);
            leftBackDrive.setPower(leftBackPower * driveSensitivity);
            rightBackDrive.setPower(rightBackPower * driveSensitivity);

            //motor intake
            if (gamepad2.x && intake) {
                leftClawServo.toggle();
                rightClawServe.toggle();
                intake = false;
            }
            else if (gamepad2.x && !intake) {
                leftClawServo.toggle();
                rightClawServe.toggle();
                intake = true;
            }

            if(gamepad2.right_bumper && !rb12ressed){
                left_angle_dir = 1;
                right_angle_dir = -1;
                rb1Pressed = true;
            }
            else if (gamepad2.right_bumper && rb2Pressed) {
                left_angle_dir = -1;
                right_angle_dir = 1;
                rb1Pressed = false;
            }
            if(gamepad2.left_bumper && gamepad2.right_bumper){
                if (!angle_brake) {
                    lAngleMotor.setPower(0);
                    rAngleMotor.setPower(0);
                    angle_brake = true;
                }
                else {
                    angle_brake = false;
                }
            }
            if(gamepad2.right_trigger > 0.1){
                // basically have the DC motor go based on the gampad max value... i assume its one....
                static int MAX_RIGHT_TRIGGER = 1; //hoenstly i have no clue baically.
                    // now scale the thing till it reaches the value. 
                if (!angle_brake) {
                    lAngleMotor.setPower(left_angle_dir * gamepad2.right_trigger/MAX_RIGHT_TRIGGER);
                    lAngleMotor.setPower(right_angle_dir * gamepad2.right_trigger/MAX_RIGHT_TRIGGER);
                }
            }

            if (gamepad2.y && !extension_brake) {
                // should i hard code a max extension? 
                // why not honestly.
                // send it
                lExtensionMotor.setPower(1);
                rExtensionMotor.setPower(1);
            }
            if (gamepad2.a && extension_brake) {
                lExtensionMotor.setPower(-1);
                rExtensionMotor.setPower(-1);
            }
            
           

            //update all button states
            b1Pressed = gamepad1.b;
            a1Pressed = gamepad1.a;
            x1Pressed = gamepad1.x;
            y1Pressed = gamepad1.y;
            down1Pressed = gamepad1.dpad_down;
            up1Pressed = gamepad1.dpad_up;
            left1Pressed = gamepad1.dpad_left;
            right1Pressed = gamepad1.dpad_right;
            lb1Pressed = gamepad1.left_bumper;
            rb1Pressed = gamepad1.right_bumper;

            b2Pressed = gamepad2.b;
            a2Pressed = gamepad2.a;
            x2Pressed = gamepad2.x;
            y2Pressed = gamepad2.y;
            down2Pressed = gamepad2.dpad_down;
            up2Pressed = gamepad2.dpad_up;
            left2Pressed = gamepad2.dpad_left;
            right2Pressed = gamepad2.dpad_right;
            lb2Pressed = gamepad2.left_bumper;
            rb2Pressed = gamepad2.right_bumper;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("ArmL", larm.getServo().getPosition());
            telemetry.addData("ArmR", rarm.getServo().getPosition());
            telemetry.addData("Elbow", elbow.getServo().getPosition());
            telemetry.addData("pivotPos", rpivot.getServo().getPosition());
            telemetry.addData("larm", larm.getServo().getPosition());
            telemetry.addData("rarm", rarm.getServo().getPosition());
            telemetry.addData("elbownum", elbow.pos);
            telemetry.addData("perpTicks", perp.getPositionAndVelocity().position);
            telemetry.addData("par0Ticks", par1.getPositionAndVelocity().position);
            telemetry.update();
        }
    }}
