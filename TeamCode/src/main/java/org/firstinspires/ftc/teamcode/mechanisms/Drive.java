
package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Drive implements Mechanism {
    public Drive(){
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);
    }
    public final NextMotor frontLeft = new NextMotor(RobotController.controlHub(), Config.fl);
    public final NextMotor frontRight = new NextMotor(RobotController.expansionHub(), Config.fr);
    public final NextMotor backLeft = new NextMotor(RobotController.controlHub(), Config.bl);
    public final NextMotor backRight =  new NextMotor(RobotController.expansionHub(), Config.br);




}