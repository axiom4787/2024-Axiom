package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class XBoxController {
    static XboxController xboxcontroller = new XboxController(Constants.XCONTROLLER_PORT);
    
    public XBoxController(){
    }
//Dpad buttons
    public int getXboxDpad(){
        if(xboxcontroller.getPOV() != -1){
            //System.out.println(xboxcontroller.getPOV());
            return xboxcontroller.getPOV();
        }
        return -1;
    }
}
