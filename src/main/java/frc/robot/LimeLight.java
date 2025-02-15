package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: Limelight
* Variables used: table, tx, ty, ta, x, y, area 
* What it does: Puts the limelights vareables on the dashborad.
*  */

public class LimeLight {
    //creates a NetworkTable based off the limelight
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-owosso");

    //creates double varibales based of the X, Y, and Area of the limelight
    static double tx = table.getEntry("tx").getDouble(0);
    static double ty = table.getEntry("ty").getDouble(0);
    static double ta = table.getEntry("ta").getDouble(0);

    /**Method: updateTable
    * Parameters: N/A
    * Variables used: tx, ty, ta, x, y, area 
    * What it does: Puts the limelights vareables on the dashborad.
    *  */
    public static void updateTable(){
        //read values periodically
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        ta = table.getEntry("ta").getDouble(0);
        
        //post to smart dashboard periodically
          SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    /**Method: getX
    * Parameters: N/A
    * Variables used: tx
    * What it does: returns the current X value of the Limelight
    *  */
    public static double getX(){
        return tx;
    }

    /**Method: getY
    * Parameters: N/A
    * Variables used: ty
    * What it does: returns the current Y value of the Limelight
    *  */
    public static double getY(){
        return ty;
    }

    /**Method: getArea
    * Parameters: N/A
    * Variables used: ta
    * What it does: returns the current Area of the Limelight
    *  */
    public static double getArea(){
        return ta;
    }
}