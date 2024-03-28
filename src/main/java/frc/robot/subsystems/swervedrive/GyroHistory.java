package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;

import java.util.ArrayList;

public class GyroHistory {
    private double[][] gyroData = new double[50][2];
    //private ArrayList<Pair<>> gyroList = new ArrayList<25>;
    private SwerveDrive swerveDrive;
    
    public GyroHistory(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }
    public void recordGyroData() {
        for(int x = gyroData.length-2; x>=0; x--) {
        gyroData[x+1][0] = gyroData[x][0];
        gyroData[x+1][1] = gyroData[x][1];
        }
        gyroData[0][0] = swerveDrive.field.getRobotPose().getRotation().getDegrees();
        //gyroData[0][0] = (Timer.getFPGATimestamp()*325)%360;
        gyroData[0][1] = Timer.getFPGATimestamp();
    }


    public Rotation2d fetchInterpolatedGyroData(double wantedTimestamp) {

    int bestID = -1;
    double bestDiff = Double.MAX_VALUE; // Initialize with maximum value

     // Find the index of the closest timestamp
    for(int x = 0; x < gyroData.length; x++) {
        double diff = Math.abs(gyroData[x][1] - wantedTimestamp);
        if(diff < bestDiff) {
            bestDiff = diff;
            bestID = x;
        }
    }
    //System.out.println("the closest real data point is array id " + bestID);
    // System.out.println("searching for: " + wantedTimestamp);

    //this will return the raw value if its on the edge, which should never happen, but saftey
    if (bestID == 0 || bestID == gyroData.length - 1) {
      return Rotation2d.fromDegrees(gyroData[bestID][0]);
    }

    double prevTimestamp = gyroData[bestID - 1][1];
    double nextTimestamp = gyroData[bestID + 1][1];
    double prevValue = gyroData[bestID - 1][0];
    double nextValue = gyroData[bestID + 1][0];
    // System.out.println("values nearby: " + prevValue + ", " + gyroData[bestID][0] + ", " + nextValue);
    // System.out.println("times nearby: " + prevTimestamp + ", " + gyroData[bestID][1] + ", " + nextTimestamp);

    if ((Math.abs(prevValue-nextValue))>100) {
      return Rotation2d.fromDegrees(gyroData[bestID][0]);
    }

    double interpolatedValue = interpolate(prevTimestamp, prevValue, nextTimestamp, nextValue, wantedTimestamp);
    // System.out.println("interpreted value is " + interpolatedValue);
    // System.out.println("---");
    return Rotation2d.fromDegrees(interpolatedValue);
  }

  private double interpolate(double x0, double y0, double x1, double y1, double x) {
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
  }
}