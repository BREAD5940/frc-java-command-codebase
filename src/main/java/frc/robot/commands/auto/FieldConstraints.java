package frc.robot.commands.auto;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;

import frc.robot.RobotConfig;

public class FieldConstraints {
  protected static double rad = RobotConfig.auto.robotRadius.getFeet();
  protected static final Length maxY = LengthKt.getFeet(27-rad);
  protected static final Length minY = LengthKt.getFeet(0+rad);
  protected static final Length maxX = LengthKt.getFeet(54-rad);
  protected static final Length minX = LengthKt.getFeet(0+rad);

  protected static final Translation2d[] cargo = {new Translation2d(LengthKt.getFeet(18.208-rad), LengthKt.getFeet(15.386+rad)),
                                                  new Translation2d(LengthKt.getFeet(36.738+rad), LengthKt.getFeet(11.717-rad))};
  protected static final Translation2d[] rocketL = {new Translation2d(LengthKt.getFeet(17.602-rad), LengthKt.getFeet(27+rad)),
                                                    new Translation2d(LengthKt.getFeet(20.422+rad), LengthKt.getFeet(24.745-rad))};
  protected static final Translation2d[] rocketR = {new Translation2d(LengthKt.getFeet(17.602-rad), LengthKt.getFeet(2.134+rad)),
                                                    new Translation2d(LengthKt.getFeet(20.422+rad), LengthKt.getFeet(0-rad))};
  protected static final Translation2d[] upperHabaDepot = {new Translation2d(LengthKt.getFeet(0-rad), LengthKt.getFeet(21+rad)),
                                                            new Translation2d(LengthKt.getFeet(4+rad), LengthKt.getFeet(6-rad))};
  protected static final Translation2d[] habRamp = {new Translation2d(LengthKt.getFeet(4-rad), LengthKt.getFeet(20+rad)),
                                                    new Translation2d(LengthKt.getFeet(8+rad), LengthKt.getFeet(7.15-rad))};

  protected static boolean isSafe(TimedTrajectory<Pose2dWithCurvature> traject){
    List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
    List<Translation2d[]> constraints = new ArrayList<Translation2d[]>(Arrays.asList(cargo,rocketL,rocketR,upperHabaDepot,habRamp));
    if(isOutsideField(traject)){
      return false; //see comment below
    }
    for(int j=0; j<constraints.size()-1; j++){
      for (int i=0; i<points.size()-1; i++){
        Translation2d point = points.get(i).getState().getPose().getTranslation();
        //translation array cycles topLeft->bottomRight
        if(!(point.getX().getFeet()>constraints.get(j)[0].getX().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet()
              &&point.getY().getFeet()>constraints.get(j)[0].getY().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet())){
          return false; //this is an if statement bc it loops again if it's not false. it's this or a while loop
        }
      }
    }
    return true;
  }

  protected static boolean isOutsideField(TimedTrajectory<Pose2dWithCurvature> traject){
    List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();

    for(int i=0; i<points.size()-1; i++){
      Translation2d point = points.get(i).getState().getPose().getTranslation();
      if(point.getX().getFeet()>maxX.getFeet()
          || point.getX().getFeet()<minX.getFeet()
          || point.getY().getFeet()>maxY.getFeet()
          || point.getY().getFeet()<minY.getFeet()){return true;}
    }
    return false;
  }

  public static TimedTrajectory<Pose2dWithCurvature> makeSafe(TimedTrajectory<Pose2dWithCurvature> traject){
    List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
    List<TimedEntry<Pose2dWithCurvature>> safePoints = new ArrayList<TimedEntry<Pose2dWithCurvature>>();
    List<Translation2d[]> constraints = new ArrayList<Translation2d[]>(Arrays.asList(cargo,rocketL,rocketR,upperHabaDepot,habRamp));
    if(isSafe(traject)){
      System.out.println("Trajectory is already safe!");
      return traject;
    }

    for(int i=0; i<points.size()-1; i++){
      Translation2d point = points.get(i).getState().getPose().getTranslation();
      Length safeX = point.getX();
      Length safeY = point.getY();
      if(point.getX().getFeet()>maxX.getFeet()){safeX=maxX;}
      if(point.getX().getFeet()<minX.getFeet()){safeX=minX;}
      if(point.getY().getFeet()>maxY.getFeet()){safeY=maxY;}
      if(point.getY().getFeet()<minY.getFeet()){safeY=minY;}

      point = new Translation2d(safeX, safeY); //set the current point to the safepoint inside the field

      for(int j=0; j<constraints.size()-1; j++){
        if(!(point.getX().getFeet()>constraints.get(j)[0].getX().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet()
              &&point.getY().getFeet()>constraints.get(j)[0].getY().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet())){
          //theoretically picks the point on the border closest to the original point
          Translation2d lastNearest = constraints.get(j)[0];
          double lastShortest = distanceFormula(lastNearest, point);
          for (double x=0; x<Math.abs(constraints.get(j)[0].getX().getFeet()-constraints.get(j)[1].getX().getFeet()); x+=0.01){
            for (double y=0; y<Math.abs(constraints.get(j)[0].getY().getFeet()-constraints.get(j)[1].getY().getFeet()); y+=0.01){
              if(distanceFormula(point, new Translation2d(x, y))<lastShortest){lastNearest=new Translation2d(x, y);}
            }
          }
          safeX=lastNearest.getX();
          safeY=lastNearest.getY();
        }
      }


      safePoints.add(i,new TimedEntry<Pose2dWithCurvature>((new Pose2dWithCurvature(new Pose2d(new Translation2d(safeX,safeY),points.get(i).getState().getPose().getRotation()),points.get(i).getState().getCurvature())),
                        points.get(i).getT(), points.get(i).getVelocity(), points.get(i).getAcceleration()));
    }

    //TODO test to see if this smoother actually works
    return new TimedTrajectory<Pose2dWithCurvature>(doublesAsPoints(safePoints, smoother(pointsAsDoubles(safePoints),0.02, 0.98, 0.001)), false);
  }

  // TODO make me use Translation2ds instead of doubles[][] // yeah we tried that and it died
  // furthermore Falconlib should do this somehow, right?
  
  protected static double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance){
    //copy array
    double[][] newPath = doubleArrayCopy(path);

    double change = tolerance;
    while(change >= tolerance){
      change = 0.0;
      for(int i=1; i<path.length-1; i++)
          for(int j=0; j<path[i].length; j++){
            double aux = newPath[i][j];
            newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
            change += Math.abs(aux - newPath[i][j]);	
          }					
    }

    return newPath;
  }

  protected static double[][] doubleArrayCopy(double[][] arr){

    //size first dimension of array
    double[][] temp = new double[arr.length][arr[0].length];

    for(int i=0; i<arr.length; i++){
      //Resize second dimension of array
      temp[i] = new double[arr[i].length];

      //Copy Contents
      for(int j=0; j<arr[i].length; j++)
        temp[i][j] = arr[i][j];
    }

    return temp;

  }


  protected static double[][] pointsAsDoubles(List<TimedEntry<Pose2dWithCurvature>> points){
    double[][] toreturn = new double[points.size()][2];

    for (int i=0; i<points.size(); i++){
        toreturn[i][0]=points.get(i).getState().getPose().getTranslation().getX().getFeet();
        toreturn[i][1]=points.get(i).getState().getPose().getTranslation().getY().getFeet();
    }

    return toreturn;
  }
  // FIXME because I"ll probubly break the rotation2d component of the pose. Solution is to approximate tangent line slopes
  // or just only run known good paths /shrug
  protected static List<TimedEntry<Pose2dWithCurvature>> doublesAsPoints(List<TimedEntry<Pose2dWithCurvature>> original, double[][] newP){
    List<TimedEntry<Pose2dWithCurvature>> toReturn = new ArrayList<TimedEntry<Pose2dWithCurvature>>();

    for (int i=0; i<newP.length-1; i++){
      toReturn.add(i,new TimedEntry<Pose2dWithCurvature>((new Pose2dWithCurvature(new Pose2d(new Translation2d(newP[i][0],newP[i][1]),original.get(i).getState().getPose().getRotation()),
            original.get(i).getState().getCurvature())),
            original.get(i).getT(), original.get(i).getVelocity(), original.get(i).getAcceleration()));
    }
    //FIXME I have none idea how this works
    new Pose2dWithCurvature(new Pose2d(new Translation2d(), new Rotation2d(0)), new Pose2dCurvature(_curvature, dkds));

    return toReturn;
  }
  public static double distanceFormula(Translation2d p1, Translation2d p2){
    return Math.sqrt(Math.abs(Math.pow(p1.getX().getFeet()-p2.getX().getFeet(),2)+Math.pow(p1.getY().getFeet()-p2.getY().getFeet(),2)));
  }
}