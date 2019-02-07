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
  protected static final int numDec = 1000;
  protected static double rad = Math.round(RobotConfig.auto.robotRadius.getFeet()*numDec);
  protected static final Length maxY = LengthKt.getFeet((27*numDec-rad)/numDec);
  protected static final Length minY = LengthKt.getFeet((0+rad)/numDec);
  protected static final Length maxX = LengthKt.getFeet((54*numDec-rad)/numDec);
  protected static final Length minX = LengthKt.getFeet((0+rad)/numDec);

  protected static final Translation2d[] cargo = {new Translation2d(LengthKt.getFeet((18.208*numDec-rad)/numDec), LengthKt.getFeet((15.386*numDec+rad)/numDec)),
                                                  new Translation2d(LengthKt.getFeet((36.738*numDec+rad)/numDec), LengthKt.getFeet((11.717*numDec-rad)/numDec))};
  protected static final Translation2d[] rocketL = {new Translation2d(LengthKt.getFeet((17.602*numDec-rad)/numDec), LengthKt.getFeet((27*numDec+rad)/numDec)),
                                                    new Translation2d(LengthKt.getFeet((20.422*numDec+rad)/numDec), LengthKt.getFeet((24.745*numDec-rad)/numDec))};
  protected static final Translation2d[] rocketR = {new Translation2d(LengthKt.getFeet((17.602*numDec-rad)/numDec), LengthKt.getFeet((2.134*numDec+rad)/numDec)),
                                                    new Translation2d(LengthKt.getFeet((20.422*numDec+rad)/numDec), LengthKt.getFeet((0-rad)/numDec))};
  protected static final Translation2d[] upperHabaDepot = {new Translation2d(LengthKt.getFeet((0-rad)/numDec), LengthKt.getFeet((21*numDec+rad)/numDec)),
                                                            new Translation2d(LengthKt.getFeet((4*numDec+rad)/numDec), LengthKt.getFeet((6*numDec-rad)/numDec))};
  protected static final Translation2d[] habRamp = {new Translation2d(LengthKt.getFeet((4*numDec-rad)/numDec), LengthKt.getFeet((20*numDec+rad)/numDec)),
                                                    new Translation2d(LengthKt.getFeet((8*numDec+rad)/numDec), LengthKt.getFeet((7.15*numDec-rad)/numDec))};


  public static TimedTrajectory<Pose2dWithCurvature> makeSafe(TimedTrajectory<Pose2dWithCurvature> traject, boolean bigSpeed){
    List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
    List<TimedEntry<Pose2dWithCurvature>> safePoints = new ArrayList<TimedEntry<Pose2dWithCurvature>>();
    List<Translation2d[]> constraints = new ArrayList<Translation2d[]>(Arrays.asList(cargo,rocketL,rocketR,upperHabaDepot,habRamp));

    System.out.print("Maximum x: ");
    System.out.println(maxX.getFeet());
    System.out.print("Minimum x: ");
    System.out.println(minX.getFeet());
    System.out.print("Maximum y: ");
    System.out.println(maxY.getFeet());
    System.out.print("Minimum y: ");
    System.out.println(minY.getFeet());

    for(int i=0; i<points.size(); i++){
      if(i==0||i==points.size()-1){
        System.out.println("No correction required!");
        safePoints.add(points.get(i));
      }else{
        Translation2d point = points.get(i).getState().getPose().getTranslation();
        Length safeX = point.getX();
        Length safeY = point.getY();
        

        point = new Translation2d(safeX, safeY); //set the current point to the safepoint inside the field

        for(int j=0; j<constraints.size()-1; j++){
          if(!(point.getX().getFeet()>constraints.get(j)[0].getX().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet()
                &&point.getY().getFeet()>constraints.get(j)[0].getY().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet())){
            //theoretically picks the point on the border closest to the original point
            Translation2d lastNearest = constraints.get(j)[0];
            double lastShortest = distanceFormula(lastNearest, point);
            double precision;
            if(bigSpeed){
              precision=1;
            }else{
              precision=0.1;
            }
            for (double x=0; x<Math.abs(constraints.get(j)[0].getX().getFeet()-constraints.get(j)[1].getX().getFeet()); x+=precision){ //IMPORTANT this currently makes the whole thing take about
              for (double y=0; y<Math.abs(constraints.get(j)[0].getY().getFeet()-constraints.get(j)[1].getY().getFeet()); y+=precision){//20sec longer to execute. we change it to 1, it's less precise, but faster
                if(distanceFormula(point, new Translation2d(x, y))<lastShortest){lastNearest=new Translation2d(x, y);}
              }
            }
            safeX=lastNearest.getX();
            safeY=lastNearest.getY();
            System.out.println(safeX.getFeet());
            System.out.println(safeY.getFeet());
          }
        }

        point = new Translation2d(safeX, safeY); //set the current point to the safepoint inside the field


        if(point.getX().getFeet()>maxX.getFeet()){
          safeX=maxX;
          System.out.print("safed to max x. safeX: ");
          System.out.println(safeX);
        }
        if(point.getX().getFeet()<minX.getFeet()){
          safeX=minX;
          System.out.print("safed to min x. safeX: ");
          System.out.println(safeX);
        }
        if(point.getY().getFeet()>maxY.getFeet()){
          safeY=maxY;
          System.out.print("safed to max y. safeY: ");
          System.out.println(safeY);
        }
        if(point.getY().getFeet()<minY.getFeet()){
          safeY=minY;
          System.out.print("safed to min Y. safeY: ");
          System.out.println(safeY);
        }

        safePoints.add(i,new TimedEntry<Pose2dWithCurvature>((new Pose2dWithCurvature(new Pose2d(new Translation2d(safeX,safeY),points.get(i).getState().getPose().getRotation()),points.get(i).getState().getCurvature())),
                          points.get(i).getT(), points.get(i).getVelocity(), points.get(i).getAcceleration()));
      }
    }

    System.out.println(safePoints.get(0).getState().getPose().getTranslation().getX().getFeet());
    System.out.println(safePoints.get(0).getState().getPose().getTranslation().getY().getFeet());
    double[][] uno = pointsAsDoubles(safePoints);
    System.out.println(uno[0][0]);
    System.out.println(uno[0][1]);
    double[][] dos = smoother(uno,0.02, 0.98, 0.001);
    System.out.println(dos[0][0]);
    System.out.println(dos[0][1]);
    List<TimedEntry<Pose2dWithCurvature>> tres = doublesAsPoints(safePoints, dos);

    System.out.println(tres.get(0).getState().getPose().getTranslation().getX().getFeet());
    System.out.println(tres.get(0).getState().getPose().getTranslation().getY().getFeet());
    //TODO test to see if this smoother actually works
    TimedTrajectory<Pose2dWithCurvature> toReturn = new TimedTrajectory<Pose2dWithCurvature>(tres, false);
    System.out.println(toReturn.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet());
    System.out.println(toReturn.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet());
    return toReturn;

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

  //FIXME something in here makes the points Excessively Wrong
  protected static List<TimedEntry<Pose2dWithCurvature>> doublesAsPoints(List<TimedEntry<Pose2dWithCurvature>> original, double[][] newP){
    List<TimedEntry<Pose2dWithCurvature>> toReturn = new ArrayList<TimedEntry<Pose2dWithCurvature>>();

    for (int i=0; i<newP.length; i++){
        double curve=original.get(i).getState().getCurvature().get_curvature$FalconLibrary();
        if(i==0||i==newP.length-1){
          curve=0;
        }else{
          if(newP[i-1][0]==newP[i][0]){
            newP[i-1][0]+=0.001;
          }
          double k1=0.5*(Math.pow(newP[i-1][0],2)+Math.pow(newP[i-1][1],2)-Math.pow(newP[i][0],2)-Math.pow(newP[i][1],2))/(newP[i-1][0]-newP[i][0]);
          double k2=(newP[i-1][1]-newP[i][1])/(newP[i-1][0]-newP[i][0]);
          double b=0.5*(Math.pow(newP[i][0],2)-2*newP[i][0]*k1+Math.pow(newP[i][1],2)-Math.pow(newP[i+1][0],2)+2*newP[i+1][0]*k1-Math.pow(newP[i+1][1],2))
              /(newP[i+1][0]*k2-newP[i+1][1]+newP[i][1]-newP[i][1]*k2);
          double a=k1-k2*b;
          double r=Math.sqrt(Math.pow((newP[i-1][1]-a),2)+Math.pow((newP[i-1][1]-b),2));
          curve = 1/r;
        }
        //FIXME i don't know what the deriv of the curvature is, so im leaving it the same
        Pose2dCurvature newCurve = new Pose2dCurvature(curve, original.get(i).getState().getCurvature().getDkds());
        Rotation2d newRot=new Rotation2d(0);
        if(i==0){
          newRot=original.get(i).getState().getPose().getRotation(); //just set it to the original
        }else{
          newRot= new Rotation2d((newP[i-1][1]-newP[i][1])/(newP[i-1][0]-newP[i][0])); //this is just the secant between the current pt and before
        }
        toReturn.add(i,new TimedEntry<Pose2dWithCurvature>((new Pose2dWithCurvature(new Pose2d(new Translation2d(LengthKt.getFeet(newP[i][0]),LengthKt.getFeet(newP[i][1])),newRot),
              original.get(i).getState().getCurvature())), original.get(i).getT(), original.get(i).getVelocity(), original.get(i).getAcceleration()));
      

    }

    return toReturn;
  }
  public static double distanceFormula(Translation2d p1, Translation2d p2){
    return Math.sqrt(Math.abs(Math.pow(p1.getX().getFeet()-p2.getX().getFeet(),2)+Math.pow(p1.getY().getFeet()-p2.getY().getFeet(),2)));
  }
}