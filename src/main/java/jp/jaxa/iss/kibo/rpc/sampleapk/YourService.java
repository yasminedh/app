package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;
import java.lang.Math;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.apache.commons.io.filefilter.TrueFileFilter;
import org.apache.commons.math3.geometry.euclidean.threed.*;

import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import org.opencv.objdetect.QRCodeDetector;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName() ;
    //@Override
    public  Mat  doTheQrStuff (){
        api.flashlightControlFront(0.05f);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            // Handle the exception if necessary
            e.printStackTrace();
        }
        Mat image = api.getMatNavCam();
        while (image == null) {
            image = api.getMatNavCam();
        }

        api.saveMatImage(image, "img.png");
        api.flashlightControlFront(0.00f);
        return image;

    }

    public static double calculateDistance(Point pt1 , Point pt2) {
        double dx = pt1.getX()- pt2.getX();
        double dy = pt1.getY()- pt2.getY();
        double dz = pt1.getZ()- pt2.getZ();

        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        return distance;
    }
    public static Integer getClosestPoint(List<Integer> activeTargets , List<DistancePointList> pointsAndDistances ){
        System.out.println("looking for the closest point");
        int minIndex = activeTargets.get(0);
        for(Integer index : activeTargets ) {
            if (pointsAndDistances.get(index).getDistance() < pointsAndDistances.get(minIndex).getDistance()) {
                minIndex = index;
            }
        }
        System.out.println("got the closest point : "+minIndex);
        return minIndex;
    }

    protected void runPlan1() {


        Point start = new Point(9.815, -9.806, 4.293);
        Point goal = new Point(11.143, -6.7607, 4.9654);
        Quaternion goalQuaternions = new Quaternion(0f, 0f, -0.707f, 0.707f);

        Point infrontOfQr = new Point(11.369, -8.5518, 4.48);
        Point infrontOfTarget1 = new Point(11.204d, -9.754899999999916d, 5.47d);
        Point infrontOfTarget2 = new Point(10.35488d, -9.192672d, 4.631600000000089d);
        Point infrontOfTarget3 = new Point(10.7106d, -7.76857d, 4.720400000000062d);
        Point infrontOfTarget4 = new Point(10.736600000000063d, -6.615472d, 5.20281d);
        Point infrontOfTarget5 = new Point(11.0435d, -7.9229d, 5.4276d);
        Point infrontOfTarget6 = new Point(11.167599999999924d, -9.0475d, 4.938d);

        List<Point> Points = new ArrayList<>();
        Points.add(new Point(11.369, -8.5518, 4.48));
        Points.add(new Point(11.204, -9.754899999999916, 5.47));
        Points.add(new Point(10.35488, -9.192672, 4.631600000000089));
        Points.add(new Point(10.7106, -7.76857, 4.720400000000062));
        Points.add(new Point(10.736600000000063, -6.615472, 5.20281));
        Points.add(new Point(11.0435, -7.9229, 5.4276));
        Points.add(new Point(11.167599999999924, -9.0475, 4.938));

        List<Quaternion> Quaternions = new ArrayList<>();
        Quaternions.add(new Quaternion(0f, 0.707f, 0f, 0.707f));
        Quaternions.add(new Quaternion(0f, 0f, -0.707f, 0.707f));
        Quaternions.add(new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
        Quaternions.add(new Quaternion(0f, 0.707f, 0f, 0.707f));
        Quaternions.add(new Quaternion(0f, 0f, -1f, 0f));
        Quaternions.add(new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f));

        List<List<DistancePointList>> pointsAndDistances = new ArrayList<>();

        //Qr
        List<DistancePointList> QrPoints = new ArrayList<>();
        DistancePointList Qr_Qr = new DistancePointList();
        Qr_Qr.setDistance(0.0);
        Qr_Qr.addPoint(new Point(0, 0, 0));
        QrPoints.add(Qr_Qr);
        DistancePointList Qr_1 = new DistancePointList();
        Qr_1.addPoint(new Point(11.323028, -8.788241, 5.141627));
        Qr_1.addPoint(infrontOfTarget1);
        Double distance1 = calculateDistance(infrontOfQr, Qr_1.getPointsList().get(0)) + calculateDistance(Qr_1.getPointsList().get(0), infrontOfTarget1);
        Qr_1.setDistance(distance1);
        QrPoints.add(Qr_1);
        DistancePointList Qr_2 = new DistancePointList();
        Qr_2.addPoint(new Point(10.781596, -8.66281, 5.035678));
        Qr_2.addPoint(infrontOfTarget2);
        Double distance2 = calculateDistance(infrontOfQr, Qr_2.getPointsList().get(0)) + calculateDistance(Qr_2.getPointsList().get(0), infrontOfTarget2);
        Qr_2.setDistance(distance2);
        QrPoints.add(Qr_2);
        DistancePointList Qr_3 = new DistancePointList();
        Qr_3.addPoint(new Point(11.183367, -8.716602, 5.371128));
        Qr_3.setDistance(calculateDistance(infrontOfQr, Qr_3.getPointsList().get(0)) + calculateDistance(Qr_3.getPointsList().get(0), infrontOfTarget3));
        QrPoints.add(Qr_3);
        DistancePointList Qr_4 = new DistancePointList();
        Qr_4.addPoint(new Point(11.183367, -8.716602, 5.371128));
        Qr_4.setDistance(calculateDistance(infrontOfQr, Qr_4.getPointsList().get(0)) + calculateDistance(Qr_4.getPointsList().get(0), infrontOfTarget4));
        QrPoints.add(Qr_4);
        DistancePointList Qr_5 = new DistancePointList();
        Qr_5.addPoint(new Point(11.183367, -8.716602, 5.371128));
        Qr_5.setDistance(calculateDistance(infrontOfQr, Qr_5.getPointsList().get(0)) + calculateDistance(Qr_5.getPointsList().get(0), infrontOfTarget5));
        QrPoints.add(Qr_5);
        DistancePointList Qr_6 = new DistancePointList();
        Qr_6.setDistance(calculateDistance(infrontOfQr, infrontOfTarget6));
        Qr_6.addPoint(infrontOfTarget6);
        QrPoints.add(Qr_6);
        DistancePointList Qr_start = new DistancePointList();
        Qr_start.setDistance(0.0);
        Qr_start.addPoint(new Point(0, 0, 0));
        QrPoints.add(Qr_start);
        DistancePointList Qr_end = new DistancePointList();
        Qr_end.setDistance(0.0);
        Qr_end.addPoint(new Point(0, 0, 0));
        QrPoints.add(Qr_end);

        pointsAndDistances.add(QrPoints);

        //Target1
        List<DistancePointList> Target1Points = new ArrayList<>();
        DistancePointList T1_Qr = new DistancePointList();
        T1_Qr.addPoint(new Point(11.323028, -8.788241, 5.141627));
        T1_Qr.addPoint(infrontOfQr);
        T1_Qr.setDistance(distance1);
        Target1Points.add(T1_Qr);
        DistancePointList T1_T1 = new DistancePointList();
        T1_T1.setDistance(0.0);
        T1_T1.addPoint(new Point(0, 0, 0));
        Target1Points.add(T1_T1);
        DistancePointList T1_T2 = new DistancePointList();
        T1_T2.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget2));
        T1_T2.addPoint(infrontOfTarget2);
        Target1Points.add(T1_T2);
        DistancePointList T1_T3 = new DistancePointList();
        T1_T3.addPoint(new Point(10.893754, -7.969446, 4.748193));
        T1_T3.addPoint(infrontOfTarget3);
        T1_T3.setDistance(calculateDistance(infrontOfTarget1, T1_T3.getPointsList().get(0)) + calculateDistance(T1_T3.getPointsList().get(0), infrontOfTarget3));
        Target1Points.add(T1_T3);
        DistancePointList T1_T4 = new DistancePointList();
        T1_T4.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget4));
        T1_T4.addPoint(infrontOfTarget4);
        Target1Points.add(T1_T4);
        DistancePointList T1_T5 = new DistancePointList();
        T1_T5.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget5));
        T1_T5.addPoint(infrontOfTarget5);
        Target1Points.add(T1_T5);
        DistancePointList T1_T6 = new DistancePointList();
        T1_T6.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget6));
        T1_T6.addPoint(infrontOfTarget6);
        Target1Points.add(T1_T6);
        DistancePointList T1_start = new DistancePointList();
        T1_start.setDistance(0.0);
        T1_start.addPoint(new Point(0, 0, 0));
        Target1Points.add(T1_start);
        DistancePointList T1_goal = new DistancePointList();
        T1_goal.addPoint(new Point(11.204, -8.542963, 4.923365));
        T1_goal.addPoint(goal);
        T1_goal.setDistance(calculateDistance(infrontOfTarget1, T1_goal.getPointsList().get(0)) + calculateDistance(T1_goal.getPointsList().get(0), goal));
        Target1Points.add(T1_goal);

        pointsAndDistances.add(Target1Points);

        //target 2
        List<DistancePointList> Target2Points = new ArrayList<>();
        DistancePointList T2_Qr = new DistancePointList();
        T2_Qr.addPoint(new Point(10.781596, -8.66281, 5.035678));
        T2_Qr.addPoint(infrontOfQr);
        T2_Qr.setDistance(distance2);
        Target2Points.add(T2_Qr);
        DistancePointList T2_T1 = new DistancePointList();
        T2_T1.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget2));
        T2_T1.addPoint(infrontOfTarget1);
        Target2Points.add(T2_T1);
        DistancePointList T2_T2 = new DistancePointList();
        T2_T2.setDistance(0.0);
        T2_T2.addPoint(new Point(0, 0, 0));
        Target2Points.add(T2_T2);
        DistancePointList T2_T3 = new DistancePointList();
        T2_T3.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T2_T3.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T2_T3.addPoint(infrontOfTarget3);
        T2_T3.setDistance(calculateDistance(infrontOfTarget2, T2_T3.getPointsList().get(0)) + calculateDistance(T2_T3.getPointsList().get(0), T2_T3.getPointsList().get(1)) + calculateDistance(T2_T3.getPointsList().get(1), infrontOfTarget3));
        Target2Points.add(T1_T3);
        DistancePointList T2_T4 = new DistancePointList();
        T2_T4.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T2_T4.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T2_T4.addPoint(infrontOfTarget4);
        T2_T4.setDistance(calculateDistance(infrontOfTarget2, T2_T4.getPointsList().get(0)) + calculateDistance(T2_T4.getPointsList().get(0), T2_T4.getPointsList().get(1)) + calculateDistance(T2_T4.getPointsList().get(1), infrontOfTarget4));
        Target2Points.add(T2_T4);
        DistancePointList T2_T5 = new DistancePointList();
        T2_T5.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T2_T5.addPoint(infrontOfTarget6);
        T2_T5.setDistance(calculateDistance(infrontOfTarget2, T2_T5.getPointsList().get(0)) + calculateDistance(T2_T5.getPointsList().get(0), infrontOfTarget5));
        Target2Points.add(T2_T5);
        DistancePointList T2_T6 = new DistancePointList();
        T2_T6.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T2_T6.addPoint(infrontOfTarget6);
        T2_T6.setDistance(calculateDistance(infrontOfTarget2, T2_T6.getPointsList().get(0)) + calculateDistance(T2_T6.getPointsList().get(0), infrontOfTarget6));
        Target2Points.add(T2_T6);
        DistancePointList T2_start = new DistancePointList();
        T2_start.setDistance(0.0);
        T2_start.addPoint(new Point(0, 0, 0));
        Target2Points.add(T2_start);
        DistancePointList T2_goal = new DistancePointList();
        T2_goal.addPoint(new Point(10.612, -8.866519, 4.976946));
        T2_goal.addPoint(new Point(11.204, -8.542963, 4.923365));
        T2_goal.addPoint(goal);
        T2_goal.setDistance(calculateDistance(infrontOfTarget2, T2_goal.getPointsList().get(0)) + calculateDistance(T2_goal.getPointsList().get(0), T2_goal.getPointsList().get(1)) + calculateDistance(T2_goal.getPointsList().get(1), goal));
        Target2Points.add(T2_goal);

        pointsAndDistances.add(Target2Points);

        // Target 3
        List<DistancePointList> Target3Points = new ArrayList<>();
        DistancePointList T3_Qr = new DistancePointList();
        T3_Qr.addPoint((new Point(11.183367, -8.716602, 5.371128)));
        T3_Qr.addPoint(infrontOfQr);
        T3_Qr.setDistance(calculateDistance(infrontOfQr, T3_Qr.getPointsList().get(0)) + calculateDistance(T3_Qr.getPointsList().get(0), infrontOfTarget3));
        Target3Points.add(T3_Qr);
        DistancePointList T3_T1 = new DistancePointList();
        T3_T1.addPoint(new Point(10.893754, -7.969446, 4.748193));
        T3_T1.addPoint(infrontOfTarget1);
        T3_T1.setDistance(calculateDistance(infrontOfTarget1, T1_T3.getPointsList().get(0)) + calculateDistance(T1_T3.getPointsList().get(0), infrontOfTarget3));
        Target3Points.add(T3_T1);
        DistancePointList T3_T2 = new DistancePointList();
        T3_T2.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T3_T2.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T3_T2.addPoint(infrontOfTarget2);
        T3_T2.setDistance(calculateDistance(infrontOfTarget2, T2_T3.getPointsList().get(0)) + calculateDistance(T2_T3.getPointsList().get(0), T2_T3.getPointsList().get(1)) + calculateDistance(T2_T3.getPointsList().get(1), infrontOfTarget3));
        Target3Points.add(T3_T2);
        DistancePointList T3_T3 = new DistancePointList();
        T3_T3.setDistance(0.0);
        T3_T3.addPoint(new Point(0, 0, 0));
        Target3Points.add(T3_T3);
        Target3Points.add(T3_T3);
        DistancePointList T3_T4 = new DistancePointList();
        T3_T4.addPoint(new Point(10.71, -7.143459, 5.151553));
        T3_T4.addPoint(infrontOfTarget4);
        T3_T4.setDistance(calculateDistance(infrontOfTarget3, T3_T4.getPointsList().get(0)) + calculateDistance(T3_T4.getPointsList().get(0), infrontOfTarget4));
        Target3Points.add(T3_T4);
        DistancePointList T3_T5 = new DistancePointList();
        T3_T5.addPoint(new Point(11.110324, -7.494255, 5.292847));
        T3_T5.addPoint(infrontOfTarget5);
        T3_T5.setDistance(calculateDistance(infrontOfTarget3, T3_T5.getPointsList().get(0)) + calculateDistance(T3_T5.getPointsList().get(0), infrontOfTarget5));
        Target3Points.add(T3_T5);
        DistancePointList T3_T6 = new DistancePointList();
        T3_T6.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T3_T6.addPoint(infrontOfTarget6);
        T3_T6.setDistance(calculateDistance(infrontOfTarget3, T3_T6.getPointsList().get(0)) + calculateDistance(T3_T6.getPointsList().get(0), infrontOfTarget6));
        Target3Points.add(T3_T6);
        DistancePointList T3_start = new DistancePointList();
        T3_start.setDistance(0.0);
        T3_start.addPoint(new Point(0, 0, 0));
        Target3Points.add(T3_start);
        DistancePointList T3_goal = new DistancePointList();
        T3_goal.addPoint(goal);
        T3_goal.setDistance(calculateDistance(infrontOfTarget3, goal));
        Target3Points.add(T3_goal);

        pointsAndDistances.add(Target3Points);

        //Target4
        List<DistancePointList> Target4Points = new ArrayList<>();
        DistancePointList T4_Qr = new DistancePointList();
        T4_Qr.addPoint(new Point(11.183367, -8.716602, 5.371128));
        T4_Qr.addPoint(infrontOfQr);
        T4_Qr.setDistance(calculateDistance(infrontOfQr, T4_Qr.getPointsList().get(0)) + calculateDistance(T4_Qr.getPointsList().get(0), infrontOfTarget4));
        Target4Points.add(T4_Qr);
        DistancePointList T4_T1 = new DistancePointList();
        T4_T1.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget4));
        T4_T1.addPoint(infrontOfTarget1);
        Target4Points.add(T4_T1);
        DistancePointList T4_T2 = new DistancePointList();
        T4_T2.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T4_T2.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T4_T2.addPoint(infrontOfTarget2);
        T4_T2.setDistance(calculateDistance(infrontOfTarget4, T4_T2.getPointsList().get(0)) + calculateDistance(T4_T2.getPointsList().get(0), T4_T2.getPointsList().get(1)) + calculateDistance(T4_T2.getPointsList().get(1), infrontOfTarget2));
        Target4Points.add(T3_T2);
        DistancePointList T4_T3 = new DistancePointList();
        T4_T3.addPoint(new Point(10.71, -7.143459, 5.151553));
        T4_T3.addPoint(infrontOfTarget3);
        T4_T3.setDistance(calculateDistance(infrontOfTarget3, T4_T3.getPointsList().get(0)) + calculateDistance(T4_T3.getPointsList().get(0), infrontOfTarget4));
        Target4Points.add(T4_T3);
        DistancePointList T4_T4 = new DistancePointList();
        T4_T4.setDistance(0.0);
        T4_T4.addPoint(new Point(0, 0, 0));
        Target4Points.add(T4_T4);
        DistancePointList T4_T5 = new DistancePointList();
        T4_T5.addPoint(infrontOfTarget5);
        T4_T5.setDistance(calculateDistance(infrontOfTarget4, infrontOfTarget5));
        Target4Points.add(T4_T5);
        DistancePointList T4_T6 = new DistancePointList();
        T4_T6.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T4_T6.addPoint(infrontOfTarget6);
        T4_T6.setDistance(calculateDistance(infrontOfTarget6, T4_T6.getPointsList().get(0)) + calculateDistance(T4_T6.getPointsList().get(0), infrontOfTarget4));
        Target4Points.add(T4_T6);
        DistancePointList T4_start = new DistancePointList();
        T4_start.setDistance(0.0);
        T4_start.addPoint(new Point(0, 0, 0));
        Target4Points.add(T4_start);
        DistancePointList T4_goal = new DistancePointList();
        T4_goal.addPoint(goal);
        T4_goal.setDistance(calculateDistance(infrontOfTarget4, goal));
        Target4Points.add(T4_goal);

        pointsAndDistances.add(Target4Points);

        //Target5
        List<DistancePointList> Target5Points = new ArrayList<>();
        DistancePointList T5_Qr = new DistancePointList();
        T5_Qr.addPoint(new Point(11.183367, -8.716602, 5.371128));
        T5_Qr.addPoint(infrontOfQr);
        T5_Qr.setDistance(calculateDistance(infrontOfQr, T5_Qr.getPointsList().get(0)) + calculateDistance(T5_Qr.getPointsList().get(0), infrontOfTarget5));
        Target5Points.add(T5_Qr);
        DistancePointList T5_T1 = new DistancePointList();
        T5_T1.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget5));
        T5_T1.addPoint(infrontOfTarget1);
        Target5Points.add(T5_T1);
        DistancePointList T5_T2 = new DistancePointList();
        T5_T2.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T5_T2.addPoint(infrontOfTarget2);
        T5_T2.setDistance(calculateDistance(infrontOfTarget2, T5_T2.getPointsList().get(0)) + calculateDistance(T5_T2.getPointsList().get(0), infrontOfTarget5));
        Target5Points.add(T5_T2);
        DistancePointList T5_T3 = new DistancePointList();
        T5_T3.addPoint(new Point(11.110324, -7.494255, 5.292847));
        T5_T3.addPoint(infrontOfTarget3);
        T5_T3.setDistance(calculateDistance(infrontOfTarget3, T5_T3.getPointsList().get(0)) + calculateDistance(T5_T3.getPointsList().get(0), infrontOfTarget5));
        Target5Points.add(T5_T3);
        DistancePointList T5_T4 = new DistancePointList();
        T5_T4.addPoint(infrontOfTarget4);
        T5_T4.setDistance(calculateDistance(infrontOfTarget4, infrontOfTarget5));
        Target5Points.add(T5_T4);
        DistancePointList T5_T5 = new DistancePointList();
        T5_T5.setDistance(0.0);
        T5_T5.addPoint(new Point(0, 0, 0));
        Target5Points.add(T5_T5);
        DistancePointList T5_T6 = new DistancePointList();
        T5_T6.addPoint(new Point(11.195232, -8.363515, 5.298435));
        T5_T6.addPoint(infrontOfTarget6);
        T5_T6.setDistance(calculateDistance(infrontOfTarget6, T5_T6.getPointsList().get(0)) + calculateDistance(T5_T6.getPointsList().get(0), infrontOfTarget5));
        Target5Points.add(T5_T6);
        DistancePointList T5_start = new DistancePointList();
        T5_start.setDistance(0.0);
        T5_start.addPoint(new Point(0, 0, 0));
        Target5Points.add(T5_start);
        DistancePointList T5_goal = new DistancePointList();
        T5_goal.addPoint(new Point(11.064038, -7.321528, 5.281515));
        T5_goal.addPoint(goal);
        T5_goal.setDistance(calculateDistance(infrontOfTarget5, T5_goal.getPointsList().get(0)) + calculateDistance(T5_goal.getPointsList().get(0), goal));
        Target5Points.add(T5_goal);

        pointsAndDistances.add(Target5Points);

        //Target6
        List<DistancePointList> Target6Points = new ArrayList<>();
        DistancePointList T6_Qr = new DistancePointList();
        T6_Qr.setDistance(calculateDistance(infrontOfQr, infrontOfTarget6));
        T6_Qr.addPoint(infrontOfQr);
        Target6Points.add(T6_Qr);
        DistancePointList T6_T1 = new DistancePointList();
        T6_T1.setDistance(calculateDistance(infrontOfTarget1, infrontOfTarget6));
        T6_T1.addPoint(infrontOfTarget1);
        Target6Points.add(T6_T1);
        DistancePointList T6_T2 = new DistancePointList();
        T6_T2.addPoint(new Point(10.91861, -9.060668, 4.924728));
        T6_T2.addPoint(infrontOfTarget2);
        T6_T2.setDistance(calculateDistance(infrontOfTarget2, T6_T2.getPointsList().get(0)) + calculateDistance(T6_T2.getPointsList().get(0), infrontOfTarget6));
        Target6Points.add(T6_T2);
        DistancePointList T6_T3 = new DistancePointList();
        T6_T3.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T6_T3.addPoint(infrontOfTarget3);
        T6_T3.setDistance(calculateDistance(infrontOfTarget3, T6_T3.getPointsList().get(0)) + calculateDistance(T6_T3.getPointsList().get(0), infrontOfTarget6));
        Target6Points.add(T6_T3);
        DistancePointList T6_T4 = new DistancePointList();
        T6_T4.addPoint(new Point(10.91861, -7.847378, 4.802304));
        T6_T4.addPoint(infrontOfTarget4);
        T6_T4.setDistance(calculateDistance(infrontOfTarget6, T6_T4.getPointsList().get(0)) + calculateDistance(T6_T4.getPointsList().get(0), infrontOfTarget4));
        Target6Points.add(T6_T4);
        DistancePointList T6_T5 = new DistancePointList();
        T6_T5.addPoint(new Point(11.195232, -8.363515, 5.298435));
        T6_T5.addPoint(infrontOfTarget5);
        T6_T5.setDistance(calculateDistance(infrontOfTarget6, T6_T5.getPointsList().get(0)) + calculateDistance(T6_T5.getPointsList().get(0), infrontOfTarget5));
        Target6Points.add(T6_T5);
        DistancePointList T6_T6 = new DistancePointList();
        T6_T6.setDistance(0.0);
        T6_T6.addPoint(new Point(0, 0, 0));
        Target6Points.add(T6_T6);
        DistancePointList T6_start = new DistancePointList();
        T6_start.setDistance(0.0);
        T6_start.addPoint(new Point(0, 0, 0));
        Target6Points.add(T6_start);
        DistancePointList T6_goal = new DistancePointList();
        T6_goal.addPoint(new Point(11.204, -8.542963, 4.923365));
        T6_goal.addPoint(goal);
        T6_goal.setDistance(calculateDistance(infrontOfTarget6, T6_goal.getPointsList().get(0)) + calculateDistance(T6_goal.getPointsList().get(0), goal));
        Target6Points.add(T6_goal);

        pointsAndDistances.add(Target6Points);

        // Start
        List<DistancePointList> startPoints = new ArrayList<>();
        DistancePointList start_Qr = new DistancePointList();
        start_Qr.addPoint(new Point(10.923028, -8.788241, 5.141627));
        start_Qr.addPoint(infrontOfQr);
        start_Qr.setDistance(calculateDistance(start, start_Qr.getPointsList().get(0)) + calculateDistance(start_Qr.getPointsList().get(0), infrontOfQr));
        startPoints.add(start_Qr);
        DistancePointList start_T1 = new DistancePointList();
        start_T1.setDistance(calculateDistance(infrontOfTarget1, start));
        start_T1.addPoint(infrontOfTarget1);
        startPoints.add(start_T1);
        DistancePointList start_T2 = new DistancePointList();
        start_T2.addPoint(new Point(10.385574, -9.695185, 4.502076));
        start_T2.addPoint(infrontOfTarget2);
        start_T2.setDistance(calculateDistance(infrontOfTarget2, start_T2.getPointsList().get(0)) + calculateDistance(start_T2.getPointsList().get(0), start));
        startPoints.add(start_T2);
        DistancePointList start_T3 = new DistancePointList();
        start_T3.addPoint(new Point(10.458145, -9.623698, 4.958129));
        start_T3.addPoint(new Point(10.458145, -7.993161, 4.958129));
        start_T3.addPoint(infrontOfTarget3);
        start_T3.setDistance(calculateDistance(start, start_T3.getPointsList().get(0)) + calculateDistance(start_T3.getPointsList().get(0), start_T3.getPointsList().get(1)) + calculateDistance(start_T3.getPointsList().get(1), infrontOfTarget3));
        startPoints.add(start_T3);
        DistancePointList start_T4 = new DistancePointList();
        start_T4.addPoint(new Point(10.385574, -9.695185, 4.502076));
        start_T4.addPoint(infrontOfTarget4);
        start_T4.setDistance(calculateDistance(start, start_T4.getPointsList().get(0)) + calculateDistance(start_T4.getPointsList().get(0), infrontOfTarget4));
        startPoints.add(start_T4);
        DistancePointList start_T5 = new DistancePointList();
        start_T5.addPoint(new Point(10.385574, -9.695185, 4.502076));
        start_T5.addPoint(infrontOfTarget5);
        start_T5.setDistance(calculateDistance(start, start_T5.getPointsList().get(0)) + calculateDistance(start_T5.getPointsList().get(0), infrontOfTarget5));
        startPoints.add(start_T5);
        DistancePointList start_T6 = new DistancePointList();
        start_T6.addPoint(new Point(10.385574, -9.695185, 4.502076));
        start_T6.addPoint(infrontOfTarget6);
        start_T6.setDistance(calculateDistance(infrontOfTarget6, start_T6.getPointsList().get(0)) + calculateDistance(start_T6.getPointsList().get(0), start));
        startPoints.add(start_T6);
        DistancePointList start_start = new DistancePointList();
        start_start.setDistance(0.0);
        start_start.addPoint(new Point(0, 0, 0));
        startPoints.add(start_start);
        DistancePointList start__goal = new DistancePointList();
        start__goal.setDistance(0.0);
        start__goal.addPoint(new Point(0, 0, 0));
        startPoints.add(start__goal);

        pointsAndDistances.add(startPoints);

        //goal
        List<DistancePointList> goalPoints = new ArrayList<>();
        DistancePointList goal_Qr = new DistancePointList();
        goal_Qr.setDistance(0.0);
        goal_Qr.addPoint(new Point(0, 0, 0));
        goalPoints.add(goal_Qr);
        DistancePointList goal_T1 = new DistancePointList();
        goal_T1.addPoint(new Point(11.204, -8.542963, 4.923365));
        goal_T1.addPoint(goal);
        goal_T1.setDistance(calculateDistance(infrontOfTarget1, goal_T1.getPointsList().get(0)) + calculateDistance(goal_T1.getPointsList().get(0), goal));
        goalPoints.add(goal_T1);
        DistancePointList goal_T2 = new DistancePointList();
        goal_T2.addPoint(new Point(11.204, -8.542963, 4.923365));
        goal_T2.addPoint(new Point(10.612, -8.866519, 4.976946));
        goal_T2.addPoint(goal);
        goal_T2.setDistance(calculateDistance(infrontOfTarget2, goal_T2.getPointsList().get(0)) + calculateDistance(goal_T2.getPointsList().get(0), goal_T2.getPointsList().get(1)) + calculateDistance(goal_T2.getPointsList().get(1), goal));
        goalPoints.add(goal_T2);

        DistancePointList goal_T3 = new DistancePointList();
        goal_T3.addPoint(goal);
        goal_T3.setDistance(calculateDistance(infrontOfTarget3, goal));
        goalPoints.add(goal_T3);
        DistancePointList goal_T4 = new DistancePointList();
        goal_T4.addPoint(goal);
        goal_T4.setDistance(calculateDistance(infrontOfTarget4, goal));
        goalPoints.add(goal_T4);
        DistancePointList goal_T5 = new DistancePointList();
        goal_T5.addPoint(new Point(11.064038, -7.321528, 5.281515));
        goal_T5.addPoint(goal);
        goal_T5.setDistance(calculateDistance(infrontOfTarget5, goal_T5.getPointsList().get(0)) + calculateDistance(goal_T5.getPointsList().get(0), goal));
        goalPoints.add(goal_T5);
        DistancePointList goal_T6 = new DistancePointList();
        goal_T6.addPoint(new Point(11.204, -8.542963, 4.923365));
        goal_T6.addPoint(goal);
        goal_T6.setDistance(calculateDistance(infrontOfTarget6, goal_T6.getPointsList().get(0)) + calculateDistance(goal_T6.getPointsList().get(0), goal));
        goalPoints.add(goal_T6);
        DistancePointList goal_start = new DistancePointList();
        T6_start.setDistance(0.0);
        T6_start.addPoint(new Point(0, 0, 0));
        goalPoints.add(goal_start);
        DistancePointList goal_goal = new DistancePointList();
        T6_T6.setDistance(0.0);
        T6_T6.addPoint(new Point(0, 0, 0));
        goalPoints.add(goal_goal);
        pointsAndDistances.add(goalPoints);

        List<Integer> numbers = new ArrayList<>();
        numbers.add(1);
        numbers.add(2);
        numbers.add(6);


        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
        List<Integer> activeTargets = api.getActiveTargets();
        Boolean scanned = false;
        String QrContent = "";
        Integer closetPointIndex0 = 0;

        //starting phase 1
        //going from start the closest active target or to the Qr code
        List<Integer> closeActiveTargets = new ArrayList<>();
        for (Integer i : activeTargets) {
            if (numbers.contains(i)) {
                closeActiveTargets.add(i);
            }
        }
        System.out.println("ActiveTargets : " + activeTargets);
        System.out.println("closeActiveTargets : " + closeActiveTargets);
        if (closeActiveTargets.size() == 0) {
            System.out.println("its the Qr");
            //if that's the case means the active targets are ab3ad mel Qr
            Mat image = doTheQrStuff();
            QrContent = QrReader(image);
            scanned = true;
            closetPointIndex0 = 0;
        } else {
            boolean firstTime = true;
            while (closeActiveTargets.size() > 0) {
                System.out.println("closeActiveTargets : " + closeActiveTargets);
                if (firstTime) {
                    closetPointIndex0 = getClosestPoint(closeActiveTargets, pointsAndDistances.get(7));
                    System.out.println("moving to the closest point");
                    for (Point point : pointsAndDistances.get(7).get(closetPointIndex0).getPointsList()) {
                        Result pointResult0 = api.moveTo(point, Quaternions.get(closetPointIndex0), true);
                        System.out.println("moved");
                        firstTime = false;
                    }
                } else {
                    Integer closetPointIndex = getClosestPoint(closeActiveTargets, pointsAndDistances.get(closetPointIndex0));
                    System.out.println("moving to the closest point");

                    for (Point point : pointsAndDistances.get(closetPointIndex0).get(closetPointIndex).getPointsList()) {
                        Result pointResulti = api.moveTo(point, Quaternions.get(closetPointIndex), true);
                        System.out.println("moved");
                    }
                    closetPointIndex0 = closetPointIndex;
                }
                System.out.println("its a target");
                Result shot = api.laserControl(true);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    // Handle the exception if necessary
                    e.printStackTrace();
                }
                shot = api.laserControl(false);
                closeActiveTargets.remove(closetPointIndex0);
            }
            for (Point point : pointsAndDistances.get(closetPointIndex0).get(0).getPointsList()) {
                Result pointResultt = api.moveTo(point, Quaternions.get(closetPointIndex0), true);
                System.out.println("moved to Qr");
            }
            System.out.println("its the Qr");
            //if that's the case means the active targets are ab3ad mel Qr
            Mat image = doTheQrStuff();
            QrContent = QrReader(image);
            scanned = true;
            closetPointIndex0 = 0;
        }
        while (api.getTimeRemaining().get(0) > 20000) {
            activeTargets = api.getActiveTargets();
            Integer closetPointIndex = getClosestPoint(activeTargets, pointsAndDistances.get(closetPointIndex0));
            Double distance = pointsAndDistances.get(closetPointIndex0).get(closetPointIndex).getDistance();
            //checking if the time taken by astroobe to get to the next target is > than the time remaining for the 1st phase : mayouselch fel wa9t --> zeyed aka break
            //reminder : durée = distance/vitesse , noting that vitesse astrobee=0.5m/s , getTimeRemaining taati bel ms
            if (api.getTimeRemaining().get(1) >60000 || activeTargets.isEmpty()  ) {
                continue;
            } else if (distance * 2000 > api.getTimeRemaining().get(1)) {
                continue;
            } else if (api.getTimeRemaining().get(1) <15000 && activeTargets.isEmpty()) {
                for (Point point : pointsAndDistances.get(closetPointIndex0).get(3).getPointsList()) {
                    Result goalResult = api.moveTo(point, Quaternions.get(3), true);
                }
            } else {
                for (Point point : pointsAndDistances.get(closetPointIndex0).get(closetPointIndex).getPointsList()) {
                    Result goalResult = api.moveTo(point, Quaternions.get(closetPointIndex), true);
                }
                Result shot = api.laserControl(true);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    // Handle the exception if necessary
                    e.printStackTrace();
                }
                shot = api.laserControl(false);
                
                closetPointIndex0 = closetPointIndex;
            }
        }

        //reporting  the mission
        for (Point point : pointsAndDistances.get(closetPointIndex0).get(8).getPointsList()) {
            Result goalResult = api.moveTo(point, goalQuaternions, true);
        }
        api.reportMissionCompletion(QrContent);


    }


    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private String QrReader(Mat image){
        String QRcode_content = "";
        try {
            api.saveMatImage(image, "QR.png");
            Mat mini_image = new Mat(image, new Rect(700, 360, 240, 240)); // ここの値は切り取る領域
            api.saveMatImage(mini_image, "QR_mini.png");

            MatOfPoint2f points = new MatOfPoint2f();
            Mat straight_qrcode = new Mat();
            QRCodeDetector qrc_detector = new QRCodeDetector();
            Boolean detect_success = qrc_detector.detect(mini_image, points);
            Log.i(TAG, "detect_success is " + detect_success.toString());

            QRcode_content = qrc_detector.detectAndDecode(mini_image, points, straight_qrcode);
            Log.i(TAG, "QRCode_content is " + QRcode_content);
            if (QRcode_content != null) {
                Mat straight_qrcode_gray = new Mat();
                straight_qrcode.convertTo(straight_qrcode_gray, CvType.CV_8UC1);
                api.saveMatImage(straight_qrcode_gray, "QR_binary.png");
            }

        } catch (Exception e) {
            ;
        }
        /**
         * QRCode_CONTENT to REPORT_MESSEGE
         */
        switch (QRcode_content) {
            case "JEM":
                QRcode_content = "STAY_AT_JEM";
                break;
            case "COLUMBUS":
                QRcode_content = "GO_TO_COLUMBUS";
                break;
            case "RACK1":
                QRcode_content = "CHECK_RACK_1";
                break;
            case "ASTROBEE":
                QRcode_content = "I_AM_HERE";
                break;
            case "INTBALL":
                QRcode_content = "LOOKING_FORWARD_TO_SEE_YOU";
                break;
            case "BLANK":
                QRcode_content = "NO_PROBLEM";
                break;
            default:
                QRcode_content = "";
                break;
        }
        return QRcode_content;
    }
}
