package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.apache.commons.math3.geometry.euclidean.threed.*;

import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName() ;
    //@Override

    public static DoubleVectorList closestPoint (List<DoubleVectorList> pointsDistances ){
        List <Double> distances = new ArrayList<>();
        for (int i=0 ; i<pointsDistances.size();i++){
            distances.add(pointsDistances.get(i).getDistance());
        }
        int minIndex = 0;
        for (int i = 1; i < distances.size(); i++) {
            if (distances.get(i) < distances.get(minIndex)) {
                minIndex = i;
            }
        }
        return pointsDistances.get(minIndex);
    }

    public static boolean isWithinBrders(Vector3D intersectionPoint , Integer bridgeNum , List<List<Vector3D>> bridgesBorders){
        System.out.println("started isWthinBorders");

        List<Vector3D> bridgeBorders = bridgesBorders.get(bridgeNum);
        boolean isWithinBrders ;
        Vector3D pointMin = bridgeBorders.get(0);
        Vector3D pointMax = bridgeBorders.get(1);
        isWithinBrders = ( intersectionPoint.getX()<pointMax.getX() && pointMin.getX() < intersectionPoint.getX()) && (( intersectionPoint.getY()<pointMax.getY() && pointMin.getY() < intersectionPoint.getY())) && ( intersectionPoint.getZ()<pointMax.getZ() && pointMin.getZ() < intersectionPoint.getZ());
        return isWithinBrders ;
    }


    public static Vector3D project(Vector3D point, Plane plane) {
        Vector3D normal = plane.getNormal();
        double distance = plane.getOffset(point);
        return point.subtract(normal.scalarMultiply(distance));
    }

    public static DoubleVectorList getDistanceOverBridge(Vector3D pt1, List<Vector3D> intersectedPoints ,List<Plane> intersectedFaces ){
        System.out.println("started getDistanceOverBridge");

        System.out.println("intersectedFaces : "+intersectedFaces.size());
        Plane face1 = intersectedFaces.get(0);
        Plane face2 = intersectedFaces.get(1);
        Vector3D faceNormal1 = intersectedFaces.get(0).getNormal();
        Vector3D faceNormal2 = intersectedFaces.get(1).getNormal();
        DoubleVectorList distanceOverBridge = new DoubleVectorList();
        if (faceNormal1.dotProduct(faceNormal2)==0){
            Vector3D pt3 = project(intersectedPoints.get(1),face2);
            double distance = pt1.distance(intersectedPoints.get(0)) + intersectedPoints.get(0).distance(pt3);
            distanceOverBridge.setDistance(distance);
            distanceOverBridge.addVector(pt1);
            distanceOverBridge.addVector(intersectedPoints.get(0));
            distanceOverBridge.addVector(pt3);

        }else{
            Vector3D pt3 = project(intersectedPoints.get(0),face1);
            Vector3D pt4 = project(intersectedPoints.get(1),face2);
            double distance = pt1.distance(intersectedPoints.get(0))+intersectedPoints.get(0).distance(pt3)+pt3.distance(pt4);
            distanceOverBridge.setDistance(distance);
            distanceOverBridge.addVector(pt1);
            distanceOverBridge.addVector(intersectedPoints.get(0));
            distanceOverBridge.addVector(pt3);
            distanceOverBridge.addVector(pt4);
        }
        Double distance = distanceOverBridge.getDistance();
        List<Vector3D> points = distanceOverBridge.getVectorList();
        System.out.println("distanceOverBridge :"+distance);
        System.out.println(points);

        return distanceOverBridge;
    }

    public static boolean pointBelongsToSegment(Vector3D pt1, Vector3D pt2, Vector3D intersecionPoint) {
        System.out.println("started pointBelongsTSegment");

        Vector3D segmentVector = pt2.subtract(pt1);
        Vector3D pointVector = intersecionPoint.subtract(pt1);
        double segmentLengthSquared = segmentVector.getNormSq();
        double dotProduct = pointVector.dotProduct(segmentVector);
        boolean pointBelongsToSegment = dotProduct > 0 && dotProduct < segmentLengthSquared;
        return pointBelongsToSegment;
    }


    public static Integer closestBridgeNum (Vector3D pt1 , Line segment , List<String> intersectedBridges ,  List<List<Plane>> bridgesFaces ){
        System.out.println("started closestBridgeNum");

        List<Integer> bridges = new ArrayList<>();
        List<Integer> faces = new ArrayList<>();
        for (String str : intersectedBridges) {
            String[] list =str.split("/");
            bridges.add(Integer.parseInt(list[0]));
            faces.add(Integer.parseInt(list[1]));
        }
        if (intersectedBridges.size()==1){
            return bridges.get(0);
        }else {
            List<Vector3D> intersectedPoints = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            for (int i =0 ; i<intersectedBridges.size() ;i++ ){
                Vector3D point = bridgesFaces.get(bridges.get(i)).get(faces.get(i)).intersection(segment);
                double distance = pt1.distance(point);
                intersectedPoints.add(point);
                distances.add(distance);
            }

            int minIndex = 0;
            for (int i = 1; i < distances.size(); i++) {
                if (distances.get(i) < distances.get(minIndex)) {
                    minIndex = i;
                }
            }
            return bridges.get(minIndex);
        }
    }

    public static List<String> intersectedBridges (Vector3D pt1, Vector3D pt2,Line segment , List<List<Plane>> bridgesFaces , List<List<Vector3D>> bridgesBorders ) {
        System.out.println("started intersectedBridges");
        List<String> intersectedBridges = new ArrayList<>() ;
        Vector3D origin = new Vector3D(0,0,0);
        for ( int i =0 ; i<5 ; i++){
            boolean isIntersected = false;
            int j =0 ;
            while (!isIntersected && j<6){
                System.out.println("checking");
                Vector3D intersectionPoint = bridgesFaces.get(i).get(j).intersection(segment);
                if ((intersectionPoint!=origin ) && (isWithinBrders(intersectionPoint , i , bridgesBorders )) && (pointBelongsToSegment(pt1, pt2, intersectionPoint )) ){
                    isIntersected = true ;
                    intersectedBridges.add(i+"/"+j);
                    System.out.println("intersected bridge num : "+i+"face num: "+j);
                }
                j++;
            }
        }
        System.out.println( "intersected bridges :" + intersectedBridges);
        return intersectedBridges;
    }


    public static DoubleVectorList getShortestDistance(Vector3D pt1 , Vector3D pt2 , List<List<Plane>> bridgesFaces , List<List<Vector3D>> bridgesBorders , Boolean firstCalculation ) {
        System.out.println("started getShortestDistance");
        Line segment = new Line(pt1 , pt2);
        Vector3D origin = new Vector3D(0,0,0);
        //Segment segment1 = new Segment(pt1 , pt2 , segment);
        List<String> intersectedBridges = intersectedBridges(pt1,pt2,segment , bridgesFaces , bridgesBorders);
        System.out.println(intersectedBridges);
        DoubleVectorList shortestDistance = new DoubleVectorList();

        if (intersectedBridges.isEmpty()) {
            if (firstCalculation){
                shortestDistance.setDistance(pt1.distance(pt2));
                System.out.println("done");
                shortestDistance.addVector(pt1);
                System.out.println("done");
                shortestDistance.addVector(pt2);
                System.out.println("done");
                System.out.println("i am here");

            }else{
                shortestDistance.setDistance(pt1.distance(pt2));
                shortestDistance.addVector(pt2);
            }
        }
        else {
            Integer closestBridgeNum = closestBridgeNum(pt1 , segment , intersectedBridges ,  bridgesFaces);
            System.out.println("closestBridgeNum : "+closestBridgeNum);
            List<Plane> intersectedFaces = new ArrayList<>();
            List<Vector3D> intersectedPoints = new ArrayList<>();
            for (int i = 0; i < 6; i++) {
                Vector3D intersecionPoint = bridgesFaces.get(closestBridgeNum).get(i).intersection(segment);
                if ((intersecionPoint != origin)  && (!intersectedPoints.contains(intersecionPoint))) {
                    intersectedFaces.add(bridgesFaces.get(closestBridgeNum).get(i));
                    intersectedPoints.add((intersecionPoint));
                    System.out.println("intersected faces : "+ intersectedFaces);
                    System.out.println("intersected Points :" + intersectedPoints);
                    System.out.println("YES");
                }
                else
                {
                    System.out.println("NO");
                }
            }
            if (intersectedPoints.size()==1 &&intersectedBridges.size()==1 ){
                shortestDistance.setDistance(pt1.distance(pt2));
                if (firstCalculation){
                    shortestDistance.addVector(pt1);
                    shortestDistance.addVector(pt2);
                }else{
                    shortestDistance.addVector(pt2);
                }

            }else if (intersectedPoints.size()==1 &&intersectedBridges.size()>1){
                Double distance = pt1.distance(intersectedPoints.get(0));
                shortestDistance.addVector(pt1);
                shortestDistance.addVector(intersectedPoints.get(0));
                shortestDistance.setDistance(distance+ getShortestDistance(intersectedPoints.get(0), pt2 ,bridgesFaces,bridgesBorders, false).getDistance());
            }else {
                DoubleVectorList distanceOverBridge = getDistanceOverBridge(pt1, intersectedPoints ,intersectedFaces );
                double distance = distanceOverBridge.getDistance();
                List<Vector3D> Points = distanceOverBridge.getVectorList();
                int l = Points.size()-1;
                for (Vector3D point : Points){
                    shortestDistance.addVector(point);
                }
                shortestDistance.setDistance(distance+ getShortestDistance(Points.get(l), pt2 ,bridgesFaces,bridgesBorders, false).getDistance());
                for (Vector3D point : getShortestDistance(Points.get(l), pt2 ,bridgesFaces,bridgesBorders, false).getVectorList() ){
                    shortestDistance.addVector(point);
                }
            }
        }
        return shortestDistance ;
    }

    public static Integer getPointIndex (Point point , List<Point> Points){
        int pointIndex =-1 ;
        Boolean found = false;
        int i =0 ;
        while (!found && i<Points.size()){
            if (Points.get(i) == point ){
                pointIndex = i;
                found= true;
            }
            i++;
        }
        return  pointIndex;
    }

    public static Plane[] getFaces (double xMin , double xMax, double yMin, double yMax, double zMin, double zMax) {
        Vector3D v1 = new Vector3D(xMin, yMin, zMin);
        Vector3D v2 = new Vector3D(xMax, yMin, zMin);
        Vector3D v3 = new Vector3D(xMax, yMax, zMin);
        Vector3D v4 = new Vector3D(xMin, yMax, zMin);
        Vector3D v5 = new Vector3D(xMin, yMin, zMax);
        Vector3D v6 = new Vector3D(xMax, yMin, zMax);
        Vector3D v7 = new Vector3D(xMax, yMax, zMax);
        Vector3D v8 = new Vector3D(xMin, yMax, zMax);

        Plane[] faces = new Plane[6];
        faces[0] = new Plane(v1, v2, v3); // Bottom face
        faces[1] = new Plane(v5, v6, v7); // Top face
        faces[2] = new Plane(v1, v5, v6); // Side face 1
        faces[3] = new Plane(v2, v6, v7); // Side face 2
        faces[4] = new Plane(v3, v7, v8); // Side face 3
        faces[5] = new Plane(v4, v8, v5); // Side face 4

        return faces;
    }

    protected void runPlan1(){
        List<List<Plane>> bridgesFaces = new ArrayList<>();
        bridgesFaces.add(Arrays.asList(getFaces(12.783, 13.071, -21.8899, -21.6929, 16.8385, 17.0665)));
        bridgesFaces.add(Arrays.asList(getFaces(12.8652, 12.9628, -21.0734, -20.7314, 16.3861, 16.6401)));
        bridgesFaces.add(Arrays.asList(getFaces(12.185, 13.665, -20.3826, -20.2826, 16.1475, 16.6725)));
        bridgesFaces.add(Arrays.asList(getFaces(12.7955, 13.3525, -20.0635, -19.7305, 17.1055, 17.1305)));
        bridgesFaces.add(Arrays.asList(getFaces(12.563, 12.709, -19.1449, -18.8099, 16.6544, 16.8164)));

        Vector3D pnt  = new Vector3D(34.783,-27.8899,26.8385);
        Vector3D pnt1 = new Vector3D(31.071,-30.6929,25.0665);
        List<List<Vector3D>> bridgesBorders = new ArrayList<>();
        // Bridge 1
        List<Vector3D> bridge1 = new ArrayList<>();
        bridge1.add(new Vector3D(12.783, -21.8899, 16.8385));
        bridge1.add(new Vector3D(13.071, -21.6929, 17.0665));
        bridgesBorders.add(bridge1);
        // Bridge 2
        List<Vector3D> bridge2 = new ArrayList<>();
        bridge2.add(new Vector3D(12.8652, -21.0734, 16.3861));
        bridge2.add(new Vector3D(12.9628, -20.7314, 16.6401));
        bridgesBorders.add(bridge2);

        // Bridge 3
        List<Vector3D> bridge3 = new ArrayList<>();
        bridge3.add(new Vector3D(12.185, -20.3826, 16.1475));
        bridge3.add(new Vector3D(13.665, -20.2826, 16.6725));
        bridgesBorders.add(bridge3);

        // Bridge 4
        List<Vector3D> bridge4 = new ArrayList<>();
        bridge4.add(new Vector3D(12.7955, -20.0635, 17.1055));
        bridge4.add(new Vector3D(13.3525, -18.8099, 17.1305));
        bridgesBorders.add(bridge4);

        // Bridge 5
        List<Vector3D> bridge5 = new ArrayList<>();
        bridge5.add(new Vector3D(12.563, -19.1449, 16.6544));
        bridge5.add(new Vector3D(12.709, -18.8099, 16.8164));
        bridgesBorders.add(bridge5);


        List<Point> Points = new ArrayList<>();
        Points.add(new Point(11.381944, -8.566172, 3.76203));
        Points.add(new Point(11.2746, -9.92284, 5.2988));
        Points.add(new Point(10.612, -9.0709, 4.48));
        Points.add(new Point(10.71, -7.7, 4.48));
        Points.add(new Point(10.51, -6.7185, 5.1804));
        Points.add(new Point(11.114, -7.9756, 5.3393));
        Points.add(new Point(11.355, -8.9929, 4.7818));
        Points.add(new Point(11.369, -8.5518, 4.48));

        List<Quaternion> Quaternions = new ArrayList<>();
        Quaternions.add(new Quaternion(0f, 0f, 0f, 1f));
        Quaternions.add(new Quaternion(0f, 0f, -0.707f, 0.707f));
        Quaternions.add(new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
        Quaternions.add(new Quaternion(0f, 0.707f, 0f, 0.707f));
        Quaternions.add(new Quaternion(0f, 0f, -1f, 0f));
        Quaternions.add(new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f));
        Quaternions.add(new Quaternion(0f, 0f, 0f, 1f));
        Quaternions.add(new Quaternion(0f, 0.707f, 0f, 0.707f));

        Point goal = new Point( 11.143 , -6.7607 , 4.9654);
        Quaternion goalQuaternions = new Quaternion(0f ,0f, -0.707f ,0.707f);
        Vector3D goalVector =  new Vector3D(goal.getX(),goal.getY(),goal.getZ());
        Log.i(TAG, "start mission") ;
        // the mission starts
        api.startMission();
        List<Integer> activeTargets = api.getActiveTargets();
        while (activeTargets.isEmpty()){
            activeTargets = api.getActiveTargets();
        }
        while ( api.getTimeRemaining().get(1) > 30000) {
            activeTargets = api.getActiveTargets();
            System.out.println("active targets : "+activeTargets);
            List<Point> activePoints = new ArrayList<>();
            Point point = api.getRobotKinematics().getPosition();
            Vector3D currentPosition = new Vector3D(point.getX(), point.getY(), point.getZ());
            for (int i : activeTargets) {
                activePoints.add(Points.get(i - 1));
            }
            List<DoubleVectorList> pointsDistances = new ArrayList<>();
            for (Point pt : activePoints) {
                Vector3D activePoint = new Vector3D(pt.getX(), pt.getY(), pt.getZ());
                DoubleVectorList shortestDistance = getShortestDistance(currentPosition, activePoint, bridgesFaces, bridgesBorders, true);
                pointsDistances.add(shortestDistance);
            }
            DoubleVectorList closestPoint = closestPoint(pointsDistances);
            List<Vector3D> vectorsPath = closestPoint.getVectorList();
            System.out.println("distance : "+closestPoint.getDistance());
            System.out.println("vector path : "+vectorsPath);
            List<Point> pointsPath = new ArrayList<>();
            for (Vector3D vector : vectorsPath) {
                Point pt = new Point(vector.getX(), vector.getY(), vector.getZ());
                pointsPath.add(pt);
            }
            Integer index = getPointIndex(pointsPath.get(pointsPath.size() - 1), Points);
            System.out.println("index :" +index);

            for (Point pt : pointsPath) {
                Result result = api.moveTo(pt, Quaternions.get(index), true);
                System.out.println("point: " + point +"result : "+result);
                final int LOOP_MAX = 7;
                int loopCounter = 0;
                while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
                    //retry
                    result = api.moveTo(pt, Quaternions.get(index), true);
                    ++loopCounter;
                }
            }
            Mat image = api.getMatNavCam();
            api.saveMatImage ( image ,"file_name.png");
            api.laserControl(true);
           // api.takeTargetSnapshot(i);
            List<Long> timeRemaining = api.getTimeRemaining();

            // check the remaining milliseconds of mission time
            if (timeRemaining.get(1) < 30000) break;

        }
        Point point = api.getRobotKinematics().getPosition();
        Vector3D currentPosition = new Vector3D(point.getX(), point.getY(), point.getZ());
        DoubleVectorList shortestDistance = getShortestDistance(currentPosition, goalVector, bridgesFaces, bridgesBorders, true);
        List<Vector3D> vectorsPath = shortestDistance.getVectorList();
        List<Point> pointsPath = new ArrayList<>();
        for (Vector3D vectort : vectorsPath) {
            Point pt = new Point(vectort.getX(), vectort.getY(), vectort.getZ());
            pointsPath.add(pt);
        }
        for (Point pt : pointsPath) {
            Result result = api.moveTo(pt, goalQuaternions, true);
            final int LOOP_MAX = 7;
            int loopCounter = 0;
            while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
                //retry
                result = api.moveTo(pt, goalQuaternions, true);
                ++loopCounter;
            }
        }
        String mQrContent = yourMethod();
        api.reportMissionCompletion(mQrContent);


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
    private String yourMethod(){
        return "your method";
    }
}
