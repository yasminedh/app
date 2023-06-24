package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import java.util.ArrayList;
import java.util.List;

public class DistancePointList {
    private Double distance;
    private List<Point> pointsList;

    public  DistancePointList() {
        this.pointsList = new ArrayList<>();
    }
    public void setDistance(Double distance) {
        this.distance = distance;
    }
    public void addPoint(Point point) {
        pointsList.add(point);
    }
    public Double getDistance() {
        return distance;
    }
    public List<Point> getPointsList() {
        return pointsList;
    }
}