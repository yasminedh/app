package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.util.ArrayList;
import java.util.List;

public class DoubleVectorList {
    private Double distance;
    private List<Vector3D> vectorList;

    public  DoubleVectorList() {
        this.vectorList = new ArrayList<>();
    }
    public void setDistance(Double distance) {
        this.distance = distance;
    }
    public void addVector(Vector3D vector) {
        vectorList.add(vector);
    }
    public Double getDistance() {
        return distance;
    }
    public List<Vector3D> getVectorList() {
        return vectorList;
    }
}