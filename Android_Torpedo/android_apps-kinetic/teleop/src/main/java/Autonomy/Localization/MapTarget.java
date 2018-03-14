package Autonomy.Localization;

import org.ros.rosjava_geometry.Vector3;

/**
 * Created by meskupie on 13/03/18.
 */

public class MapTarget {
    private int id;

    private double x;
    private double y;
    private double z;
    private Vector3 point;

    public MapTarget(double _x, double _y, double _z, int _id) {
        x = _x;
        y = _y;
        z = _z;
        point = new Vector3(x,y,z);
        id = _id;
    }

    // Accessors
    public int getTargetId(){return id;}
    public Vector3  getPoint(){return point;}
    public double getX(){return x;}
    public double getY(){return y;}
    public double getZ(){return z;}
}
