package Autonomy.Localization;

/**
 * Created by meskupie on 13/03/18.
 */

public class MapTarget {
    private int id;

    private double x;
    private double y;
    private double z;

    public MapTarget(double _x, double _y, double _z, int _id) {
        x = _x;
        y = _y;
        z = _z;
        id = _id;
    }

    // Accessors
    public int getTargetId(){return id;}
    public double getX(){return x;}
    public double getY(){return y;}
    public double getZ(){return z;}
}
