package Autonomy.Localization;

/**
 * Created by meskupie on 13/03/18.
 */

// Camera targets class
public class CameraTarget {
    private int id;

    private double altitude;
    private double azimuth;

    private boolean hasCorrespondence;
    private MapTarget correspondence;

    public CameraTarget(double _altitude, double _azimuth, int _id) {
        altitude = _altitude;
        azimuth = _azimuth;
        id = _id;
        hasCorrespondence = false;
    }

    // Mutator
    public void setCorrespondence(MapTarget _correspondence) {
        correspondence = _correspondence;
        hasCorrespondence = true;
    }

    // Accessors
    public int getTargetId(){return id;}
    public double getAltitude(){return altitude;}
    public double getAzimuth(){return azimuth;}
    public boolean getHasCorrespondence(){return hasCorrespondence;}
    public MapTarget getCorrespondence(){return correspondence;}
}
