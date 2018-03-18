package Autonomy;

import org.ros.rosjava_geometry.Quaternion;

import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.sin;
import static java.lang.Math.cos;

/**
 * Created by meskupie on 07/03/18.
 */

public class MyQuaternion extends Quaternion{
    public MyQuaternion(double x, double y, double z, double w){
        super(x,y,z,w);
    }

    public MyQuaternion(Quaternion q){
        super(q.getX(),q.getY(),q.getZ(),q.getW());
    }

    public static MyQuaternion createFromEuler(double r, double p, double y){
        double cr = cos(r/2.0);
        double sr = sin(r/2.0);
        double cp = cos(p/2.0);
        double sp = sin(p/2.0);
        double cy = cos(y/2.0);
        double sy = sin(y/2.0);

        return new MyQuaternion((cy*sr*cp)-(sy*cr*sp),(cy*cr*sp)+(sy*sr*cp),(sy*cr*cp)-(cy*sr*sp),(cy*cr*cp)+(sy*sr*sp));
    }

    public double getRoll(){
        double qw = this.getW();
        double qx = this.getX();
        double qy = this.getY();
        double qz = this.getZ();
        return atan2(qy*qz+qw*qx,0.5-(qx*qx+qy*qy));
    }

    public double getPitch(){
        double qw = this.getW();
        double qx = this.getX();
        double qy = this.getY();
        double qz = this.getZ();
        return asin(-2*(qx*qz-qw*qy));
    }

    public double getYaw(){
        double qw = this.getW();
        double qx = this.getX();
        double qy = this.getY();
        double qz = this.getZ();
        return atan2(qx*qy+qw*qz,0.5-(qy*qy+qz*qz));
    }
}
