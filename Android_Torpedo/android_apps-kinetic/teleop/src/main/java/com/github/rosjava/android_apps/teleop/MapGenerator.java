package com.github.rosjava.android_apps.teleop;

import android.content.Context;

import org.ros.rosjava_geometry.Vector3;
import java.util.*;

/**
 * Created by isaiah on 28/02/18.
 */

public class MapGenerator {

    private Map<Integer, Vector3> pointMap;
    private Context context; // store a reference to context if map creation depends on phone params

   public MapGenerator(Context c) {
       this.pointMap = new HashMap<>();
       this.context = c;
       this.createMap();
   }

    private void createMap() {
        // call addPointToMap for a variety of points
        this.addPointToMap(0, new Vector3(1, 1, 1));
        this.addPointToMap(1, new Vector3(2, 15, 42));
        this.addPointToMap(2, new Vector3(3, 12, 2));
        this.addPointToMap(3, new Vector3(4, 2, 3));
        this.addPointToMap(4, new Vector3(5, 7, 2));
    }

    private void addPointToMap(int key, Vector3 point) {
        this.pointMap.put(key, point);
    }

    public Vector3 getPoint(int key) {
        return this.pointMap.get(key);
    }

    public Map<Integer, Vector3> getPointMap() {
        return this.pointMap;
    }

    @Override
    public String toString() {
       String str = "";

       for(int i : this.pointMap.keySet()) {
           Vector3 vect = this.pointMap.get(i);
           str = str + "(x: " + vect.getX() + ", y: " + vect.getY() + ", z: " + vect.getZ() + ")\n";
       }
       return str;
    }
}
