package com.github.rosjava.android_apps.teleop;

import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 * Created by meskupie on 17/02/18.
 */

public class cameraTarget {
    Transform target_transform;
    Vector3 target_unit_vector;
    Vector3 estimated_position;
    double scale;
    double distance;
    double score;
    int corresponding_map_index;
}
