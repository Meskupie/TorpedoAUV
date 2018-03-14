package Autonomy.Localization;


import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.ArrayList;

/**
 * Created by meskupie on 14/03/18.
 */

public class TargetProcessing {

    // Camera Data
    CameraTarget[] camera_targets;
    Vector3[] points_target;
    Vector3[] points_map;

    MapTarget[] map;

    public TargetProcessing(MapTarget[] _map){
        map = _map;
    }

    public void matchCorrespondences(Transform pose){
        ArrayList<Vector3> points_target_list = new ArrayList<>();
        ArrayList<Vector3> points_map_list = new ArrayList<>();
        for(int i = 0; i < camera_targets.length; i++){
            double best_score = 0.5; // Score threshold that has to be beet
            // Best correspondence
            boolean found_option = false;
            Vector3 best_map = new Vector3(0,0,0);
            Vector3 best_target = new Vector3(0,0,0);
            // Calculate a geometry related to the camera target
            Transform target_inertial_trans = pose.multiply(camera_targets[i].getTargetTransform());
            Quaternion target_inertial_quat = target_inertial_trans.getRotationAndScale();
            Vector3 target_inertial_vect = target_inertial_trans.getTranslation();
            Transform unit_vector_trans = new Transform(new Vector3(1,0,0),new Quaternion(0,0,0,1));
            // Calculate a unit direction vector of the inertial camera laser
            Vector3 target_unit_vect = new Transform(new Vector3(0,0,0),target_inertial_quat).multiply(unit_vector_trans).getTranslation();
            for(int j = 0; j < map.length; j++){
                if(camera_targets[i].getTargetId() == map[j].getTargetId()){ // colours are the same
                    Vector3 map_test_point = map[j].getPoint().subtract(target_inertial_vect);
                    double scale = map_test_point.dotProduct(target_unit_vect)/target_unit_vect.dotProduct(target_unit_vect);
                    if(scale > 0){
                        Vector3 camera_test_point = target_unit_vect.scale(scale);
                        double score = camera_test_point.subtract(map_test_point).getMagnitudeSquared();
                        if(score < best_score){
                            best_score = score;
                            found_option = true;
                            best_map = map_test_point;
                            best_target = camera_test_point;
                        }
                    }
                }
            }
            if(found_option){
                points_map_list.add(best_map);
                points_target_list.add(best_target);
            }
        }
        points_map = points_map_list.toArray(new Vector3[0]);
        points_target = points_target_list.toArray(new Vector3[0]);
    }

    public void setCameraTargets(CameraTarget[] _data_camera_targets){
        camera_targets = _data_camera_targets;}

    public Vector3[] getTargetCorrespondences(){return points_target;}
    public Vector3[] getMapCorrespondences(){return points_map;}
}
