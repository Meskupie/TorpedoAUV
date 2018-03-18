package Autonomy.Localization;

import android.view.animation.TranslateAnimation;

import org.ejml.simple.SimpleBase;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.ArrayList;

/**
 * Created by meskupie on 14/03/18.
 */

public class PoseFitting {

    private Transform pose_transform;
    private double pose_fitness;

    // Input Data
    private Quaternion data_imu = new Quaternion(0,0,0,1);
    private double data_depth = 0;
    private CameraTarget[] data_camera_targets = new CameraTarget[0];
    private MapTarget[] map = new MapTarget[0];

    // Processed Data
    private SimpleMatrix point_sense_imu;
    private SimpleMatrix point_sense_depth;
    private SimpleMatrix[] points_target;
    private SimpleMatrix[] points_map;

    // Fitting Data
    private SimpleMatrix H;
    private SimpleMatrix centroid_start;
    private SimpleMatrix centroid_end;
    private SimpleMatrix centroid_start_camera;
    private SimpleMatrix centroid_end_camera;

    // Tuning
    double scale_translation = 1;
    double min_translation = 0.2;
    double offset_translation = min_translation=min_translation*scale_translation;
    double scale_rotation = 1;
    double min_rotation = 0.2;
    double offset_rotation = min_rotation-min_rotation*scale_rotation;
    int weight_imu = 2;
    int weight_depth = 2;
    int weight_camera_trans = 1;
    int weight_camera_rot = 1;
    double fitness_imu = 2.0;
    double fitness_depth = 2.0;
    double fitness_camera = 1.0;


    public PoseFitting(){
    }

    private void matchCorrespondences(Particle particle){
        Transform pose = particle.getPose();
        ArrayList<SimpleMatrix> points_target_list = new ArrayList<>();
        ArrayList<SimpleMatrix> points_map_list = new ArrayList<>();
        for(int i = 0; i < data_camera_targets.length; i++){
            double best_score = 0.5; // Score threshold that has to be beet
            // Best correspondence
            boolean found_option = false;
            Vector3 best_map = new Vector3(0,0,0);
            Vector3 best_target = new Vector3(0,0,0);
            // Calculate a geometry related to the camera target
            Transform target_inertial_trans = pose.multiply(data_camera_targets[i].getTargetTransform());
            Vector3 target_inertial_vect = target_inertial_trans.getTranslation();
            // Calculate a unit direction vector of the inertial camera laser
            Quaternion target_inertial_quat = target_inertial_trans.getRotationAndScale();
            Vector3 target_unit_vect = getUnitVector(target_inertial_quat,new Vector3(1,0,0));
            for(int j = 0; j < map.length; j++){
                if(data_camera_targets[i].getTargetId() == map[j].getTargetId()){ // colours are the same
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
                points_map_list.add(vecToMat(best_map));
                points_target_list.add(vecToMat(best_target));
            }
        }
        points_map = points_map_list.toArray(new SimpleMatrix[0]);
        points_target = points_target_list.toArray(new SimpleMatrix[0]);
    }

    public void fitTransform(Particle particle,int type){
        Transform pose = particle.getPose();
        boolean use_imu = false;
        boolean use_depth = false;
        boolean use_camera = false;
        switch(type){
            case 1:
                use_imu = true;
                break;
            case 2:
                use_imu = true;
                use_depth = true;
                break;
            case 3:
                use_imu = true;
                use_camera = true;
                break;
            case 4:
                use_imu = true;
                use_depth = true;
                use_camera = true;
                break;
            default:
                break;
        }

        // Reset rotation matrix generator
        H = new SimpleMatrix(3,3);

        // Reset centroid calculators
        SimpleMatrix mean_count = new SimpleMatrix(3,1);
        centroid_start = new SimpleMatrix(3,1);
        centroid_end = new SimpleMatrix(3,1);

        // Fitness checking

        if(use_imu){
            SimpleMatrix point_pose_orientation = vecToMat(getUnitVector(pose.getRotationAndScale(),new Vector3(0,0,1)));
            for(int i = 0; i < weight_imu; i++) {
                H = H.plus(point_pose_orientation.mult(point_sense_imu.transpose()));
            }
        }

        if(use_depth){
            SimpleMatrix point_pose_depth = new SimpleMatrix(3,1);
            point_pose_depth.set(2,0,pose.getTranslation().getZ());
            for(int i = 0; i < weight_depth; i++){
                centroid_start = centroid_start.plus(point_pose_depth);
                centroid_end = centroid_end.plus(point_sense_depth);
            }
            mean_count = mean_count.plus(new SimpleMatrix(3,1,false,new double[]{0,0,weight_depth}));
        }

        if(use_camera){
            // find correspondences
            this.matchCorrespondences(particle);

            // Reset centroid calculators
            SimpleMatrix mean_count_camera = new SimpleMatrix(3,1);
            centroid_start_camera = new SimpleMatrix(3,1);
            centroid_end_camera = new SimpleMatrix(3,1);

            // Calculate mean of map/target points and of all points
            for(int i = 0; i < points_map.length; i++){
                centroid_start_camera = centroid_start_camera.plus(points_target[i].scale(weight_camera_trans));
                centroid_end_camera = centroid_end_camera.plus(points_map[i].scale(weight_camera_trans));
            }
            int camera_count = weight_camera_trans*points_map.length;
            mean_count = mean_count.plus(new SimpleMatrix(3,1,false,new double[]{camera_count,camera_count,camera_count}));
            centroid_start = centroid_start.plus(centroid_start_camera).elementDiv(mean_count);
            centroid_end = centroid_end.plus(centroid_end_camera).elementDiv(mean_count);
            centroid_start_camera = centroid_start_camera.divide(camera_count);
            centroid_end_camera = centroid_end_camera.divide(camera_count);

            // Run rotation transform
            for(int i = 0; i < points_map.length; i++) {
                SimpleMatrix H_delta = points_target[i].minus(centroid_start_camera).mult(points_map[i].minus(centroid_end_camera).transpose());
                for (int j = 0; j < weight_camera_rot; j++) {
                    H = H.plus(H_delta);
                }
            }
        }

        // Calculate final translation
        SimpleMatrix trans_vect = centroid_end.minus(centroid_start);
        Vector3 final_trans = new Vector3(trans_vect.get(1,1),trans_vect.get(2,1),trans_vect.get(3,1));
        double distance = final_trans.getMagnitude();
        // Check if we should scale the amount we shift
        if(distance > min_translation){
            final_trans.scale((distance*scale_translation+offset_translation)/distance);
        }
        // Run SVD and calculate rotation matrix
        SimpleSVD<SimpleMatrix> rot_svd = H.svd();
        SimpleMatrix rot_U = rot_svd.getU();
        SimpleMatrix rot_V = rot_svd.getV();
        SimpleMatrix rot_matrix = rot_U.mult(rot_V.transpose());

        // Scale the rotation by converting to axis angle, limiting angle rotation and converting to quaternion
        double angle = Math.acos((rot_matrix.trace()-1)/2);
        double scale = (1/(2*Math.sin(angle)));
        double axis_x = scale*(rot_matrix.get(3,2)-rot_matrix.get(3,2));
        double axis_y = scale*(rot_matrix.get(1,3)-rot_matrix.get(3,1));
        double axis_z = scale*(rot_matrix.get(2,1)-rot_matrix.get(1,2));
        if(angle > min_rotation){
            angle = angle*scale_rotation+offset_rotation;
        }
        Quaternion final_rot = new Quaternion(axis_x*Math.sin(angle/2),axis_y*Math.sin(angle/2),axis_z*Math.sin(angle/2),Math.cos(angle/2));

        // Set pose transform
        pose_transform = new Transform(final_trans,final_rot);

        // Calculate fitness of solution
        double fitness_sum = 0;
        Transform new_pose = particle.getPose().multiply(pose_transform);

        if(use_imu){
            SimpleMatrix pose_orientation_point = vecToMat(getUnitVector(new_pose.getRotationAndScale(),new Vector3(0,0,1)));
            fitness_sum += matToVec(pose_orientation_point.minus(point_sense_imu)).getMagnitudeSquared();
        }

        if(use_depth){
            fitness_sum += Math.pow((new_pose.getTranslation().getZ()-point_sense_depth.get(2,0)),2)*fitness_depth;
        }

        if(use_camera){
            Transform point_target_transform;
            Transform delta_rotation = new Transform(new Vector3(1,0,0),final_rot);
            Vector3 point_target_new;

            for(int i = 0; i < points_map.length; i++){
                // Move target point to origin reference
                point_target_transform = new Transform(matToVec(points_target[i].minus(centroid_start)),new Quaternion(0,0,0,1));
                // Rotate origin referenced point
                point_target_new = point_target_transform.multiply(delta_rotation).getTranslation();
                // Shift to map origin
                point_target_new = point_target_new.add(matToVec(centroid_start)).add(final_trans);
                // Update fitness
                fitness_sum += matToVec(points_map[i]).subtract(point_target_new).getMagnitudeSquared()*fitness_camera;
            }
        }

        if(use_camera||use_depth||use_imu){
            pose_fitness = 1.0/fitness_sum;
        }else{
            pose_fitness = particle.getFitness();
        }
    }

    public void weighParticle(Particle particle){

    }

    private Vector3 getUnitVector(Quaternion q,Vector3 dir){
        Transform rot = new Transform(new Vector3(0,0,0),q);
        Transform unit = new Transform(dir,new Quaternion(0,0,0,1));
        return rot.multiply(unit).getTranslation();
    }

    private SimpleMatrix vecToMat(Vector3 vect){
        return new SimpleMatrix(3,1,false,new double[]{vect.getX(),vect.getY(),vect.getZ()});
    }

    private Vector3 matToVec(SimpleMatrix mat){
        return new Vector3(mat.get(0,0),mat.get(1,0),mat.get(2,0));
    }

    // Mutators
    public void setIMU(Quaternion _data_imu){
        data_imu = _data_imu;
        point_sense_imu = vecToMat(getUnitVector(data_imu,new Vector3(0,0,1)));
    }
    public void setDepth(double _data_depth){
        data_depth = _data_depth;
        point_sense_depth = new SimpleMatrix(3,1);
        point_sense_depth.set(2,0,data_depth);
    }

    public void setMap(MapTarget[] _map){
        map = _map;}
    public void setCameraTargets(CameraTarget[] _camera_targets){
        data_camera_targets = _camera_targets;}

    // Accessor
    public Transform getPoseTransform(){return pose_transform;}
    public double getFitness(){return pose_fitness;}

}
