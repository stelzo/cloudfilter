use std::iter::Map;
use rospcl::Pointcloud3F32;
use rosrust_msg::sensor_msgs::PointCloud2;
use tf_rosrust::{TfListener};

struct AABB {
    min: [f32; 3],
    max: [f32; 3],
}

#[inline]
fn aabb_filter(input: Pointcloud3F32, aabb: AABB, tf: &tf_rosrust::transforms::geometry_msgs::Transform) -> Pointcloud3F32 {
    // transformation from the pointcloud frame to the global zero frame
    let tf = nalgebra::geometry::Isometry3::from_parts(
        nalgebra::geometry::Translation3::new(tf.translation.x as f32, tf.translation.y as f32, tf.translation.z as f32),
        nalgebra::geometry::UnitQuaternion::from_quaternion(nalgebra::geometry::Quaternion::new(
            tf.rotation.w as f32,
            tf.rotation.x as f32,
            tf.rotation.y as f32,
            tf.rotation.z as f32,
        )),
    );

    Pointcloud3F32::new(input.map(|point| {
        let point = nalgebra::Point3::new(point[0], point[1], point[2]);
        tf.transform_point(&point) // transform
    }).filter(|point| { // aabb filter
        point[0] >= aabb.min[0]
            && point[0] <= aabb.max[0]
            && point[1] >= aabb.min[1]
            && point[1] <= aabb.max[1]
            && point[2] >= aabb.min[2]
            && point[2] <= aabb.max[2]
    }).map(|point| {
        [point.x, point.y, point.z] // convert back to array
    }).collect())
}

fn main() {
    rosrust::init("cloudfilter");
    let listener = TfListener::new();

    // advertise a topic
    let pubber = match rosrust::publish("filtered", 100) {
        Ok(p) => p,
        Err(_) => panic!("Failed to publish to filtered topic"),
    };

    let filter_type: String = rosrust::param("~filter_type").unwrap().get().unwrap_or("aabb".to_string());
    let aabb_min_x = rosrust::param("~aabb_min_x").unwrap().get().unwrap_or(f32::MIN) as f32;
    let aabb_max_x = rosrust::param("~aabb_max_x").unwrap().get().unwrap_or(f32::MAX) as f32;
    let aabb_min_y = rosrust::param("~aabb_min_y").unwrap().get().unwrap_or(f32::MIN) as f32;
    let aabb_max_y = rosrust::param("~aabb_max_y").unwrap().get().unwrap_or(f32::MAX) as f32;
    let aabb_min_z = rosrust::param("~aabb_min_z").unwrap().get().unwrap_or(f32::MIN) as f32;
    let aabb_max_z = rosrust::param("~aabb_max_z").unwrap().get().unwrap_or(f32::MAX) as f32;

    let global_zero_frame: String = rosrust::param("~global_zero_frame").unwrap().get().unwrap_or("base_footprint".to_string());

    // subscribe to a topic
    let subber = rosrust::subscribe("cloud", 100, move |msg: PointCloud2| {
        let tf = listener.lookup_transform(global_zero_frame.as_str(), msg.header.frame_id.as_str(), msg.header.stamp);
        let tf = match tf {
            Ok(t) => t,
            Err(e) => {
                rosrust::ros_err!("TF: {:?}", e);
                return;
            }
        };
        let incoming_header = msg.header.clone();
        match Pointcloud3F32::try_from(msg) {
            Err(e) => {
                rosrust::ros_err!("Error: {:?}", e);
            }
            Ok(cloud) => {
                match filter_type.as_str() {
                    "aabb"=> {
                        let aabb = AABB {
                            min: [aabb_min_x as f32, aabb_min_y as f32, aabb_min_z as f32],
                            max: [aabb_max_x as f32, aabb_max_y as f32, aabb_max_z as f32],
                        };
                        let filtered_cloud = aabb_filter(cloud, aabb, &tf.transform);
                        let mut out_msg: PointCloud2 = filtered_cloud.try_into().unwrap();
                        out_msg.header = incoming_header;
                        out_msg.header.frame_id = global_zero_frame.clone();

                        if let Err(e) = pubber.send(out_msg) {
                            rosrust::ros_err!("Could not send filtered pointcloud: {:?}", e);
                        }
                    }
                    _ => {
                        rosrust::ros_err!("Unknown filter type: {}", filter_type);
                    }
                }
            }
        }
    });

    if let Err(e) = subber {
        rosrust::ros_err!("Error: {:?}", e);
    }

    rosrust::spin();
}
