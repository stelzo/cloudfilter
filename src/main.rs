use ros_pointcloud2::fallible_iterator::FallibleIterator;
use ros_pointcloud2::ros_types::PointCloud2Msg;
use rosrust_msg::sensor_msgs::PointCloud2;
use tf_rosrust::TfListener;

struct AABB {
    min: [f32; 3],
    max: [f32; 3],
}
type Point = ros_pointcloud2::pcl_utils::PointXYZI;
type Convert = ros_pointcloud2::ConvertXYZI;

#[inline]
fn aabb_filter(
    input: Convert,
    aabb: AABB,
    tf: &tf_rosrust::transforms::geometry_msgs::Transform,
    invert: bool,
) -> Vec<Point> {
    let tf = nalgebra::geometry::Isometry3::from_parts(
        nalgebra::geometry::Translation3::new(
            tf.translation.x as f32,
            tf.translation.y as f32,
            tf.translation.z as f32,
        ),
        nalgebra::geometry::UnitQuaternion::from_quaternion(nalgebra::geometry::Quaternion::new(
            tf.rotation.w as f32,
            tf.rotation.x as f32,
            tf.rotation.y as f32,
            tf.rotation.z as f32,
        )),
    );

    input
        .filter_map(|point| {
            let transformed_point =
                tf.transform_point(&nalgebra::Point3::new(point.x, point.y, point.z));
            let inside = transformed_point[0] >= aabb.min[0]
                && transformed_point[0] <= aabb.max[0]
                && transformed_point[1] >= aabb.min[1]
                && transformed_point[1] <= aabb.max[1]
                && transformed_point[2] >= aabb.min[2]
                && transformed_point[2] <= aabb.max[2];
            if (!invert && inside) || (invert && !inside) {
                return Ok(Some(Point {
                    x: transformed_point[0],
                    y: transformed_point[1],
                    z: transformed_point[2],
                    intensity: point.intensity,
                }));
            }
            Ok(None)
        })
        .collect::<Vec<Point>>()
        .unwrap_or(Vec::new())
}

fn main() {
    rosrust::init("cloudfilter");
    let listener = TfListener::new();

    let pubber = match rosrust::publish("filtered", 100) {
        Ok(p) => p,
        Err(_) => panic!("Failed to publish to filtered topic"),
    };

    let filter_type: String = rosrust::param("~filter_type")
        .unwrap()
        .get()
        .unwrap_or("aabb".to_string());
    let aabb_min_x = rosrust::param("~aabb_min_x")
        .unwrap()
        .get()
        .unwrap_or(f32::MIN) as f32;
    let aabb_max_x = rosrust::param("~aabb_max_x")
        .unwrap()
        .get()
        .unwrap_or(f32::MAX) as f32;
    let aabb_min_y = rosrust::param("~aabb_min_y")
        .unwrap()
        .get()
        .unwrap_or(f32::MIN) as f32;
    let aabb_max_y = rosrust::param("~aabb_max_y")
        .unwrap()
        .get()
        .unwrap_or(f32::MAX) as f32;
    let aabb_min_z = rosrust::param("~aabb_min_z")
        .unwrap()
        .get()
        .unwrap_or(f32::MIN) as f32;
    let aabb_max_z = rosrust::param("~aabb_max_z")
        .unwrap()
        .get()
        .unwrap_or(f32::MAX) as f32;
    let invert = rosrust::param("~invert").unwrap().get().unwrap_or(false);

    let global_zero_frame: String = rosrust::param("~global_zero_frame")
        .unwrap()
        .get()
        .unwrap_or("base_footprint".to_string());

    // subscribe to a topic
    let subber = rosrust::subscribe("cloud", 100, move |msg: PointCloud2| {
        let tf = listener.lookup_transform(
            global_zero_frame.as_str(),
            msg.header.frame_id.as_str(),
            msg.header.stamp,
        );
        let tf = match tf {
            Ok(t) => t,
            Err(e) => {
                rosrust::ros_warn!("TF: {:?}", e);
                return;
            }
        };
        let incoming_header = msg.header.clone();
        let msg: PointCloud2Msg = msg.into();

        match Convert::try_from(msg) {
            Err(e) => {
                rosrust::ros_err!("Error: {:?}", e);
            }
            Ok(cloud) => match filter_type.as_str() {
                "aabb" => {
                    let aabb = AABB {
                        min: [aabb_min_x as f32, aabb_min_y as f32, aabb_min_z as f32],
                        max: [aabb_max_x as f32, aabb_max_y as f32, aabb_max_z as f32],
                    };
                    let filtered_cloud = aabb_filter(cloud, aabb, &tf.transform, invert);
                    if filtered_cloud.is_empty() {
                        return;
                    }
                    let convert: Result<Convert, ros_pointcloud2::ConversionError> =
                        Convert::try_from(filtered_cloud);
                    match convert {
                        Err(e) => {
                            rosrust::ros_err!("Error: {:?}", e);
                            return;
                        }
                        Ok(converted) => {
                            let filtered_msg: Result<
                                PointCloud2Msg,
                                ros_pointcloud2::ConversionError,
                            > = converted.try_into();
                            match filtered_msg {
                                Ok(out_msg) => {
                                    let mut out_msg: PointCloud2 = out_msg.into();
                                    out_msg.header = incoming_header;
                                    out_msg.header.frame_id = global_zero_frame.clone();

                                    if let Err(e) = pubber.send(out_msg) {
                                        rosrust::ros_err!(
                                            "Could not send filtered pointcloud: {:?}",
                                            e
                                        );
                                    }
                                }
                                Err(e) => {
                                    rosrust::ros_err!(
                                        "Could not convert filtered pointcloud: {:?}",
                                        e
                                    );
                                }
                            }
                        }
                    }
                }
                _ => {
                    rosrust::ros_err!("Unknown filter type: {}", filter_type);
                }
            },
        }
    });

    if let Err(e) = subber {
        rosrust::ros_err!("Error: {:?}", e);
    }

    rosrust::spin();
}
