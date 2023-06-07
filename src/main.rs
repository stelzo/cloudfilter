use cupcl::Point2;
use ros_pointcloud2::fallible_iterator::FallibleIterator;
use ros_pointcloud2::pcl_utils::PointXYZI;
use ros_pointcloud2::ros_types::PointCloud2Msg;
use ros_pointcloud2::ConvertXYZI;
use rosrust::{ros_err, ros_info, ros_warn};
use rosrust_msg::geometry_msgs::PolygonStamped;
use rosrust_msg::sensor_msgs::PointCloud2;
use std::sync::{Arc, Mutex};
use tf_rosrust::TfListener;

type PointType = PointXYZI;
type Converter = ConvertXYZI;

struct Filter {
    filtered_pub: rosrust::Publisher<PointCloud2>,
    listener: TfListener,
    verbose: bool,
    aabb_min_x: f32,
    aabb_max_x: f32,
    aabb_min_y: f32,
    aabb_max_y: f32,
    aabb_min_z: f32,
    aabb_max_z: f32,
    invert_aabb: bool,
    min_dist: f32,
    max_dist: f32,
    invert_distance: bool,
    global_zero_frame: String,
    polygon: Option<Vec<cupcl::Point2>>,
    invert_polygon: bool,
    point_buffer: Vec<cupcl::Point>,
    tf: Option<nalgebra::geometry::Isometry3<f32>>,
    fov_right: f32,
    fov_left: f32,
    fov_enabled: bool,
}

impl Filter {
    pub fn new() -> Self {
        let verbose = rosrust::param("~verbose").unwrap().get().unwrap_or(false);
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
        let invert_aabb = rosrust::param("~invert_aabb")
            .unwrap()
            .get()
            .unwrap_or(false);
        let min_dist = rosrust::param("~min_dist").unwrap().get().unwrap_or(0.0) as f32;
        let max_dist = rosrust::param("~max_dist")
            .unwrap()
            .get()
            .unwrap_or(f32::MAX / 2.5) as f32;
        let invert_distance = rosrust::param("~invert_distance")
            .unwrap()
            .get()
            .unwrap_or(false);
        let invert_polygon = rosrust::param("~invert_polygon")
            .unwrap()
            .get()
            .unwrap_or(false);

        let global_zero_frame: String = rosrust::param("~global_zero_frame")
            .unwrap()
            .get()
            .unwrap_or("base_footprint".to_string());

        let fov_right = rosrust::param("~fov_right").unwrap().get().unwrap_or(0.0) as f32;
        let fov_left = rosrust::param("~fov_left").unwrap().get().unwrap_or(0.0) as f32;
        let fov_enabled = rosrust::param("~enable_fov")
            .unwrap()
            .get()
            .unwrap_or(false);

        Self {
            filtered_pub: rosrust::publish("~filtered", 100).unwrap(),
            listener: TfListener::new(),
            verbose,
            aabb_min_x,
            aabb_max_x,
            aabb_min_y,
            aabb_max_y,
            aabb_min_z,
            aabb_max_z,
            invert_aabb,
            min_dist,
            max_dist,
            invert_distance,
            global_zero_frame,
            polygon: None,
            invert_polygon,
            point_buffer: Vec::new(),
            tf: None,
            fov_right,
            fov_left,
            fov_enabled,
        }
    }

    pub fn polygon_callback(&mut self, msg: PolygonStamped) {
        let tf = match self.tf {
            Some(tf) => tf,
            None => {
                ros_warn!("No TF available for polygon");
                return;
            }
        };

        let mut poly = Vec::new();
        for p in msg.polygon.points {
            let p = tf * nalgebra::geometry::Point3::new(p.x as f32, p.y as f32, p.z as f32);
            poly.push(cupcl::Point2::new(p.x, p.y));
        }
        self.polygon = Some(poly);
    }

    pub fn cloud_callback(&mut self, msg: PointCloud2) {
        let start_callback = std::time::Instant::now();
        self.point_buffer.clear();
        let stream = cupcl::CudaStream::new();

        let tf = match self.tf {
            Some(tf) => tf,
            None => {
                let tf = self.listener.lookup_transform(
                    self.global_zero_frame.as_str(),
                    msg.header.frame_id.as_str(),
                    msg.header.stamp,
                );
                let tf = match tf {
                    Ok(t) => t,
                    Err(e) => {
                        ros_warn!("TF: {:?}", e);
                        return;
                    }
                };
                self.tf = Some(nalgebra::geometry::Isometry3::from_parts(
                    nalgebra::geometry::Translation3::new(
                        tf.transform.translation.x as f32,
                        tf.transform.translation.y as f32,
                        tf.transform.translation.z as f32,
                    ),
                    nalgebra::geometry::UnitQuaternion::from_quaternion(
                        nalgebra::geometry::Quaternion::new(
                            tf.transform.rotation.w as f32,
                            tf.transform.rotation.x as f32,
                            tf.transform.rotation.y as f32,
                            tf.transform.rotation.z as f32,
                        ),
                    ),
                ));

                self.tf.expect("TF was just set")
            }
        };

        let in_msg: PointCloud2Msg = msg.into();
        let incoming_header = in_msg.header.clone();
        let points_n = (in_msg.width * in_msg.height) as usize;
        if points_n == 0 {
            return;
        }

        if self.point_buffer.capacity() < points_n {
            self.point_buffer = Vec::with_capacity(points_n);
        }

        let mut convert = match Converter::try_from(in_msg) {
            Ok(c) => c,
            Err(e) => {
                ros_warn!("Failed to create pointcloud converter: {:?}", e);
                return;
            }
        };
        while let Ok(Some(point)) = convert.next() {
            self.point_buffer.push(cupcl::Point::new(
                point.x,
                point.y,
                point.z,
                point.intensity,
            ));
        }

        let cloud = cupcl::CudaPointCloud::from_full_cloud(&stream, self.point_buffer.clone());
        let mut params = cupcl::PassthroughFilterParameters::default();
        params.min = (self.aabb_min_x, self.aabb_min_y, self.aabb_min_z);
        params.max = (self.aabb_max_x, self.aabb_max_y, self.aabb_max_z);
        params.invert_bounding_box = self.invert_aabb;
        params.min_dist = self.min_dist;
        params.max_dist = self.max_dist;
        params.invert_distance = self.invert_distance;
        params.rotation = (tf.rotation.i, tf.rotation.j, tf.rotation.k, tf.rotation.w);
        params.translation = (tf.translation.x, tf.translation.y, tf.translation.z);
        params.fov_left = self.fov_left;
        params.fov_right = self.fov_right;
        params.enable_horizontal_fov = self.fov_enabled;

        match self.polygon.as_ref() {
            None => {}
            Some(poly) => {
                let poly = cupcl::CudaBuffer::from_vec(
                    &stream,
                    poly.clone(),
                    std::mem::size_of::<Point2>(),
                );
                params.polygon = Some(poly);
                params.invert_polygon = self.invert_polygon;
            }
        }

        let start_filter = std::time::Instant::now();
        let output = cupcl::passthrough_filter(&stream, &cloud.buffer, &params);
        let end_filter = std::time::Instant::now();

        let out_points = output
            .as_slice()
            .iter()
            .map(|p| PointType {
                x: p.x,
                y: p.y,
                z: p.z,
                intensity: p.i,
            })
            .collect::<Vec<_>>();
        if out_points.is_empty() {
            ros_warn!("Filtered pointcloud is empty");
            return;
        }
        let mut internal_msg: PointCloud2Msg = match Converter::try_from(out_points) {
            Ok(c) => match c.try_into() {
                Ok(m) => m,
                Err(e) => {
                    ros_err!("Failed to convert pointcloud: {:?}", e);
                    return;
                }
            },
            Err(e) => {
                ros_err!("Failed to convert pointcloud: {:?}", e);
                return;
            }
        };

        internal_msg.header = incoming_header;
        internal_msg.header.frame_id = self.global_zero_frame.clone();

        let out_msg: PointCloud2 = internal_msg.into();
        if let Err(e) = self.filtered_pub.send(out_msg) {
            ros_err!("Could not send filtered pointcloud: {:?}", e);
        }
        let end_callback = std::time::Instant::now();
        if self.verbose {
            ros_info!("Filtering: {:?}", end_filter - start_filter);
            ros_info!("Callback: {:?}", end_callback - start_callback);
        }
    }
}

fn main() {
    rosrust::init("cloudfilter");

    let filter = Arc::new(Mutex::new(Filter::new()));

    let state_clone = Arc::clone(&filter);
    let _cloud_subber = rosrust::subscribe("~cloud", 100, move |msg: PointCloud2| {
        match state_clone.lock() {
            Ok(mut state) => state.cloud_callback(msg),
            Err(e) => {
                ros_err!(
                    "Failed to lock state in filter for point cloud callback: {:?}",
                    e
                );
            }
        };
    });

    let state_clone = Arc::clone(&filter);
    let _poly_subber = rosrust::subscribe("~polygon", 100, move |msg: PolygonStamped| {
        match state_clone.lock() {
            Ok(mut state) => state.polygon_callback(msg),
            Err(e) => {
                ros_err!(
                    "Failed to lock state in filter for polygon service: {:?}",
                    e
                );
            }
        };
    });

    let state_clone = Arc::clone(&filter);
    let _service_set_fov =
        rosrust::service::<rosrust_msg::cloudfilter::SetFov, _>("~set_fov", move |req| {
            let left: f32 = req.left;
            let right: f32 = req.right;
            let enable: bool = req.enable;

            Ok(rosrust_msg::cloudfilter::SetFovRes {
                success: match state_clone.lock() {
                    Ok(mut state) => {
                        state.fov_enabled = enable;
                        state.fov_left = left;
                        state.fov_right = right;
                        true
                    }
                    Err(e) => {
                        ros_err!(
                            "Failed to lock state in filter for polygon service: {:?}",
                            e
                        );
                        false
                    }
                },
            })
        });

    rosrust::spin();
}
