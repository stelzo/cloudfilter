fn filter(input: Vec<nalgebra::Point3<f32>>) -> Vec<nalgebra::Point3<f32>> {
    let mut output = Vec::new();
    for point in input {
        if point.z > 0.0 {
            output.push(point);
        }
    }
    output
}

fn main() {
    rosrust::init("cloudfilter");

    // advertise a topic
    let pubber = match rosrust::publish("filtered", 100) {
        Ok(p) => {
            rosrust::ros_info!("Publishing to filtered topic");
            p
        },
        Err(_) => panic!("Failed to publish to filtered topic"),
    };

    // subscribe to a topic
    let subber_ = rosrust::subscribe("/virtual_edge/merged_points", 100, move |msg: rosrust_msg::sensor_msgs::PointCloud2| {
        let pointcloud = rospcl::xzy_pointcloud_f32_from_msg(&msg);
        match pointcloud {
            Ok(cloud) => {
                let filtered_cloud = filter(cloud);
                let mut out_msg = rospcl::xzy_pointcloud_f32_to_msg(&filtered_cloud);
                out_msg.header = msg.header;
                match pubber.send(out_msg) {
                    Ok(_) => {
                        rosrust::ros_info!("Sent filtered pointcloud with {} points", filtered_cloud.len());
                    }
                    Err(err) => {
                        rosrust::ros_err!("Could not send filtered pointcloud: {:?}", err);
                    }
                }
            }
            Err(e) => {
                rosrust::ros_err!("Error: {:?}", e);
            }
        }
        rosrust::ros_info!("Received pointcloud with {} points", msg.width * msg.height);
    });

    match subber_ {
        Ok(_) => {
            rosrust::ros_info!("Subscribed to pointcloud topic");
        }
        Err(_) => {
            panic!("Failed to subscribe to pointcloud topic");
        }
    }

    rosrust::ros_info!("Spinning...");
    rosrust::spin();
}
