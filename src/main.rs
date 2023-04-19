fn filter(input: rospcl::Pointcloud3F32) -> rospcl::Pointcloud3F32 {
    let mut output = Vec::new();
    for point in input {
        output.push(point); // TODO: implement filtering arguments here
    }

    rospcl::Pointcloud3F32::new(output)
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
    let subber = rosrust::subscribe("/velodyne_points", 100, move |msg: rosrust_msg::sensor_msgs::PointCloud2| {
        let incoming_header = msg.header.clone();
        match rospcl::Pointcloud3F32::from_msg(msg) {
            Err(e) => {
                rosrust::ros_err!("Error: {:?}", e);
            }
            Ok(cloud) => {
                let filtered_cloud = filter(cloud);
                let mut out_msg = filtered_cloud.to_msg().unwrap();
                out_msg.header = incoming_header;

                if let Err(e) = pubber.send(out_msg) {
                    rosrust::ros_err!("Could not send filtered pointcloud: {:?}", e);
                }
            }
        }
    });

    if let Err(e) = subber {
        rosrust::ros_err!("Error: {:?}", e);
    }

    rosrust::ros_info!("Spinning...");
    rosrust::spin();
}
