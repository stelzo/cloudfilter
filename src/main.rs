use rospcl::Pointcloud3F32;
use rosrust_msg::sensor_msgs::PointCloud2;

fn filter(input: Pointcloud3F32) -> Pointcloud3F32 {
    let mut output = Vec::new();
    for point in input {
        output.push(point); // TODO: implement filtering arguments here
    }

    Pointcloud3F32::new(output)
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
    let subber = rosrust::subscribe("/velodyne_points", 100, move |msg: PointCloud2| {
        let incoming_header = msg.header.clone();
        match Pointcloud3F32::try_from(msg) {
            Err(e) => {
                rosrust::ros_err!("Error: {:?}", e);
            }
            Ok(cloud) => {
                let filtered_cloud = filter(cloud);
                let mut out_msg: PointCloud2 = filtered_cloud.try_into().unwrap();
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
