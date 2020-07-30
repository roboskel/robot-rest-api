#![feature(proc_macro_hygiene, decl_macro)]

#[macro_use]
extern crate mysql;
#[macro_use]
extern crate rocket;
#[macro_use]
extern crate serde_derive;

use rocket_contrib::json;

use rosrust;
use rosrust_msg;

// TODO
const mysqldb: &str = "mysql://root:password@localhost:3307/mysql";

#[derive(Deserialize)]
struct Pose2D {
    x: f64,
    y: f64,
    stay: f64,
}

#[derive(Deserialize)]
struct Mission {
    id: i64,
    name: String,
    poses: Vec<Pose2D>,
}

#[post("/robot_goal", format = "json", data = "<j>")]
fn robot_goal(j: json::Json<Pose2D>) -> json::JsonValue {
    let goal_pub = rosrust::publish("move_base_simple/goal", 1).unwrap();

    rosrust::rate(1.0).sleep();

    let mut msg = rosrust_msg::geometry_msgs::PoseStamped::default();
    msg.header.frame_id = format!("map");
    msg.pose.position.x = j.x;
    msg.pose.position.y = j.y;
    msg.pose.orientation.w = 1.0;
    // TODO create custom PoseStamped msg for the "stay" field
    // msg.stay = j.stay;

    goal_pub.send(msg).unwrap();
    json!({ "status": "ok" })
}

#[post("/create_mission", format = "json", data = "<j>")]
fn create_mission(j: json::Json<Mission>) -> json::JsonValue {
    let mut mission = Mission {
        id: 0,
        name: String::new(),
        poses: Vec::new(),
    };

    let pool = mysql::Pool::new(mysqldb).unwrap();
    // TODO INSERT

    json!({ "status" : "ok" })
}

#[post("/update_mission", format = "json", data = "<j>")]
fn update_mission(j: json::Json<Mission>) -> json::JsonValue {
    let mut mission = Mission {
        id: 0,
        name: String::new(),
        poses: Vec::new(),
    };

    // TODO UPDATE based on id

    json!({ "status" : "ok" })
}

fn main() {
    rosrust::init("robot_rest_api");

    rocket::ignite()
        .mount("/api", routes![robot_goal, create_mission, update_mission])
        .launch();
}
