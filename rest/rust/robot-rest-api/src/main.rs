#![feature(proc_macro_hygiene, decl_macro)]

#[macro_use]
extern crate mysql;
#[macro_use]
extern crate rocket;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate serde_derive;

use rocket_contrib::json;

use rosrust;
use rosrust_msg;

const MYSQLDB: &str = "mysql://roboskel:r0b0sk3l@localhost:3306/roboDB";

lazy_static! {
    pub static ref pool: mysql::Pool = mysql::Pool::new(MYSQLDB).unwrap();
    pub static ref goal_pub: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseStamped> =
        rosrust::publish("/move_base_simple/goal", 1).unwrap();
    pub static ref mission_pub: rosrust::Publisher<rosrust_msg::roboskel_msgs::Mission> =
        rosrust::publish("/mission_manager/mission", 1).unwrap();
}

#[derive(Deserialize, Debug)]
struct Pose2D {
    x: f64,
    y: f64,
    stay: f64,
}

#[derive(Deserialize)]
struct Mission {
    name: String,
    poses: Vec<Pose2D>,
}

#[derive(Deserialize)]
struct State {
    state: String,
}

// TODO handle error during database access or ROS communications
// TODO get maps
// TODO add reports
// TODO get reports

#[post("/robot_goal", format = "json", data = "<j>")]
fn robot_goal(j: json::Json<Pose2D>) -> json::JsonValue {
    let mut msg = rosrust_msg::geometry_msgs::PoseStamped::default();
    msg.header.frame_id = format!("map");
    msg.pose.position.x = j.x;
    msg.pose.position.y = j.y;
    msg.pose.orientation.w = 1.0;

    goal_pub.send(msg).unwrap();
    json!({ "status": "ok" })
}

#[post("/create_mission", format = "json", data = "<j>")]
fn create_mission(j: json::Json<Mission>) -> json::JsonValue {
    // Populate the missions table
    let mut s = format!("INSERT INTO missions (name) VALUES ('{}');", j.name);
    let mid = pool
        .prepare(s)
        .unwrap()
        .execute(())
        .unwrap()
        .last_insert_id();

    // Populate the poses table
    for p in j.poses.iter() {
        s = format!(
            "INSERT INTO poses (x, y, stay) VALUES ({}, {}, {});",
            p.x, p.y, p.stay
        );
        let pid = pool
            .prepare(s)
            .unwrap()
            .execute(())
            .unwrap()
            .last_insert_id();

        // Populate the poses_in_missions table
        s = format!(
            "INSERT INTO poses_in_missions (mid,pid) VALUES ({},{})",
            mid, pid
        );
        pool.prepare(s).unwrap().execute(()).unwrap();
    }

    json!({ "status" : "ok" })
}

#[post("/play_mission", format = "json", data = "<j>")]
fn play_mission(j: json::Json<Mission>) -> json::JsonValue {
    let mut msg = rosrust_msg::roboskel_msgs::Mission::default();
    msg.name = j.name.clone();

    let s = format!("SELECT x,y,stay FROM poses INNER JOIN (SELECT * FROM poses_in_missions WHERE mid=(SELECT id FROM missions WHERE name='{}')) AS A ON id=pid", j.name);

    for r in pool.prepare(s).unwrap().execute(()).unwrap() {
        let o = r.unwrap().clone();
        let x: f64 = (o.get(0)).unwrap();
        let y: f64 = (o.get(1)).unwrap();
        let stay: f64 = (o.get(2)).unwrap();
        let mut ps = rosrust_msg::geometry_msgs::PoseStamped::default();
        ps.header.frame_id = String::from("map");
        ps.header.stamp = rosrust::now();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.orientation.w = 1.0;
        msg.poses.push(ps);
        msg.stay.push(stay);
    }

    mission_pub.send(msg).unwrap();

    json!({ "status" : "ok" })
}

#[get("/get_missions")]
fn get_missions() -> json::JsonValue {
    let s = format!("SELECT name FROM missions");

    let mut v: Vec<String> = vec![];
    for r in pool.prepare(s).unwrap().execute(()).unwrap() {
        let mut o = r.unwrap().clone();
        // let mut row = r.unwrap().next().unwrap().unwrap();
        let name: String = o.take("name").unwrap();
        v.push(name.clone());
        println!("{}", name);
    }

    json!({ "missions": v })
}

#[post("/estop", format = "json", data = "<j>")]
fn estop(j: json::Json<State>) -> json::JsonValue {
    // TODO implement twist mux for software e-stop
    if j.state.eq("on") {
        // TODO
        println!("estop on");
    } else {
        // TODO
        println!("estop off");
    }
    json!({ "status" : "ok" })
}

#[get("/ping")]
fn ping() -> json::JsonValue {
    // TODO send a keep alive message
    json!({ "status" : "ok" })
}

fn lazy_and_unsafe() {
    lazy_static::initialize(&pool);
    lazy_static::initialize(&goal_pub);
    lazy_static::initialize(&mission_pub);
}

fn main() {
    rosrust::init("robot_rest_api");

    lazy_and_unsafe();

    rocket::ignite()
        .mount(
            "/api",
            routes![
                robot_goal,
                create_mission,
                play_mission,
                get_missions,
                estop,
                ping
            ],
        )
        .launch();
}
