use crate::motion::Motion;
use crate::motion::pathplanning::environment::Environment;
use crate::motion::pathplanning::FastPathPlanner;
use crate::sender::radio::Radio;
use crate::types::{KickerCommand, MotionCommand, RobotState, Vec2D};
use crate::world::World;
use mlua::prelude::*;
use std::sync::{Arc, Mutex, RwLock};

pub(super) fn register_control_functions(
    lua: &Lua,
    radio: Arc<Mutex<Radio>>,
    world: Arc<RwLock<World>>,
) {
    let globals = lua.globals();

    // ── send_velocity(id, team, vx, vy, omega) ──
    {
        let r = Arc::clone(&radio);
        let w = Arc::clone(&world);
        let f = lua
            .create_function(move |_, (id, team, vx, vy, omega): (i32, i32, f64, f64, f64)| {
                let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                let rs = w.get_robot_state(id, team);
                if !rs.active {
                    return Ok(());
                }
                let mut cmd = MotionCommand::new(id, team, vx, vy);
                cmd.angular = Some(omega);
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_motion_command(cmd);
                Ok(())
            })
            .unwrap();
        globals.set("send_velocity", f).unwrap();
    }

    // ── move_to(id, team, {x=, y=}) ──
    {
        let r = Arc::clone(&radio);
        let w = Arc::clone(&world);
        let f = lua
            .create_function(move |_, (id, team, point): (i32, i32, LuaTable)| {
                let x: f64 = point.get("x")?;
                let y: f64 = point.get("y")?;
                let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                let rs = w.get_robot_state(id, team);
                if !rs.active {
                    return Ok(());
                }
                let motion = Motion::new();
                let cmd = motion.move_to(&rs, Vec2D::new(x, y), &w);
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_motion_command(cmd);
                Ok(())
            })
            .unwrap();
        globals.set("move_to", f).unwrap();
    }

    // ── move_to_path(id, team, {x=, y=}) -> { {x=, y=}, ... } ──
    {
        let r = Arc::clone(&radio);
        let w = Arc::clone(&world);
        let f = lua
            .create_function(move |lua, (id, team, point): (i32, i32, LuaTable)| {
                let x: f64 = point.get("x")?;
                let y: f64 = point.get("y")?;
                let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                let rs = w.get_robot_state(id, team);

                let result = lua.create_table()?;
                if !rs.active {
                    return Ok(result);
                }

                let target = Vec2D::new(x, y);
                let env = Environment::new(&w, &rs);
                let planner = FastPathPlanner::default();
                let path = planner.get_path(rs.position, target, &env);

                let motion = Motion::new();
                let cmd = motion.move_to(&rs, target, &w);
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_motion_command(cmd);

                for (i, point) in path.iter().enumerate() {
                    let t = lua.create_table()?;
                    t.set("x", point.x)?;
                    t.set("y", point.y)?;
                    result.set(i + 1, t)?;
                }

                Ok(result)
            })
            .unwrap();
        globals.set("move_to_path", f).unwrap();
    }

    // ── move_direct(id, team, {x=, y=}) ──
    {
        let r = Arc::clone(&radio);
        let w = Arc::clone(&world);
        let f = lua
            .create_function(move |_, (id, team, point): (i32, i32, LuaTable)| {
                let x: f64 = point.get("x")?;
                let y: f64 = point.get("y")?;
                let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                let rs = w.get_robot_state(id, team);
                if !rs.active {
                    return Ok(());
                }
                let motion = Motion::new();
                let cmd = motion.move_direct(&rs, Vec2D::new(x, y));
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_motion_command(cmd);
                Ok(())
            })
            .unwrap();
        globals.set("move_direct", f).unwrap();
    }

    // ── plan_path({x=, y=}, {x=, y=}) ──
    {
        let w = Arc::clone(&world);
        let f = lua
            .create_function(move |lua, (from_tbl, to_tbl): (LuaTable, LuaTable)| {
                let from = Vec2D::new(from_tbl.get("x")?, from_tbl.get("y")?);
                let to = Vec2D::new(to_tbl.get("x")?, to_tbl.get("y")?);

                let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                let planner = FastPathPlanner::default();
                let self_robot = RobotState::new(-1, 0);
                let env = Environment::new(&w, &self_robot);
                let path = planner.get_path(from, to, &env);

                let result = lua.create_table()?;
                for (i, point) in path.iter().enumerate() {
                    let t = lua.create_table()?;
                    t.set("x", point.x)?;
                    t.set("y", point.y)?;
                    result.set(i + 1, t)?;
                }

                Ok(result)
            })
            .unwrap();
        globals.set("plan_path", f).unwrap();
    }

    // ── face_to(id, team, {x=, y=}, kp?, ki?, kd?) ──
    {
        let r = Arc::clone(&radio);
        let w = Arc::clone(&world);
        let f = lua
            .create_function(
                move |_,
                      (id, team, point, kp, ki, kd): (
                    i32,
                    i32,
                    LuaTable,
                    Option<f64>,
                    Option<f64>,
                    Option<f64>,
                )| {
                    let x: f64 = point.get("x")?;
                    let y: f64 = point.get("y")?;
                    let w = w.read().map_err(|e| LuaError::external(format!("{e}")))?;
                    let rs = w.get_robot_state(id, team);
                    if !rs.active {
                        return Ok(());
                    }
                    let motion = Motion::new();
                    let cmd = motion.face_to(
                        &rs,
                        Vec2D::new(x, y),
                        kp.unwrap_or(1.0),
                        ki.unwrap_or(1.0),
                        kd.unwrap_or(0.1),
                    );
                    r.lock()
                        .map_err(|e| LuaError::external(format!("{e}")))?
                        .add_motion_command(cmd);
                    Ok(())
                },
            )
            .unwrap();
        globals.set("face_to", f).unwrap();
    }

    // ── kickx(id, team) ──
    {
        let r = Arc::clone(&radio);
        let f = lua
            .create_function(move |_, (id, team): (i32, i32)| {
                let mut kicker = KickerCommand::new(id, team);
                kicker.kick_x = true;
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_kicker_command(kicker);
                Ok(())
            })
            .unwrap();
        globals.set("kickx", f).unwrap();
    }

    // ── kickz(id, team) ──
    {
        let r = Arc::clone(&radio);
        let f = lua
            .create_function(move |_, (id, team): (i32, i32)| {
                let mut kicker = KickerCommand::new(id, team);
                kicker.kick_z = true;
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_kicker_command(kicker);
                Ok(())
            })
            .unwrap();
        globals.set("kickz", f).unwrap();
    }

    // ── dribbler(id, team, speed) ──
    {
        let r = Arc::clone(&radio);
        let f = lua
            .create_function(move |_, (id, team, speed): (i32, i32, f64)| {
                let speed = speed.clamp(0.0, 10.0);
                let mut kicker = KickerCommand::new(id, team);
                kicker.dribbler = speed;
                r.lock()
                    .map_err(|e| LuaError::external(format!("{e}")))?
                    .add_kicker_command(kicker);
                Ok(())
            })
            .unwrap();
        globals.set("dribbler", f).unwrap();
    }

    // ── start_ekf_telemetry(filename) ──
    {
        let f = lua
            .create_function(move |_, filename: String| {
                crate::receiver::tracker::start_telemetry(filename);
                Ok(())
            })
            .unwrap();
        globals.set("start_ekf_telemetry", f).unwrap();
    }

    // ── stop_ekf_telemetry() ──
    {
        let f = lua
            .create_function(move |_, ()| {
                crate::receiver::tracker::stop_telemetry();
                Ok(())
            })
            .unwrap();
        globals.set("stop_ekf_telemetry", f).unwrap();
    }
}