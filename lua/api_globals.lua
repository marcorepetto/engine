---@meta
---@diagnostic disable: lowercase-global

-- Engine Lua API declarations for static analysis and autocomplete.

---@class Vec2
---@field x number
---@field y number

---@class RobotState
---@field id integer
---@field team integer
---@field x number
---@field y number
---@field vel_x number
---@field vel_y number
---@field orientation number
---@field omega number
---@field active boolean

---@class BallState
---@field x number
---@field y number
---@field vel_x number
---@field vel_y number

---@param id integer
---@param team integer
---@param vx number
---@param vy number
---@param omega number
function send_velocity(id, team, vx, vy, omega) end

---@param id integer
---@param team integer
---@param point Vec2
function move_to(id, team, point) end

---@param id integer
---@param team integer
---@param point Vec2
function move_direct(id, team, point) end

---@param id integer
---@param team integer
---@param point Vec2
---@param kp? number
---@param ki? number
---@param kd? number
function face_to(id, team, point, kp, ki, kd) end

---@param id integer
---@param team integer
function kickx(id, team) end

---@param id integer
---@param team integer
function kickz(id, team) end

---@param id integer
---@param team integer
---@param speed number
function dribbler(id, team, speed) end

---@param id integer
---@param team integer
---@return RobotState
function get_robot_state(id, team) end

---@return BallState
function get_ball_state() end

---@return RobotState[]
function get_blue_team_state() end

---@return RobotState[]
function get_yellow_team_state() end

---@return string
function get_ref_message() end

---@param filename string
function start_ekf_telemetry(filename) end

function stop_ekf_telemetry() end

---@param x number
---@param y number
---@param draw_x_or_color? boolean|number[]|{r:number,g:number,b:number}
---@param color? number[]|{r:number,g:number,b:number}
function draw_point(x, y, draw_x_or_color, color) end

---@param id integer
---@param team integer
function highlight_robot(id, team) end

---@param points table
---@param draw_points_between_or_color? boolean|number[]|{r:number,g:number,b:number}
---@param color? number[]|{r:number,g:number,b:number}
function draw_line(points, draw_points_between_or_color, color) end

---@param x number
---@param y number
---@param text string
---@param color? number[]|{r:number,g:number,b:number}
function draw_text(x, y, text, color) end

---@class GrSimApi
---@field teleport_robot fun(id: integer, team: integer, x: number, y: number, dir: number)
---@field teleport_ball fun(x: number, y: number)

---@type GrSimApi
grsim = {
	teleport_robot = function(id, team, x, y, dir) end,
	teleport_ball = function(x, y) end,
}
