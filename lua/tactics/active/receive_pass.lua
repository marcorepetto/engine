local utils = require("utils.utils")

local ReceivePass = { State = { playing = "playing", failed = "failed", done = "done" } }
ReceivePass.__index = ReceivePass

function ReceivePass.new()
    return setmetatable({ state = ReceivePass.State.playing }, ReceivePass)
end

--- Executes the receive pass tactic with predictive interception
--- @param robotId number
--- @param team number
function ReceivePass:process(robotId, team)
    -- Early exit if already finished or failed
    if self.state ~= ReceivePass.State.playing then
        return self.state == ReceivePass.State.done
    end

    local mu = 0.05 -- Friction coefficient
    local g  = 9.81 -- Gravity

    local ball = get_ball_state()
    local robot = get_robot_state(robotId, team)

    if not ball or not robot then
        self.state = ReceivePass.State.failed
        return false
    end

    local dist_to_ball = utils.getDistance(robot, ball)
    
    -- Default to waiting in the current position
    local target_x, target_y = robot.x, robot.y 

    -- Extract ball velocity (Update .vx and .vy if your API uses different names)
    local bvx, bvy = ball.vel_x or 0, ball.vel_y or 0
    local ball_speed = math.sqrt(bvx^2 + bvy^2)

    -- If the ball is moving fast enough, calculate interception
    if ball_speed > 0.1 then
        -- 1. Get the direction the ball is traveling (normalized vector)
        dir_x = bvx / ball_speed
        dir_y = bvy / ball_speed
        
        -- 2. Get the vector from the ball to our robot
        local dx = robot.x - ball.x
        local dy = robot.y - ball.y
        
        -- 3. Calculate the dot product to project our robot onto the ball's path
        local dot = (dx * dir_x) + (dy * dir_y)

        -- local time_to_intercept = dot / ball_speed
        local max_dist = ball_speed*ball_speed/(2*1.42857*mu*g)

        -- 4. If dot > 0, the ball is traveling towards our general direction
        if dot > 0 then
            if dot > max_dist then
                dot = max_dist
            end

            -- Set target to the closest point on the ball's trajectory line
            target_x = ball.x + (dir_x * dot)
            target_y = ball.y + (dir_y * dot)
        end

        -- 5. Face the robot towards the ball's direction of travel
        local target_orientation = {
            x = robot.x - dir_x*10,
            y = robot.y - dir_y*10
        }
        face_to(robotId, team, target_orientation)
    else
        -- If the ball is slow or stopped, just face it and wait
        face_to(robotId, team, {x = ball.x, y = ball.y})
    end
    -- Move to interception point and always keep eyes on the ball
    move_to(robotId, team, {x = target_x, y = target_y})

    -- Draw a green dot at our interception/waiting point so you can debug it
    draw_point(target_x, target_y, true, {r=0.0, g=1.0, b=0.0})
    

    -- Check if we are close enough to consider the receive successful (e.g., we trapped it)
    if dist_to_ball < 0.15 then
        self.state = ReceivePass.State.done
    end

    return self.state == ReceivePass.State.done
end

function ReceivePass:get_state()
    return self.state
end

function ReceivePass:reset()
    self.state = ReceivePass.State.playing
end

-- Default instance and module export
local def = ReceivePass.new()

return {
    State = ReceivePass.State,
    new = ReceivePass.new,
    process = function(r, t) return def:process(r, t) end,
    get_state = function() return def:get_state() end,
    reset = function() def:reset() end
}