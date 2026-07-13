-- test_estimation.lua
-- Script to automate ball kicking and robot telemetry gathering.

local test_estimation = {}

local state = 0 -- 0: reset/setup, 1: wait for teleport, 2: kick, 3: track ball, 4: wait for stop, 5: robot experiments, 6: done
local run_id = 0
local max_runs = 20
local tick = 0
local wait_ticks = 0
local stop_counter = 0

local team = 0
local robot_id = 0

-- Buffers
local ball_records = {}
local current_run_records = {}
local robot_records = {}

-- Robot Experiment parameters
local robot_tick = 0
local robot_state = 0 -- 0: static, 1: step X, 2: step Y, 3: circular, 4: done
local robot_wait_ticks = 0

-- Write the ball data to CSV
local function write_ball_csv()
    local file, err = io.open("estimation_test.csv", "w")
    if not file then
        print("[Lua Test] Error opening estimation_test.csv: " .. tostring(err))
        return
    end
    file:write("run_id,tick,ball_x,ball_y,speed,empirical_accel,est_x,est_y,actual_stop_x,actual_stop_y,error_distance\n")
    for _, r in ipairs(ball_records) do
        file:write(string.format("%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            r.run_id, r.tick, r.ball_x, r.ball_y, r.speed, r.empirical_accel, r.est_x, r.est_y, r.actual_stop_x, r.actual_stop_y, r.error_distance))
    end
    file:close()
    print("[Lua Test] Successfully saved estimation_test.csv with " .. #ball_records .. " records.")
end

-- Write the robot data to CSV
local function write_robot_csv()
    local file, err = io.open("robot_test.csv", "w")
    if not file then
        print("[Lua Test] Error opening robot_test.csv: " .. tostring(err))
        return
    end
    file:write("run_id,tick,cmd_vx,cmd_vy,cmd_omega,vx,vy,omega,x_meas,y_meas,theta_meas\n")
    for _, r in ipairs(robot_records) do
        file:write(string.format("%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            r.run_id, r.tick, r.cmd_vx, r.cmd_vy, r.cmd_omega, r.vx, r.vy, r.omega, r.x_meas, r.y_meas, r.theta_meas))
    end
    file:close()
    print("[Lua Test] Successfully saved robot_test.csv with " .. #robot_records .. " records.")
end

function test_estimation.process()
    if state == 0 then
        -- ── RESET / SETUP BALL RUN ──
        if run_id >= max_runs then
            print("[Lua Test] Finished all ball runs. Moving to Robot Experiments...")
            state = 5 -- Transition to Robot Experiments
            robot_state = 0
            robot_tick = 0
            robot_wait_ticks = 40
        else
            run_id = run_id + 1
            tick = 0
            stop_counter = 0
            current_run_records = {}
            
            -- Teleport ball and robot
            grsim.teleport_robot(robot_id, team, -3.5, 0.0, 0.0)
            grsim.teleport_ball(-3.3, 0.0)
            
            wait_ticks = 40
            state = 1
        end
        
    elseif state == 1 then
        -- ── WAIT STABILIZATION ──
        send_velocity(robot_id, team, 0.0, 0.0, 0.0)
        wait_ticks = wait_ticks - 1
        if wait_ticks <= 0 then
            state = 2
        end
        
    elseif state == 2 then
        -- ── KICK THE BALL ──
        -- Charge towards the ball with varying speeds depending on the run
        local target_speed = 1.0 + (run_id % 5) * 0.5 -- speed from 1.0 to 3.0
        send_velocity(robot_id, team, target_speed, 0.0, 0.0)
        kickx(robot_id, team)
        
        local ball = get_ball_state()
        local speed = math.sqrt(ball.vel_x^2 + ball.vel_y^2)
        if speed > 0.2 then
            -- Kick detected! Stop robot and track ball.
            send_velocity(robot_id, team, 0.0, 0.0, 0.0)
            state = 3
        end
        
    elseif state == 3 then
        -- ── TRACK & LOG BALL ──
        send_velocity(robot_id, team, 0.0, 0.0, 0.0)
        tick = tick + 1
        
        local ball = get_ball_state()
        local speed = math.sqrt(ball.vel_x^2 + ball.vel_y^2)
        
        -- Compute empirical acceleration
        local empirical_accel = 0.0
        if #current_run_records > 0 then
            local prev = current_run_records[#current_run_records]
            empirical_accel = (speed - prev.speed) / 0.0166 -- 60 FPS nominal dt
        end
        
        -- Predict stop position using measured deceleration (approx 0.37 m/s^2)
        local d_est = 0.0
        local est_x = ball.x
        local est_y = ball.y
        if speed > 0.01 then
            d_est = (speed * speed) / (2 * 0.37)
            est_x = ball.x + (ball.vel_x / speed) * d_est
            est_y = ball.y + (ball.vel_y / speed) * d_est
        end
        
        table.insert(current_run_records, {
            run_id = run_id,
            tick = tick,
            ball_x = ball.x,
            ball_y = ball.y,
            speed = speed,
            empirical_accel = empirical_accel,
            est_x = est_x,
            est_y = est_y,
            actual_stop_x = 0.0,
            actual_stop_y = 0.0,
            error_distance = 0.0
        })
        
        -- Check if stopped
        if speed < 0.02 then
            stop_counter = stop_counter + 1
        else
            stop_counter = 0
        end
        
        if stop_counter >= 15 or tick > 300 then
            state = 4
        end
        
    elseif state == 4 then
        -- ── STOPPED: BACKFILL AND RESET ──
        send_velocity(robot_id, team, 0.0, 0.0, 0.0)
        local ball = get_ball_state()
        local actual_stop_x = ball.x
        local actual_stop_y = ball.y
        
        for _, rec in ipairs(current_run_records) do
            rec.actual_stop_x = actual_stop_x
            rec.actual_stop_y = actual_stop_y
            rec.error_distance = math.sqrt((rec.est_x - actual_stop_x)^2 + (rec.est_y - actual_stop_y)^2)
            table.insert(ball_records, rec)
        end
        
        print(string.format("[Ball Run] Run %d/%d finished. Ticks: %d, Stop: (%.3f, %.3f)", run_id, max_runs, tick, actual_stop_x, actual_stop_y))
        state = 0
        
    elseif state == 5 then
        -- ── ROBOT EXPERIMENTS STATE MACHINE ──
        if robot_state == 0 then
            -- ── TEST 1: STATIC JITTER TEST ──
            if robot_wait_ticks > 0 then
                grsim.teleport_robot(robot_id, team, 0.0, 0.0, 0.0)
                send_velocity(robot_id, team, 0.0, 0.0, 0.0)
                robot_wait_ticks = robot_wait_ticks - 1
            else
                robot_tick = robot_tick + 1
                local robot = get_robot_state(robot_id, team)
                table.insert(robot_records, {
                    run_id = 1, -- Run 1: Static Jitter
                    tick = robot_tick,
                    cmd_vx = 0.0,
                    cmd_vy = 0.0,
                    cmd_omega = 0.0,
                    vx = robot.vel_x,
                    vy = robot.vel_y,
                    omega = robot.omega,
                    x_meas = robot.x,
                    y_meas = robot.y,
                    theta_meas = robot.orientation
                })
                
                if robot_tick >= 150 then -- 2.5 seconds of static logs
                    print("[Robot Test] Finished Static Jitter Test. Moving to Step X...")
                    robot_state = 1
                    robot_tick = 0
                    robot_wait_ticks = 40
                end
            end
            
        elseif robot_state == 1 then
            -- ── TEST 2: STEP RESPONSE X-DIRECTION ──
            if robot_wait_ticks > 0 then
                grsim.teleport_robot(robot_id, team, -2.0, 0.0, 0.0)
                send_velocity(robot_id, team, 0.0, 0.0, 0.0)
                robot_wait_ticks = robot_wait_ticks - 1
            else
                robot_tick = robot_tick + 1
                local cmd_vx = 0.0
                if robot_tick <= 60 then
                    cmd_vx = 1.5 -- Step command of 1.5 m/s
                end
                
                send_velocity(robot_id, team, cmd_vx, 0.0, 0.0)
                local robot = get_robot_state(robot_id, team)
                table.insert(robot_records, {
                    run_id = 2, -- Run 2: Step X
                    tick = robot_tick,
                    cmd_vx = cmd_vx,
                    cmd_vy = 0.0,
                    cmd_omega = 0.0,
                    vx = robot.vel_x,
                    vy = robot.vel_y,
                    omega = robot.omega,
                    x_meas = robot.x,
                    y_meas = robot.y,
                    theta_meas = robot.orientation
                })
                
                if robot_tick >= 120 then -- 2 seconds total
                    print("[Robot Test] Finished Step X Test. Moving to Step Y...")
                    robot_state = 2
                    robot_tick = 0
                    robot_wait_ticks = 40
                end
            end
            
        elseif robot_state == 2 then
            -- ── TEST 3: STEP RESPONSE Y-DIRECTION ──
            if robot_wait_ticks > 0 then
                grsim.teleport_robot(robot_id, team, 0.0, -2.0, 0.0)
                send_velocity(robot_id, team, 0.0, 0.0, 0.0)
                robot_wait_ticks = robot_wait_ticks - 1
            else
                robot_tick = robot_tick + 1
                local cmd_vy = 0.0
                if robot_tick <= 60 then
                    cmd_vy = 1.5 -- Step command of 1.5 m/s
                end
                
                send_velocity(robot_id, team, 0.0, cmd_vy, 0.0)
                local robot = get_robot_state(robot_id, team)
                table.insert(robot_records, {
                    run_id = 3, -- Run 3: Step Y
                    tick = robot_tick,
                    cmd_vx = 0.0,
                    cmd_vy = cmd_vy,
                    cmd_omega = 0.0,
                    vx = robot.vel_x,
                    vy = robot.vel_y,
                    omega = robot.omega,
                    x_meas = robot.x,
                    y_meas = robot.y,
                    theta_meas = robot.orientation
                })
                
                if robot_tick >= 120 then -- 2 seconds total
                    print("[Robot Test] Finished Step Y Test. Moving to Circular Trajectory...")
                    robot_state = 3
                    robot_tick = 0
                    robot_wait_ticks = 40
                end
            end
            
        elseif robot_state == 3 then
            -- ── TEST 4: CIRCULAR TRAJECTORY ──
            if robot_wait_ticks > 0 then
                grsim.teleport_robot(robot_id, team, 1.0, 0.0, 0.0)
                send_velocity(robot_id, team, 0.0, 0.0, 0.0)
                robot_wait_ticks = robot_wait_ticks - 1
            else
                robot_tick = robot_tick + 1
                -- R = 1.0 m, speed = 1.0 m/s, angular = 1.0 rad/s
                local phi = (robot_tick - 1) * 0.025
                local cmd_vx = -1.0 * math.sin(phi)
                local cmd_vy = 1.0 * math.cos(phi)
                local cmd_omega = 1.0
                
                send_velocity(robot_id, team, cmd_vx, cmd_vy, cmd_omega)
                
                local robot = get_robot_state(robot_id, team)
                table.insert(robot_records, {
                    run_id = 4, -- Run 4: Circular
                    tick = robot_tick,
                    cmd_vx = cmd_vx,
                    cmd_vy = cmd_vy,
                    cmd_omega = cmd_omega,
                    vx = robot.vel_x,
                    vy = robot.vel_y,
                    omega = robot.omega,
                    x_meas = robot.x,
                    y_meas = robot.y,
                    theta_meas = robot.orientation
                })
                
                if robot_tick >= 250 then -- one full rotation + extra
                    print("[Robot Test] Finished Circular Test. Saving data files...")
                    robot_state = 4
                end
            end
            
        elseif robot_state == 4 then
            -- ── WRITE OUT CSV FILES ──
            send_velocity(robot_id, team, 0.0, 0.0, 0.0)
            write_ball_csv()
            write_robot_csv()
            print("[Lua Test] All experiments complete! You can open the Jupyter notebook now.")
            state = 6
        end
    end
end

return test_estimation
