local demo = require("AI.scenarios.two_vs_one")

local tick = 0
local log_filename = "robot_ball_log.csv"

-- Your main engine loop
function process()
    -- Run scenario first
    demo.process()

    tick = tick + 1

    -- Initialize the file with headers on the first tick (overwrite existing file)
    if tick == 1 then
        local file, err = io.open(log_filename, "w")
        if file then
            file:write("tick,x,y,robot_id,team\n")
            file:close()
        else
            print("[Lua Log] Error initializing csv: " .. tostring(err))
        end
    end

    -- Open the CSV file for appending this tick's records
    local file, err = io.open(log_filename, "a")
    if not file then
        print("[Lua Log] Error opening csv for append: " .. tostring(err))
        return
    end

    -- 1. Log Ball State (-1 for robot_id, -1 for team)
    local ball = get_ball_state()
    if ball then
        file:write(string.format("%d,%.5f,%.5f,-1,-1\n", tick, ball.x, ball.y))
    end

    -- 2. Log Blue Robots (team 0)
    local blue_robots = get_blue_team_state()
    if blue_robots then
        for _, robot in ipairs(blue_robots) do
            if robot.active then
                file:write(string.format("%d,%.5f,%.5f,%d,0\n", tick, robot.x, robot.y, robot.id))
            end
        end
    end

    -- 3. Log Yellow Robots (team 1)
    local yellow_robots = get_yellow_team_state()
    if yellow_robots then
        for _, robot in ipairs(yellow_robots) do
            if robot.active then
                file:write(string.format("%d,%.5f,%.5f,%d,1\n", tick, robot.x, robot.y, robot.id))
            end
        end
    end

    file:close()
end