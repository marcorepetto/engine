-- Move the ball to a desired target

local move_ball = {}
-- Ya no necesitamos go_to_ball porque haremos nuestro propio acercamiento
local utils = require("utils.utils")
local has_the_ball = utils.has_the_ball

function move_ball.process(robotId, team, target)
    local ball_pos = get_ball_state()
    local robot_pos = get_robot_state(robotId, team)
    local tolerance = 0.116
    
    local robot_has_ball = has_the_ball(robotId, team)
    
    -- If we have the ball, move towards the target
    if robot_has_ball and math.sqrt((ball_pos.x - target.x)^2 + (ball_pos.y - target.y)^2) > tolerance then
        move_direct(robotId, team, target)
        dribbler(robotId, team, 5)
        return false
    elseif not robot_has_ball then
        -- 1. Calcular el vector direccional desde la pelota hacia el objetivo
        local dx = target.x - ball_pos.x
        local dy = target.y - ball_pos.y
        local dist = math.sqrt(dx^2 + dy^2)
        
        -- Evitar división por cero
        if dist == 0 then dist = 0.001 end
        
        -- 2. Calcular el punto de aproximación detrás de la pelota
        -- offset_dist es la distancia detrás de la pelota. 
        -- Debería ser aprox el radio del robot + un margen (ej: 0.15 metros)
        local offset_dist = 0.15 
        local approach_point = {
            x = ball_pos.x - (dx / dist) * offset_dist,
            y = ball_pos.y - (dy / dist) * offset_dist
        }

        -- Dibujar el punto de aproximación para depuración
        draw_point(approach_point.x, approach_point.y, true, {r=1.0, g=0.0, b=0.0})
        
        -- 3. Calcular qué tan cerca está el robot de ese punto de aproximación ideal
        local dist_to_approach = math.sqrt((robot_pos.x - approach_point.x)^2 + (robot_pos.y - approach_point.y)^2)
        
        -- 4. Máquina de estados de acercamiento
        if dist_to_approach > 0.08 then
            -- Si estamos lejos del punto ideal, rodeamos la pelota hasta llegar a esa posición
            move_to(robotId, team, approach_point)
            face_to(robotId, team, target)
        else
            -- Si ya estamos posicionados atrás, avanzamos directo (sin esquivar la pelota) hacia ella
            move_direct(robotId, team, target)
            face_to(robotId, team, target)
            dribbler(robotId, team, 5) -- Activar dribbler para asegurar la posesión
        end
        return false
    end

    return true
end

return move_ball