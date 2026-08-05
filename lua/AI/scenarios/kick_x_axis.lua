local m = {}
local kick_to_point = require("skills.kick_to_point")
local utils = require("utils.utils")

-- Configuración inicial del equipo y robot
local team = 0    -- 0: Azul, 1: Amarillo
local robotId = 0 -- ID del robot atacante

-- Definición de las posiciones de los arcos en el eje Y (x = 0.0 estricto)
local START_GOAL = { x = -3.0, y = 0.0 }  -- Arco de origen (sur)
local TARGET_GOAL = { x = 3.0, y = 0.0 }  -- Arco objetivo (norte)

-- Posicionar robot y pelota en el simulador (grSim)
-- Pelota en el arco de origen (0.0, -3.0)
-- Robot ubicado detrás de la pelota en (0.0, -3.3) mirando hacia el arco objetivo (pi/2 rads)
grsim.teleport_robot(robotId, team, -3.3, 0.0, 0)
grsim.teleport_ball(START_GOAL.x, START_GOAL.y)

--- Proceso principal de la prueba
function m.process()
    -- Visualizaciones en el visor del engine
    -- Línea guía estrictamente sobre el eje Y (de y = -3.0 a y = 3.0)
    draw_line({ { x = -3.0, y = 0.0 }, { x = 3.0, y = 0.0 } }, true, { r = 0.0, g = 0.8, b = 1.0 })
    
    -- Puntos de origen (verde) y objetivo (rojo)
    draw_point(START_GOAL.x, START_GOAL.y, true, { r = 0.0, g = 1.0, b = 0.0 })
    draw_point(TARGET_GOAL.x, TARGET_GOAL.y, true, { r = 1.0, g = 0.0, b = 0.0 })

    local robot = get_robot_state(robotId, team)
    if robot then
        draw_text(robot.x + 0.2, robot.y + 0.4, "Prueba Patada Eje Y (Goal to Goal)", { r = 1.0, g = 1.0, b = 0.0 })
    end

    -- Ejecutar la skill kick_to_point hacia el arco objetivo
    local is_done = kick_to_point.process(robotId, team, TARGET_GOAL)
    
    if is_done then
        draw_text(0.0, 0.0, "¡PATADA COMPLETADA DE ARCO A ARCO!", { r = 0.0, g = 1.0, b = 0.0 })
    end

    return is_done
end

-- Permitir ejecución directa cuando se carga este script en el motor
function process()
    m.process()
end

return m
