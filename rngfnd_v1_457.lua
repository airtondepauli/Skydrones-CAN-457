-- Skydrones 2025
-- Skydrones CAN Driver for Firmware higher or equal than 4.5.7

-- WARNING
-- Minimum Arducopter Version: 4.5.7

-- Created 03/02/2025 by Airton Depauli Junior

-- Essa versão remove a criação de parâmetros via lua script e espera que eles
-- estejam implementados via firmware. Os bindings de anticolisão e bateria são
-- os padrões do ardupilot, e não mais os customizados desenvolvidos por Rishabh e
-- Airton. 

-- Microbrain:
-- R21 e H30 
-- Bateria inteligente Herewin 
-- Célula de carga EFT
-- Topradar 100 metros (DR24200)
-- Batteria tattu 12 e 14S

-- Version 2.0
-- 03/02/2025

--------- CONST DEFINITION -------------
local MAV_SEVERITY_EMERGENCY = 0
local MAV_SEVERITY_ALERT = 1
local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_DEBUG = 7
local SCRIPT_NAME = "SD_CAN_DRIVER"
----------------------------------------

---------- MAIN CONTROL ----------------
-- UPDATE LOOP
local loop_duration_ms = 1
local UPDATE_FREQUENCY = 1000 -- freq = (1000/loop_duration_ms)

--  CAN PORT
local can_driver = CAN:get_device(20)

-- RANGEFINDER CONF
local param_num_lua_rfnd_backend = 36
---@type AP_RangeFinder_Backend_ud
local lua_rfnd_backend 
local lua_rfnd_driver_found = false
local param_num_topradar_rfnd_backend = 26      -- Lanbao
local param_num_nanoradar_rfnd_backend = 11     -- USD11
local param_num_topradar_rfnd_can = 34          -- Benewake CAN Topradar TR24
local param_num_lightware_rfnd = 8              -- Lightware Laser
local isMicrobrainRFND = true

-- PRX CONF
local param_num_lua_prx_backend = 15
---@type AP_Proximity_Backend_ud
local lua_prx_backend
local lua_prx_driver_found = false
local prx_max_range = 20     -- max range of sensor [m]
local prx_min_range = 1.5    -- minimum range of sensor [m]
local param_num_topradar_prx_backend = 4        -- Lanbao
local param_num_nanoradar_prx_backend = 3       -- TeraRangeFinder
local isMicrobrainPRX = true
local proximity_selector = 7

-- BATT CONF
---@type BattMonitorScript_State_ud
local BATT_TEMP_STATE_HERE
local IS_HEREWIN_30AH = true
local batt_counter = 2001

-- SMART BATTERY TATTU
---@type BattMonitorScript_State_ud
local BATT_TEMP_STATE
local TENSAO_LSB = 0
local CELL2LSB = 0
local CELL9LSB = 0
local BATT_MANUF = 0
local BATT_MODEL = 0
local BATT_FULL_14S = 0
local BATT_CONSUMED_MAH_LSB = 0
----------------------------------

-- GLOBAL COUNTER
local DEBUG_MSG = 0

-- PARAMETERS CHEKC
local RNGFND1TYPE = Parameter()
RNGFND1TYPE:init('RNGFND1_TYPE')
local PRX1TYPE = Parameter()
PRX1TYPE:init('PRX1_TYPE')
local PRXSELECTOR = Parameter()
PRXSELECTOR:init('USR_SD_SEL')

local LAST_LEVEL_BROADCASTED = true
local FAIL_TIMER = 0
local LAND_TIMER = 0
local INFO_TIMER = 0
local ALT_HOLD_TIMER = 0

-- VEHICLE MODES
local COPTER_LAND_MODE = 9
local COPTER_ALT_HOLD = 2
local COPTER_RTL_MODE = 6
local COPTER_LOITER_MODE = 5
local COPTER_AUTO_MODE = 3


-- ULTRA FLOW 
local CONT_FLOW_NOTIFY = 0

-- AGRO CONTROL TO AVOID MULTIPLE CALLS TO MODE CHANGE
local HasSwitchedToRTL = false

local lastAccessParameters = millis()

function SEND_MSG_WITH_REC(msg, notification_rate, globalCounter)
    local expireCount = UPDATE_FREQUENCY*notification_rate
    if globalCounter == expireCount then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, msg)
        globalCounter = 0
    else
        globalCounter = globalCounter + 1
    end
end

---CombineBytes
---@param lsb integer
---@param msb integer
---@return integer
function CombineBytes(lsb, msb)
    return msb*0x100 + lsb
end

---CombineBytes2Compl
---@param lsb integer
---@param msb integer
---@return integer
function CombineBytes2Compl(lsb, msb)
    if lsb == 0 and msb == 0 then
        return 0
    end
    return 65535 - (msb*0x100 + lsb)
end

function GetBattVoltage(lsb, msb)
    return CombineBytes(lsb, msb)*0.001
end

------------------------------- RNGFND --------------------------------

function Setup_rfnd_sensor()
    if not can_driver then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "No scripting CAN interface found")
        return
    end

    local sensor_count = rangefinder:num_sensors() -- number of sensors connected
    for j = 0, sensor_count-1 do
        local device = rangefinder:get_backend(j)
        if ((not lua_rfnd_driver_found) and device and (device:type() == param_num_lua_rfnd_backend)) then
            -- LUA DRIVER
            lua_rfnd_driver_found = true
            lua_rfnd_backend = device
        end
    end
    if not lua_rfnd_driver_found then
        -- Could not find a lua backend
        SEND_MSG_WITH_REC("CONFIGURE LUA RFND", 1, DEBUG_MSG)
        return
    end
end

function Parse_rfnd_can_frame(frame_rfnd)
    local height_cm = (frame_rfnd:data(0)*256 + frame_rfnd:data(1))
    return height_cm * 0.01
end

function Handle_rfnd_frame(frame_rfnd)
    local rfnd_dist = Parse_rfnd_can_frame(frame_rfnd)

    if (rfnd_dist >= 0) then
        local sent_success = lua_rfnd_backend:handle_script_msg(rfnd_dist)
        if not sent_success then
            SEND_MSG_WITH_REC("RFND Lua Script Error", 1, DEBUG_MSG)
            return
        end
    end
end

-------------------------------------- PRX -----------------------------------
function Setup_prx_sensor()
    if not can_driver then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "No scripting CAN interface found")
        return
    end

    local sensor_count_prx = proximity:num_sensors()
    for j = 0, sensor_count_prx-1 do
        local device_prx = proximity:get_backend(j)
        if ((not lua_prx_driver_found) and device_prx and (device_prx:type() == param_num_lua_prx_backend)) then
            -- LUA DRIVER
            lua_prx_driver_found = true
            lua_prx_backend = device_prx
            lua_prx_backend:set_distance_min_max(prx_min_range, prx_max_range)
        end
    end
    if not lua_prx_driver_found then
        -- Could not find lua backend
        gcs:send_text(MAV_SEVERITY_EMERGENCY, string.format("Configure Lua Proximity Sensor"))
        return
    end
end

function Parse_can_frame(frame_prx)
    local object_x_cm = (frame_prx:data(0)*256 + frame_prx:data(1)) - 32768
    local object_y_cm = (frame_prx:data(2)*256 + frame_prx:data(3))
    return object_x_cm*0.01, object_y_cm*0.01
end

function Handle_prx_frame(frame_prx, prx_orientation_yaw_deg)

    local object_x_cm, object_y_cm = Parse_can_frame(frame_prx)
    local object_valid = false
    if (object_y_cm >= 0) then
        object_valid = true
    end

    if object_valid then
        local object_distance = math.sqrt(object_x_cm * object_x_cm + object_y_cm * object_y_cm)
        local sent_success = lua_prx_backend:handle_script_distance_msg(object_distance, prx_orientation_yaw_deg, 0, true)
        if not sent_success then
            gcs:send_text(MAV_SEVERITY_EMERGENCY, string.format("Proximity Lua Script Error"))
            return
        end
    end
end

---------------------------------- MAIN --------------------------------
-- @calibrateGravimetric(): Performs a gravimetric calibration on the strain gauges
function CalibrateGravimetric(CAN_driver, weight)
    local calibrationWeightHigh = math.floor((weight) / 256)
    local calibrationWeightLow = (weight) % 256

    local calibrateMessage = GetCANMessage(0xF2, 0x88, 0x0, 0x0, 0xF7, calibrationWeightHigh, calibrationWeightLow, 0x0, 0x0, 0xF1, 0xF3, 0x0)
    CAN_driver:write_frame(calibrateMessage, 50) -- Send CAN_msg and set 50 us timeout
end

-- @calibrateTare(): Performs a tare calibration on the weight modules
function CalibrateTare(CAN_driver)
    local calibrateMessage = GetCANMessage(0xF2, 0x88, 0x0, 0x0, 0xF6, 0x0, 0x0, 0x0, 0x0, 0xF1, 0xF3, 0x0)
    CAN_driver:write_frame(calibrateMessage, 50) -- Send CAN_msg and set 50 us timeout    
end

-- @getCANMessage(): Returns an extended CAN msg
function GetCANMessage(id_b1, id_b2, id_b3, id_b4, b1, b2, b3, b4, b5, b6, b7, b8)
    local msg = CANFrame()
    msg:id(uint32_t(1) << 31 | uint32_t(id_b4) << 24 | uint32_t(id_b3) << 16 | uint32_t(id_b2) << 8 | uint32_t(id_b1)) -- Set the extended CAN ID (0x88F2)
    msg:data(0, b1) -- Switch source to signal switching
    msg:data(1, b2) -- Switch to CANBUS mode
    msg:data(2, b3) -- Empty
    msg:data(3, b4) -- Empty
    msg:data(4, b5) -- Empty
    msg:data(5, b6) -- Empty
    msg:data(6, b7) -- Empty
    msg:data(7, b8) -- Empty
    msg:dlc(8)-- sending 8 bytes of data
    return msg
end

-- -------------------------------- BATT --------------------------------
function SendHeartbeat()
    local msg = CANFrame()
    msg:id( (uint32_t(1) << 31) | (uint32_t(0x18) << 24) | (uint32_t(0x43) << 16) | (uint32_t(0x00) << 8) | (uint32_t(0xFE)) )
    msg:dlc(0)
    can_driver:write_frame(msg, uint32_t(100))
end

------------------------PF 22------------------------
-- handle PF22
---@param frame CANFrame_ud 
function HandleChargeRequestPF22(frame)
    local max_voltage = (frame:data(1)*256 + frame:data(0))/100.0
    local max_curr = (frame:data(3)*256 + frame:data(2))/100.0
    local max_cell_volt = (frame:data(5)*256 + frame:data(4))/1000.0
end

-----------------PF 24----------------------
-- handle PF24
---@param frame CANFrame_ud
function HandleAlarmInfoPF24(frame)
    gcs:send_named_float("BAT_AL0", frame:data(0))
    gcs:send_named_float("BAT_AL1", frame:data(1))
end

---------------PF 26 ----------------
--handle PF26
---@param frame CANFrame_ud
function HandleBattOperationPF26(frame)
    local current_voltage = (frame:data(1)*256 + frame:data(0))*0.01
    local current_amp_raw = (frame:data(3)*256 + frame:data(2))
    local current_amp_draw = 0
    if current_amp_raw == 0 then
        current_amp_draw = 0
    else
        current_amp_draw = (65535 - current_amp_raw)*0.01
    end
    local soc = frame:data(4)
    local soh = frame:data(5)
    local sop15 = (frame:data(7)*256 + frame:data(6))*10.0

    local num_cells = 14
    local battState = BattMonitorScript_State()
    battState:healthy(true)
    battState:voltage(current_voltage)
    battState:cell_count(num_cells)
    battState:capacity_remaining_pct(soc)
    battState:current_amps(current_amp_draw)
    for i=1, num_cells do
        battState:cell_voltages(i-1, math.floor(current_voltage/14.0))
    end
    battery:handle_scripting(0, battState)

    gcs:send_named_float("BAT_SOH", soh)
    --gcs:send_named_float("BAT_SOP", sop15)
end
-- parsing:
-- packet C4 16 92 DC 3E 64 00 00
-- voltage = 0x16 * 0x100 + C4 = 16C4*0.01 = 58.28
-- current = 0xFFFF - (0xDC*0x100 + 92) = 0xFFFF - 0xDC92 = 0x236D = 9096*0.01 = 90.96
-- Corrente funciona em complemento de 2
-- SOC = 0x3E = 62%
-- SOH = 0x64 = 100%

--handle Herewin Battery
---@param frame CANFrame_ud
function HandleHerewinBattery0(frame)
    local voltage = CombineBytes(frame:data(2), frame:data(3))*0.01
    local current = CombineBytes2Compl(frame:data(4), frame:data(5))*0.01
    local soc = math.floor(CombineBytes(frame:data(6), frame:data(7))*0.1)
    BATT_TEMP_STATE_HERE = BattMonitorScript_State()
    BATT_TEMP_STATE_HERE:healthy(true)
    BATT_TEMP_STATE_HERE:cell_count(12)
    BATT_TEMP_STATE_HERE:voltage(voltage)
    BATT_TEMP_STATE_HERE:capacity_remaining_pct(soc)
    BATT_TEMP_STATE_HERE:current_amps(current)
    IS_HEREWIN_30AH = false
end

function HandleHerewinBattery014S(frame)
    local voltage = CombineBytes(frame:data(2), frame:data(3))*0.01
    local current = CombineBytes2Compl(frame:data(4), frame:data(5))*0.01
    local soc = math.floor(CombineBytes(frame:data(6), frame:data(7))*0.1)
    BATT_TEMP_STATE_HERE = BattMonitorScript_State()
    BATT_TEMP_STATE_HERE:healthy(true)
    BATT_TEMP_STATE_HERE:cell_count(14)
    BATT_TEMP_STATE_HERE:voltage(voltage)
    BATT_TEMP_STATE_HERE:capacity_remaining_pct(soc)
    BATT_TEMP_STATE_HERE:current_amps(current)
    IS_HEREWIN_30AH = false
end

function HandleHerewinBattery1(frame)
    local temperature = CombineBytes(frame:data(0), frame:data(1))*0.1
    local status = CombineBytes(frame:data(2), frame:data(3))
    local cell1 = CombineBytes(frame:data(4), frame:data(5))
    local cell2 = CombineBytes(frame:data(6), frame:data(7))
    BATT_TEMP_STATE_HERE:temperature(temperature)
    BATT_TEMP_STATE_HERE:cell_voltages(0, cell1)
    BATT_TEMP_STATE_HERE:cell_voltages(1, cell2)
    IS_HEREWIN_30AH = false
end

function HandleHerewinBattery2(frame)
    local cell3 = CombineBytes(frame:data(0), frame:data(1))
    local cell4 = CombineBytes(frame:data(2), frame:data(3))
    local cell5 = CombineBytes(frame:data(4), frame:data(5))
    local cell6 = CombineBytes(frame:data(6), frame:data(7))
    BATT_TEMP_STATE_HERE:cell_voltages(2, cell3)
    BATT_TEMP_STATE_HERE:cell_voltages(3, cell4)
    BATT_TEMP_STATE_HERE:cell_voltages(4, cell5)
    BATT_TEMP_STATE_HERE:cell_voltages(5, cell6)
    IS_HEREWIN_30AH = false
end

function HandleHerewinBattery3(frame)
    local cell7 = CombineBytes(frame:data(0), frame:data(1))
    local cell8 = CombineBytes(frame:data(2), frame:data(3))
    local cell9 = CombineBytes(frame:data(4), frame:data(5))
    local cell10 = CombineBytes(frame:data(6), frame:data(7))
    BATT_TEMP_STATE_HERE:cell_voltages(6, cell7)
    BATT_TEMP_STATE_HERE:cell_voltages(7, cell8)
    BATT_TEMP_STATE_HERE:cell_voltages(8, cell9)
    BATT_TEMP_STATE_HERE:cell_voltages(9, cell10)
    IS_HEREWIN_30AH = false
end

function HandleHerewinBattery4(frame)
    local cell11 = CombineBytes(frame:data(0), frame:data(1))
    local cell12 = CombineBytes(frame:data(2), frame:data(3))
    BATT_TEMP_STATE_HERE:cell_voltages(10, cell11)
    BATT_TEMP_STATE_HERE:cell_voltages(11, cell12)
    IS_HEREWIN_30AH = false
    battery:handle_scripting(0, BATT_TEMP_STATE_HERE)
end

function HandleHerewinBattery14SEnd(frame)
    local cell11 = CombineBytes(frame:data(0), frame:data(1))
    local cell12 = CombineBytes(frame:data(2), frame:data(3))
    local cell13 = CombineBytes(frame:data(4), frame:data(5))
    local cell14 = CombineBytes(frame:data(6), frame:data(7))
    BATT_TEMP_STATE_HERE:cell_voltages(10, cell11)
    BATT_TEMP_STATE_HERE:cell_voltages(11, cell12)
    BATT_TEMP_STATE_HERE:cell_voltages(12, cell13)
    BATT_TEMP_STATE_HERE:cell_voltages(13, cell14)
    IS_HEREWIN_30AH = false
    battery:handle_scripting(0, BATT_TEMP_STATE_HERE)
end
-----------------------------------------------

function Update()

    local rng1 = RNGFND1TYPE:get()
    if rng1 == param_num_topradar_rfnd_backend or rng1 == param_num_nanoradar_rfnd_backend or
     rng1 == param_num_topradar_rfnd_can or rng1 == param_num_lightware_rfnd or rng1 == 0 then
        isMicrobrainRFND = false
    end

    if PRX1TYPE:get() == param_num_topradar_prx_backend or PRX1TYPE:get() == param_num_nanoradar_prx_backend or PRX1TYPE:get() == param_num_lightware_rfnd or PRX1TYPE:get() == 0 then
        isMicrobrainPRX = false
    end

    if not lua_rfnd_driver_found and isMicrobrainRFND then
        Setup_rfnd_sensor()
    end

    if not lua_prx_driver_found and isMicrobrainPRX then
        Setup_prx_sensor()
    end

    if batt_counter > 2000 then
        SendHeartbeat()
        batt_counter = 0 
    end

    batt_counter = batt_counter + 1
    local remaining = battery:capacity_remaining_pct(0)

    if remaining ~= nil and remaining <= 30 and FAIL_TIMER >= 5000 and remaining > 20 then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "Pouca Bateria, retornando para casa")
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "<SKY>Mode_set_reason:FAILSAFE_BAT\n")
        if not HasSwitchedToRTL then
            vehicle:set_mode(COPTER_RTL_MODE)
            HasSwitchedToRTL = true
        end
        FAIL_TIMER = 0
    end

    if remaining ~= nil and remaining <= 10 and LAND_TIMER >= 8000 then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "BATERIA FRACA - POUSANDO")
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "<SKY>Mode_set_reason:FAILSAFE_BAT\n")
        vehicle:set_mode(COPTER_LAND_MODE)
        LAND_TIMER = 0
    end

    FAIL_TIMER = FAIL_TIMER + 1
    LAND_TIMER = LAND_TIMER + 1
    if FAIL_TIMER >= 15000 or LAND_TIMER >= 15000 then
        FAIL_TIMER = 0
        LAND_TIMER = 0
    end

    local frame = can_driver:read_frame()
    if not frame then
        -- no frame to parse
        return
    end

    if (tostring(frame:id()) == "2147485468") then
        --gcs:send_text(MAV_SEVERITY_EMERGENCY, "PASSEI AQUI 2" .. sensor_select:get())
        Handle_prx_frame(frame, 0)
    end

    if (tostring(frame:id()) == "2147485484") then
        --gcs:send_text(MAV_SEVERITY_EMERGENCY, "PASSEI AQUI 3" .. sensor_select:get())
        Handle_prx_frame(frame, 180)
    end

     ------- WEIGHT MODULES CAN -------
     if tostring(frame:id()) == "2147518546" then -- Weight modules CAN frame
        local tank_weight = frame:data(1)*256 + frame:data(2)
        gcs:send_named_float('TANK_WEIGHT', tank_weight)
    end

    -- If 5 seconds have passed since last access of parameters, check if they have been changed
    if millis() - lastAccessParameters > 5000 then

        local graviParam = Parameter()
        if graviParam:init("USR_CALIB_GRAVI") then
            local graviCalibValue = graviParam:get()
            if graviCalibValue > 0 then
                CalibrateGravimetric(can_driver, graviCalibValue)
                graviParam:set_and_save(0)
            end
        end

        local tareParam = Parameter()
        if tareParam:init("USR_CALIB_TARE") then
            local shouldCalibTare = tareParam:get()
            if shouldCalibTare > 0 then
                CalibrateTare(can_driver)
                tareParam:set_and_save(0)
            end
        end
        
        lastAccessParameters = millis()
    end


    if (tostring(frame:id()) == "2147485500" or tostring(frame:id()) == "2147483649") then
        --gcs:send_text(MAV_SEVERITY_EMERGENCY, "PASSEI AQUI 1 " .. sensor_select:get())
        Handle_rfnd_frame(frame)
    end

    if(tostring(frame:id()) == "2525234179") then
        HandleHerewinBattery0(frame)
        return
    end

    if(tostring(frame:id()) == "2525234195") then
        HandleHerewinBattery1(frame)
        return
    end

    if(tostring(frame:id()) == "2525234211") then
        HandleHerewinBattery2(frame)
        return
    end

    if(tostring(frame:id()) == "2525234227") then
        HandleHerewinBattery3(frame)
        return
    end

    if(tostring(frame:id()) == "2525234251") then
        HandleHerewinBattery4(frame)
        return
    end

    if(tostring(frame:id()) == "2525758467") then
        --battery 14s herewin 0
        HandleHerewinBattery014S(frame)
        return
    end

    if(tostring(frame:id()) == "2525758483") then
        --battery 14s herewin 1
        HandleHerewinBattery1(frame)
        return
    end

    if(tostring(frame:id()) == "2525758499") then
        HandleHerewinBattery2(frame)
        --battery 14s herewin 2
        return
    end

    if(tostring(frame:id()) == "2525758515") then
        HandleHerewinBattery3(frame)
        --battery 14s herewin 3
        return
    end

    if(tostring(frame:id()) == "2525758539") then
        HandleHerewinBattery14SEnd(frame)
        --battery 14s herewin 4
        return
    end

    if(tostring(frame:id()) == "2552692224" and IS_HEREWIN_30AH) then
        --gcs:send_text(0, "BATTERY OPERATION")
        HandleBattOperationPF26(frame)
        return
    end

    if(tostring(frame:id()) == "2530477059" or tostring(frame:id()) == "2530477075" or
        tostring(frame:id()) == "2530477091" or tostring(frame:id()) == "2530477107" or
        tostring(frame:id()) == "2530477123" or tostring(frame:id()) == "2530477147") then
        -- BATT STATUS IGNORE
        return
    end

end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(Update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    --gcs:send_text(MAV_SEVERITY_EMERGENCY, string.format('PARAM VALUE update_rate_ms: ' .. tostring(update_rate_ms)))
    return protected_wrapper, loop_duration_ms
end

-- Start running update loop
return protected_wrapper()
