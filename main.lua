local astar = require 'astar'
local astar8, astar4

local time = 0 -- average path finding time
local repeatcount = 100 -- (average) time = totaltime / repeatcount

local _floor, _abs, _sqrt = math.floor, math.abs, math.sqrt
local _sf = string.format

love.keyboard.setKeyRepeat(true)

local mapw, maph = 32, 32
local mapx, mapy, mapcw = 5, 80, 20
local startx, starty = 0, 15
local goalx, goaly = 31, 15
local tiebreaker = 1 + 1 / mapw -- to reduce search time "H := tiebreaker * H"
local mapdata, mappath = {}, nil
local numvisited, pathcost, costmap = 0, 0
local currentastar

local maxcost = 4

local mapget = function(data, x, y)
	local row = data[y]
	if not row then return end
	return row[x]
end

local mapset = function(data, x, y, value)
	local row = data[y]
	if not row then
		if not value then return end
		row = {}; data[y] = row
	end
	row[x] = value
end

 -- set costmap(x, y) and return (do not set if it exists already)
local setcost = function(x, y, cost)
	local row = costmap[y]
	if not row then
		row = {}; costmap[y] = row
	end
	
	local node = row[x]
	if node then return node end
	numvisited = numvisited + 1
	node = {x = x, y = y, cost = cost}
	row[x] = node
	return node
end

local path2polyline = function(path)
	local polyline, cw = {}, mapcw
	for i = 1, #path do
		polyline[2 * i - 1] = _floor(cw * (path[i].x + 0.5))
		polyline[2 * i    ] = _floor(cw * (path[i].y + 0.5))
	end
	return polyline
end

local findpath = function()
	local startcost = mapget(mapdata, startx, starty)
	local goalcost = mapget(mapdata, goalx, goaly)
	numvisited, pathcost, costmap = 0, 0, nil
	time = 0
	if startcost and goalcost then
		time = love.timer.getTime()
		for i = 1, repeatcount do
			costmap = {}
			local start = setcost(startx, starty, startcost)
			local goal  = setcost(goalx, goaly, goalcost)
			mappath, pathcost = currentastar:find(start, goal)
		end
		time = ((love.timer.getTime() - time) * 1000) / repeatcount
		numvisited = numvisited / repeatcount
		if mappath then mappath.polyline = path2polyline(mappath) end
	else
		mappath = nil
	end
end


local mapfill = function(data)
	for y = 0, maph - 1 do
		local row = {}
		for x = 0, mapw - 1 do
			local c = love.math.random(0, maxcost)
			if c >= 1 then row[x] = c end
		end
		data[y] = row
	end
end

-- for cost based coloring
local cR, cG, cB = (1 - 0.2) / (maxcost - 1), (1 - 0.4) / (maxcost - 1), (1 - 0.5) / (maxcost - 1)

local mapdraw = function()
	local ox, oy, cw = mapx, mapy, mapcw
	love.graphics.translate(ox, oy)
	
	love.graphics.setColor(1, 1, 1)
	love.graphics.rectangle("line", -0.5, -0.5, mapw * cw + 1, maph * cw + 1)
	
	for y, row in pairs(mapdata) do
		for x, id in pairs(row) do
			local c = id
			love.graphics.setColor(1 + cR * (1 - c), 1 + cG * (1 - c), 1 + cB * (1 - c))
			love.graphics.rectangle("fill", x * cw + 1, y * cw + 1, cw - 2, cw - 2)
		end
	end
	
	if costmap then
		love.graphics.setColor(1, 0, 0)
		for y, row in pairs(costmap) do
			for x, node in pairs(row) do
				love.graphics.rectangle("fill", x * cw + 1, y * cw + 1, 3, 3)
			end
		end
	end

	love.graphics.setColor(0, 0, 1)
	love.graphics.circle("fill", (goalx + 0.5) * cw, (goaly + 0.5) * cw, cw / 3)

	love.graphics.setColor(0, 1, 0)
	love.graphics.circle("fill", (startx + 0.5) * cw, (starty + 0.5) * cw, cw / 3)

	if mappath then
		love.graphics.setColor(0, 0, 1)
		love.graphics.line(mappath.polyline)
	end
	love.graphics.translate(-ox, -oy)
end

local neighbors8 = function(context, node)
	-- It is likely that we don't need to check this with A*
	if node.neighbors then return node.neighbors end
	
	local x, y = node.x, node.y
	local rt, rc, rb = mapdata[y - 1], mapdata[y], mapdata[y + 1]
	local t = {}
	local w, e
	if rc then
		e, w = rc[x - 1], rc[x + 1]
		if e then t[#t + 1] = setcost(x - 1, y, e) end
		if w then t[#t + 1] = setcost(x + 1, y, w) end
	end
	if rt and rt[x] then
		t[#t + 1] = setcost(x, y - 1, rt[x])
		if e and rt[x - 1] then t[#t + 1] = setcost(x - 1, y - 1, rt[x - 1]) end
		if w and rt[x + 1] then t[#t + 1] = setcost(x + 1, y - 1, rt[x + 1]) end
	end
	if rb and rb[x] then
		t[#t + 1] = setcost(x, y + 1, rb[x])
		if e and rb[x - 1] then t[#t + 1] = setcost(x - 1, y + 1, rb[x - 1]) end
		if w and rb[x + 1] then t[#t + 1] = setcost(x + 1, y + 1, rb[x + 1]) end
	end

	node.neighbors = t
	return t
end

local neighbors4 = function(context, node)
	if node.neighbors then return node.neighbors end

	local x, y = node.x, node.y
	local rt, rc, rb = mapdata[y - 1], mapdata[y], mapdata[y + 1]
	local t = {}
	
	if rc then
		if rc[x - 1] then t[#t + 1] = setcost(x - 1, y, rc[x - 1]) end
		if rc[x + 1] then t[#t + 1] = setcost(x + 1, y, rc[x + 1]) end
	end
	if rt and rt[x] then t[#t + 1] = setcost(x, y - 1, rt[x]) end
	if rb and rb[x] then t[#t + 1] = setcost(x, y + 1, rb[x]) end

	node.neighbors = t
	return t
end

local root2 = _sqrt(2)

local distance8 = function(context, current, neighbor)
	if current.x == neighbor.x or current.y == neighbor.y then
		return 0.5 * (current.cost + neighbor.cost)
	end
	return 0.5 * root2 * (current.cost + neighbor.cost)
end

local distance4 = function(context, current, neighbor)
	return 0.5 * (current.cost + neighbor.cost)
end

local heuristic8 = function(context, from, goal)
	--visited[from] = true; numvisited = numvisited + 1
	local dx, dy = _abs(from.x - goal.x), _abs(from.y - goal.y)
	return tiebreaker * (dx + dy + (root2 - 2) * math.min(dx, dy))
end

local heuristic4 = function(context, from, goal)
	--visited[from] = true; numvisited = numvisited + 1
	return tiebreaker * (_abs(from.x - goal.x) + _abs(from.y - goal.y))
end

local nodetypes = {['s'] = 'start', ['g'] = 'goal', ['c'] = 'clear'}
for i = 1, maxcost do nodetypes[tostring(i)] = i end 
local nodetype = 'start'

local refreshmap

local function update_node(cx, cy)
	if not cx or not cy then return end
	
	if nodetype == 'start' then
		if startx ~= cx or starty ~= cy then
			refreshmap = true
			startx, starty = cx, cy
		end
	elseif nodetype == 'goal' then
		if goalx ~= cx or goaly ~= cy then
			refreshmap = true
			goalx, goaly = cx, cy
		end
	else
		local newcost = nodetype
		if type(newcost) ~= 'number' then newcost = nil end
		local oldcost = mapget(mapdata, cx, cy)
		if oldcost ~= newcost then
			refreshmap = true
			mapset(mapdata, cx, cy, newcost)
		end
	end
	--return refreshmap
end

local arrowkeymove = {left = {-10, 0}, right = {10, 0},
	up = {0, -10}, down = {0, 10}}

function love.keyreleased(k)
	if k == "escape" then return love.event.quit() end
	if k == "space" then
		if currentastar == astar8 then currentastar = astar4
		else currentastar = astar8 end
		findpath()
		return
	end
	local nt = nodetypes[k]
	if nt then nodetype = nt end
end

function love.keypressed(k, isrepeat)
	local move = arrowkeymove[k]
	if move then
		mapx, mapy = mapx + move[1], mapy + move[2]
	end
end

local mousedown1, mousedown2, cellx, celly, cellx1, celly1, cellnode

function love.wheelmoved(x, y)
	y = _floor(y)
	local newcw = mapcw + y
	if newcw > 64 then newcw = 64
	elseif newcw < 10 then newcw = 10 end
	if newcw ~= mapcw then
		mapcw = newcw
		if cellx and celly and y then
			mapx = mapx - y * cellx
			mapy = mapy - y * celly
		end
		if mappath then mappath.polyline = path2polyline(mappath) end
	end
end

function love.mousepressed(mx,my,b)
	if b == 1 then
		cellx1, celly1, mousedown1 = cellx, celly, true
		return
	end
	if b == 2 then
		mousedown2 = true
	end
end

function love.mousereleased(mx, my, b)
	if b == 1 then
		mousedown1 = false
		if cellx1 == cellx and celly1 == celly then
			update_node(cellx, celly)
		end
		if refreshmap then findpath(); refreshmap = false end
		return
	end
	if b == 2 then
		mousedown2 = false
	end
end

function love.mousemoved(mx, my, dx, dy)
	local cw = mapcw
	cellx, celly = _floor((mx - mapx) / cw), _floor((my - mapy) / cw)
	if cellx >= mapw or celly >= maph or cellx < 0 or celly < 0 then
		cellx, celly, cellnode = nil, nil, nil
	else
		cellnode = mapget(mapdata, cellx, celly)
	end
	if mousedown1 then
		if cellx and celly then update_node(cellx, celly) end
	elseif mousedown2 then
			mapx, mapy = mapx + dx, mapy + dy
	end
end

mapfill(mapdata)
astar8 = astar.new(nil, neighbors8, distance8, heuristic8)
astar4 = astar.new(nil, neighbors4, distance4, heuristic4)

currentastar = astar8
findpath()

love.draw = function()
	mapdraw()
	
	love.graphics.setColor(1, 1, 1)
	love.graphics.print(_sf("Cell (%s, %s) Cost: %s",
		cellx, celly, cellnode), 5, 5)
	love.graphics.print(_sf("Nodetype: %s", nodetype), 5, 20)
	love.graphics.print(_sf("Keys: [s]tart, [c]lear, [g]oal, Cost [1]...[%i]", maxcost), 5, 35)
	love.graphics.print("\t\t [space] toggle diagonal move", 5, 50)
	love.graphics.print(_sf("Time: %.3fms, Visited: %i, Path cost: %.3f",
		time, numvisited or 0, pathcost or 0), 5, 65)
end
