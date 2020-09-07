local astar = require 'astar'
local astar8, astar4

local time = 0 -- average path finding time
local repeatcount = 100 -- (average) time = totaltime / repeatcount

local _floor, _abs, _sqrt = math.floor, math.abs, math.sqrt
local _sf = string.format

local mapw, maph = 32, 32
local mapx, mapy, mapcw = 5, 80, 20
local startx, starty = 0, 15
local goalx, goaly = 31, 15
local tiebreaker = 1 + 1 / mapw -- to reduce search time "H := tiebreaker * H"
local mapdata, mappath, visited = {}, nil, nil
local numvisited, pathcost = 0, 0
local currentastar

local maxcost = 4

local mapget = function(x, y)
	local row = mapdata[y]
	if not row then return end
	return row[x]
end

local findpath = function()
	local start = mapget(startx, starty)
	local goal = mapget(goalx, goaly)
	visited = {}
	numvisited, pathcost = 0, 0
	time = 0
	if start and goal then
		time = love.timer.getTime()
		for i = 1, repeatcount do
			mappath, pathcost = currentastar:find(start, goal)
		end
		time = ((love.timer.getTime() - time) * 1000) / repeatcount
		numvisited = numvisited / repeatcount
	else
		mappath = nil
	end
end

local mappop = function(x, y)
	local row = mapdata[y]
	if not row then return end
	
	local node = row[x]
	if not node then return end
	row[x] = nil
	return node
end

local mapset = function(x, y, cost)
	local row = mapdata[y]
	if not row then
		if not cost then return end
		row = {}; mapdata[y] = row
	end
	
	local node = row[x]
	if not node then
		if not cost then return end
		node = {}; row[x] = node
	elseif not cost then
		row[x] = nil
	end
	
	node.x, node.y, node.cost = x, y, cost
	return node
end

local mapfill = function()
	for y = 0, maph - 1 do
		local row = {}
		for x = 0, mapw - 1 do
			local c = love.math.random(0, maxcost)
			if c >= 1 then row[x] = {x = x, y = y, cost = c} end
		end
		mapdata[y] = row
	end
end

-- for cost based coloring
local cR, cG, cB = (1 - 0.2) / (maxcost - 1), (1 - 0.4) / (maxcost - 1), (1 - 0.5) / (maxcost - 1)

local mapdraw = function()
	local ox, oy, cw = mapx, mapy, mapcw
	
	love.graphics.setColor(1, 1, 1)
	love.graphics.rectangle("line", ox - 0.5, oy - 0.5, mapw * cw + 1, maph * cw + 1)
	
	for y, row in pairs(mapdata) do
		for x, node in pairs(row) do
			local c = node.cost
			love.graphics.setColor(1 + cR * (1 - c), 1 + cG * (1 - c), 1 + cB * (1 - c))
			love.graphics.rectangle("fill",
				x * cw + ox + 1, y * cw + oy + 1, cw - 2, cw - 2)
		end
	end
	
	love.graphics.setColor(0, 0, 1)
	love.graphics.circle("fill", (goalx + 0.5) * cw + ox, (goaly + 0.5) * cw + oy, cw / 3)

	love.graphics.setColor(0, 1, 0)
	love.graphics.circle("fill", (startx + 0.5) * cw + ox, (starty + 0.5) * cw + oy, cw / 3)

	if visited then
		love.graphics.setColor(1, 0, 0)
		for v, _ in pairs(visited) do
			love.graphics.rectangle("line",
				v.x * cw + ox + 2.5, v.y * cw + oy + 2.5, cw - 5, cw - 5)
		end
	end

	if mappath then
		love.graphics.setColor(0, 0, 1)
		local x1, y1 = (mappath[1].x + 0.5) * cw + ox, (mappath[1].y + 0.5) * cw + oy
		local x2, y2
		for i = 2, #mappath do
			x2, y2 = (mappath[i].x + 0.5) * cw + ox, (mappath[i].y + 0.5) * cw + oy
			love.graphics.line(x1, y1, x2, y2)
			x1, y1 = x2, y2
		end
	end
end

local neighbors8 = function(context, node)
	local x, y = node.x, node.y
	local rt, rc, rb = mapdata[y - 1], mapdata[y], mapdata[y + 1]
	local t = {}
	local w, e
	if rc then
		e, w = rc[x - 1], rc[x + 1]
		if e then t[#t + 1] = e end
		if w then t[#t + 1] = w end
	end
	if rt and rt[x] then
		t[#t + 1] = rt[x]
		if e and rt[x - 1] then t[#t + 1] = rt[x - 1] end
		if w and rt[x + 1] then t[#t + 1] = rt[x + 1] end
	end
	if rb and rb[x] then
		t[#t + 1] = rb[x]
		if e and rb[x - 1] then t[#t + 1] = rb[x - 1] end
		if w and rb[x + 1] then t[#t + 1] = rb[x + 1] end
	end

	return t
end

local neighbors4 = function(context, node)
	local x, y = node.x, node.y
	local rt, rc, rb = mapdata[y - 1], mapdata[y], mapdata[y + 1]
	local t = {}
	
	if rc then
		if rc[x - 1] then t[#t + 1] = rc[x - 1] end
		if rc[x + 1] then t[#t + 1] = rc[x + 1] end
	end
	if rt and rt[x] then t[#t + 1] = rt[x] end
	if rb and rb[x] then t[#t + 1] = rb[x] end

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
	visited[from] = true; numvisited = numvisited + 1
	local dx, dy = _abs(from.x - goal.x), _abs(from.y - goal.y)
	return tiebreaker * (dx + dy + (root2 - 2) * math.min(dx, dy))
end

local heuristic4 = function(context, from, goal)
	visited[from] = true; numvisited = numvisited + 1
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
		local cost = nodetype
		if type(cost) ~= 'number' then cost = nil end
		local node = mapget(cx, cy)
		if node then
			if cost then
				if node.cost ~= cost then -- modify cost
					refreshmap = true
					node.cost = cost
				end
			else  -- remove node
				refreshmap = true
				mappop(cx, cy)
			end
		elseif cost then -- add node
			refreshmap = true
			mapset(cx, cy, cost)
		end
	end
	--return refreshmap
end

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

local mousedown1, cellx, celly, cellx1, celly1, cellnode

function love.mousepressed(mx,my,b)
	if b == 1 then
		cellx1, celly1, mousedown1 = cellx, celly, true
	end
end

function love.mousereleased(mx, my, b)
	if b == 1 then
		mousedown1 = false
		if cellx1 == cellx and celly1 == celly then
			update_node(cellx, celly)
		end
		if refreshmap then findpath(); refreshmap = false end
	end
end

function love.mousemoved(mx, my)
	local cw = mapcw
	cellx, celly = _floor((mx - mapx) / cw), _floor((my - mapy) / cw)
	if cellx >= mapw or celly >= maph or cellx < 0 or celly < 0 then
		cellx, celly, cellnode = nil, nil, nil
	else
		cellnode = mapget(cellx, celly)
	end
	if mousedown1 then
		update_node(cellx, celly)
	end
end

mapfill()
astar8 = astar.new(nil, neighbors8, distance8, heuristic8)
astar4 = astar.new(nil, neighbors4, distance4, heuristic4)

currentastar = astar8
findpath()

love.draw = function()
	love.graphics.setColor(1, 1, 1)
	love.graphics.print(_sf("Cell (%s, %s) Cost: %s",
		cellx, celly, cellnode and cellnode.cost or "inf"), 5, 5)
	love.graphics.print(_sf("Nodetype: %s", nodetype), 5, 20)
	love.graphics.print(_sf("Keys: [s]tart, [c]lear, [g]oal, Cost [1]...[%i]", maxcost), 5, 35)
	love.graphics.print("\t\t [space] toggle diagonal move", 5, 50)
	love.graphics.print(_sf("Time: %.3fms, Visited: %i, Path cost: %.3f",
		time, numvisited or 0, pathcost or 0), 5, 65)

	mapdraw()
end
