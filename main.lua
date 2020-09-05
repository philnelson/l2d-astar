local astar = require 'astar'
local astar8, astar4

local time = 0

local _floor, _abs, _sqrt = math.floor, math.abs, math.sqrt
local _sf = string.format

local mapw, maph = 32, 32
local mapx, mapy, mapcw = 5, 80, 20
local startx, starty = 0, 15
local goalx, goaly = 31, 15
local mapdata, mappath, visited = {}, nil, nil
local currentfinder

local costcolors = {
	[1] = {1, 1, 1},
	[2] = {0.5, 0.7, 0.7},
	[3] = {0.3, 0.4, 0.4},
	[4] = {0.1, 0.2, 0.2},
}

local mapget = function(x, y)
	local row = mapdata[y]
	if not row then return end
	return row[x]
end

local findpath = function()
	local start = mapget(startx, starty)
	local goal = mapget(goalx, goaly)
	visited = {}
	time = 0
	if start and goal then
		time = love.timer.getTime()
		mappath = currentastar:find(start, goal)
		time = (love.timer.getTime() - time) * 1000
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
			local c = love.math.random(0,4)
			if c ~= 0 then row[x] = {x = x, y = y, cost = c} end
		end
		mapdata[y] = row
	end
end


local mapdraw = function()
	local ox, oy, cw = mapx, mapy, mapcw
	
	love.graphics.setColor(1, 1, 1)
	love.graphics.rectangle("line", ox - 0.5, oy - 0.5, mapw * cw, maph * cw)
	
	for y, row in pairs(mapdata) do
		for x, node in pairs(row) do
			local c = node.cost
			love.graphics.setColor(1.25 - 0.25 * c, 1.20 - 0.20 * c, 1.15 - 0.15 * c)
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
		for _, v in ipairs(visited) do
			love.graphics.rectangle("line",
				v.x * cw + ox + 2.5, v.y * cw + oy + 2.5, cw - 5, cw - 5)
		end
	end

	if mappath then
		love.graphics.setColor(0, 1, 0)
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
	local n, e, s, w
	if rc then
		e, w = rc[x - 1], rc[x + 1]
		if e then t[#t + 1] = e end
		if w then t[#t + 1] = w end
	end
	if rt then
		n = rt[x]
		if n then
			t[#t + 1] = n
			if e and rt[x - 1] then t[#t + 1] = rt[x - 1] end
			if w and rt[x + 1] then t[#t + 1] = rt[x + 1] end
		end
	end
	if rb then
		s = rb[x]
		if s then
			t[#t + 1] = s
			if e and rb[x - 1] then t[#t + 1] = rb[x - 1] end
			if w and rb[x + 1] then t[#t + 1] = rb[x + 1] end
		end
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

local distance8 = function(context, from, to)
	local dx, dy = from.x - to.x, from.y - to.y
	return 0.5 * _sqrt(dx * dx + dy * dy) * (from.cost + to.cost)
end

local distance4 = function(context, from, to)
	local dx, dy = from.x - to.x, from.y - to.y
	return 0.5 * (from.cost + to.cost)
end

local heuristic8 = function(context, from, goal)
	visited[#visited + 1] = from
	--local dx, dy = from.x - goal.x, from.y - goal.y
	--return _sqrt(dx * dx + dy * dy) * 1.1
	--(dx + dy) + (sqrt(2) - 2) * min(dx, dy)
	local dx, dy = _abs(from.x - goal.x), _abs(from.y - goal.y)
	return (1 + 1 / mapw) * (dx + dy + (root2 - 2) * math.min(dx, dy))
end

local heuristic4 = function(context, from, goal)
	visited[#visited + 1] = from 
	return (1 + 1 / mapw) * (_abs(from.x - goal.x) + _abs(from.y - goal.y))
end

local nodetypes = {['s']='start', ['g']='goal', ['c']='clear', 
	['1']=1, ['2']=2, ['3']=3, ['4']=4}
local nodetype = 'start'

local mousedown1, cellx, celly, cellx1, celly1, cellnode

local function update_node(cx, cy)
	if not cx or not cy then return end
	
	local refresh = false
	if nodetype == 'start' then
		if startx ~= cx or starty ~= cy then
			refresh = true
			startx, starty = cx, cy
		end
	elseif nodetype == 'goal' then
		if goalx ~= cx or goaly ~= cy then
			refresh = true
			goalx, goaly = cx, cy
		end
	else
		local cost = nodetype
		if type(cost) ~= 'number' then cost = nil end
		local node = mapget(cx, cy)
		if node then
			if cost then
				refresh = (node.cost ~= cost) -- modify
				node.cost = cost
			else
				refresh = true -- remove
				mappop(cx, cy)
			end
		elseif cost then
			refresh = true -- new
			mapset(cx, cy, cost)
		end
	end

	if refresh then
		findpath()
	end
end
--]]
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
	love.graphics.print("Keys: [s]tart, [c]lear, [g]oal, Cost [1][2][3][4]", 5, 35)
	love.graphics.print("\t\t [space] toggle diagonal move", 5, 50)
	love.graphics.print(_sf("Time: %.3fms, Visited %s", time, #visited), 5, 65)

	mapdraw()
end
