local inf = 1 / 0
local insert = table.insert

local function findlowest(set, score)
	local min, lowest = inf, nil

	for node, _ in pairs(set) do
		local score = score[node]
		if score < min then
			min, lowest = score, node
		end
	end
	
	return lowest
end

local function reconstruct(camefrom, current)
	local path = {current}
	while camefrom[current] do
		current = camefrom[current]
		insert(path, current)
	end
	return path
end


-- This version uses a closed set
local find_wc = function(astar, start, goal)
	local ctx = astar.context
	local neighbors, d, h = astar.neighbors, astar.distance, astar.heuristic
	local openset = {[start] = true}
	
	local closedset = {}
	local camefrom = {}

	local gscore = {[start] = 0}
	local hscore = {[start] = h(ctx, start, goal)} -- cached hscore
	local fscore = {[start] = hscore[start]}

	while next(openset) do
		local current = findlowest(openset, fscore)

		if current == goal then
			return reconstruct(camefrom, goal)
		end
		openset[current] = nil
		closedset[current] = true

		local from_node = camefrom[current]
		local tentative_gscore
		for _, neighbor in ipairs(neighbors(ctx, current)) do
			if not closedset[neighbor] then
				tentative_gscore = gscore[current] + d(ctx, current, neighbor)

				if not gscore[neighbor] or tentative_gscore < gscore[neighbor] then
					camefrom[neighbor] = current
					gscore[neighbor] = tentative_gscore
					if not hscore[neighbor] then
						hscore[neighbor] = h(ctx, neighbor, goal)
					end
					fscore[neighbor] = tentative_gscore + hscore[neighbor]

					openset[neighbor] = true
				end
			end
		end
	end

	return nil
end

-- This version does not use a closed set
local find_nc = function(astar, start, goal)
	local ctx = astar.context
	local neighbors, d, h = astar.neighbors, astar.distance, astar.heuristic

	local openset = {[start] = true}
	local camefrom = {}

	local gscore = {[start] = 0}
	local hscore = {[start] = h(ctx, start, goal)} -- cached hscore
	local fscore = {[start] = hscore[start]}

	while next(openset) do
		local current = findlowest(openset, fscore)

		if current == goal then
			return reconstruct(camefrom, goal)
		end
		openset[current] = nil

		local from_node = camefrom[current]
		local tentative_gscore
		for _, neighbor in ipairs(neighbors(ctx, current)) do
			tentative_gscore = gscore[current] + d(ctx, current, neighbor)

			if not gscore[neighbor] or tentative_gscore < gscore[neighbor] then
				camefrom[neighbor] = current
				gscore[neighbor] = tentative_gscore
				if not hscore[neighbor] then
					hscore[neighbor] = h(ctx, neighbor, goal)
				end
				fscore[neighbor] = tentative_gscore + hscore[neighbor]

				openset[neighbor] = true
			end
		end
	end

	return nil
end

local M = {}
M.find = find_wc
M.new = function(Map, Neighbors, Distance, Heuristic)
	local astar = {
		map = Map,
		neighbors = Neighbors,
		distance = Distance,
		heuristic = Heuristic,
	}
	
	astar.find = find_wc
	
	return astar
end

return M
