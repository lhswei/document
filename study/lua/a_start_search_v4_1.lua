-- auther: luhengsi
-- date:   2018-04-12
-- update: 2018-04-20

-- a * 寻路算法
-- 基本思想：
-- 1. 建立一个开放列表和一个关闭列表
-- 2. 将其点放入开放列表
-- 3. 从开放列表中取一个预估到达终点路径最短（F值最小）的点：point_current
-- 	  将这个点从开放列表中移除并放入关闭列表
-- 	  	开放列表中计算到达终点路径的计算方：
-- 		a. 起点到达这个点的移动路径值之和G（若斜线方向上点四周没有障碍则可通行，但步长比平行和上下移动距离要远一点）
-- 		b. 这个点到达终点的移动路径值之和H（不可斜线通行，不考虑障碍），对于任何点，H值是不变的
-- 		c. 预估到达终点的路径值假设为 F = G + H
--		d. 其点的 G = 0
-- 4. 获取 point_current 点临近的非障碍点，并对每个点做一下处理：
--    	a. 如果这个点在关闭列表中存在，则什么都不做
-- 		b. 如这个点在开放列表中则更新这个点的G值，更新规则：
-- 		   取原来的G值G1， 这个点到point_current需要的G值加上point_current本身的G值得到G2
-- 		   如果G1不比G2大，则什么都不做
-- 		   如果G2比G1小，这个点的G设置为G2，并把它的父节点更新为point_current
--      c. 如果这个点不在开放列表，则计算这个点的G 和 H值
-- 		   G的计算是这个点到point_current需要的G值加上point_current本身的G值得到的
--		   并把point_current设置为他的父节点。
-- 5. 重复步骤3和步骤4，直到：
-- 		a. 找到终点，结束查找，通过反查找终点父节点一直到起点便是寻路路径。
-- 		b. 开放列表没有点了，查找失败，没有路径可以从起点到达终点。

-- lua 代码实现:
-- 设计5个类：Point, PathPoint, PathList, AStart, Map
-- Point: 地图点点坐标 {x, y, block}, x坐标， y坐标， block障碍物标记
--      	  类方法处理获取和设置x, y, block 的值外，还有计算的指定点到这个点的G和H值
-- PathPoint：辅助计算移动的路径点，包含一个Point的实例点，和查找过程中记录的G和H值
-- 			  还有指向父节点的成员变量
-- PathList： 开放列表和关闭列表的抽象数据结构，用来保存PathPoint类的实例
-- 			  有基本的增加、查找和删除操作
-- AStart：   算法抽象类型， 主要用于寻路查找计算过程中的技术和数据保存
--  		  find_path 为核心的寻路a*寻路算法
-- Map：      地图的抽象类型，包含一个二维坐标的矩阵 Matrix和一个AStart类的对象，用于a*寻路查找
--  		  

local fn_abs = abs
local fn_max = max
local fn_min = min
local fn_remove = tremove
local fn_insert = tinsert
local fn_format = format
local fn_concat = concat
local fn_getn = getn
local DEF_G1_STEP = 10			-- 直线移动值
local DEF_G2_STEP = 14			-- 斜线移动值


--=================================================
-- 模拟lua5里面的 unpcak
function unpack(table, i)
	i = i or 1
	if table[i] then
		return table[i], unpack(table, i + 1)
	end
end


-- 模拟lua5里面的 setmetatable
function setmetatable(obj, base)
	if not obj.__tag then
		obj.__tag = newtag()
	end
	
	obj.__parent = base
	settag(obj, obj.__tag)
	
	local tm = gettagmethod(obj.__tag, "index")
	if not tm then
		local foo = function(table, key)
			local val = rawget (table, key) -- 访问类本身
			if val then return key end -- 如果自身有则直接返回	

			return table.__parent[key]
		end
		settagmethod(obj.__tag, "index", foo)
	end
end


local class = function(base_class)
	if not base then
		__foo = function(obj,...)
			local parent = rawget(obj, "__parent")
			if parent then
				__foo(parent, unpack(arg))	-- 递归调用，保证最先调用最底层父类的__init__
			end
			local __init__ = rawget(obj, "__init__")
			if not __init__ then
				print("undefine __init__.")
				assert(false)	-- 子类必须实现自己的 __init__
			end
			call(__init__, arg)
		end
		
		base = {}	-- 所有类的基类
	
		function base:__init__(...)
			-- print(self, "base:__init__", unpack(arg))
		end
		
		base.new = function(...)
			local self = arg[1]
			local obj = {}
			
			setmetatable(obj, self)
			
			arg[1] = obj
			__foo(self, unpack(arg))
			return obj
		end
	end
	
	base_class = base_class or base
	
	local new_class = {}
	new_class.__tag = base_class.__tag
	setmetatable(new_class, base_class)
	
	return new_class;
end

-- local class_a = class()
-- function class_a:__init__(name)
	-- print(self, "class_a:__init__", name)
-- end

-- function class_a:test()
	-- print(self, "a")
-- end
-- local obj_a = class_a:new("a")

-- local class_b = class()
-- function class_b:__init__(name)
	-- print(self, "class_b:__init__", name)
-- end
-- obj_b = class_b:new("b")

-- local class_c = class(class_a)
-- function class_c:__init__(name)
	-- print(self, "class_c:__init__", name)
-- end
-- obj_c = class_c:new("c")
-- obj_c:test()
-- print(111, obj_c.sk)
--============================================================
-- point 
local Point = class()

function Point:__init__(x, y, is_barrier)
	self.pos_x = x or 0
	self.pox_y = y or 0
	self.is_barrier = is_barrier or 0
end

function Point:get_x()
	return self.pos_x
end

function Point:set_x(x)
	self.pos_x = x or 0
end

function Point:get_y()
	return self.pox_y
end

function Point:set_y(y)
	self.pox_y = y or 0
end

function Point:is_samexy(point)
	return ((self.pos_x == point.pos_x) and (self.pox_y == point.pox_y))
end

function Point:set_barrier(is_barrier)
	self.is_barrier = is_barrier or 0
end

-- calc H value
function Point:calc_h(point)
	local step = %fn_abs(point.pos_x - self.pos_x) + %fn_abs(point.pox_y - self.pox_y)
	return step * %DEF_G1_STEP
end

-- calc G value
function Point:calc_g(point)
	local xstep = %fn_abs(point.pos_x - self.pos_x)
	local ystep = %fn_abs(point.pox_y - self.pox_y)
	local step1 = %fn_max(xstep, ystep)
	local step2 = %fn_min(xstep, ystep)

	return step2 * %DEF_G2_STEP + (step1 - step2) * %DEF_G1_STEP
end

--==================================================================================
-- path point
local PathPoint = class()

function PathPoint:__init__(point)
	self.point = point
	self.parent_ppoint = nil
	self.value_h = 0
	self.value_g = 0
end

function PathPoint:get_point()
	return self.point
end

function PathPoint:set_parent(path_point)
	self.parent_ppoint = path_point
end

function PathPoint:get_parent()
	return self.parent_ppoint
end

function PathPoint:get_x()
	return self.point.pos_x
end

function PathPoint:get_y()
	return self.point.pox_y
end

function PathPoint:get_h()
	return self.value_h
end

function PathPoint:set_h(h)
	self.value_h = h
end

function PathPoint:get_g()
	return self.value_g
end

function PathPoint:set_g(g)
	self.value_g = g
end

function PathPoint:calc_f()
	return self.value_h + self.value_g
end

function PathPoint:is_samexy(point)
	return self.point:is_samexy(point)
end

function PathPoint:update_data(parent, value_h, value_g)
	self.parent_ppoint = parent
	self.value_h = value_h
	self.value_g = value_g
end

function PathPoint:update_g(path_point_current)
	local point_current = path_point_current:get_point()
	local g = point_current:calc_g(self.point) + path_point_current:get_g()
	if g < self:get_g() then
		self:set_g(g)
		self:set_parent(path_point_current)
	end
end

--======================================================
-- path list
local PathList = class()

function PathList:__init__()
	self.list = {}
end

function PathList:get_min_ppoint()
	local min_path_point, min_f
	for i = 1, %fn_getn(self.list) do 
		local path_point = self.list[i]
		local f = path_point:calc_f()
		if not min_path_point or f < min_f then
			min_path_point = path_point
			min_f = f
		end
	end
		
	return min_path_point
end

function PathList:count()
	local n = %fn_getn(self.list)
	return n
end

function PathList:add(path_point)
	%fn_insert(self.list, path_point)
end
	
function PathList:remove(path_point)
	for i = 1, %fn_getn(self.list) do 
		local v_path_point = self.list[i]
		local point = v_path_point:get_point()
		if path_point:is_samexy(point) then
			%fn_remove(self.list, i)
			break
		end
	end
end
	
function PathList:get_exist(point)
	for i = 1, %fn_getn(self.list) do 
		local path_point = self.list[i]
		if path_point:is_samexy(point) then 
			return path_point
		end
	end
	return nil
end
--=========================================================================================
-- a start
local AStart = class()
function AStart:__init__(map)
	self.map = map
	local origin = map:get_point(0, 0);
	self:reset(origin, origin)
end

function AStart:reset(point_start, point_des)
	self.point_list = {}
	self.point_start = point_start
	self.point_des = point_des
	self.is_find = 0
	self.ppoint_start = %PathPoint:new(point_start)
	self.ppoint_des = %PathPoint:new(point_des)
	self.open_list = %PathList:new()					-- 开放列表
	self.close_list = %PathList:new()					-- 关闭列表
end

function AStart:find_path(point_start, point_des)
	self:reset(point_start, point_des)
	local open_list = self.open_list
	local close_list = self.close_list
	open_list:add(self.ppoint_start)
	
	while (open_list:count() > 0) do 
		local path_point_current = open_list:get_min_ppoint()		-- 从开启列表中取出F值最小的路径点
		open_list:remove(path_point_current)					-- 从开启列表中删除这个点
		close_list:add(path_point_current)						-- 并加入到关闭列表
		
		local point_current = path_point_current:get_point()
		
		local surround_list = self.map:surround_points(point_current) 	-- 获取周围可通行的点
		local n = %fn_getn(surround_list)
		for i = 1, n do 
			local s_point = surround_list[i]
			self:calc_and_updata(path_point_current, s_point)			-- 计算并更新路径点数据
			if self.is_find == 1 then
				return self.point_list
			end
		end
	end
	
	return self.point_list
end

function AStart:calc_and_updata(path_point_current, s_point)
	local open_list = self.open_list
	local close_list = self.close_list
	local ppoint_start = self.ppoint_start
	local ppoint_des = self.ppoint_des
	
	local point_current = path_point_current:get_point()
	-- 如果是终点，则更新终点路径点父节点，结束查找
	if ppoint_des:is_samexy(s_point) then
		local value_h = 0
		local value_g = point_current:calc_g(s_point) + path_point_current:get_g()
		ppoint_des:update_data(path_point_current, value_h, value_g)
		self.is_find = 1
		close_list:add(ppoint_start)
		self:make_list()
		return
	end
			
	-- 如果此路径点的不在关闭列表
	if not close_list:get_exist(s_point) then
		local exist_point = open_list:get_exist(s_point)
		if exist_point then
			-- 如果此路径点在开放列表，则更新相关参数
			exist_point:update_g(path_point_current)
		else
			-- 如果不在开放列表，则计算并更新参数，并加入开放列表
			local value_h = self.point_des:calc_h(s_point)
			local value_g = point_current:calc_g(s_point) + path_point_current:get_g()
			local path_s_point = %PathPoint:new(s_point)
			path_s_point:update_data(path_point_current, value_h, value_g)
			open_list:add(path_s_point)
		end
	end
end

function AStart:make_list()
	self.point_list = {}
	local temp_path_point = self.ppoint_des
	while (temp_path_point) do 
		local temp_point = temp_path_point:get_point()
		temp_path_point = temp_path_point:get_parent()
		local x = temp_point:get_x()
		local y = temp_point:get_y()
		local n = %fn_getn(self.point_list);
		self.point_list[n + 1] = {x, y}
	end
	
	return self.point_list
end

--======================================================
-- map
local Map = class()
-- 八个方向
Map.dirs = {{-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}}

function Map:__init__(matrix)
	self.map_points = {}
	for y = 1, %fn_getn(matrix) do 
		local list_y = matrix[y]
		for x = 1, %fn_getn(list_y) do 
			local is_barrier = list_y[x]
			local point = %Point:new(x, y, is_barrier)
			self.map_points[x] = self.map_points[x] or {}
			self.map_points[x][y] = point
		end
	end
	self.a_start = %AStart:new(self)
end

function Map:get_point(x, y)
	if self.map_points[x] then
		return self.map_points[x][y]
	end
	
	return nil
end

-- 检查点 point_s 是否能移动到 point_e
-- point_s 和 point_e 一定是相邻的点，包括斜线上的点
-- 1. 若不是相邻的点则返回 0
-- 2. 若 point_e 是障碍点，则返回 0
-- 3. 若 point_e 不是障碍点，此时 point_s 和 point_e 的x, y 都不相等，且他们的公共相邻点是障碍，返回0
-- 4. 其他返回 1
function Map:check_can_move(point_s, point_e)
	local diff_x = point_s:get_x() - point_e:get_x()
	local diff_y = point_s:get_y() - point_e:get_y()
	
	if %fn_abs(diff_x) > 1 or %fn_abs(diff_y) > 1 then
		return 0
	end
	
	if point_e.is_barrier == 1 then
		return 0
	end
	
	if  diff_x ~= 0 and diff_y ~= 0 then
		local point_x = self.map_points[point_e:get_x()][point_s:get_y()]
		if point_x.is_barrier == 1 then
			return 0
		end
		
		local point_y = self.map_points[point_s:get_x()][point_e:get_y()]
		if point_y.is_barrier == 1 then
			return 0
		end
	end
	
	return 1
end

-- 获得指定点周围可以通行的点，若此点的水平和垂直方向上的相邻点是障碍，则这个方向上的斜线不可通过。
function Map:surround_points(point)
	local surround_list = {}
	local pos_x = point:get_x()
	local pos_y = point:get_y()
		
	local count = 0
	for i = 1, %fn_getn(self.dirs) do 
		local dir = self.dirs[i]
		local point_e = self:get_point(pos_x + dir[1], pos_y + dir[2])
		if point_e and self:check_can_move(point, point_e) == 1 then
			count = count + 1
			surround_list[count] = point_e
		end
	end

	return surround_list
end

function Map:find_path(start, des)
	local point_start = self:get_point(start[1], start[2])
	local point_des = self:get_point(des[1], des[2])
	if not point_start or not point_des then
		print("start or des erro.")
		return nil
	end
	
	if point_start.is_barrier == 1 or point_des.is_barrier == 1 then
		print("start or des is barrier.")
		return nil
	end
	
	local point_list = self.a_start:find_path(point_start, point_des)
	return point_list
end

-- 地图矩阵 [y][x] = is_barrier
local Matrix = {
	[1] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[2] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[3] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[4] = {[1] = 0, [2] = 0, [3] = 0, [4] = 1, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[5] = {[1] = 0, [2] = 0, [3] = 0, [4] = 1, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[6] = {[1] = 0, [2] = 0, [3] = 0, [4] = 1, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[7] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[8] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
	[9] = {[1] = 0, [2] = 0, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0, [8] = 0, [9] = 0},
}

local Start = {1, 5}
local Des = {9, 9}
local game_map = Map:new(Matrix)
local point_list = game_map:find_path(Start, Des)
local szHead = fn_format("find path {%d, %d} to {%d, %d}\n", Start[1], Start[2], Des[1], Des[2])
print(szHead)
local szPath = ""
local temp = {}
local n = fn_getn(point_list)
for i = n, 1, -1 do 
	local v = point_list[i]
	local szPoint = fn_format("{%d, %d}", v[1], v[2])
	szPath = szPath..szPoint
	if i > 1 then
		szPath = szPath.."==>"
	end
end

print(szPath)












