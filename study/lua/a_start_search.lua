-- auther: luhengsi
-- date:   2018-04-12

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
-- 设计4个类：PointBase, PathPoint, PathList, Map
-- PointBase: 地图点点坐标 {x, y, block}, x坐标， y坐标， block障碍物标记
--      	  类方法处理获取和设置x, y, block 的值外，还有计算的指定点到这个点的G和H值
-- PathPoint：辅助计算移动的路径点，包含一个PointBase的实例点，和查找过程中记录的G和H值
-- 			  还有指向父节点的成员变量
-- PathList： 开放列表和关闭列表的抽象数据结构，用来保存PathPoint类的实例
-- 			  有基本的增加、查找和删除操作
-- Map：      地图的抽象类型，包含一个二维坐标的矩阵 Matrix
--  		  FindPath为核心的寻路a*寻路算法

local fn_abs = math.abs;
local fn_max = math.max;
local fn_min = math.min;
local fn_remove = table.remove;
local fn_unpack = unpack;
local fn_format = string.format;
local fn_concat = table.concat;

local DEF_G1_STEP = 10;			-- 直线移动值
local DEF_G2_STEP = 14;			-- 斜线移动值

--=================================================
-- point
local PointBase = {};

function PointBase:NewPoint(...)
	local point = {};
	setmetatable(point, self);
	self.__index = self;
	point:Init(...);
	
	return point;
end

function PointBase:Init(x, y, is_barrier)
	self.m_nX = x or 0;
	self.m_nY = y or 0;
	self.m_nIsBarrier = is_barrier or 0;
end

function PointBase:GetX()
	return self.m_nX;
end

function PointBase:SetX(x)
	self.m_nX = x or 0;
end

function PointBase:GetY()
	return self.m_nY;
end

function PointBase:SetY(y)
	self.m_nY = y or 0;
end

function PointBase:IsSameXY(point)
	return ((self.m_nX == point.m_nX) and (self.m_nY == point.m_nY));
end

function PointBase:IsBarrier()
	return self.m_nIsBarrier == 1;
end

function PointBase:SetBarrier(is_barrier)
	self.m_nIsBarrier = is_barrier or 0;
end

-- calc H value
function PointBase:CalcH(point)
	local step = fn_abs(point.m_nX - self.m_nX) + fn_abs(point.m_nY - self.m_nY);
	return step * DEF_G1_STEP;
end

-- calc G value
function PointBase:CalcG(point)
	local xstep = fn_abs(point.m_nX - self.m_nX);
	local ystep = fn_abs(point.m_nY - self.m_nY);
	local step1 = fn_max(xstep, ystep);
	local step2 = fn_min(xstep, ystep);

	return step2 * DEF_G2_STEP + (step1 - step2) * DEF_G1_STEP;
end

--=======================================================================
-- path point
local PathPoint = {};

function PathPoint:NewPathPoint(...)
	local ppoint = {};
	setmetatable(ppoint, self);
	self.__index = self;
	ppoint:Init(...);
	
	return ppoint;
end

function PathPoint:Init(point)
	self.m_Point = point;
	self.m_Parent = nil;
	self.m_nH = 0;
	self.m_nG = 0;
end

function PathPoint:GetPoint()
	return self.m_Point;
end

function PathPoint:SetParent(path_point)
	self.m_Parent = path_point;
end

function PathPoint:GetParent()
	return self.m_Parent;
end

function PathPoint:GetH()
	return self.m_nH;
end

function PathPoint:SetH(h)
	self.m_nH = h;
end

function PathPoint:GetG()
	return self.m_nG;
end

function PathPoint:SetG(g)
	self.m_nG = g;
end

function PathPoint:CalcF()
	return self.m_nH + self.m_nG;
end

function PathPoint:IsSamePoint(point)
	return self.m_Point:IsSameXY(point);
end

function PathPoint:UpdateG(path_point_current)
	local point_current = path_point_current:GetPoint();
	local g = point_current:CalcG(self.m_Point) + path_point_current:GetG();
	if g < self:GetG() then
		self:SetG(g);
		self:SetParent(path_point_current);
	end
end
--======================================================
-- path list
local PathList = {};
function PathList:NewPathList(...)
	local plist = {};
	setmetatable(plist, self);
	self.__index = self;
	plist:Init(...);
	
	return plist;
end

function PathList:Init()
	self.m_List = {};
end

function PathList:MinPPoint()
	local min_path_point, min_f;
	for _, path_point in ipairs(self.m_List) do 
		local f = path_point:CalcF();
		if not min_path_point or f < min_f then
			min_path_point = path_point;
			min_f = f;
		end
	end
		
	return min_path_point;
end

function PathList:Count()
	return #(self.m_List);
end

function PathList:Add(path_point)
	local num = #(self.m_List);
	self.m_List[num + 1] = path_point;
end
	
function PathList:Remove(path_point)
	for i, v_path_point in ipairs(self.m_List) do 
		local point = v_path_point:GetPoint();
		if path_point:IsSamePoint(point) then
			fn_remove(self.m_List, i);
			break;
		end
	end
end
	
function PathList:GetExist(point)
	for _, path_point in ipairs(self.m_List) do 
		if path_point:IsSamePoint(point) then 
			return path_point;
		end
	end
	return nil;
end

--======================================================
-- map
local Map = {};
function Map:NewMap(...)
	local game_map = {};
	setmetatable(game_map, self);
	self.__index = self;
	game_map:Init(...);
	
	return game_map;
end

function Map:Init(org_matrix, width, height)
	self.m_Matrix = org_matrix;
	self.m_Width = width;
	self.m_Height = height;
end

function Map:CalcIndex(x, y)
	return y * self.m_Height + x + 1;
end

function Map:CreatePoint(pxy)
	local x, y = pxy[1], pxy[2];
	local index = self:CalcIndex(x, y);
	local pxyb = self.m_Matrix[index];
	if pxyb then
		return PointBase:NewPoint(fn_unpack(pxyb));
	end
	
	return nil;
end

-- 四周的八个点，辅助查找周围可通行的点
local quadrant = {{-1, -1, 0}, {0, -1, 1}, {1, -1, 0}, {1, 0, 1}, {1, 1, 0}, {0, 1, 1}, {-1, 1, 0}, {-1, 0, 1}};
-- 获得指定点周围可以通行的点，若此点的水平和垂直方向上的相邻点是障碍，则这个方向上的斜线不可通过。
function Map:SurroundPoints(point)
	local surround_points = {};
	local x = point:GetX();
	local y = point:GetY();
	
	local add_tag = 1;
	for _, v in ipairs(quadrant) do 
		local temp_point = self:CreatePoint({x + v[1], y + v[2]});
		local num = #surround_points;
		if temp_point then
			if temp_point:IsBarrier() == false then
				if add_tag == 1 then
					surround_points[num + 1] = temp_point;
				else
					add_tag = 1;
				end
			else
				if v[3] == 1 then
					if num > 0 then
						-- 退回去一格，前面一个不可通过
						surround_points[num] = nil;
					end
					
					-- 下一格也不可通过
					add_tag = 0;
				end
			end
		end
	end
	
	return surround_points;
end

-- a * 寻路算法的实现
function Map:FindPath(start, des)
	local point_list = {};
	local point_start = self:CreatePoint(start);
	local point_des = self:CreatePoint(des);
	if not point_start or not point_des then
		print("start or des erro.");
		return nil;
	end
	
	if point_start:IsBarrier() or point_des:IsBarrier() then
		print("start or des is barrier.");
		return nil;
	end
	
	local open_list = PathList:NewPathList();					-- 开放列表
	local close_list = PathList:NewPathList();					-- 关闭列表
	
	local ppoint_start = PathPoint:NewPathPoint(point_start);
	local ppoint_des = PathPoint:NewPathPoint(point_des);
	open_list:Add(ppoint_start);
	
	local is_find = 0;
	while (open_list:Count() > 0) do 
		local path_point_current = open_list:MinPPoint();		-- 从开启列表中取出F值最小的路径点
		open_list:Remove(path_point_current);					-- 从开启列表中删除这个点
		close_list:Add(path_point_current);						-- 并加入到关闭列表
		
		local point_current = path_point_current:GetPoint();
		
		local surround_points = self:SurroundPoints(point_current); 	-- 获取周围的点
		for _, s_point in ipairs(surround_points) do 
			-- 如果是终点，则更新终点路径点父节点，结束查找
			if ppoint_des:IsSamePoint(s_point) then
				local h = 0;
				local g = point_current:CalcG(s_point) + path_point_current:GetG();
				ppoint_des:SetH(h);
				ppoint_des:SetG(g);
				ppoint_des:SetParent(path_point_current);
				
				close_list:Add(ppoint_des);
				
				local ppt = ppoint_des:GetParent();
				local pt = ppt:GetPoint();
				is_find = 1;
				break;
			end
			
			-- 如果此路径点的不在关闭列表
			if not close_list:GetExist(s_point) then
				local exist_point = open_list:GetExist(s_point);
				-- 如果此路径点在开放列表，则更新相关参数
				if exist_point then
					exist_point:UpdateG(path_point_current);
				else
					-- 如果不在开放列表，则计算并更新参数，并加入开放列表
					local h = point_des:CalcH(s_point);
					local g = point_current:CalcG(s_point) + path_point_current:GetG();
					local path_s_point = PathPoint:NewPathPoint(s_point);
					path_s_point:SetH(h);
					path_s_point:SetG(g);
					path_s_point:SetParent(path_point_current);
					open_list:Add(path_s_point);
				end
			end
		end
		
		if is_find == 1 then
			break;
		end
	end

	local temp_path_point = ppoint_des;
	while (temp_path_point) do 
		local temp_point = temp_path_point:GetPoint();
		temp_path_point = temp_path_point:GetParent();
		local x = temp_point:GetX();
		local y = temp_point:GetY();
		point_list[#point_list + 1] = {x, y};
	end
	
	return point_list;
end

local DEF_MAP_WIDTH = 10; -- 宽度
local DEF_MAP_HEIGHT = 10; -- 高度
local Matrix = {
	{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {8, 0, 0}, {9, 0, 0}, 
	{0, 1, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {8, 1, 0}, {9, 1, 0}, 
	{0, 2, 0}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {8, 2, 0}, {9, 2, 0}, 
	{0, 3, 0}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {8, 3, 0}, {9, 3, 0}, 
	{0, 4, 0}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 1}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {8, 4, 0}, {9, 4, 0}, 
	{0, 5, 0}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 1}, {5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {8, 5, 0}, {9, 5, 0}, 
	{0, 6, 0}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {4, 6, 1}, {5, 6, 0}, {6, 6, 0}, {7, 6, 0}, {8, 6, 0}, {9, 6, 0}, 
	{0, 7, 0}, {1, 7, 0}, {2, 7, 0}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, {6, 7, 0}, {7, 7, 0}, {8, 7, 0}, {9, 7, 0}, 
	{0, 8, 0}, {1, 8, 0}, {2, 8, 0}, {3, 8, 0}, {4, 8, 0}, {5, 8, 0}, {6, 8, 0}, {7, 8, 0}, {8, 8, 0}, {9, 8, 0}, 
	{0, 9, 0}, {1, 9, 0}, {2, 9, 0}, {3, 9, 0}, {4, 9, 0}, {5, 9, 0}, {6, 9, 0}, {7, 9, 0}, {8, 9, 0}, {9, 9, 0}, 
}

local Start = {1, 4};
local Des = {9, 9};

local game_map = Map:NewMap(Matrix, DEF_MAP_WIDTH, DEF_MAP_HEIGHT);
local point_list = game_map:FindPath(Start, Des);

local temp = {};
for i = #point_list, 1, -1 do 
	local v = point_list[i];
	local szPoint = fn_format("{%d, %d}", v[1], v[2]);
	temp[#temp + 1] = szPoint;
end

local szHead = fn_format("find path {%d, %d} to {%d, %d}\n", Start[1], Start[2], Des[1], Des[2]);
print(szHead);
local szPath = fn_concat(temp, "==>");
print(szPath);

