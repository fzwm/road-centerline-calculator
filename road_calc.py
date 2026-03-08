"""
道路中心线计算 - Python 版
从 road-centerline-calculator (HTML/JS) 移植

功能:
- 解析纬地 HintCAD 的 .pm / .zdm / .sup 文件
- 计算三维道路中心线坐标
"""

import math
from dataclasses import dataclass
from typing import List, Optional


# ========================================
# 基础工具函数
# ========================================

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180


def rad_to_deg(rad: float) -> float:
    return rad * 180 / math.pi


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def is_finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


def parse_station(sta: str) -> float:
    """解析桩号: '0+100.000' -> 100, '-1+50.000' -> -1050"""
    s = str(sta).strip()
    if not s:
        raise ValueError('空桩号')
    
    if '+' in s:
        negative = s.startswith('-')
        clean = s[1:] if negative else s
        parts = clean.split('+')
        km = float(parts[0])
        m = float(parts[1])
        if not (math.isfinite(km) and math.isfinite(m)):
            raise ValueError(f'桩号解析失败: {sta}')
        result = km * 1000 + m
        return -result if negative else result
    
    v = float(s)
    if not math.isfinite(v):
        raise ValueError(f'桩号解析失败: {sta}')
    return v


def format_station(val: float) -> str:
    """格式化桩号: 100 -> '0+100.000'"""
    sign = '-' if val < 0 else ''
    abs_val = abs(val)
    km = int(abs_val // 1000)
    m = abs_val % 1000
    return f'{sign}{km}+{m:03.3f}'


# ========================================
# 数据结构
# ========================================

@dataclass
class PlaneElement:
    """平面线元"""
    e_type: str      # 'LS', 'L', 'C', 'TS', 'SC', 'CS', 'ST'
    sta_start: float # 起点桩号
    sta_end: float   # 终点桩号
    x: float         # 起点 X 坐标
    y: float         # 起点 Y 坐标
    azimuth: float   # 起点方位角 (度)
    r: float         # 曲率半径 (0 表示直线)
    length: float    # 线元长度


@dataclass
class VGradePoint:
    """纵断面变坡点"""
    sta: float       # 桩号
    elevation: float # 高程
    r: float         # 竖曲线半径 (0 表示直线)


@dataclass
class Superelevation:
    """超高渐变段"""
    sta_start: float  # 起点桩号
    sta_end: float    # 终点桩号
    left: float       # 左超高 (%)
    right: float      # 右超高 (%)


@dataclass
class Point3D:
    """三维坐标点"""
    station: float  # 桩号
    x: float        # X 坐标
    y: float        # Y 坐标
    z: float        # 高程
    azimuth: float  # 方位角/航向角 (度)
    slope: float    # 纵坡 (%)
    roll: float     # 横向倾角/旋转角 (度)


# ========================================
# 平面线元计算
# ========================================

def calc_line(e: PlaneElement, station: float) -> tuple:
    """
    直线计算
    返回: (x, y, azimuth)
    """
    ds = station - e.sta_start
    az_rad = deg_to_rad(e.azimuth)
    
    x = e.x + ds * math.cos(az_rad)
    y = e.y + ds * math.sin(az_rad)
    
    return x, y, e.azimuth


def calc_circle(e: PlaneElement, station: float) -> tuple:
    """
    圆曲线计算
    返回: (x, y, azimuth)
    """
    ds = station - e.sta_start
    az_start = deg_to_rad(e.azimuth)
    
    # 圆心角
    angle = ds / e.r
    
    # 圆心坐标
    cx = e.x + e.r * math.cos(az_start + math.pi / 2)
    cy = e.y + e.r * math.sin(az_start + math.pi / 2)
    
    # 当前点坐标
    current_angle = az_start + math.pi / 2 + angle
    x = cx + e.r * math.cos(current_angle)
    y = cy + e.r * math.sin(current_angle)
    
    # 当前方位角
    azimuth = rad_to_deg(current_angle - math.pi / 2)
    
    return x, y, azimuth


def calc_spiral(e: PlaneElement, station: float) -> tuple:
    """
    缓和曲线计算 (Clothoid)
    返回: (x, y, azimuth)
    
    使用近似公式: A^2 = R * L
    """
    ds = station - e.sta_start
    az_rad = deg_to_rad(e.azimuth)
    
    # 缓和曲线参数
    A2 = e.r * e.length  # A^2 = R * L
    
    # 弧长参数 t
    t = ds / A2 if A2 != 0 else 0
    
    # 近似计算 (适用于小角度)
    x = e.x + ds * (
        1 - t**2 / 10 + t**4 / 216 - t**6 / 9360
    ) * math.cos(az_rad) - ds * (
        t / 3 - t**3 / 42 + t**5 / 1320
    ) * math.sin(az_rad)
    
    y = e.y + ds * (
        t / 3 - t**3 / 42 + t**5 / 1320
    ) * math.cos(az_rad) + ds * (
        1 - t**2 / 10 + t**4 / 216 - t**6 / 9360
    ) * math.sin(az_rad)
    
    # 当前方位角
    azimuth = e.azimuth + rad_to_deg(t**2 / 2)
    
    return x, y, azimuth


def calc_plane_point(elements: List[PlaneElement], station: float) -> tuple:
    """
    计算平面坐标
    返回: (x, y, azimuth)
    """
    for e in elements:
        if e.sta_start <= station <= e.sta_end:
            if e.e_type == 'L':
                return calc_line(e, station)
            elif e.e_type == 'C':
                return calc_circle(e, station)
            else:
                # 其他类型按缓和曲线处理
                return calc_spiral(e, station)
    
    raise ValueError(f'桩号 {station} 超出平面线元范围')


# ========================================
# 纵断面计算
# ========================================

def calc_vertical(vgrades: List[VGradePoint], station: float) -> tuple:
    """
    计算高程
    返回: (elevation, slope)
    """
    for i, vg in enumerate(vgrades):
        if i < len(vgrades) - 1:
            if vg.sta <= station <= vgrades[i + 1].sta:
                next_vg = vgrades[i + 1]
                
                # 坡度
                dz = next_vg.elevation - vg.elevation
                ds = next_vg.sta - vg.sta
                slope = (dz / ds * 100) if ds != 0 else 0  # 百分比坡度
                
                # 直线段
                if vg.r == 0:
                    elev = vg.elevation + (station - vg.sta) * dz / ds
                    return elev, slope
                
                # 竖曲线段
                # 简化计算
                L = next_vg.sta - vg.sta
                p = (station - vg.sta) / L if L != 0 else 0
                
                # 竖曲线偏移
                if vg.r != 0:
                    R = vg.r
                    # 抛物线近似
                    offset = (p * (1 - p) * L**2) / (2 * R)
                    elev = vg.elevation + dz * p + offset
                else:
                    elev = vg.elevation + dz * p
                
                return elev, slope
    
    raise ValueError(f'桩号 {station} 超出纵断面范围')


# ========================================
# 主计算函数
# ========================================

class RoadCenterline:
    """道路中心线计算器"""
    
    def __init__(self):
        self.plane_elements: List[PlaneElement] = []
        self.vgrades: List[VGradePoint] = []
        self.supers: List[Superelevation] = []
    
    def parse_pm(self, content: str) -> List[PlaneElement]:
        """
        解析 PM 文件 (平面线元)
        格式: 类型 起点桩号 终点桩号 X Y 方位角 曲率半径
        """
        elements = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 7:
                e = PlaneElement(
                    e_type=parts[0],
                    sta_start=parse_station(parts[1]),
                    sta_end=parse_station(parts[2]),
                    x=float(parts[3]),
                    y=float(parts[4]),
                    azimuth=float(parts[5]),
                    r=float(parts[6]),
                    length=float(parts[2]) - float(parts[1])
                )
                elements.append(e)
        
        self.plane_elements = elements
        return elements
    
    def parse_zdm(self, content: str) -> List[VGradePoint]:
        """
        解析 ZDM 文件 (纵断面)
        格式: 桩号 高程 竖曲线半径
        """
        vgrades = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 2:
                vg = VGradePoint(
                    sta=parse_station(parts[0]),
                    elevation=float(parts[1]),
                    r=float(parts[2]) if len(parts) > 2 else 0
                )
                vgrades.append(vg)
        
        self.vgrades = vgrades
        return vgrades
    
    def parse_sup(self, content: str) -> List[Superelevation]:
        """
        解析 SUP 文件 (超高)
        格式: 起点桩号 终点桩号 左超高(%) 右超高(%)
        """
        supers = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 4:
                s = Superelevation(
                    sta_start=parse_station(parts[0]),
                    sta_end=parse_station(parts[1]),
                    left=float(parts[2]),
                    right=float(parts[3])
                )
                supers.append(s)
        
        self.supers = supers
        return supers
    
    def calculate(self, station: float) -> Point3D:
        """计算指定桩号的三维坐标"""
        x, y, azimuth = calc_plane_point(self.plane_elements, station)
        z, slope = calc_vertical(self.vgrades, station)
        
        # 计算横向倾角 (roll) - 从超高数据获取
        roll = 0.0
        for sup in self.supers:
            if sup.sta_start <= station <= sup.sta_end:
                # 取左右超高的平均值作为横向倾角
                roll = (sup.left + sup.right) / 2
                break
        
        return Point3D(
            station=station,
            x=x,
            y=y,
            z=z,
            azimuth=azimuth,
            slope=slope,
            roll=roll
        )
    
    def calculate_range(self, start: float, end: float, step: float = 10) -> List[Point3D]:
        """计算桩号范围内的三维坐标点"""
        results = []
        station = start
        while station <= end:
            results.append(self.calculate(station))
            station += step
        
        # 确保包含终点
        if results and results[-1].station != end:
            results.append(self.calculate(end))
        
        return results
    
    def query(self, station: float) -> dict:
        """查询指定桩号的信息"""
        pt = self.calculate(station)
        
        return {
            'station': format_station(station),
            'x': round(pt.x, 4),
            'y': round(pt.y, 4),
            'z': round(pt.z, 4),
            'azimuth': round(pt.azimuth, 4),
            'slope': round(pt.slope, 4),
            'roll': round(pt.roll, 4)
        }


# ========================================
# 使用示例
# ========================================

if __name__ == '__main__':
    # 示例: 创建计算器并计算
    road = RoadCenterline()
    
    # 示例平面线元数据
    pm_content = """
    L 0 50 500000 3000000 90 0
    C 50 100 500050 3000050 90 500
    L 100 150 500100 3000100 120 0
    """
    
    # 示例纵断面数据
    zdm_content = """
    0 100 0
    50 110 5000
    100 105 0
    150 95 0
    """
    
    road.parse_pm(pm_content)
    road.parse_zdm(zdm_content)
    
    # 查询桩号
    result = road.query(25)
    print(f"桩号 {result['station']}:")
    print(f"  X = {result['x']}")
    print(f"  Y = {result['y']}")
    print(f"  Z = {result['z']}")
    print(f"  方位角 = {result['azimuth']}°")
    print(f"  坡度 = {result['slope']}%")
    print(f"  横向倾角 = {result['roll']}°")
