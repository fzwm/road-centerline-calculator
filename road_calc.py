"""
道路中心线计算 - Python 版
从 road-centerline-calculator (HTML/JS) 移植

功能:
- 解析纬地 HintCAD 的 .pm / .zdm / .sup 文件
- 计算三维道路中心线坐标

修复记录 (2026-03-08):
- 修复圆曲线计算方向错误 (r可正可负)
- 添加 Simpson 数值积分用于缓和曲线
- 修复数据结构，添加曲率信息
- 修复纵断面竖曲线计算公式
- 修复超高单位 (使用比例而非百分比)
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Callable


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
# 数值积分 (Simpson法)
# ========================================

def simpson_integrate(f: Callable[[float], float], s: float, n: int = 400) -> float:
    """
    Simpson 数值积分
    用于缓和曲线计算，比泰勒级数精度更高
    """
    if s == 0:
        return 0
    N = max(20, n)
    if N % 2 == 1:
        N += 1
    h = s / N
    total = f(0) + f(s)
    for i in range(1, N):
        t = i * h
        total += (4 if i % 2 == 1 else 2) * f(t)
    return (h / 3) * total


# ========================================
# 数据结构
# ========================================

@dataclass
class PlaneElement:
    """
    平面线元
    包含完整的曲率信息
    """
    e_type: str       # 'LS', 'L', 'C', 'TS', 'SC', 'CS', 'ST'
    sta_start: float # 起点桩号
    sta_end: float   # 终点桩号
    x: float         # 起点 X 坐标
    y: float         # 起点 Y 坐标
    azimuth: float   # 起点方位角 (度)
    r: float         # 曲率半径 (可正可负，r>0左转，r<0右转)
    length: float    # 线元长度
    
    # 曲率信息
    k: float = 0.0    # 起点曲率 (1/r)
    k0: float = 0.0   # 终点曲率
    k1: float = 0.0   # 曲率变化率
    ck: float = 0.0   # 累积曲率变化
    
    def __post_init__(self):
        # 计算曲率
        if self.length > 0 and self.r != 0:
            self.k = 1.0 / self.r  # 起点曲率
            # 根据线元类型计算终点曲率和曲率变化率
            if self.e_type == 'L':
                self.k0 = 0.0
                self.k1 = 0.0
            elif self.e_type == 'C':
                self.k0 = self.k
                self.k1 = 0.0
            else:  # 缓和曲线
                # 假设线性变化
                self.k0 = 0.0 if self.e_type in ['TS', 'ST'] else self.k
                self.k1 = (self.k0 - self.k) / self.length
                self.ck = (self.k0 - self.k) * self.length / 2  # 累积曲率变化


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
    left: float       # 左超高 (比例，如 -0.02 表示 -2%)
    right: float      # 右超高 (比例)


@dataclass
class Point3D:
    """三维坐标点"""
    station: float  # 桩号
    x: float        # X 坐标
    y: float        # Y 坐标
    z: float        # 高程
    azimuth: float  # 方位角/航向角 (度)
    slope: float    # 纵坡 (%)
    roll: float     # 横向倾角/旋转角 (比例)


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
    支持任意方向 (r可正可负)
    
    修复: 使用曲率 k = 1/r 处理方向
    """
    ds = station - e.sta_start
    az_start = deg_to_rad(e.azimuth)
    
    # 曲率 (可正可负)
    k = 1.0 / e.r if e.r != 0 else 0.0
    
    # 近似直线处理
    if abs(k) < 1e-12:
        return calc_line(e, station)
    
    # 当前方位角 (带符号)
    az = az_start + ds * k
    
    # 积分计算坐标
    # X = X0 + (sin(az) - sin(a0)) / k
    # Y = Y0 + (-cos(az) + cos(a0)) / k
    x = e.x + (math.sin(az) - math.sin(az_start)) / k
    y = e.y + (-math.cos(az) + math.cos(az_start)) / k
    
    return x, y, rad_to_deg(az)


def calc_spiral(e: PlaneElement, station: float) -> tuple:
    """
    缓和曲线计算 (Clothoid)
    使用 Simpson 数值积分，精度更高
    """
    ds = station - e.sta_start
    az_start = deg_to_rad(e.azimuth)
    
    # 获取曲率参数
    k = e.k if e.k != 0 else (1.0 / e.r if e.r != 0 else 0.0)
    k0 = e.k0
    ck = e.ck if e.ck != 0 else (k0 - k) * ds / 2
    
    # 积分函数
    def fx(t: float) -> float:
        """X坐标积分: cos(a0 + k0*t + 0.5*ck*t^2 / ds)"""
        angle = az_start + k0 * t + 0.5 * ck * t * t / ds if ds != 0 else az_start
        return math.cos(angle)
    
    def fy(t: float) -> float:
        """Y坐标积分: sin(a0 + k0*t + 0.5*ck*t^2 / ds)"""
        angle = az_start + k0 * t + 0.5 * ck * t * t / ds if ds != 0 else az_start
        return math.sin(angle)
    
    # Simpson 数值积分
    if ds == 0:
        return e.x, e.y, e.azimuth
    
    # 如果曲率变化率很小，退化为简单积分
    if abs(ck) < 1e-12:
        # 简化为定曲率积分
        k_mid = (k + k0) / 2
        az_mid = az_start + ds * k_mid
        x = e.x + (math.sin(az_mid) - math.sin(az_start)) / k_mid if abs(k_mid) > 1e-12 else ds * math.cos(az_start)
        y = e.y + (-math.cos(az_mid) + math.cos(az_start)) / k_mid if abs(k_mid) > 1e-12 else ds * math.sin(az_start)
        azimuth = rad_to_deg(az_mid)
    else:
        # 完整数值积分
        x = e.x + simpson_integrate(fx, ds)
        y = e.y + simpson_integrate(fy, ds)
        
        # 当前方位角
        azimuth = rad_to_deg(az_start + k0 * ds + 0.5 * ck * ds / ds) if ds != 0 else e.azimuth
    
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
                # 缓和曲线类型
                return calc_spiral(e, station)
    
    raise ValueError(f'桩号 {station} 超出平面线元范围')


# ========================================
# 纵断面计算
# ========================================

def calc_vertical(vgrades: List[VGradePoint], station: float) -> tuple:
    """
    计算高程
    返回: (elevation, slope)
    
    修复: 使用道路工程标准公式
    Z = Z_BVC + g1 * x + (A / (2 * Lv)) * x^2
    其中 A = g2 - g1 是坡度代数差
    """
    for i, vg in enumerate(vgrades):
        if i < len(vgrades) - 1:
            if vg.sta <= station <= vgrades[i + 1].sta:
                next_vg = vgrades[i + 1]
                
                # 坡度 (百分比)
                dz = next_vg.elevation - vg.elevation
                ds = next_vg.sta - vg.sta
                g1 = (dz / ds * 100) if ds != 0 else 0  # 起点坡度 (%)
                g2 = 0.0  # 终点坡度
                
                # 如果下一段有数据，获取终点坡度
                if i + 2 < len(vgrades):
                    next_next = vgrades[i + 2]
                    dz2 = next_next.elevation - next_vg.elevation
                    ds2 = next_next.sta - next_vg.sta
                    g2 = (dz2 / ds2 * 100) if ds2 != 0 else 0
                
                # 直线段
                if vg.r == 0:
                    x = station - vg.sta
                    elev = vg.elevation + x * dz / ds
                    return elev, g1
                
                # 竖曲线段
                # 竖曲线长度
                Lv = next_vg.sta - vg.sta
                x = station - vg.sta  # 距离起点的距离
                
                # 坡度代数差
                A = g2 - g1
                
                # 标准公式: Z = Z_BVC + g1*x + (A/(2*Lv))*x^2
                elev = vg.elevation + g1 * x / 100 + (A * x * x) / (2 * Lv * 100)
                
                # 当前坡度
                slope = g1 + (A * x) / Lv
                
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
        
        修复: 正确处理曲率半径的正负
        """
        elements = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 7:
                # 解析曲率半径 (可正可负)
                r = float(parts[6])
                
                e = PlaneElement(
                    e_type=parts[0],
                    sta_start=parse_station(parts[1]),
                    sta_end=parse_station(parts[2]),
                    x=float(parts[3]),
                    y=float(parts[4]),
                    azimuth=float(parts[5]),
                    r=r,
                    length=parse_station(parts[2]) - parse_station(parts[1])
                )
                
                # 计算曲率信息
                if e.length > 0:
                    if e.e_type == 'L':
                        e.k = 0.0
                        e.k0 = 0.0
                        e.k1 = 0.0
                    elif e.e_type == 'C':
                        e.k = 1.0 / r if r != 0 else 0.0
                        e.k0 = e.k
                        e.k1 = 0.0
                    else:  # 缓和曲线
                        # 假设从直线进入圆曲线
                        e.k = 1.0 / r if r != 0 else 0.0
                        e.k0 = 0.0  # 起点在直线上
                        e.k1 = (e.k - e.k0) / e.length
                        e.ck = (e.k - e.k0) * e.length / 2
                
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
        格式: 起点桩号 终点桩号 左超高(比例) 右超高(比例)
        
        修复: 使用比例而非百分比
        """
        supers = []
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 4:
                # 转换为比例 (如 2% -> 0.02)
                left = float(parts[2]) / 100.0
                right = float(parts[3]) / 100.0
                
                s = Superelevation(
                    sta_start=parse_station(parts[0]),
                    sta_end=parse_station(parts[1]),
                    left=left,
                    right=right
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
                # 取左右超高的平均值作为横向倾角 (比例)
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
    
    # 示例平面线元数据 (注意 r 可正可负)
    # r > 0: 左转曲线, r < 0: 右转曲线
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
    
    # 示例超高数据 (使用比例)
    sup_content = """
    0 50 0.02 0.02
    50 100 0.04 0.04
    """
    
    road.parse_pm(pm_content)
    road.parse_zdm(zdm_content)
    road.parse_sup(sup_content)
    
    # 查询桩号
    result = road.query(25)
    print(f"桩号 {result['station']}:")
    print(f"  X = {result['x']}")
    print(f"  Y = {result['y']}")
    print(f"  Z = {result['z']}")
    print(f"  方位角 = {result['azimuth']}°")
    print(f"  坡度 = {result['slope']}%")
    print(f"  横向倾角 = {result['roll']}")
