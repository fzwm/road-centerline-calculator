"""
道路中心线计算 - Python 版
从 road-centerline-calculator (HTML/JS) 忠实移植

功能:
- 解析纬地 HintCAD 的 .pm / .zdm / .sup 文件（原始格式 + 标准中间格式）
- 计算三维道路中心线坐标（平面 + 纵断面 + 超高）
- 输出与 JS 版完全一致的结果
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Callable, Dict


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


def parse_station(sta) -> float:
    """解析桩号: '0+100.000' -> 100, '-1+050.000' -> -1050"""
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
    return f'{sign}{km}+{m:07.3f}'


def clean_lines(text: str) -> List[str]:
    """清洗文本行：去掉注释和空行"""
    lines = []
    for line in text.replace('\r', '').split('\n'):
        # 去掉 // 注释
        idx = line.find('//')
        if idx >= 0:
            line = line[:idx]
        # 去掉 # 开头注释
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        if stripped:
            lines.append(stripped)
    return lines


# ========================================
# 数值积分 (Simpson法)
# ========================================

def simpson_integrate(f: Callable[[float], float], s: float, n: int = 200) -> float:
    """Simpson 数值积分，对应 JS simpsonIntegrate()"""
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
    平面线元 (标准格式)
    对应 JS parsePM() 的输出结构
    """
    id: int           # 序号
    type: int         # 1=直线, 2=圆曲线, 3=缓和曲线
    L0: float         # 起点桩号
    L1: float         # 终点桩号
    L_len: float      # 线元长度
    X0: float         # 起点 X 坐标
    Y0: float         # 起点 Y 坐标
    alpha0: float     # 起点方位角 (度)
    last: float       # 曲率 k (1/m)

    # 计算后的曲率信息
    k: float = 0.0    # 当前曲率
    k0: float = 0.0   # 起点曲率 (缓和曲线积分用)
    k1: float = 0.0   # 终点曲率
    ck: float = 0.0   # 曲率变化率 (dk/ds)


@dataclass
class VGradePoint:
    """纵断面变坡点"""
    L: float          # 桩号
    Z: float          # 高程
    Rv: float         # 竖曲线半径 (0 表示无竖曲线)


@dataclass
class VerticalCurve:
    """竖曲线参数"""
    index: int        # 变坡点索引
    VPI: float        # 变坡点桩号
    Z_VPI: float      # 变坡点高程
    Rv: float         # 竖曲线半径
    g1: float         # 前坡度 (比例)
    g2: float         # 后坡度 (比例)
    A: float          # 坡度代数差 g2-g1
    Lv: float         # 竖曲线长度
    T: float          # 切线长
    BVC: float        # 竖曲线起点桩号
    EVC: float        # 竖曲线终点桩号
    Z_BVC: float      # 竖曲线起点高程


@dataclass
class ZdmModel:
    """纵断面模型"""
    pts: List[VGradePoint]
    g: List[float]                # 各段坡度 (比例，非百分比)
    curves: List[VerticalCurve]


@dataclass
class SuperSegment:
    """超高段"""
    L0: float          # 起点桩号
    L1: float          # 终点桩号
    left: float        # 终点左超高 (比例)
    right: float       # 终点右超高 (比例)
    gradRate: float    # 渐变率
    startLeft: float = 0.0
    startRight: float = 0.0
    endLeft: float = 0.0
    endRight: float = 0.0


@dataclass
class Point3D:
    """三维坐标点，对应 JS solveAtStation() 的返回值"""
    L: float                # 桩号
    X: float                # X 坐标
    Y: float                # Y 坐标
    Z: float                # 高程
    azimuthDeg: float       # 方位角 (度)
    gradeDeg: float         # 坡度角 (度)
    crossDeg: float         # 横向倾角 (度)
    compositeSlope: float   # 合成坡度
    grade: float            # 纵坡 (比例)
    crossSlope: float       # 横坡 (比例)
    tangent: Dict[str, float] = field(default_factory=dict)


# ========================================
# 原始 HintCAD 文件解析
# ========================================

PM_TYPE_MAP = {1: 1, 3: 2, 21: 3, 22: 3}
PM_TYPE_NAME = {1: '直线', 2: '圆曲线', 3: '缓和曲线'}


def parse_pm_raw(content: str) -> dict:
    """
    解析原始纬地 .pm 文件
    对应 JS parsePmFile()
    """
    lines = [l for l in content.replace('\r', '').split('\n') if l.strip()]
    if len(lines) < 3:
        raise ValueError('pm 文件内容不完整（至少需要文件头、全局参数和基础参数行）')

    header = lines[0].strip()
    if 'HINTCAD' not in header or 'PM' not in header:
        raise ValueError('不是有效的纬地 .pm 文件（文件头标识不匹配）')

    global_tokens = [t for t in lines[1].strip().split() if t != 'NULL']
    if not global_tokens:
        raise ValueError(f'pm 全局参数行无有效数据（line 2）: {lines[1]}')

    try:
        element_count = int(global_tokens[0])
        global_start_station = float(global_tokens[-1])
    except ValueError as e:
        raise ValueError(f'pm 全局参数解析失败（line 2）: {lines[1]}') from e

    elements = []
    idx = 2

    # 跳过基础参数行（6字段）
    idx += 1

    while idx < len(lines):
        tokens = lines[idx].strip().split()

        if len(tokens) == 7:
            # 扩展参数行 → 读取三行构成一个线元
            ext = tokens
            idx += 1
            if idx + 1 >= len(lines):
                raise ValueError(f'pm 线元数据不完整（line {idx + 1} 附近，第{len(elements) + 1}个线元缺少坐标或桩号行）')

            try:
                type_code = int(float(ext[-1]))
                std_type = PM_TYPE_MAP.get(type_code, type_code)
                val5 = float(ext[4])
                val6 = float(ext[5])

                coord_line_no = idx + 1
                coord = lines[idx].strip().split()
                idx += 1
                if len(coord) < 6:
                    raise ValueError(f'pm 坐标行字段不足（line {coord_line_no}）: {lines[idx - 1]}')
                start_x = float(coord[0])
                start_y = float(coord[1])
                end_x = float(coord[4])
                end_y = float(coord[5])

                sta_line_no = idx + 1
                sta = lines[idx].strip().split()
                idx += 1
                if len(sta) < 4:
                    raise ValueError(f'pm 桩号行字段不足（line {sta_line_no}）: {lines[idx - 1]}')
                start_station = float(sta[0])
                end_station = float(sta[1])
                start_az_rad = float(sta[2])
                end_az_rad = float(sta[3])
            except ValueError as e:
                raise ValueError(f'pm 线元解析失败（line {idx + 1} 附近，第{len(elements) + 1}个线元）: {e}') from e

            length = end_station - start_station

            delta_az = end_az_rad - start_az_rad
            if delta_az > math.pi:
                delta_az -= 2 * math.pi
            if delta_az < -math.pi:
                delta_az += 2 * math.pi
            sign = -1 if delta_az < 0 else 1

            curvature = 0.0
            if std_type == 1:  # 直线
                curvature = 0
            elif std_type == 3:  # 缓和曲线
                if val6 != 9999 and val6 != 0:
                    curvature = sign / val6
            elif std_type == 2:  # 圆曲线
                R = 0
                if val5 != 9999 and val5 != 0:
                    R = val5
                elif val6 != 9999 and val6 != 0:
                    R = val6
                if R != 0:
                    TAU = 2 * math.pi
                    expected_abs = length / R
                    raw_delta = end_az_rad - start_az_rad
                    raw_mod = ((raw_delta % TAU) + TAU) % TAU
                    pos_mod = ((expected_abs % TAU) + TAU) % TAU
                    neg_mod = ((-expected_abs % TAU) + TAU) % TAU
                    d_pos = min(abs(raw_mod - pos_mod), TAU - abs(raw_mod - pos_mod))
                    d_neg = min(abs(raw_mod - neg_mod), TAU - abs(raw_mod - neg_mod))
                    curvature = (1 / R) if d_pos <= d_neg else (-1 / R)

            elements.append({
                'index': len(elements) + 1,
                'type': std_type,
                'typeName': PM_TYPE_NAME.get(std_type, '未知'),
                'startStation': start_station,
                'endStation': end_station,
                'length': length,
                'startX': start_x,
                'startY': start_y,
                'endX': end_x,
                'endY': end_y,
                'startAzDeg': rad_to_deg(start_az_rad),
                'endAzDeg': rad_to_deg(end_az_rad),
                'curvature': curvature,
            })
        else:
            idx += 1

    return {
        'header': header,
        'elementCount': len(elements),
        'globalStartStation': global_start_station,
        'elements': elements,
    }


def format_output_pm(parsed: dict) -> str:
    """转换为标准格式文本，对应 JS formatOutputPM()"""
    lines = [
        '# 纬地 .pm 文件解析结果',
        f'# 文件标识: {parsed["header"]}',
        f'# 线元数量: {parsed["elementCount"]}',
        f'# 全局起始桩号: {parsed["globalStartStation"]}',
        '#',
        '# 序号, 类型(1=直线/2=圆曲线/3=缓和曲线), 起始桩号, 终点桩号, 长度, 起点X, 起点Y, 起始方位角(度), 曲率k(1/m)',
        '#',
    ]
    for e in parsed['elements']:
        lines.append(', '.join([
            str(e['index']), str(e['type']),
            format_station(e['startStation']), format_station(e['endStation']),
            f"{e['length']:.3f}", f"{e['startX']:.3f}", f"{e['startY']:.3f}",
            f"{e['startAzDeg']:.3f}", f"{e['curvature']:.6f}",
        ]))
    return '\n'.join(lines)


def parse_zdm_raw(content: str) -> dict:
    """
    解析原始纬地 .zdm 文件
    对应 JS parseZdmFile()
    """
    lines = [l for l in content.replace('\r', '').split('\n') if l.strip()]
    if len(lines) < 2:
        raise ValueError('zdm 文件内容不完整（至少需要文件头和点数行）')

    header = lines[0].strip()
    if 'HINTCAD' not in header or 'ZDM' not in header:
        raise ValueError('不是有效的纬地 .zdm 文件（文件头标识不匹配）')

    try:
        point_count = int(lines[1].strip())
    except ValueError as e:
        raise ValueError(f'zdm 变坡点数量解析失败（line 2）: {lines[1]}') from e
    if point_count < 2:
        raise ValueError('变坡点数量无效（至少需要2个）')
    if len(lines) < 2 + point_count:
        raise ValueError(f'zdm 文件行数不足（声明{point_count}个变坡点，实际仅{max(0, len(lines) - 2)}个）')

    points = []
    for i in range(point_count):
        line_no = 3 + i
        tokens = lines[2 + i].strip().split()
        if len(tokens) < 3:
            raise ValueError(f'第{i + 1}个变坡点字段不足（line {line_no}）: {lines[2 + i]}')
        station = float(tokens[0])
        elevation = float(tokens[1])
        radius = float(tokens[2])
        if not all(math.isfinite(v) for v in [station, elevation, radius]):
            raise ValueError(f'第{i + 1}个变坡点数据无效: {lines[2 + i]}')
        points.append({
            'station': station,
            'elevation': elevation,
            'radius': radius,
        })

    return {'header': header, 'pointCount': point_count, 'points': points}


def format_output_zdm(parsed: dict) -> str:
    """转换为标准格式文本，对应 JS formatOutputZDM()"""
    lines = [
        '# 纬地 .zdm 文件解析结果',
        f'# 文件标识: {parsed["header"]}',
        f'# 变坡点数量: {parsed["pointCount"]}',
        '#',
        '# 格式: 桩号 高程 竖曲线半径',
        '#',
        str(parsed['pointCount']),
    ]
    for p in parsed['points']:
        lines.append(
            f"{format_station(p['station'])}  {p['elevation']:.3f}  {p['radius']:.0f}"
        )
    return '\n'.join(lines)


def interpolate_9999(points: List[dict], field_name: str):
    """对 9999 标记值进行插值，对应 JS interpolate9999()"""
    THRESHOLD = 9000
    defined = []
    for p in points:
        if abs(p[field_name]) < THRESHOLD:
            defined.append({'station': p['station'], 'value': p[field_name]})

    if not defined:
        for p in points:
            p[field_name] = 0
        return

    for pt in points:
        if abs(pt[field_name]) < THRESHOLD:
            continue
        station = pt['station']
        before = None
        after = None
        for d in defined:
            if d['station'] <= station + 1e-9:
                before = d
        for d in defined:
            if d['station'] >= station - 1e-9:
                after = d
                break

        if before and after and abs(before['station'] - after['station']) > 1e-9:
            t = (station - before['station']) / (after['station'] - before['station'])
            pt[field_name] = before['value'] + t * (after['value'] - before['value'])
        elif before:
            pt[field_name] = before['value']
        elif after:
            pt[field_name] = after['value']


def parse_sup_raw(content: str) -> dict:
    """
    解析原始纬地 .sup 文件
    对应 JS parseSupFile()
    """
    lines = [l for l in content.replace('\r', '').split('\n') if l.strip()]
    if not lines:
        raise ValueError('sup 文件为空')

    header = lines[0].strip()
    if 'HINTCAD' not in header or 'SUP' not in header:
        raise ValueError('不是有效的纬地 .sup 文件（文件头标识不匹配）')

    points = []
    for i in range(1, len(lines)):
        parts = lines[i].strip().split()
        if len(parts) < 7:
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in vals[:7]):
            continue
        points.append({
            'leftLane': vals[2],
            'station': vals[3],
            'rightLane': vals[4],
        })

    if len(points) < 2:
        raise ValueError('控制点数量不足（至少需要 2 个）')

    points.sort(key=lambda p: p['station'])

    interpolate_9999(points, 'leftLane')
    interpolate_9999(points, 'rightLane')

    raw_segs = []
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        seg_len = p1['station'] - p0['station']
        if seg_len < 1e-9:
            continue
        left_change = abs(p1['leftLane'] - p0['leftLane'])
        right_change = abs(p1['rightLane'] - p0['rightLane'])
        is_constant = left_change < 0.01 and right_change < 0.01
        grad_rate = 0.0
        if not is_constant:
            grad_rate = max(left_change, right_change) / seg_len / 100
        raw_segs.append({
            'L0': p0['station'], 'L1': p1['station'],
            'leftEnd': p1['leftLane'], 'rightEnd': p1['rightLane'],
            'gradRate': grad_rate, 'isConstant': is_constant,
        })

    # 合并相邻常值段
    segments = []
    for seg in raw_segs:
        if segments:
            prev = segments[-1]
            if (seg['isConstant'] and prev.get('isConstant')
                    and abs(prev['leftEnd'] - seg['leftEnd']) < 0.01
                    and abs(prev['rightEnd'] - seg['rightEnd']) < 0.01):
                prev['L1'] = seg['L1']
                continue
        segments.append(dict(seg))

    return {'header': header, 'pointCount': len(points), 'segments': segments}


def format_output_sup(parsed: dict) -> str:
    """转换为标准格式文本，对应 JS formatOutputSUP()"""
    lines = [
        '# 纬地 .sup 文件解析结果',
        f'# 文件标识: {parsed["header"]}',
        f'# 控制点数量: {parsed["pointCount"]}',
        '#',
        '# 序号, 起始桩号, 终点桩号, 旋转轴, 左行车道超高(%), 右行车道超高(%), 渐变率',
        '#',
    ]
    for i, s in enumerate(parsed['segments']):
        lines.append(', '.join([
            str(i + 1),
            format_station(s['L0']), format_station(s['L1']),
            '0', f"{s['leftEnd']:.1f}", f"{s['rightEnd']:.1f}",
            f"{s['gradRate']:.6f}",
        ]))
    return '\n'.join(lines)


# ========================================
# 标准格式解析（对应 JS 的 parsePM/parseZDM/parseSUP）
# ========================================

def parse_pm_standard(text: str) -> List[PlaneElement]:
    """
    解析标准格式 PM 文本
    格式: id, type, L0, L1, L_len, X0, Y0, alpha0, curvature
    对应 JS parsePM()
    """
    lines = clean_lines(text)
    elems_raw = []

    for line in lines:
        parts = [p.strip() for p in line.replace(',', ' ').split()]
        parts = [p for p in parts if p]
        if len(parts) < 9:
            continue

        try:
            elem_id = int(float(parts[0]))
            elem_type = int(float(parts[1]))
            L0 = parse_station(parts[2])
            L1 = parse_station(parts[3])
            L_len = float(parts[4])
            X0 = float(parts[5])
            Y0 = float(parts[6])
            alpha0 = float(parts[7])
            last = float(parts[8])
        except (ValueError, IndexError):
            continue

        if elem_type not in (1, 2, 3):
            continue
        if not all(math.isfinite(v) for v in [elem_id, L0, L1, L_len, X0, Y0, alpha0, last]):
            continue

        elems_raw.append(PlaneElement(
            id=elem_id, type=elem_type,
            L0=L0, L1=L1, L_len=L_len,
            X0=X0, Y0=Y0, alpha0=alpha0, last=last,
        ))

    elems_raw.sort(key=lambda e: e.L0)

    # 计算曲率信息（关键！缓和曲线需要从前一线元继承起点曲率）
    elems = elems_raw
    for i, e in enumerate(elems):
        if e.type == 1:  # 直线
            e.k = 0
            e.k0 = 0
            e.k1 = 0
            e.ck = 0
        elif e.type == 2:  # 圆曲线
            e.k = e.last
            e.k0 = e.last
            e.k1 = e.last
            e.ck = 0
        elif e.type == 3:  # 缓和曲线
            k1 = e.last  # 终点曲率
            k0 = 0.0
            # 从前一线元继承起点曲率
            if i > 0:
                prev = elems[i - 1]
                if prev.type == 2:
                    k0 = prev.k
                elif prev.type == 3:
                    k0 = prev.k1 if is_finite(prev.k1) else 0
                else:
                    k0 = 0
            e.k0 = k0
            e.k1 = k1
            e.ck = (k1 - k0) / e.L_len if e.L_len > 0 else 0
            e.k = k0

    return elems


def parse_zdm_standard(text: str) -> ZdmModel:
    """
    解析标准格式 ZDM 文本
    对应 JS parseZDM()
    """
    lines = clean_lines(text)
    idx = 0

    # 检查第一行是否为点数
    try:
        N = int(float(lines[0]))
        if N > 0 and len(lines) >= N + 1:
            idx = 1
        else:
            idx = 0
    except (ValueError, IndexError):
        idx = 0

    pts = []
    for i in range(idx, len(lines)):
        parts = [p.strip() for p in lines[i].replace(',', ' ').split()]
        parts = [p for p in parts if p]
        if len(parts) < 3:
            continue
        try:
            L = parse_station(parts[0])
            Z = float(parts[1])
            Rv = float(parts[2])
        except (ValueError, IndexError):
            continue
        if not all(math.isfinite(v) for v in [L, Z, Rv]):
            continue
        pts.append(VGradePoint(L=L, Z=Z, Rv=Rv))

    pts.sort(key=lambda p: p.L)
    if len(pts) < 2:
        raise ValueError('zdm 至少需要 2 个变坡点')

    # 计算各段坡度 (比例，不是百分比！)
    g = [0.0] * len(pts)
    for i in range(len(pts) - 1):
        dL = pts[i + 1].L - pts[i].L
        if dL <= 0:
            raise ValueError('zdm 桩号必须递增')
        g[i] = (pts[i + 1].Z - pts[i].Z) / dL
    g[len(pts) - 1] = g[len(pts) - 2]

    # 识别竖曲线（仅在中间变坡点处，不含首尾）
    curves = []
    for i in range(1, len(pts) - 1):
        Rv = pts[i].Rv
        if not (Rv > 0):
            continue
        g1 = g[i - 1]       # 前坡度
        g2 = g[i]           # 后坡度
        A = g2 - g1          # 坡度代数差
        Lv = Rv * abs(A)     # 竖曲线长度
        if Lv <= 0:
            continue
        T = Lv / 2           # 切线长
        VPI = pts[i].L
        BVC = VPI - T        # 竖曲线起点
        EVC = VPI + T        # 竖曲线终点
        Z_VPI = pts[i].Z
        Z_BVC = Z_VPI - g1 * T  # 竖曲线起点高程

        curves.append(VerticalCurve(
            index=i, VPI=VPI, Z_VPI=Z_VPI, Rv=Rv,
            g1=g1, g2=g2, A=A, Lv=Lv, T=T,
            BVC=BVC, EVC=EVC, Z_BVC=Z_BVC,
        ))

    return ZdmModel(pts=pts, g=g, curves=curves)


def parse_sup_standard(text: str) -> List[SuperSegment]:
    """
    解析标准格式 SUP 文本
    格式: id, L0, L1, axis, leftPct, rightPct, gradRate
    对应 JS parseSUP()
    """
    lines = clean_lines(text)
    segs = []

    for line in lines:
        parts = [p.strip() for p in line.replace(',', ' ').split()]
        parts = [p for p in parts if p]
        if len(parts) < 7:
            continue
        try:
            seg_id = int(float(parts[0]))
            L0 = parse_station(parts[1])
            L1 = parse_station(parts[2])
            axis = float(parts[3])
            leftPct = float(parts[4])
            rightPct = float(parts[5])
            gradRate = float(parts[6])
        except (ValueError, IndexError):
            continue
        if not all(math.isfinite(v) for v in [seg_id, L0, L1, axis, leftPct, rightPct, gradRate]):
            continue

        segs.append(SuperSegment(
            L0=L0, L1=L1,
            left=leftPct / 100, right=rightPct / 100,
            gradRate=gradRate,
        ))

    segs.sort(key=lambda s: s.L0)

    # 计算起终点超高（渐变段起点取前段终点值）
    for i, cur in enumerate(segs):
        prev = segs[i - 1] if i > 0 else None
        if abs(cur.gradRate) < 1e-12:
            # 常值段
            cur.startLeft = cur.left
            cur.startRight = cur.right
        else:
            # 渐变段：起点取前段终点
            cur.startLeft = prev.endLeft if prev else 0
            cur.startRight = prev.endRight if prev else 0
        cur.endLeft = cur.left
        cur.endRight = cur.right

    return segs


# ========================================
# 平面坐标求解
# ========================================

def solve_planar(L: float, line_elems: List[PlaneElement]) -> dict:
    """
    计算平面坐标
    对应 JS solvePlanarAtStation()
    返回: {'X': ..., 'Y': ..., 'azimuthRad': ...}
    """
    if not line_elems:
        raise ValueError('平面线元为空，请先加载 pm 数据')

    e = None
    for elem in line_elems:
        if L >= elem.L0 - 1e-9 and L <= elem.L1 + 1e-9:
            e = elem
            break
    if e is None:
        raise ValueError(f'桩号 {L:.3f} 超出平面范围')

    s = L - e.L0
    a0 = deg_to_rad(e.alpha0)

    if e.type == 1:  # 直线
        return {
            'X': e.X0 + s * math.cos(a0),
            'Y': e.Y0 + s * math.sin(a0),
            'azimuthRad': a0,
        }

    if e.type == 2:  # 圆曲线
        k = e.k
        if abs(k) < 1e-12:
            return {
                'X': e.X0 + s * math.cos(a0),
                'Y': e.Y0 + s * math.sin(a0),
                'azimuthRad': a0,
            }
        az = a0 + s * k
        return {
            'X': e.X0 + (math.sin(az) - math.sin(a0)) / k,
            'Y': e.Y0 + (-math.cos(az) + math.cos(a0)) / k,
            'azimuthRad': az,
        }

    # 缓和曲线 (type == 3)
    k0 = e.k0
    ck = e.ck
    azimuth_rad = a0 + k0 * s + 0.5 * ck * s * s

    fx = lambda t: math.cos(a0 + k0 * t + 0.5 * ck * t * t)
    fy = lambda t: math.sin(a0 + k0 * t + 0.5 * ck * t * t)

    dX = simpson_integrate(fx, s, 400)
    dY = simpson_integrate(fy, s, 400)

    return {
        'X': e.X0 + dX,
        'Y': e.Y0 + dY,
        'azimuthRad': azimuth_rad,
    }


# ========================================
# 纵断面求解
# ========================================

def solve_vertical(L: float, zdm_model: ZdmModel) -> dict:
    """
    计算高程和坡度
    对应 JS solveVerticalAtStation()
    返回: {'Z': ..., 'grade': ...}  (grade 为比例，非百分比)
    """
    if zdm_model is None or len(zdm_model.pts) < 2:
        raise ValueError('纵断面数据无效，请先加载 zdm 数据')

    pts = zdm_model.pts
    g = zdm_model.g
    curves = zdm_model.curves

    # 优先检查是否在竖曲线范围内
    for c in curves:
        if L >= c.BVC - 1e-9 and L <= c.EVC + 1e-9:
            x = L - c.BVC
            Z = c.Z_BVC + c.g1 * x + (c.A / (2 * c.Lv)) * x * x
            grade = c.g1 + (c.A / c.Lv) * x
            return {'Z': Z, 'grade': grade}

    # 不在竖曲线内 → 直线段
    i_seg = len(pts) - 2
    for i in range(len(pts) - 1):
        if L >= pts[i].L - 1e-9 and L <= pts[i + 1].L + 1e-9:
            i_seg = i
            break

    p0 = pts[i_seg]
    grade = g[i_seg]
    Z = p0.Z + (L - p0.L) * grade
    return {'Z': Z, 'grade': grade}


# ========================================
# 超高求解
# ========================================

def solve_superelevation(L: float, sup_segs: List[SuperSegment]) -> dict:
    """
    计算超高
    对应 JS solveSuperelevationAtStation()
    返回: {'crossSlope': ...}  (比例)
    """
    seg = None
    for s in sup_segs:
        if L >= s.L0 - 1e-9 and L <= s.L1 + 1e-9:
            seg = s
            break

    if seg is None:
        return {'crossSlope': 0}

    denom = seg.L1 - seg.L0
    t = clamp((L - seg.L0) / denom, 0, 1) if denom > 1e-12 else 0
    right = seg.startRight + (seg.endRight - seg.startRight) * t
    return {'crossSlope': right}


# ========================================
# 主计算类
# ========================================

class RoadCenterline:
    """道路中心线计算器，对应 JS buildSolver()"""

    def __init__(self):
        self.line_elems: List[PlaneElement] = []
        self.zdm_model: Optional[ZdmModel] = None
        self.sup_segs: List[SuperSegment] = []

    # ---------- 加载原始 HintCAD 文件 ----------

    def load_pm_raw(self, content: str):
        """加载原始 .pm 文件 → 解析 → 标准格式 → 计算模型"""
        try:
            parsed = parse_pm_raw(content)
        except Exception as e:
            raise ValueError(f'加载原始 PM 数据失败: {e}') from e

        try:
            standard_text = format_output_pm(parsed)
            self.line_elems = parse_pm_standard(standard_text)
        except Exception as e:
            raise ValueError(f'PM 标准化或建模失败: {e}') from e

        return parsed

    def load_zdm_raw(self, content: str):
        """加载原始 .zdm 文件 → 解析 → 标准格式 → 计算模型"""
        try:
            parsed = parse_zdm_raw(content)
        except Exception as e:
            raise ValueError(f'加载原始 ZDM 数据失败: {e}') from e

        try:
            standard_text = format_output_zdm(parsed)
            self.zdm_model = parse_zdm_standard(standard_text)
        except Exception as e:
            raise ValueError(f'ZDM 标准化或建模失败: {e}') from e

        return parsed

    def load_sup_raw(self, content: str):
        """加载原始 .sup 文件 → 解析 → 标准格式 → 计算模型"""
        try:
            parsed = parse_sup_raw(content)
        except Exception as e:
            raise ValueError(f'加载原始 SUP 数据失败: {e}') from e

        try:
            standard_text = format_output_sup(parsed)
            self.sup_segs = parse_sup_standard(standard_text)
        except Exception as e:
            raise ValueError(f'SUP 标准化或建模失败: {e}') from e

        return parsed

    # ---------- 加载标准格式文本 ----------

    def load_pm(self, text: str):
        """加载标准格式 PM 文本"""
        try:
            self.line_elems = parse_pm_standard(text)
        except Exception as e:
            raise ValueError(f'加载标准 PM 文本失败: {e}') from e

    def load_zdm(self, text: str):
        """加载标准格式 ZDM 文本"""
        try:
            self.zdm_model = parse_zdm_standard(text)
        except Exception as e:
            raise ValueError(f'加载标准 ZDM 文本失败: {e}') from e

    def load_sup(self, text: str):
        """加载标准格式 SUP 文本"""
        try:
            self.sup_segs = parse_sup_standard(text)
        except Exception as e:
            raise ValueError(f'加载标准 SUP 文本失败: {e}') from e

    # ---------- 求解 ----------

    def solve_at_station(self, L: float) -> Point3D:
        """
        计算指定桩号的三维坐标
        对应 JS solveAtStation()
        """
        planar = solve_planar(L, self.line_elems)
        vert = solve_vertical(L, self.zdm_model) if self.zdm_model else {'Z': 0, 'grade': 0}
        sup = solve_superelevation(L, self.sup_segs) if self.sup_segs else {'crossSlope': 0}

        azimuth_deg = rad_to_deg(planar['azimuthRad'])
        grade_deg = rad_to_deg(math.atan(vert['grade']))
        cross_deg = rad_to_deg(math.atan(sup['crossSlope']))
        composite_slope = math.sqrt(vert['grade'] ** 2 + sup['crossSlope'] ** 2)

        # 切线方向向量
        tx0 = math.cos(planar['azimuthRad'])
        ty0 = math.sin(planar['azimuthRad'])
        tz0 = vert['grade']
        length = math.sqrt(tx0 ** 2 + ty0 ** 2 + tz0 ** 2) or 1

        return Point3D(
            L=L,
            X=planar['X'],
            Y=planar['Y'],
            Z=vert['Z'],
            azimuthDeg=azimuth_deg,
            gradeDeg=grade_deg,
            crossDeg=cross_deg,
            compositeSlope=composite_slope,
            grade=vert['grade'],
            crossSlope=sup['crossSlope'],
            tangent={'x': tx0 / length, 'y': ty0 / length, 'z': tz0 / length},
        )

    def sample_alignment(self, L0: float, L1: float, step: float = 10) -> List[Point3D]:
        """采样对齐线，对应 JS sampleAlignment()"""
        if L1 < L0:
            raise ValueError('L1 必须 >= L0')
        if step <= 0:
            raise ValueError('step 必须 > 0')
        out = []
        L = L0
        while L <= L1 + 1e-9:
            out.append(self.solve_at_station(L))
            L += step
        if not out or abs(out[-1].L - L1) > 1e-6:
            out.append(self.solve_at_station(L1))
        return out

    def query(self, station: float) -> dict:
        """查询指定桩号的信息"""
        pt = self.solve_at_station(station)
        return {
            'station': format_station(station),
            'X': round(pt.X, 4),
            'Y': round(pt.Y, 4),
            'Z': round(pt.Z, 4),
            'azimuthDeg': round(pt.azimuthDeg, 6),
            'gradeDeg': round(pt.gradeDeg, 6),
            'crossDeg': round(pt.crossDeg, 6),
            'compositeSlope': round(pt.compositeSlope, 6),
            'grade': round(pt.grade, 6),
            'crossSlope': round(pt.crossSlope, 6),
            'tangent': {k: round(v, 9) for k, v in pt.tangent.items()},
        }

    def to_csv(self, points: List[Point3D]) -> str:
        """导出 CSV，对应 JS toCSV()"""
        header = 'L,X,Y,Z,azimuthDeg,gradeDeg,crossDeg,compositeSlope,grade,crossSlope,tx,ty,tz'
        rows = []
        for p in points:
            rows.append(','.join([
                f'{p.L:.3f}', f'{p.X:.3f}', f'{p.Y:.3f}', f'{p.Z:.3f}',
                f'{p.azimuthDeg:.6f}', f'{p.gradeDeg:.6f}', f'{p.crossDeg:.6f}',
                f'{p.compositeSlope:.6f}', f'{p.grade:.6f}', f'{p.crossSlope:.6f}',
                f'{p.tangent["x"]:.9f}', f'{p.tangent["y"]:.9f}', f'{p.tangent["z"]:.9f}',
            ]))
        return header + '\n' + '\n'.join(rows)


# ========================================
# 使用示例
# ========================================

if __name__ == '__main__':
    road = RoadCenterline()

    # 使用 HTML 中内嵌的示例数据（原始 HintCAD 格式）
    SAMPLE_PM = (
        "HINTCAD5.83_PM_SHUJU_PM\n"
        "         5\tNULL\tNULL\t0.000000\n"
        "         1\t0.0000\t6.252194703415\t0.000000000000\t41801.83399000\t36973.58929000\n"
        "         1\t0.000000\t36.39140501\t0.00000000\t9999.00000000\t9999.00000000\t         1\n"
        "41801.83399000\t36973.58929000\t41820.02095547\t36973.02548445\t41838.20792094\t36972.46167890\t0.00000000\t0.00000000\n"
        "0.00000000\t36.39140501\t6.252194703415\t6.252194703415\n"
        "         1\t0.000000\t-35.00000000\t43.47413024\t9999.00000000\t54.00000000\t        21\n"
        "41838.20792094\t36972.46167890\t41861.65965448\t36971.73466276\t41872.94176615\t36975.13931009\t41857.34086496\t37026.83662029\n"
        "36.39140501\t71.39140501\t6.252194703415\t0.293083470331\n"
        "         1\t0.000000\t-50.30392997\t0.00000000\t54.00000000\t54.00000000\t         3\n"
        "41872.94176615\t36975.13931010\t41898.92818123\t36982.98133307\t41908.13774627\t37008.51513429\t41857.34086496\t37026.83662030\n"
        "71.39140501\t121.69533498\t0.293083470309\t1.224637729013\n"
        "         1\t0.000000\t-35.00000000\t43.47413024\t54.00000000\t9999.00000000\t        22\n"
        "41908.13774626\t37008.51513428\t41912.13611775\t37019.60074174\t41912.65424481\t37043.05801999\t0.00000000\t0.00000000\n"
        "121.69533498\t156.69533498\t1.224637728940\t1.548711803035\n"
        "         1\t0.000000\t22.44628361\t0.00000000\t9999.00000000\t9999.00000000\t         1\n"
        "41912.65424481\t37043.05801999\t41912.90208240\t37054.27842500\t41913.14992000\t37065.49883000\t0.00000000\t0.00000000\n"
        "156.69533498\t179.14161859\t1.548711803014\t1.548711803014"
    )

    SAMPLE_ZDM = (
        "HINTCAD5.83_ZDM_SHUJU\n"
        "\t         3\n"
        "   -90.000\t9.15160000\t0.00000000\t     0.000\t0.00000000\n"
        "    40.000\t14.09170000\t1200.00000000\t     0.000\t0.00000000\n"
        "   179.142\t11.27300000\t0.00000000\t     0.000\t0.00000000"
    )

    SAMPLE_SUP = (
        "HINTCAD5.83_SUP_SHUJU\n"
        "     -3.00\t     -2.00\t     -2.00\t     0.000\t     -2.00\t     -2.00\t     -3.00\n"
        "   9999.00\t   9999.00\t   9999.00\t    35.391\t   9999.00\t   9999.00\t     -3.00\n"
        "     -3.00\t     -2.00\t     -2.00\t    36.391\t     -2.00\t     -2.00\t     -2.00\n"
        "   9999.00\t   9999.00\t   9999.00\t    49.119\t     -2.00\t     -2.00\t     -2.00\n"
        "     -3.00\t      9.00\t      9.00\t    71.391\t     -9.00\t     -9.00\t     -9.00\n"
        "     -3.00\t      9.00\t      9.00\t   121.695\t     -9.00\t     -9.00\t     -9.00\n"
        "   9999.00\t   9999.00\t   9999.00\t   143.968\t     -2.00\t     -2.00\t     -2.00\n"
        "     -3.00\t     -2.00\t     -2.00\t   156.695\t     -2.00\t     -2.00\t     -2.00\n"
        "   9999.00\t   9999.00\t   9999.00\t   157.695\t   9999.00\t   9999.00\t     -3.00\n"
        "     -3.00\t     -2.00\t     -2.00\t   179.142\t     -2.00\t     -2.00\t     -3.00"
    )

    road.load_pm_raw(SAMPLE_PM)
    road.load_zdm_raw(SAMPLE_ZDM)
    road.load_sup_raw(SAMPLE_SUP)

    # 查询桩号
    test_stations = [0, 25, 50, 75, 100, 150, 179]
    for sta in test_stations:
        result = road.query(sta)
        print(f"桩号 {result['station']}:")
        print(f"  X = {result['X']},  Y = {result['Y']},  Z = {result['Z']}")
        print(f"  方位角 = {result['azimuthDeg']}°,  坡度角 = {result['gradeDeg']}°")
        print(f"  横坡角 = {result['crossDeg']}°,  合成坡度 = {result['compositeSlope']}")
        print(f"  切线向量 = ({result['tangent']['x']}, {result['tangent']['y']}, {result['tangent']['z']})")
        print()

    # 采样并导出 CSV
    points = road.sample_alignment(0, 179, step=10)
    csv = road.to_csv(points)
    print("=== CSV 输出 (前5行) ===")
    for line in csv.split('\n')[:6]:
        print(line)
