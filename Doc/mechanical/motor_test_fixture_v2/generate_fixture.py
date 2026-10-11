"""生成小于 60 mm 轮子的通用无刷电机测试架。单位均为 mm。

立板只保留 48×48 mm 的 M4 接口和 Φ42 中心孔。
电机孔全部做在 60×60×3 mm 可换适配片上。
"""

import math
import os
from pathlib import Path

import ezdxf
from build123d import (
    Align,
    Axis,
    Box,
    BuildLine,
    BuildSketch,
    Compound,
    Cone,
    Cylinder,
    Line,
    Plane,
    Pos,
    Rectangle,
    ThreePointArc,
    chamfer,
    extrude,
    fillet,
    make_face,
    export_step,
    export_stl,
)


OUT = Path(__file__).resolve().parent / "out"
AXIS_Z = 84.0  # 底板顶面 z=6，轴心在顶面以上 78 mm。角码竖边按 40 mm，要离开适配片。
BRACKET_X = 40.0  # 角码孔。再靠里会和底板 M4 沉孔穿通。
CENTER_D = 42.0  # 中心通孔。M2.5 头大约占到半径 12 mm，Φ42 给扳手留斜着进去的空。
PLATE = 60.0
PLATE_T = 3.0
# 立板厚 8 mm 且关于 y=0 对称，电机侧表面在 y=-4。适配片背面贴在该面上。
ADAPTER_Y = -4.0 - PLATE_T / 2.0


def y_hole(x: float, z: float, diameter: float, depth: float = 40):
    # 孔以 y=0 为中心。立板在 y=-4..4，只往负方向打会在背面留下 4 mm 实心。
    return Pos(x, 0, z) * Cylinder(diameter / 2, depth).rotate(Axis.X, 90)


def y_slot(x1: float, z1: float, x2: float, z2: float, diameter: float, depth: float = 30):
    """两端为圆、中间等宽的槽。孔心可以落在两端之间的中心线上。"""
    cutter = y_hole(x1, z1, diameter, depth) + y_hole(x2, z2, diameter, depth)
    dx, dz = x2 - x1, z2 - z1
    length = math.hypot(dx, dz)
    if length < 1e-6:
        return cutter
    angle = math.degrees(math.atan2(dz, dx))
    box = Box(length, depth, diameter, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # build123d 绕 +Y 的正角度把 +X 转到 -Z，所以这里取负角。
    box = box.rotate(Axis.Y, -angle)
    return cutter + Pos((x1 + x2) / 2, 0, (z1 + z2) / 2) * box


def base_plate():
    plate = extrude(fillet(Rectangle(160, 100).vertices(), 8), 6)
    plate = chamfer(plate.edges().group_by(Axis.Z)[-1], 0.8)
    # 离 R8 圆角远一点，否则 Φ6.5 会把圆角铣破。
    for x in (-60, 60):
        for y in (-32, 32):
            plate -= Pos(x, y, -1) * Cylinder(3.25, 9, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 欧标 4040 压铸角码，孔心离内角 20 mm。沉头从底面拧入，头与底面齐平。
    for x in (-BRACKET_X, BRACKET_X):
        plate -= Pos(x, 24, -1) * Cylinder(4.5, 9, align=(Align.CENTER, Align.CENTER, Align.MIN))
        plate -= Pos(x, 24, 0) * Cone(8.5, 4.5, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 与立板相同的适配片接口，卸下立板后可把电机平放、轴朝上。
    # 中心通孔让 M2.5 螺钉头落在孔内，并能从背面斜着伸进内六角扳手。
    plate -= Pos(0, 0, -1) * Cylinder(CENTER_D / 2, 9, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x in (-24, 24):
        for y in (-24, 24):
            plate -= Pos(x, y, -1) * Cylinder(2.15, 9, align=(Align.CENTER, Align.CENTER, Align.MIN))
            # M4 内六角头高 4 mm。沉孔让头低于底面，平放时桌子上仍然是平的。
            plate -= Pos(x, y, 0) * Cylinder(4, 4.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plate


def upright_wall():
    # 只圆顶部两角。电机安装面保持平面，适配片才能贴死。
    with BuildSketch(Plane.XZ) as sketch:
        with BuildLine():
            Line((-55, 6), (55, 6))
            Line((55, 6), (55, 100))
            ThreePointArc((55, 100), (49.728, 112.728), (37, 118))
            Line((37, 118), (-37, 118))
            ThreePointArc((-37, 118), (-49.728, 112.728), (-55, 100))
            Line((-55, 100), (-55, 6))
        make_face()
    # Plane.XZ 的拉伸沿 -Y。平移后立板占 y=-4..4，电机侧在 y=-4。
    return Pos(0, 4, 0) * extrude(sketch.sketch, amount=8)


def cast_angle_bracket(x_center: float):
    """欧标 4040 压铸角码的示意件，只放进装配体，不单独下单加工。

    内角贴在立板背面 y=4、底板顶面 z=6。孔心离内角 20 mm，与底板上 y=24、
    立板上 z=26 的 Φ9 孔同心。腿长按 40 mm 留空，厚 6、宽 28，不是图纸逐尺寸复刻。
    """
    width = 28.0
    thick = 6.0
    leg = 40.0
    x0 = x_center - width / 2
    vertical = Pos(x0, 4, 6) * Box(width, thick, leg, align=(Align.MIN, Align.MIN, Align.MIN))
    horizontal = Pos(x0, 4, 6) * Box(width, leg, thick, align=(Align.MIN, Align.MIN, Align.MIN))
    part = vertical + horizontal
    # 竖边 y=4..10，横边 z=6..12。孔只打穿贴合的那一壁，螺栓头才能落在外壁上。
    part -= Pos(x_center, 6, 26) * Cylinder(4.25, 8).rotate(Axis.X, 90)
    part -= Pos(x_center, 24, 5) * Cylinder(4.25, 7, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return part


def m8_countersink_from_front(x: float, z: float):
    """90° 沉头。大端在立板正面 y=-4，向板内收到和 Φ9 通孔相接。"""
    cone = Cone(4.5, 8.5, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(x, 0, z) * cone.rotate(Axis.X, 90)


def m8_bolt_y(x: float, z: float):
    """M8×20 沉头，从立板正面拧入。薄螺母贴在角码背面，螺杆停在另一颗螺丝前面。"""
    head = Pos(x, 0, z) * Cone(4, 8.2, 4, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.X, 90)
    shank = Pos(x, 8, z) * Cylinder(4, 16).rotate(Axis.X, 90)
    nut = Pos(x, 12.5, z) * Cylinder(6.5, 5).rotate(Axis.X, 90)
    return head + shank + nut


def m8_bolt_z(x: float, y: float):
    """M8×20 沉头，从底板底面拧入。薄螺母贴在角码上表面，不碰到立板上的那颗。"""
    head = Pos(x, y, 0) * Cone(8.2, 4, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    shank = Pos(x, y, 4) * Cylinder(4, 16, align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Pos(x, y, 12) * Cylinder(6.5, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return head + shank + nut


def m4_bolt_y(x: float, z: float):
    """M4×20 内六角。头在适配片电机侧，平垫和螺母在立板背面。长度是头下的长度。"""
    head = Pos(x, -9, z) * Cylinder(3.5, 4).rotate(Axis.X, 90)
    shank = Pos(x, 3, z) * Cylinder(2, 20).rotate(Axis.X, 90)
    washer = Pos(x, 4.5, z) * Cylinder(4.5, 1).rotate(Axis.X, 90)
    nut = Pos(x, 6.6, z) * Cylinder(3.8, 3.2).rotate(Axis.X, 90)
    return head + shank + washer + nut, nut


def upright_bracket():
    # 一块 8 mm 平板，底部用现成角码固定。不再铣底脚和弧形撑。
    part = upright_wall()
    part -= y_hole(0, AXIS_Z, CENTER_D, 40)
    for x in (-24, 24):
        for z in (AXIS_Z - 24, AXIS_Z + 24):
            part -= y_hole(x, z, 4.3, 30)
    # 离底板顶面 20 mm，与角码孔心对齐。
    for x in (-BRACKET_X, BRACKET_X):
        part -= y_hole(x, 26, 9, 30)
        part -= m8_countersink_from_front(x, 26)
    return part


def adapter_blank(size: float = PLATE, corner: float = 6):
    face = Plane.XZ * fillet(Rectangle(size, size).vertices(), corner)
    # 拉伸沿 -Y，得到 y=-3..0；再平移到贴紧立板的位置。
    return Pos(0, -4, AXIS_Z) * extrude(face, amount=PLATE_T)


def add_carrier(part, center_d: float):
    part -= y_hole(0, AXIS_Z, center_d, 30)
    # 适配片是打印件，Φ4.8 给 M4 留收缩量。铝板上仍是 Φ4.3。
    for x in (-24, 24):
        for z in (AXIS_Z - 24, AXIS_Z + 24):
            part -= y_hole(x, z, 4.8, 30)
    return part


def cut_slots(part, slots):
    for slot in slots:
        part -= slot
    return part


def square_slots(pitch_a: float, pitch_b: float, diameter: float):
    """方孔距的四孔都在 45° 对角线上，孔心坐标是 (±P/2, ±P/2)。"""
    slots = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            slots.append(
                y_slot(
                    sx * pitch_a / 2,
                    AXIS_Z + sz * pitch_a / 2,
                    sx * pitch_b / 2,
                    AXIS_Z + sz * pitch_b / 2,
                    diameter,
                )
            )
    return slots


def rect_16_19_slots(diameter: float = 3.4):
    """覆盖 16×16、16×19、19×16、19×19。每象限用四条边槽连成一个窗口。"""
    slots = []
    corners = ((8.0, 8.0), (8.0, 9.5), (9.5, 9.5), (9.5, 8.0))
    for sx in (-1, 1):
        for sz in (-1, 1):
            signed = [(sx * a, AXIS_Z + sz * b) for a, b in corners]
            for i in range(4):
                x1, z1 = signed[i]
                x2, z2 = signed[(i + 1) % 4]
                slots.append(y_slot(x1, z1, x2, z2, diameter))
    return slots


def three_hole_slots(pcd_a: float, pcd_b: float, diameter: float):
    """三孔中心圆直径从 pcd_a 到 pcd_b。一个槽朝上，另外两个间隔 120°。"""
    slots = []
    for deg in (90.0, 210.0, 330.0):
        rad = math.radians(deg)
        ca, sa = math.cos(rad), math.sin(rad)
        slots.append(
            y_slot(
                (pcd_a / 2) * ca,
                AXIS_Z + (pcd_a / 2) * sa,
                (pcd_b / 2) * ca,
                AXIS_Z + (pcd_b / 2) * sa,
                diameter,
            )
        )
    return slots


def adapter_2804():
    """当前电机定子端：13.4×13.4，4×M2.5。中心孔让开图上 Φ12/Φ13 和 14 mm 凸台。"""
    part = add_carrier(adapter_blank(), 14.5)
    for x in (-6.7, 6.7):
        for z in (AXIS_Z - 6.7, AXIS_Z + 6.7):
            part -= y_hole(x, z, 3.0, 30)
    return part


def adapter_m25_12_to_16():
    part = add_carrier(adapter_blank(), 12)
    return cut_slots(part, square_slots(12, 16, 3.0))


def adapter_m3_16_to_40():
    part = add_carrier(adapter_blank(), 16)
    part = cut_slots(part, rect_16_19_slots(3.4))
    # 19×19 到 40×40。再往外会切到 48 mm 的 M4 安装孔。
    return cut_slots(part, square_slots(19, 40, 3.4))


def adapter_m4_25_to_38():
    part = add_carrier(adapter_blank(), 22)
    return cut_slots(part, square_slots(25, 38, 4.5))


def cross_slots(radius_a: float, radius_b: float, diameter: float):
    """十字槽。方孔电机转 45° 后，四颗螺丝落在坐标轴上。"""
    slots = []
    for sx, sz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        slots.append(
            y_slot(
                sx * radius_a,
                AXIS_Z + sz * radius_a,
                sx * radius_b,
                AXIS_Z + sz * radius_b,
                diameter,
            )
        )
    return slots


def adapter_multi():
    """60×60 加工用一片。对角是 M2.5/M3，十字是转 45° 后的 M4。

    对角槽宽 3.4，方孔距 16–40，含 16×19 窗口。13.4 的 2804 落在槽内端，略有活动。
    十字槽对应方孔距约 26–36 的 M4。再往外会切到板边或 48 mm 安装孔。
    """
    part = add_carrier(adapter_blank(), 14.5)
    part = cut_slots(part, rect_16_19_slots(3.4))
    part = cut_slots(part, square_slots(16, 40, 3.4))
    part = cut_slots(part, cross_slots(26 / math.sqrt(2), 36 / math.sqrt(2), 4.5))
    for x in (-6.7, 6.7):
        for z in (AXIS_Z - 6.7, AXIS_Z + 6.7):
            part -= y_hole(x, z, 3.0, 40)
    return part


def adapter_m3_cross():
    """100 mm 片。十字槽离中心 28–42 mm，对应方孔距约 40–59 mm。"""
    part = add_carrier(adapter_blank(100, 8), 32)
    return cut_slots(part, cross_slots(28, 42, 3.4))


def adapter_3hole(pcd_a: float, pcd_b: float, center_d: float):
    part = add_carrier(adapter_blank(), center_d)
    return cut_slots(part, three_hole_slots(pcd_a, pcd_b, 3.4))


def dxf_point(x: float, dz: float, size: float = PLATE):
    return (size / 2 + x, size / 2 + dz)


def add_dxf_slot(msp, x1, dz1, x2, dz2, diameter: float, size: float = PLATE):
    r = diameter / 2
    a = dxf_point(x1, dz1, size)
    b = dxf_point(x2, dz2, size)
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    msp.add_circle(a, r)
    msp.add_circle(b, r)
    if length < 1e-6:
        return
    px, py = -dy / length * r, dx / length * r
    msp.add_line((a[0] + px, a[1] + py), (b[0] + px, b[1] + py))
    msp.add_line((a[0] - px, a[1] - py), (b[0] - px, b[1] - py))


def add_rounded_rect(msp, size: float, radius: float):
    span = size - radius
    msp.add_line((radius, 0), (span, 0))
    msp.add_arc((span, radius), radius, 270, 360)
    msp.add_line((size, radius), (size, span))
    msp.add_arc((span, span), radius, 0, 90)
    msp.add_line((span, size), (radius, size))
    msp.add_arc((radius, span), radius, 90, 180)
    msp.add_line((0, span), (0, radius))
    msp.add_arc((radius, radius), radius, 180, 270)


def write_adapter_dxf(path: Path, center_d: float, circles, slots, size: float = PLATE, corner: float = 6):
    doc = ezdxf.new("R12")
    msp = doc.modelspace()
    add_rounded_rect(msp, size, corner)
    msp.add_circle(dxf_point(0, 0, size), center_d / 2)
    for x in (-24, 24):
        for dz in (-24, 24):
            msp.add_circle(dxf_point(x, dz, size), 2.4)
    for x, dz, diameter in circles:
        msp.add_circle(dxf_point(x, dz, size), diameter / 2)
    for slot in slots:
        add_dxf_slot(msp, *slot, size=size)
    doc.saveas(path)


def open_at(part, x: float, z: float) -> bool:
    clipped = part & y_hole(x, z, 0.6, 12)
    return clipped.volume < 0.05


def solid_at(part, x: float, z: float) -> bool:
    clipped = part & y_hole(x, z, 0.6, 12)
    return clipped.volume > 0.2


def expect(part, name: str, voids, solids):
    for x, z in voids:
        if not open_at(part, x, z):
            raise RuntimeError(f"{name} 应为通孔: ({x:.2f}, {z:.2f})")
    for x, z in solids:
        if not solid_at(part, x, z):
            raise RuntimeError(f"{name} 应为实体: ({x:.2f}, {z:.2f})")
    box = part.bounding_box()
    if abs(box.max.Y - (-4.0)) > 0.05 or abs(box.min.Y - (-7.0)) > 0.05:
        raise RuntimeError(f"{name} 没有贴在立板电机侧，Y={box.min.Y:.2f}..{box.max.Y:.2f}")


def through_z(part, x: float, y: float, z0: float, z1: float) -> bool:
    probe = Pos(x, y, (z0 + z1) / 2) * Cylinder(0.5, z1 - z0)
    return (part & probe).volume < 0.02


def through_y(part, x: float, z: float, y0: float, y1: float) -> bool:
    probe = Pos(x, (y0 + y1) / 2, z) * Cylinder(0.5, y1 - y0).rotate(Axis.X, 90)
    return (part & probe).volume < 0.02


def three_point(pcd: float, deg: float):
    rad = math.radians(deg)
    return ((pcd / 2) * math.cos(rad), AXIS_Z + (pcd / 2) * math.sin(rad))


def main():
    OUT.mkdir(exist_ok=True)
    base = base_plate()
    bracket = upright_bracket()
    a2804 = adapter_2804()
    am25 = adapter_m25_12_to_16()
    am3 = adapter_m3_16_to_40()
    am4 = adapter_m4_25_to_38()
    across = adapter_m3_cross()
    multi = adapter_multi()
    a3s = adapter_3hole(16, 25, 10)
    a3l = adapter_3hole(25, 36, 18)

    expect(
        a2804,
        "2804",
        [(6.7, AXIS_Z + 6.7), (-6.7, AXIS_Z - 6.7), (0, AXIS_Z)],
        [(0, AXIS_Z + 15), (15, AXIS_Z)],
    )
    expect(
        am25,
        "M2.5槽",
        [(6, AXIS_Z + 6), (6.7, AXIS_Z + 6.7), (8, AXIS_Z + 8), (0, AXIS_Z)],
        [(0, AXIS_Z + 15)],
    )
    expect(
        am3,
        "M3槽",
        [
            (8, AXIS_Z + 8),
            (8, AXIS_Z + 9.5),
            (9.5, AXIS_Z + 8),
            (9.5, AXIS_Z + 9.5),
            (11, AXIS_Z + 11),
            (12.5, AXIS_Z + 12.5),
            (16, AXIS_Z + 16),
            (20, AXIS_Z + 20),
            (0, AXIS_Z),
        ],
        [(0, AXIS_Z + 20), (8, AXIS_Z + 14), (21.8, AXIS_Z + 21.8)],
    )
    expect(
        am4,
        "M4槽",
        [(12.5, AXIS_Z + 12.5), (15, AXIS_Z + 15), (18, AXIS_Z + 18), (19, AXIS_Z + 19), (0, AXIS_Z)],
        [(0, AXIS_Z + 16), (24, AXIS_Z + 16)],
    )
    expect(
        a3s,
        "三孔小",
        [three_point(16, 90), three_point(20, 210), three_point(25, 330), (0, AXIS_Z)],
        [(12, AXIS_Z)],
    )
    for label, part, holes in (
        ("底板", base, ((60, 32, 0, 6), (BRACKET_X, 24, 0, 6), (24, 24, 0, 6), (0, 0, 0, 6))),
    ):
        for x, y, z0, z1 in holes:
            if not through_z(part, x, y, z0, z1):
                raise RuntimeError(f"{label} 孔未打透: ({x}, {y})")
    if not open_at(bracket, 6.7, AXIS_Z + 6.7):
        raise RuntimeError("2804 的 M2.5 孔被立板挡住，背面无法上螺钉")
    if not through_y(bracket, 24, AXIS_Z - 24, -4, 4) or not through_y(bracket, BRACKET_X, 26, -4, 4):
        raise RuntimeError("立板安装孔或角码孔没有打透")
    if not solid_at(bracket, 12, 30) or not solid_at(bracket, 0, 114):
        raise RuntimeError("立板安装面被切掉了")
    if not solid_at(bracket, 17, AXIS_Z - 17):
        raise RuntimeError("中心孔离 M4 太近")
    if not through_z(bracket, 24, 24, 6, 14):
        raise RuntimeError("立板压住了底板的平放接口")

    expect(
        a3l,
        "三孔大",
        [three_point(25, 90), three_point(30, 210), three_point(36, 330), (0, AXIS_Z)],
        [(0, AXIS_Z + 10)],
    )
    expect(
        across,
        "大M3十字槽",
        [(42, AXIS_Z), (0, AXIS_Z + 28), (0, AXIS_Z - 42), (0, AXIS_Z), (24, AXIS_Z - 24)],
        [(20, AXIS_Z), (0, AXIS_Z + 20)],
    )
    if across.bounding_box().min.Z < 12:
        raise RuntimeError("大适配片碰到了底板")
    expect(
        multi,
        "加工合片",
        [
            (6.7, AXIS_Z + 6.7),
            (8, AXIS_Z + 8),
            (8, AXIS_Z + 9.5),
            (9.5, AXIS_Z + 8),
            (16, AXIS_Z + 16),
            (20, AXIS_Z + 20),
            (26 / math.sqrt(2), AXIS_Z),
            (36 / math.sqrt(2), AXIS_Z),
            (0, AXIS_Z),
            (24, AXIS_Z - 24),
        ],
        [(21.8, AXIS_Z + 21.8), (0, AXIS_Z + 12), (29, AXIS_Z)],
    )

    # 角码和螺栓只进装配体，方便查看。不要把这个装配体拿去加工。
    angles = [cast_angle_bracket(x) for x in (-BRACKET_X, BRACKET_X)]
    span = angles[1].bounding_box()
    if span.max.Z > AXIS_Z - 30 - 5:
        raise RuntimeError("角码顶到适配片了")
    if abs(span.min.Y - 4) > 0.05 or abs(span.min.Z - 6) > 0.05:
        raise RuntimeError(f"角码没有贴在立板背面和底板顶面: {span}")
    if not through_z(angles[1], BRACKET_X, 24, 6, 12):
        raise RuntimeError("角码底孔没有和底板 Φ9 对齐")
    if not through_y(angles[1], BRACKET_X, 26, 4, 10):
        raise RuntimeError("角码立孔没有和立板 Φ9 对齐")
    web = Pos(30, 24, 0.3) * Cylinder(0.4, 0.4)
    if (base & web).volume < 0.02:
        raise RuntimeError("沉头孔和旁边的 M4 孔穿通了")
    corner = Pos(66.6, 37.5, 3) * Cylinder(0.6, 2)
    if (base & corner).volume < 0.02:
        raise RuntimeError("底板圆角被固定孔铣破了")
    if abs(base.bounding_box().min.Z) > 0.05:
        raise RuntimeError("底板底面不在 z=0，放不平")
    face = Pos(BRACKET_X, -3.8, 26) * Box(0.6, 0.3, 0.6)
    if (bracket & face).volume > 0.02:
        raise RuntimeError("立板正面沉头没有开到表面")
    beside = Pos(BRACKET_X, -3.8, 40) * Box(0.6, 0.3, 0.6)
    if (bracket & beside).volume < 0.02:
        raise RuntimeError("立板沉头碰到了适配片区域")
    bolts = [m8_bolt_y(x, 26) for x in (-BRACKET_X, BRACKET_X)]
    bolts += [m8_bolt_z(x, 24) for x in (-BRACKET_X, BRACKET_X)]
    m4_bolts = []
    for x in (-24, 24):
        for z in (AXIS_Z - 24, AXIS_Z + 24):
            bolt, nut = m4_bolt_y(x, z)
            m4_bolts.append(bolt)
            for angle in angles:
                if (nut & angle).volume > 0.02:
                    raise RuntimeError(f"M4 螺母碰到角码: ({x}, {z})")
    if (bolts[0] & bolts[2]).volume > 0.01 or (bolts[1] & bolts[3]).volume > 0.01:
        raise RuntimeError("同一只角码上的两颗 M8 相撞")
    if (bolts[0] & bracket).volume > 1 or (m4_bolts[0] & bracket).volume > 1:
        raise RuntimeError("螺栓埋进立板了，孔没有打透")
    if min(bolt.bounding_box().min.Z for bolt in bolts[2:]) < -0.05:
        raise RuntimeError("底板螺栓伸到了底面以下")
    head_y = bolts[0].bounding_box().min.Y
    if head_y < -4.05 or head_y > -3.9:
        raise RuntimeError(f"立板螺栓头没有和正面齐平: y={head_y:.2f}")
    assembly = Compound([base, bracket, multi, *angles, *bolts, *m4_bolts])
    solids = {
        "01_base": base,
        "02_upright_bracket": bracket,
        "03_adapter_2804": a2804,
        "04_adapter_m25_12to16": am25,
        "05_adapter_m3_16to40": am3,
        "06_adapter_m4_25to38": am4,
        "09_adapter_m3_cross": across,
        "10_adapter_multi": multi,
        "07_adapter_3hole_p16to25": a3s,
        "08_adapter_3hole_p25to36": a3l,
        "00_assembly_2804": assembly,
    }
    # STL 写入走系统 ANSI 路径，中文绝对路径会失败。切到 out 后只用文件名。
    cwd = Path.cwd()
    os.chdir(OUT)
    try:
        for name, solid in solids.items():
            export_step(solid, f"{name}.step")
            # 这个参数是相对偏差。0.002 能网格化槽片；过小会失败，过大又保不住小孔。
            if not export_stl(solid, f"{name}.stl", tolerance=0.002, angular_tolerance=0.2):
                raise RuntimeError(f"STL 导出失败: {name}")
    finally:
        os.chdir(cwd)

    write_adapter_dxf(
        OUT / "03_adapter_2804.dxf",
        14.5,
        [(sx * 6.7, sz * 6.7, 3.0) for sx in (-1, 1) for sz in (-1, 1)],
        [],
    )
    write_adapter_dxf(
        OUT / "04_adapter_m25_12to16.dxf",
        12,
        [],
        [(sx * 6, sz * 6, sx * 8, sz * 8, 3.0) for sx in (-1, 1) for sz in (-1, 1)],
    )
    m3_slots = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            corners = ((8.0, 8.0), (8.0, 9.5), (9.5, 9.5), (9.5, 8.0))
            signed = [(sx * a, sz * b) for a, b in corners]
            for i in range(4):
                x1, z1 = signed[i]
                x2, z2 = signed[(i + 1) % 4]
                m3_slots.append((x1, z1, x2, z2, 3.4))
            m3_slots.append((sx * 9.5, sz * 9.5, sx * 20, sz * 20, 3.4))
    write_adapter_dxf(OUT / "05_adapter_m3_16to40.dxf", 16, [], m3_slots)
    write_adapter_dxf(
        OUT / "06_adapter_m4_25to38.dxf",
        22,
        [],
        [(sx * 12.5, sz * 12.5, sx * 19, sz * 19, 4.5) for sx in (-1, 1) for sz in (-1, 1)],
    )
    small_3 = []
    large_3 = []
    for deg in (90.0, 210.0, 330.0):
        rad = math.radians(deg)
        ca, sa = math.cos(rad), math.sin(rad)
        small_3.append((8 * ca, 8 * sa, 12.5 * ca, 12.5 * sa, 3.4))
        large_3.append((12.5 * ca, 12.5 * sa, 18 * ca, 18 * sa, 3.4))
    write_adapter_dxf(OUT / "07_adapter_3hole_p16to25.dxf", 10, [], small_3)
    write_adapter_dxf(OUT / "08_adapter_3hole_p25to36.dxf", 18, [], large_3)
    write_adapter_dxf(
        OUT / "09_adapter_m3_cross.dxf",
        32,
        [],
        [(28, 0, 42, 0, 3.4), (-28, 0, -42, 0, 3.4), (0, 28, 0, 42, 3.4), (0, -28, 0, -42, 3.4)],
        size=100,
        corner=8,
    )
    multi_slots = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            corners = ((8.0, 8.0), (8.0, 9.5), (9.5, 9.5), (9.5, 8.0))
            signed = [(sx * a, sz * b) for a, b in corners]
            for i in range(4):
                x1, z1 = signed[i]
                x2, z2 = signed[(i + 1) % 4]
                multi_slots.append((x1, z1, x2, z2, 3.4))
            multi_slots.append((sx * 8, sz * 8, sx * 20, sz * 20, 3.4))
    r0, r1 = 26 / math.sqrt(2), 36 / math.sqrt(2)
    multi_slots.extend(((r0, 0, r1, 0, 4.5), (-r0, 0, -r1, 0, 4.5), (0, r0, 0, r1, 4.5), (0, -r0, 0, -r1, 4.5)))
    write_adapter_dxf(
        OUT / "10_adapter_multi.dxf",
        14.5,
        [(sx * 6.7, sz * 6.7, 3.0) for sx in (-1, 1) for sz in (-1, 1)],
        multi_slots,
    )

    for stale in (
        *OUT.glob("04_adapter_16x16_16x19.*"),
        *OUT.glob("05_adapter_m3_16to32.*"),
        *OUT.glob("06_adapter_m4_25to36.*"),
    ):
        stale.unlink()


if __name__ == "__main__":
    main()
