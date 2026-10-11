/*
 * 通用小型无刷电机测试固定架
 * 单位：mm
 * OpenSCAD 中将 part 改成 base/upright/adapter/assembly 后导出对应 STL。
 */

$fn = 96;
part = "assembly";

base_l = 180;
base_w = 110;
base_t = 6;
axis_h = 75;

upright_w = 90;
upright_h = 100;
upright_t = 8;
foot_depth = 34;

adapter_size = 60;
adapter_t = 3;
carrier_pitch = 44;
carrier_hole = 4.3;
center_hole = 13;

motor_pitch = 13.4;
motor_hole = 2.7;

module rounded_box(size, r = 4) {
    hull() {
        for (x = [r, size[0] - r], y = [r, size[1] - r])
            translate([x, y, 0]) cylinder(h = size[2], r = r);
    }
}

module base_plate() {
    difference() {
        translate([-base_l / 2, -base_w / 2, 0]) rounded_box([base_l, base_w, base_t], 6);
        for (x = [-75, 75], y = [-40, 40])
            translate([x, y, -1]) cylinder(h = base_t + 2, d = 6.5);
        /* 立板底脚的 M5 安装孔 */
        for (x = [-32, 32], y = [-12, 12])
            translate([x, y, -1]) cylinder(h = base_t + 2, d = 5.5);
    }
}

module upright_plate() {
    difference() {
        union() {
            /* 竖直安装面：局部坐标 X 为宽，Z 为高 */
            translate([-upright_w / 2, -upright_t / 2, 0])
                cube([upright_w, upright_t, upright_h]);
            /* 底脚 */
            translate([-upright_w / 2, -foot_depth / 2, 0])
                cube([upright_w, foot_depth, 8]);
            /* 两侧角撑 */
            for (x = [-upright_w / 2 + 8, upright_w / 2 - 8])
                translate([x, 0, 8]) rotate([90, 0, 0])
                    linear_extrude(height = upright_t + 16, center = true)
                        polygon([[0, 0], [0, 35], [18 * sign(-x), 0]]);
        }
        /* 电机中心和适配片连接孔，轴心高 75 mm */
        translate([0, 0, axis_h]) rotate([90, 0, 0])
            cylinder(h = upright_t + 2, d = 24, center = true);
        for (x = [-carrier_pitch / 2, carrier_pitch / 2], z = [-carrier_pitch / 2, carrier_pitch / 2])
            translate([x, 0, axis_h + z]) rotate([90, 0, 0])
                cylinder(h = upright_t + 2, d = carrier_hole, center = true);
        for (x = [-32, 32], y = [-12, 12])
            translate([x, y, -1]) cylinder(h = 10, d = 5.5);
    }
}

module adapter_2804() {
    difference() {
        translate([-adapter_size / 2, -adapter_size / 2, 0])
            rounded_box([adapter_size, adapter_size, adapter_t], 4);
        translate([0, 0, -1]) cylinder(h = adapter_t + 2, d = center_hole);
        for (x = [-carrier_pitch / 2, carrier_pitch / 2], y = [-carrier_pitch / 2, carrier_pitch / 2])
            translate([x, y, -1]) cylinder(h = adapter_t + 2, d = carrier_hole);
        for (x = [-motor_pitch / 2, motor_pitch / 2], y = [-motor_pitch / 2, motor_pitch / 2])
            translate([x, y, -1]) cylinder(h = adapter_t + 2, d = motor_hole);
    }
}

module assembly() {
    color("silver") base_plate();
    color("gainsboro") upright_plate();
    /* 适配片位于立板外侧；装配示意，不包含垫柱与螺钉 */
    color("orange") translate([0, -upright_t / 2 - adapter_t, axis_h])
        rotate([90, 0, 0]) translate([0, 0, -adapter_t]) adapter_2804();
}

if (part == "base") base_plate();
else if (part == "upright") upright_plate();
else if (part == "adapter") adapter_2804();
else assembly();

