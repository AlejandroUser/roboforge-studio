// ============================================================================
// Franka Emika Panda 7-DOF Cobot — Parametric Multi-Part Benchmark (OpenSCAD)
// Authentic DH-inspired geometry: reach 855 mm, payload 3.0 kg, 45+ parts
// Reference offsets: d1=333, d3=316, d5=384, a4=+82.5, a5=-82.5, a7=+88, dF=107
// ============================================================================

$fn = 48;

// Global kinematic pose (degrees)
q1 = 0; q2 = 20; q3 = 0; q4 = -75; q5 = 0; q6 = 60; q7 = 0;
gripper_open_mm = 32; // 0..80 mm stroke
explode = 0;          // Set to 15..35 for exploded view

// Authentic Franka Panda link parameters (mm)
D1 = 333; D3 = 316; D5 = 384; DF = 107;
A4 = 82.5; A5 = -82.5; A7 = 88.0;

// Colors (Panda White + Dark Graphite Rings + Satin Aluminum)
COL_BODY   = [0.94, 0.95, 0.96];
COL_RING   = [0.18, 0.20, 0.24];
COL_METAL  = [0.68, 0.72, 0.76];
COL_BOLT   = [0.25, 0.27, 0.30];

// ---------- Reusable Mechanical Sub-Parts ----------

module bolted_flange(outer_d=96, bore_d=36, h=10, bolt_pcd=78, n_bolts=6, bolt_d=4.3) {
    color(COL_METAL)
    difference() {
        cylinder(h=h, d=outer_d, center=true);
        cylinder(h=h+2, d=bore_d, center=true);
        for (i = [0:n_bolts-1]) {
            a = i * 360 / n_bolts;
            translate([bolt_pcd/2*cos(a), bolt_pcd/2*sin(a), 0])
                cylinder(h=h+2, d=bolt_d, center=true);
        }
    }
    // ISO 4762 M4 socket head cap screws
    color(COL_BOLT)
    for (i = [0:n_bolts-1]) {
        a = i * 360 / n_bolts;
        translate([bolt_pcd/2*cos(a), bolt_pcd/2*sin(a), h/2 + 2])
            cylinder(h=4, d=7.0, center=true);
    }
}

module harmonic_joint_module(outer_d=108, length=92, bore_d=28) {
    // 1. Hollow-shaft aluminum housing
    color(COL_BODY)
    difference() {
        cylinder(h=length, d=outer_d, center=true);
        cylinder(h=length+2, d=bore_d, center=true);
    }
    // 2. Strain-wave harmonic drive ring (dark graphite band)
    color(COL_RING)
    difference() {
        cylinder(h=length*0.24, d=outer_d+3.5, center=true);
        cylinder(h=length*0.24+2, d=bore_d, center=true);
    }
    // 3. Integrated torque-sensor ring (offset toward output side)
    color(COL_METAL)
    translate([0, 0, length*0.36])
    difference() {
        cylinder(h=10, d=outer_d+2.0, center=true);
        cylinder(h=12, d=bore_d, center=true);
    }
}

module hollow_link_tube(len=180, d_root=94, d_tip=82, wall=8) {
    color(COL_BODY)
    difference() {
        cylinder(h=len, d1=d_root, d2=d_tip);
        translate([0, 0, -1])
            cylinder(h=len+2, d1=d_root-2*wall, d2=d_tip-2*wall);
    }
}

module offset_elbow_bridge(dx=82.5, d_end=84, h=52) {
    color(COL_BODY)
    difference() {
        hull() {
            cylinder(h=h, d=d_end, center=true);
            translate([dx, 0, 0]) cylinder(h=h, d=d_end*0.92, center=true);
        }
        cylinder(h=h+2, d=26, center=true);
        translate([dx, 0, 0]) cylinder(h=h+2, d=24, center=true);
    }
}

module franka_hand_gripper(open_mm=32) {
    // Gripper body housing
    color(COL_BODY)
    translate([0, 0, 26])
        cube([42, 136, 52], center=true);
    // Linear guide rail
    color(COL_METAL)
    translate([0, 0, 54])
        cube([16, 120, 8], center=true);
    // Left & right parallel fingers
    for (s = [-1, 1]) {
        color(COL_RING)
        translate([0, s*(open_mm/2 + 8), 80])
            cube([22, 14, 48], center=true);
        // Rubberized fingertip pad
        color(COL_METAL)
        translate([0, s*(open_mm/2 + 2), 96])
            cube([20, 4, 18], center=true);
    }
}

// ---------- Full 7-DOF Articulated Panda Assembly ----------

module franka_panda_assembly() {
    // Base Pedestal + Anchor Flange (8x M6)
    color(COL_RING)
    cylinder(h=18, d=186);
    bolted_flange(outer_d=186, bore_d=44, h=14, bolt_pcd=156, n_bolts=8, bolt_d=6.4);
    translate([0, 0, 18]) hollow_link_tube(len=125, d_root=142, d_tip=116, wall=12);

    // Joint 1 (Base Yaw, Z axis)
    translate([0, 0, 180 + explode*0.5])
    rotate([0, 0, q1]) {
        harmonic_joint_module(outer_d=116, length=104, bore_d=32);
        translate([0, 0, 52]) hollow_link_tube(len=100, d_root=110, d_tip=102, wall=10);

        // Joint 2 (Shoulder Pitch, Y axis)
        translate([0, 0, D1 - 180 + explode])
        rotate([0, q2, 0]) {
            rotate([90, 0, 0]) harmonic_joint_module(outer_d=110, length=116, bore_d=30);
            translate([0, 0, 46]) hollow_link_tube(len=140, d_root=100, d_tip=94, wall=9);

            // Joint 3 (Upper-Arm Roll, Z axis)
            translate([0, 0, 215 + explode])
            rotate([0, 0, q3]) {
                harmonic_joint_module(outer_d=98, length=94, bore_d=28);
                // Panda characteristic +82.5 mm elbow offset
                translate([0, 0, D3 - 215]) offset_elbow_bridge(dx=A4, d_end=90, h=52);

                // Joint 4 (Elbow Pitch, Y axis, offset +A4)
                translate([A4, 0, D3 - 215 + explode])
                rotate([0, q4, 0]) {
                    rotate([90, 0, 0]) harmonic_joint_module(outer_d=94, length=102, bore_d=26);
                    // Return offset -82.5 mm into forearm
                    translate([0, 0, 42]) offset_elbow_bridge(dx=A5, d_end=84, h=46);
                    translate([A5, 0, 62]) hollow_link_tube(len=205, d_root=84, d_tip=76, wall=8);

                    // Joint 5 (Forearm Roll, Z axis)
                    translate([A5, 0, 295 + explode])
                    rotate([0, 0, q5]) {
                        harmonic_joint_module(outer_d=82, length=82, bore_d=22);

                        // Joint 6 (Wrist Pitch, Y axis)
                        translate([0, 0, D5 - 295 + explode])
                        rotate([0, q6, 0]) {
                            rotate([90, 0, 0]) harmonic_joint_module(outer_d=78, length=84, bore_d=20);
                            // Characteristic +88 mm wrist offset
                            translate([0, 0, 32]) offset_elbow_bridge(dx=A7, d_end=72, h=38);

                            // Joint 7 (Flange Roll, Z axis)
                            translate([A7, 0, 62 + explode])
                            rotate([0, 0, q7]) {
                                harmonic_joint_module(outer_d=70, length=64, bore_d=18);
                                translate([0, 0, 38])
                                    bolted_flange(outer_d=68, bore_d=18, h=10, bolt_pcd=50, n_bolts=6, bolt_d=4.3);
                                // Franka Hand Parallel Gripper
                                translate([0, 0, 44 + explode*0.5])
                                    franka_hand_gripper(open_mm=gripper_open_mm);
                            }
                        }
                    }
                }
            }
        }
    }
}

franka_panda_assembly();
