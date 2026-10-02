// Anthropomorphic 3-DOF arm (right arm).
// J1 shoulder flexion (swing forward/back): SG90 in the shoulder mount.
// J2 shoulder abduction (raise sideways) and J3 elbow flexion: 3.3 g micro servos
// (16.7 x 8.2 x 17 mm, 0.3 kg.cm). The arm hangs down at rest, palm facing the body.
// Datum: J1 axis is the Y axis; Y=0 is the front face of the shoulder mount.
// +X forward, +Y outward (away from the body), +Z up.
// Single parts render in print orientation; all print without supports.
part = "assembly"; // "assembly" | "mount" | "cap" | "upper_arm" | "forearm" | "hand"
flex = 0;          // deg, J1: + swings the arm forward (preview only)
abduct = 00;        // deg, J2: + raises the arm sideways (preview only)
elbow = 0;         // deg, J3: + bends the forearm forward (preview only)
animate = false;   // true: joints follow $t (View > Animate in the GUI, or --animate on the CLI)

// Joint angles actually shown: a waving loop when animating, the sliders above otherwise.
cycle = 360 * $t;
flex_shown = animate ? 30 * sin(cycle) : flex;
abduct_shown = animate ? 30 * (1 - cos(cycle)) : abduct;
elbow_shown = animate ? 55 + 45 * sin(3 * cycle) : elbow;

$fn = 48;
eps = 0.01;
// FDM fits. Pockets print undersized and the J2 pocket and the J2/J3 horn pockets print on
// their side, where the bridged roof sags, so these are looser than the nominal sizes suggest.
clearance = 0.4;   // per side, servo body in its pocket (clone servos vary by +/-0.2)
chamfer = 2;       // 45 degree edge chamfer on the organic parts
horn_clearance = 0.3; // per side, horn in its pocket
lead_in = 0.4;     // 45 degree entry chamfer on horn pockets and the wrist socket; the J1 horn
                   // pocket and the wrist socket open onto the bed, where elephant foot narrows them
wire_notch_w = 8.5;   // passes a 3-pin Dupont connector
wire_notch_h = 6;     // wire exit zone above the servo bottom

// --- servo specs (measure yours), indexed by the S_ and H_ constants below ---
S_LEN = 0;        // body length along the tab axis
S_W = 1;          // body width
S_H = 2;          // bottom to top of main body (excluding gear cap)
S_TAB_H = 3;      // body bottom to underside of mounting tabs
S_TAB_T = 4;      // tab thickness
S_TAB_SPAN = 5;   // tab tip to tab tip
S_HOLE_PITCH = 6; // mounting hole centre distance
S_SHAFT_OFF = 7;  // shaft axis to the near end of the body
S_CAP_D = 8;      // gear cap diameter
S_CAP_H = 9;      // gear cap height above body top
S_PILOT_D = 10;   // pilot for the self-tapping tab screws
S_PILOT_DEPTH = 11;
H_ARM_LEN = 12;   // single-arm horn: hub centre to arm tip
H_HUB_D = 13;
H_TIP_D = 14;
H_T = 15;         // horn thickness (preview only)
H_TOP_Z = 16;     // horn top face above servo body top
H_POCKET = 17;    // horn pocket depth
H_SCREW_D = 18;   // clearance hole for the horn screw
H_HEAD_D = 19;    // horn screw head counterbore
H_FLOOR = 20;     // material left under the horn screw head

SG90 = [22.8, 12.2, 22.7, 15.9, 2.5, 32.3, 27.8, 5.9, 11.8, 4.0, 1.8, 7,
        18, 7.0, 4.0, 1.6, 6.5, 1.8, 2.5, 4.8, 1.0];
// 3.3 g micro servo: body size given, tabs/shaft/horn estimated -- check against yours.
MICRO = [16.7, 8.2, 17, 12.5, 1.2, 23.5, 20.5, 4.1, 7.5, 2.0, 1.3, 5,
         12, 5.0, 3.0, 1.2, 4.5, 1.4, 2.0, 3.6, 0.8];
J1 = SG90;
J2 = MICRO;
J3 = MICRO;

wall = 2;           // material around each servo pocket
tab_margin = 1;     // seat face beyond each end of the servo tabs

// --- shoulder mount (screws to a wall or an upright) ---
flange_t = 3;
flange_wing = 8;         // flange beyond each side of the servo housing, along X
flange_hole_d = 3.4;     // M3 clearance
flange_hole_inset = 4;   // hole centre from the wing's outer edge and ends

// --- shoulder cap (J1 output, carries the J2 horn) ---
cap_t = 3;
cap_j1_r = 8.5;          // plate radius around the J1 horn hub
cap_j2_r = 6.5;          // plate radius around the J2 horn hub
cap_horn_margin = 1.2;   // plate beyond the J1 horn tip
shoulder_offset = 11.5;  // cap J1 plate to J2 axis, along Y (room to raise the arm)

// --- upper arm (holds J2 at the top and J3 at the elbow) ---
arm_w = 15;              // thickness along Y, set by the J3 pocket depth
upper_len = 58;          // J2 axis to elbow axis
bicep_disc = [0.5, -29, 7.5]; // [x, z, r] profile circles, J2 axis at origin
elbow_disc = [0.5, -59, 8];
j2_seat_x = 7.5;         // front face where the J2 tabs sit
elbow_x = 0.5;           // elbow axis X in the upper arm frame
boss_r = 3.5;            // elbow pivot boss on the inner face
boss_h = 0.5;
pivot_pilot_d = 1.7;     // M2 self-tapping pivot screw

// --- forearm (yoke around the elbow, J3 horn on the outer cheek) ---
forearm_len = 48;        // elbow axis to wrist face
cheek_t = 2.5;
cheek_gap = 0.4;         // per side, cheek to upper arm
elbow_clear_r = 10;      // free radius around the elbow inside the yoke
elbow_max = 115;         // deg, elbow flexion the yoke is carved to allow
elbow_gap = 1;           // clearance between the upper arm and the carved yoke
elbow_sweep_step = 10;   // deg per step of the carved sweep
forearm_disc = [0.5, -9, 8]; // [x, z, r] forearm muscle
wrist_x = 12;            // wrist block width along X
wrist_y_range = [-1, 10]; // wrist block along Y, reaching past the back of the hand
wrist_h = 5;             // wrist block height along Z
wrist_y = 3;             // hand centre Y in the forearm frame
pivot_hole_d = 2.5;      // M2 clearance; printed on its side, so it comes out undersized

// --- hand (glued onto a keyed peg in the wrist) ---
hand_t = 8;              // palm thickness along Y
peg_size = [5, 5.5, 5];  // X, Y, Z; flush with the back of the hand
peg_clearance = 0.25;    // per side, room for glue
palm_w = 19;
palm_len = 22;
palm_top = 6;            // wrist face to the top of the palm
finger_d = 4.2;
finger_x = [7.2, 2.4, -2.4, -7.2];   // index..little, thumb side is +X
finger_len = [14, 17, 15.5, 12];
thumb_d = 4.8;           // at the root
thumb_tip_d = 3.65;      // tapers evenly to this at the tip
thumb_segments = [[9.5, 30], [8, 12]]; // [length, lean toward +X in deg]

// --- skeleton windows: [[a, b], half_a, half_b] diamonds cut straight through a part ---
// "_x" windows run along X with [a, b] = [y, z]; "_y" windows run along Y with [a, b] = [x, z].
// Every window that lies horizontal when printed is taller (in print) than it is wide,
// so its edges stay steeper than 45 degrees and need no support.
upper_windows_y = [[[0.5, -30], 4, 8]];
upper_windows_x = [[[0, -26], 3.5, 3.2], [[0, -34], 3.5, 3.2]];
forearm_windows_y = [[[0.5, -19], 5, 5.5], [[0.5, -32], 4, 4.5],
                     [[7.5, 1.5], 2, 3], [[-7.5, 1.5], 2, 3]]; // last two: cheeks
forearm_windows_x = [[[0, -19], 3.5, 5.5], [[8.5, -19], 3, 5.5], [[4, -32], 3.5, 4.5]];
mount_windows_x = [[[-8, -5.5], 6, 5]];                        // both housing side walls
mount_windows_y = [[[12.4, -5.5], 1.8, 8], [[-12.4, -5.5], 1.8, 8]]; // flange wings
palm_slot_x = [4.8, 0, -4.8]; // slots between the finger bones
palm_slot_w = 2;
palm_slot_z = [-8.5, -18.5];  // slot ends

// --- servo helpers ---
function center_x(s) = s[S_SHAFT_OFF] - s[S_LEN] / 2; // body centre, shaft at x=0
function tab_bottom_z(s) = s[S_TAB_H] - s[S_H];       // tab underside below body top
function horn_seat(s) = s[H_TOP_Z] - s[H_POCKET];      // servo body top to mating face
// Seat face extent along the servo's length (its local X), tabs plus margin.
function tab_range(s) = [center_x(s) - s[S_TAB_SPAN] / 2 - tab_margin,
                         center_x(s) + s[S_TAB_SPAN] / 2 + tab_margin];

// --- derived ---
j1_body_top_y = -tab_bottom_z(J1);                // J1 tabs sit on the mount face
j1_horn_y = j1_body_top_y + horn_seat(J1);
mount_half_x = J1[S_W] / 2 + clearance + wall;    // J1 housing half width
mount_back_y = j1_body_top_y - J1[S_H] - clearance - wall;
mount_z = tab_range(J1);                          // J1 lies with its length along Z
j1_horn_tip_x = J1[H_ARM_LEN] - J1[H_TIP_D] / 2;  // centre of the J1 horn tip
j2_y = j1_horn_y + cap_t + shoulder_offset;
j2_box = [[j2_seat_x - J2[S_TAB_H] - clearance - wall, -arm_w / 2, tab_range(J2)[0]],
          [j2_seat_x, arm_w / 2, tab_range(J2)[1]]]; // J2 housing at the top of the arm
j2_body_top_x = j2_seat_x - tab_bottom_z(J2);
j2_horn_x = j2_body_top_x + horn_seat(J2);
j3_body_top_y = arm_w / 2 - tab_bottom_z(J3);
j3_horn_y = j3_body_top_y + horn_seat(J3);
j3_floor_y = j3_body_top_y - J3[S_H] - clearance; // bottom of the J3 pocket
inner_cheek_y = -arm_w / 2 - boss_h - cheek_gap;  // inner face of the inner cheek
outer_cheek_y = j3_horn_y;                        // inner face of the outer cheek
forearm_y0 = inner_cheek_y - cheek_t;
forearm_y1 = outer_cheek_y + cheek_t;
peg_y0 = hand_t / 2 - peg_size[1];                // peg spans peg_y0..hand_t/2
finger_center_y = hand_t / 2 - finger_d / 2;      // fingers flush with the back of the hand

for (s = [[J1, cap_t], [J2, cap_t], [J3, cheek_t]])
    assert(s[0][H_POCKET] + s[0][H_FLOOR] < s[1], "plate too thin for horn screw");
for (s = [J1, J2, J3]) assert(horn_seat(s) > s[S_CAP_H], "horn seat hits the gear cap");
assert(norm([elbow_disc[0] - elbow_x, elbow_disc[1] + upper_len]) + elbow_disc[2] + 0.5
       < elbow_clear_r, "yoke rubs the elbow");
assert(j3_floor_y > -arm_w / 2 + 1.5, "J3 pocket breaks through the inner face");
echo(j2_y = j2_y, j2_horn_x = j2_horn_x, forearm_width = forearm_y1 - forearm_y0);
echo(total_length = upper_len + forearm_len + palm_len + max(finger_len));

// Place children in a frame whose local X points along xdir and local Z along zdir.
module frame(origin, xdir, zdir) {
    ydir = cross(zdir, xdir);
    multmatrix([[xdir[0], ydir[0], zdir[0], origin[0]],
                [xdir[1], ydir[1], zdir[1], origin[1]],
                [xdir[2], ydir[2], zdir[2], origin[2]],
                [0, 0, 0, 1]]) children();
}

// Disc in the XZ plane at [x, z, r], spanning y0..y1, with 45 degree chamfered faces.
module chamfer_disc(disc, y0, y1, c = chamfer) {
    hull() {
        translate([disc[0], y0, disc[1]]) rotate([-90, 0, 0])
            cylinder(r = disc[2] - c, h = y1 - y0);
        translate([disc[0], y0 + c, disc[1]]) rotate([-90, 0, 0])
            cylinder(r = disc[2], h = y1 - y0 - 2 * c);
    }
}

// Box from corner p0 to corner p1 with 45 degree chamfered edges.
module chamfer_box(p0, p1, c = chamfer) {
    size = p1 - p0;
    hull() for (i = [0 : 2])
        translate(p0 + [i == 0 ? 0 : c, i == 1 ? 0 : c, i == 2 ? 0 : c])
            cube(size - [i == 0 ? 0 : 2 * c, i == 1 ? 0 : 2 * c, i == 2 ? 0 : 2 * c]);
}

// Diamond window cut straight through along X (see the skeleton windows parameters).
module window_x(w) {
    translate([0, w[0][0], w[0][1]]) rotate([0, 90, 0])
        linear_extrude(height = 200, center = true)
            polygon([[w[2], 0], [0, w[1]], [-w[2], 0], [0, -w[1]]]);
}

// Diamond window cut straight through along Y (see the skeleton windows parameters).
module window_y(w) {
    translate([w[0][0], 0, w[0][1]]) rotate([90, 0, 0])
        linear_extrude(height = 200, center = true)
            polygon([[w[1], 0], [0, w[2]], [-w[1], 0], [0, -w[2]]]);
}

// All windows of one part.
module windows(along_x, along_y) {
    for (w = along_x) window_x(w);
    for (w = along_y) window_y(w);
}

// Single-arm horn outline of servo s, hub on the shaft, arm along +X.
module horn_2d(s, extra) {
    hull() {
        circle(d = s[H_HUB_D] + 2 * extra);
        translate([s[H_ARM_LEN] - s[H_TIP_D] / 2, 0]) circle(d = s[H_TIP_D] + 2 * extra);
    }
}

// Horn pocket, screw hole and counterbore; mating face at z=0, part body in +Z.
module horn_socket(s, part_t) {
    translate([0, 0, -eps]) linear_extrude(s[H_POCKET] + eps) horn_2d(s, horn_clearance);
    hull() {
        translate([0, 0, -eps]) linear_extrude(eps) horn_2d(s, horn_clearance + lead_in);
        translate([0, 0, lead_in]) linear_extrude(eps) horn_2d(s, horn_clearance);
    }
    translate([0, 0, -eps]) cylinder(d = s[H_SCREW_D], h = part_t + 2 * eps);
    translate([0, 0, s[H_POCKET] + s[H_FLOOR]]) cylinder(d = s[H_HEAD_D], h = part_t);
}

// Pocket for servo s in its own frame (shaft on local Z, shaft end of the body toward +X,
// origin at the body top). Clears the body, the tab seat and the tab pilots, plus a wire
// notch at the far end that leaves through the floor ("floor") or along -X ("end").
module servo_mount_cut(s, wire_exit) {
    tab_z = tab_bottom_z(s);
    far_end = center_x(s) - s[S_LEN] / 2 - clearance;
    translate([far_end, -s[S_W] / 2 - clearance, -s[S_H] - clearance])
        cube([s[S_LEN] + 2 * clearance, s[S_W] + 2 * clearance, s[S_H] + clearance + tab_z + eps]);
    translate([center_x(s) - s[S_TAB_SPAN] / 2 - 1, -s[S_W] / 2 - 1, tab_z])
        cube([s[S_TAB_SPAN] + 2, s[S_W] + 2, 60]);
    for (side = [-1, 1])
        translate([center_x(s) + side * s[S_HOLE_PITCH] / 2, 0, tab_z - s[S_PILOT_DEPTH]])
            cylinder(d = s[S_PILOT_D], h = s[S_PILOT_DEPTH] + eps);
    if (wire_exit == "floor")
        translate([far_end - 5, -wire_notch_w / 2, -s[S_H] - 40])
            cube([8, wire_notch_w, 40 + wire_notch_h]);
    else
        translate([far_end - 60, -wire_notch_w / 2, -s[S_H] - clearance])
            cube([60 + eps, wire_notch_w, wire_notch_h + clearance]);
}

// Body of servo s without its horn, in its own frame (see servo_mount_cut).
module servo_body(s) {
    color("RoyalBlue") {
        translate([center_x(s) - s[S_LEN] / 2, -s[S_W] / 2, -s[S_H]])
            cube([s[S_LEN], s[S_W], s[S_H]]);
        translate([center_x(s) - s[S_TAB_SPAN] / 2, -s[S_W] / 2, tab_bottom_z(s)])
            cube([s[S_TAB_SPAN], s[S_W], s[S_TAB_T]]);
        cylinder(d = s[S_CAP_D], h = s[S_CAP_H]);
    }
}

// Servo s with its horn hub, for preview.
module servo(s) {
    servo_body(s);
    color("White") translate([0, 0, s[H_TOP_Z] - s[H_T]]) cylinder(d = s[H_HUB_D], h = s[H_T]);
}

// Servo frames, each in the frame of the part that holds the servo body.
module j1_frame() { frame([0, j1_body_top_y, 0], [0, 0, 1], [0, 1, 0]) children(); }
module j2_frame() { frame([j2_body_top_x, 0, 0], [0, 0, 1], [1, 0, 0]) children(); }
module j3_frame() { frame([elbow_x, j3_body_top_y, -upper_len], [0, 0, -1], [0, 1, 0]) children(); }

// Shoulder mount: housing for J1 plus a flange that screws to a wall or upright.
module mount() {
    difference() {
        union() {
            chamfer_box([-mount_half_x, mount_back_y, mount_z[0]], [mount_half_x, 0, mount_z[1]], 1);
            chamfer_box([-mount_half_x - flange_wing, mount_back_y, mount_z[0]],
                        [mount_half_x + flange_wing, mount_back_y + flange_t, mount_z[1]], 1);
        }
        j1_frame() servo_mount_cut(J1, "end");
        windows(mount_windows_x, mount_windows_y);
        for (x = [-1, 1], z = [mount_z[1] - flange_hole_inset, mount_z[0] + flange_hole_inset])
            translate([x * (mount_half_x + flange_wing - flange_hole_inset), mount_back_y - eps, z])
                rotate([-90, 0, 0]) cylinder(d = flange_hole_d, h = flange_t + 2 * eps);
    }
}

// Shoulder cap: L-plate from the J1 horn (normal Y) to the J2 horn (normal X).
module cap() {
    bar = [[j2_horn_x, j1_horn_y, -cap_j2_r], [j2_horn_x + cap_t, j1_horn_y + cap_t, cap_j2_r]];
    difference() {
        union() {
            hull() {
                translate([0, j1_horn_y, 0]) rotate([-90, 0, 0]) cylinder(r = cap_j1_r, h = cap_t);
                translate([j1_horn_tip_x, j1_horn_y, 0]) rotate([-90, 0, 0])
                    cylinder(r = J1[H_TIP_D] / 2 + horn_clearance + cap_horn_margin, h = cap_t);
                translate(bar[0]) cube(bar[1] - bar[0]);
            }
            hull() {
                translate(bar[0]) cube(bar[1] - bar[0]);
                translate([j2_horn_x, j2_y, 0]) rotate([0, 90, 0]) cylinder(r = cap_j2_r, h = cap_t);
            }
        }
        frame([0, j1_horn_y, 0], [1, 0, 0], [0, 1, 0]) horn_socket(J1, cap_t);
        frame([j2_horn_x, j2_y, 0], [0, -1, 0], [1, 0, 0]) horn_socket(J2, cap_t);
    }
}

// Upper arm: J2 in the shoulder end (tabs on the front), J3 in the elbow (tabs outward).
// Frame: J2 axis at the origin, arm centred on y=0, hanging along -Z.
module upper_arm() {
    pivot_y0 = -arm_w / 2 - boss_h;
    difference() {
        union() {
            hull() {
                chamfer_box(j2_box[0], j2_box[1], 1.5);
                chamfer_disc(bicep_disc, -arm_w / 2, arm_w / 2);
            }
            hull() {
                chamfer_disc(bicep_disc, -arm_w / 2, arm_w / 2);
                chamfer_disc(elbow_disc, -arm_w / 2, arm_w / 2);
            }
            translate([elbow_x, -arm_w / 2 + eps, -upper_len]) rotate([90, 0, 0])
                cylinder(r = boss_r, h = boss_h + eps);
        }
        j2_frame() servo_mount_cut(J2, "floor");
        j3_frame() servo_mount_cut(J3, "floor");
        windows(upper_windows_x, upper_windows_y);
        translate([elbow_x, pivot_y0 - eps, -upper_len]) rotate([-90, 0, 0])
            cylinder(d = pivot_pilot_d, h = j3_floor_y - 0.5 - pivot_y0);
    }
}

// Lower upper arm profile (bicep to elbow) in the forearm frame, grown by elbow_gap,
// spanning the space between the cheeks.
module elbow_profile() {
    translate([-elbow_x, inner_cheek_y, upper_len]) rotate([-90, 0, 0])
        linear_extrude(outer_cheek_y - inner_cheek_y) hull()
            for (d = [bicep_disc, elbow_disc])
                translate([d[0], -d[1]]) circle(r = d[2] + elbow_gap);
}

// Volume the upper arm sweeps through the yoke as the elbow bends from 0 to elbow_max.
// The forearm turns by -elbow about Y, so in its frame the upper arm turns by +elbow.
module elbow_sweep() {
    steps = ceil(elbow_max / elbow_sweep_step);
    for (i = [0 : steps - 1])
        hull() for (a = [i, i + 1]) rotate([0, a * elbow_max / steps, 0]) elbow_profile();
}

// Forearm: yoke around the elbow (horn on the outer cheek, M2 pivot on the inner one),
// tapering to a wrist with a keyed socket for the hand.
// Frame: elbow axis at the origin, y as in the upper arm, hanging along -Z.
module forearm() {
    difference() {
        hull() {
            // 1 mm chamfer: a 2 mm one would leave a feather edge on the thin cheeks
            chamfer_disc([0, 0, elbow_clear_r + cheek_t], forearm_y0, forearm_y1, 1);
            chamfer_disc(forearm_disc, forearm_y0, forearm_y1, 1);
            chamfer_box([-wrist_x / 2, wrist_y_range[0], -forearm_len],
                        [wrist_x / 2, wrist_y_range[1], -forearm_len + wrist_h], 1);
        }
        translate([0, inner_cheek_y, 0]) rotate([-90, 0, 0])
            cylinder(r = elbow_clear_r, h = outer_cheek_y - inner_cheek_y);
        translate([-2 * elbow_clear_r, inner_cheek_y, 0])
            cube([4 * elbow_clear_r, outer_cheek_y - inner_cheek_y, 2 * elbow_clear_r]);
        elbow_sweep();
        windows(forearm_windows_x, forearm_windows_y);
        frame([0, outer_cheek_y, 0], [0, 0, -1], [0, 1, 0]) horn_socket(J3, cheek_t);
        translate([0, forearm_y0 - eps, 0]) rotate([-90, 0, 0])
            cylinder(d = pivot_hole_d, h = cheek_t + 2 * eps);
        translate([-peg_size[0] / 2 - peg_clearance, wrist_y + peg_y0 - peg_clearance,
                   -forearm_len - eps])
            cube([peg_size[0] + 2 * peg_clearance, peg_size[1] + 2 * peg_clearance,
                  peg_size[2] + 1]);
        hull() for (g = [0, lead_in])
            translate([-peg_size[0] / 2 - peg_clearance - lead_in + g,
                       wrist_y + peg_y0 - peg_clearance - lead_in + g, -forearm_len - eps + g])
                cube([peg_size[0] + 2 * (peg_clearance + lead_in - g),
                      peg_size[1] + 2 * (peg_clearance + lead_in - g), eps]);
    }
}

// Octagonal finger segment along -Z from the origin, flat faces toward +/-Y, tapered tip.
module finger_rod(len, d) {
    r = d / 2 / cos(22.5);
    hull() {
        rotate([0, 0, 22.5]) cylinder(r = r, h = eps, $fn = 8);
        translate([0, (d - 0.8 * d) / 2, -len]) rotate([0, 0, 22.5])
            cylinder(r = 0.8 * r, h = eps, $fn = 8);
    }
}

// Thin octagonal finger cross-section centred at [x, z], tilted by lean degrees toward +X,
// its flat face flush with the back of the hand (+Y).
module finger_section(xz, d, lean) {
    translate([xz[0], hand_t / 2 - d / 2, xz[1]]) rotate([0, -lean, 0]) rotate([0, 0, 22.5])
        cylinder(r = d / 2 / cos(22.5), h = eps, center = true, $fn = 8);
}

// Thumb: one tapered rod bent at the knuckle. Both halves share the knuckle section,
// which is tilted halfway between them, so the outline runs on without a notch.
module thumb() {
    a0 = thumb_segments[0][1];
    a1 = thumb_segments[1][1];
    root = [palm_w / 2 - 3, -palm_top];
    knuckle = root + thumb_segments[0][0] * [sin(a0), -cos(a0)];
    tip = knuckle + thumb_segments[1][0] * [sin(a1), -cos(a1)];
    knuckle_d = thumb_d + (thumb_tip_d - thumb_d)
                * thumb_segments[0][0] / (thumb_segments[0][0] + thumb_segments[1][0]);
    hull() {
        finger_section(root, thumb_d, a0);
        finger_section(knuckle, knuckle_d, (a0 + a1) / 2);
    }
    hull() {
        finger_section(knuckle, knuckle_d, (a0 + a1) / 2);
        finger_section(tip, thumb_tip_d, a1);
    }
}

// Hand: palm, four fingers and a thumb, all flush with the back of the hand (+Y).
// Frame: wrist face centre at the origin, hand along -Z, palm facing -Y, thumb toward +X.
module hand() {
    translate([-peg_size[0] / 2, peg_y0, -eps]) cube([peg_size[0], peg_size[1], peg_size[2] + eps]);
    difference() {
        union() {
            hull() {
                chamfer_box([-wrist_x / 2, -hand_t / 2, -palm_top / 2], [wrist_x / 2, hand_t / 2, 0], 1.5);
                chamfer_box([-palm_w / 2, -hand_t / 2, -palm_len], [palm_w / 2, hand_t / 2, -palm_top]);
            }
            thumb(); // inside the difference so its root does not fill the outer palm slot
        }
        for (x = palm_slot_x)
            hull() for (z = palm_slot_z)
                translate([x, -hand_t, z]) rotate([-90, 0, 0])
                    cylinder(d = palm_slot_w, h = 2 * hand_t);
    }
    for (i = [0 : len(finger_x) - 1])
        translate([finger_x[i], finger_center_y, -palm_len + 2]) finger_rod(finger_len[i], finger_d);
}

// Everything the elbow servo moves, in the forearm frame.
module forearm_payload() {
    color("Tan") forearm();
    translate([0, wrist_y, -forearm_len]) color("Wheat") hand();
}

// Everything the J2 servo moves, in the upper arm frame.
module upper_arm_payload() {
    color("Tan") upper_arm();
    j2_frame() servo(J2);
    j3_frame() servo(J3);
    translate([elbow_x, 0, -upper_len]) rotate([0, -elbow_shown, 0]) forearm_payload();
}

module assembly() {
    color("DimGray") mount();
    j1_frame() servo(J1);
    rotate([0, -flex_shown, 0]) {
        color("Peru") cap();
        translate([0, j2_y, 0]) rotate([abduct_shown, 0, 0]) upper_arm_payload();
    }
}

if (part == "assembly") assembly();
else if (part == "mount") translate([0, 0, -mount_back_y]) rotate([90, 0, 0]) mount();
else if (part == "cap") translate([0, 0, -j1_horn_y]) rotate([90, 0, 0]) cap();
else if (part == "upper_arm") translate([0, 0, arm_w / 2]) rotate([-90, 0, 0]) upper_arm();
else if (part == "forearm") translate([0, 0, forearm_len]) forearm();
else if (part == "hand") translate([0, 0, hand_t / 2]) rotate([-90, 0, 0]) hand();
