// Cardboard layer templates for human_arm.scad.
// Every part is sliced along Y (the arm's side-to-side axis) into sheets of card_t,
// each slice taken through the middle of its layer with projection(cut = true).
// Layers are laid out in rows, labelled "<part> <layer>/<count>" from the -Y (body) side.
// Export one sheet at a time, e.g.
//   openscad cardboard.scad -D 'sheet="upper_arm"' -o upper_arm_layers.svg
include <human_arm.scad>

part = "none";     // silence the arm model's own output
sheet = "all";     // "all" | "mount" | "cap" | "upper_arm" | "forearm" | "hand"
card_t = 2;        // card thickness, mm
label_size = 3;
test_square = 40;  // printed-scale check: measure it before cutting

// Windows along X cut straight across a Y-slice and would split a layer in two.
mount_windows_x = [];
upper_windows_x = [];
forearm_windows_x = [];

// [name, y0, y1, pitch_x, pitch_y, label_z, per_row]: Y span to slice, layout pitch,
// label height and layers per row (rows fit an A4 page).
SHEETS = [
    ["mount", mount_back_y, 0, 40, 45, mount_z[0] - 5, 4],
    ["cap", j1_horn_y, j2_y + cap_j2_r, 34, 25, -cap_j1_r - 4, 5],
    ["upper_arm", -arm_w / 2 - boss_h, arm_w / 2, 34, 88, elbow_disc[1] - elbow_disc[2] - 5, 5],
    ["forearm", forearm_y0, forearm_y1, 32, 72, -forearm_len - 5, 5],
    ["hand", -hand_t / 2, hand_t / 2, 34, 52, -palm_len - max(finger_len) - 6, 4],
];

function layer_count(s) = ceil((s[2] - s[1]) / card_t - 0.01);
function row_count(s) = ceil(layer_count(s) / s[6]);

// One part in its own frame (Y is side-to-side in every frame).
module part_geometry(name) {
    if (name == "mount") mount();
    else if (name == "cap") cap();
    else if (name == "upper_arm") upper_arm();
    else if (name == "forearm") forearm();
    else if (name == "hand") hand();
}

// Outline of one layer: the part cut at height y, seen from +Y, with Z up the page.
module layer(name, y) {
    mirror([0, 1]) projection(cut = true) translate([0, 0, -y]) rotate([90, 0, 0])
        part_geometry(name);
}

// All layers of one part, laid out in rows with labels, plus a test square below them.
module layer_sheet(s) {
    count = layer_count(s);
    for (i = [0 : count - 1])
        translate([(i % s[6]) * s[3], -floor(i / s[6]) * s[4]]) {
            // Cut through the layer centre, kept inside the part for a partial last layer and
            // nudged so a cut never lands exactly on a flat face.
            layer(s[0], min(s[1] + (i + 0.5) * card_t, s[2] - 0.5) + 0.037);
            translate([0, s[5]])
                text(str(s[0], " ", i + 1, "/", count), size = label_size, halign = "center");
        }
    translate([-s[3] / 2, -(row_count(s) - 1) * s[4] + s[5] - 8 - test_square]) {
        square(test_square);
        translate([test_square + 4, test_square / 2])
            text(str(test_square, " mm test square"), size = label_size, valign = "center");
    }
    echo(str(s[0], ": ", count, " layers of ", card_t, " mm"));
}

// Vertical offset of sheet k in the "all" layout: the heights of the sheets above it.
function sheet_offset(k) = k == 0 ? 0
    : sheet_offset(k - 1) + row_count(SHEETS[k - 1]) * SHEETS[k - 1][4] + test_square + 20;

for (k = [0 : len(SHEETS) - 1])
    if (sheet == "all" || sheet == SHEETS[k][0])
        translate([0, sheet == "all" ? -sheet_offset(k) : 0]) layer_sheet(SHEETS[k]);
