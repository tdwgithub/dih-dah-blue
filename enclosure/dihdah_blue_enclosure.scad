// ============================================================================
// Radio PCB Enclosure - Parametric OpenSCAD Design
// For 50.8mm x 50.8mm (2" square) radio board with XIAO ESP32-C6
// Copyright 2026 by Todd D. Wade
// ============================================================================

// ----------------------------------------------------------------------------
// PARAMETERS - Adjust these as needed
// ----------------------------------------------------------------------------

// Board dimensions
board_width = 50.8;          // X dimension
board_depth = 50.8;          // Y dimension
board_thickness = 1.6;       // PCB thickness

// Enclosure parameters
wall_thickness = 2.0;        // Wall thickness
lid_thickness = 2.0;         // Lid thickness
board_clearance = 0.3;       // Gap around board edges for fit

// Vertical spacing
standoff_height = 7.0;       // Height from enclosure floor to bottom of PCB
clearance_above = 17.35;     // Clearance from top of PCB to inside of lid (increased by 0.25" / 6.35mm)

// Mounting
mounting_hole_dia = 3.5;     // Mounting hole diameter in PCB
screw_hole_dia = 2.8;        // Hole for #4-40 screw (tap drill ~2.7mm, clearance ~3mm)
boss_outer_dia = 7.0;        // Outer diameter of screw boss

// Mounting hole positions (relative to board origin at bottom-left)
mount_holes = [
    [3.873, 3.937],          // H4 - bottom-left
    [46.927, 3.937],         // H3 - bottom-right
    [3.873, 46.863],         // H1 - top-left
    [46.927, 46.863]         // H2 - top-right
];

// Corner screw boss parameters (for lid attachment)
corner_boss_dia = 7.0;       // Outer diameter of corner boss
corner_screw_dia = 2.2;      // Pilot hole for M2.5 screw (self-tap into PLA)
corner_clearance_dia = 2.7;  // Clearance hole in lid for M2.5
corner_countersink_dia = 5.5; // Countersink diameter for M2.5 head
corner_boss_height = 7.0;    // Height of boss (lid_overlap + extra engagement)

// Lid overlap
lid_overlap = 2.5;           // How far lid walls extend down into base (reduced - screws handle alignment)

// ----------------------------------------------------------------------------
// COMPONENT POSITIONS (relative to board origin, Y=0 at bottom)
// ----------------------------------------------------------------------------

// XIAO ESP32-C6 (U1) - USB-C faces right edge
// USB-C is centered on the module width, so footprint center Y = USB-C center Y after rotation
xiao_pos = [39.815, 17.399];
xiao_rotation = -90;
usb_c_width = 12.0;          // USB-C port opening width
usb_c_height = 10.175;       // USB-C port opening height (increased by 0.125" / 3.175mm)
usb_z_offset = 5.5875;       // Height of USB-C center above PCB top (moved up 1/16" / 1.5875mm)

// Terminal block J1 - plug insertion from left
terminal_pos = [6.857, 33.655];
terminal_rotation = -90;
terminal_cutout_width = 18.0;  // Width of cutout for mating plug
terminal_cutout_height = 13.175; // Height of cutout (increased by 0.125" / 3.175mm)
terminal_z_offset = 5.5875;    // Center height above PCB (moved up 1/16" / 1.5875mm)

// LED D1 - protrudes through right wall
led_footprint_pos = [47.774, 29.210];
led_y_offset = 1.15;         // Adjust to align with actual LED position
led_pos = [led_footprint_pos[0], led_footprint_pos[1] + led_y_offset];
led_dia = 4.79375;           // LED diameter + clearance (matched to sw2_hole_dia)
led_z_height = 4.0;          // LED center height above PCB (matches SW1)

// Button SW1 - right wall access
// Footprint origin is at pad 1, button actuator is offset in local coords
sw1_footprint_pos = [47.815, 40.132];  // Footprint origin in board coords
sw1_actuator_offset_y = -2.00;         // Actuator toward USB-C (front)
sw1_pos = [sw1_footprint_pos[0], sw1_footprint_pos[1] + sw1_actuator_offset_y];
sw1_hole_dia = 4.79375;      // Access hole diameter (matched to sw2_hole_dia)
sw1_z_height = 4.0;          // Button center height above PCB

// Button SW2 - left wall access
// With +90° rotation: local +Y becomes global -X, local +X becomes global +Y
// Actuator offset moved 1/16" (1.5875mm) away from terminal block (toward lower Y)
sw2_footprint_pos = [2.907, 11.029];   // Footprint origin in board coords
sw2_actuator_offset_y = +2.25;         // Adjusted: moved 1/16" (1.5875mm) toward terminal block
sw2_pos = [sw2_footprint_pos[0], sw2_footprint_pos[1] + sw2_actuator_offset_y];
sw2_hole_dia = 4.79375;      // Access hole diameter (increased by 1/32" / 0.79375mm for button plunger clearance)
sw2_z_height = 4.0;          // Button center height above PCB (moved down 1/16" / 1.5875mm)

// ----------------------------------------------------------------------------
// CALCULATED DIMENSIONS
// ----------------------------------------------------------------------------

// Internal cavity dimensions
cavity_width = board_width + board_clearance * 2;
cavity_depth = board_depth + board_clearance * 2;
cavity_height = standoff_height + board_thickness + clearance_above;

// External dimensions (base)
ext_width = cavity_width + wall_thickness * 2;
ext_depth = cavity_depth + wall_thickness * 2;
ext_height_base = cavity_height + wall_thickness;  // Floor thickness = wall_thickness

// Lid dimensions
lid_ext_height = lid_thickness + lid_overlap;

// Board position offset (center of cavity)
board_offset = [wall_thickness + board_clearance, wall_thickness + board_clearance, wall_thickness + standoff_height];

// ----------------------------------------------------------------------------
// MODULES
// ----------------------------------------------------------------------------

// Screw boss for PCB mounting
module screw_boss(height) {
    difference() {
        cylinder(h=height, d=boss_outer_dia, $fn=32);
        translate([0, 0, -0.1])
            cylinder(h=height + 0.2, d=screw_hole_dia, $fn=24);
    }
}

// Corner screw boss for lid attachment (solid - holes drilled separately)
module corner_boss(height) {
    cylinder(h=height, d=corner_boss_dia, $fn=32);
}

// USB-C cutout - simple rectangular opening
module usb_cutout() {
    translate([0, -usb_c_width/2, -usb_c_height/2])
        cube([wall_thickness + 0.2, usb_c_width, usb_c_height]);
}

// Terminal block cutout
module terminal_cutout() {
    translate([0, -terminal_cutout_width/2, -terminal_cutout_height/2])
        cube([wall_thickness + 0.2, terminal_cutout_width, terminal_cutout_height]);
}

// ----------------------------------------------------------------------------
// BASE
// ----------------------------------------------------------------------------

module base() {
    // Corner boss positions (at wall intersections)
    corner_boss_offset = wall_thickness / 2;
    corner_positions = [
        [corner_boss_offset, corner_boss_offset],
        [ext_width - corner_boss_offset, corner_boss_offset],
        [corner_boss_offset, ext_depth - corner_boss_offset],
        [ext_width - corner_boss_offset, ext_depth - corner_boss_offset]
    ];

    difference() {
        union() {
            // Main box
            difference() {
                // Outer shell
                cube([ext_width, ext_depth, ext_height_base]);

                // Inner cavity
                translate([wall_thickness, wall_thickness, wall_thickness])
                    cube([cavity_width, cavity_depth, cavity_height + 0.1]);
            }

            // PCB screw bosses
            for (hole = mount_holes) {
                translate([board_offset[0] + hole[0], board_offset[1] + hole[1], wall_thickness])
                    screw_boss(standoff_height);
            }

            // Corner screw bosses for lid attachment
            for (pos = corner_positions) {
                translate([pos[0], pos[1], ext_height_base - corner_boss_height])
                    corner_boss(corner_boss_height);
            }
        }

        // === CUTOUTS ===

        // USB-C cutout (right wall)
        // USB-C is centered on XIAO module width; after -90° rotation, module width is along Y
        usb_y = board_offset[1] + xiao_pos[1];
        usb_z = board_offset[2] + board_thickness + usb_z_offset;
        translate([ext_width - wall_thickness - 0.1, usb_y, usb_z])
            usb_cutout();

        // Terminal block cutout (left wall)
        term_y = board_offset[1] + terminal_pos[1];
        term_z = board_offset[2] + board_thickness + terminal_z_offset;
        translate([-0.1, term_y, term_z])
            terminal_cutout();

        // LED hole (right wall) - LED protrudes
        led_y = board_offset[1] + led_pos[1];
        led_z = board_offset[2] + board_thickness + led_z_height;
        translate([ext_width - wall_thickness - 0.1, led_y, led_z])
            rotate([0, 90, 0])
                cylinder(h=wall_thickness + 0.2, d=led_dia, $fn=24);

        // SW1 access hole (right wall)
        sw1_y = board_offset[1] + sw1_pos[1];
        sw1_z = board_offset[2] + board_thickness + sw1_z_height;
        translate([ext_width - wall_thickness - 0.1, sw1_y, sw1_z])
            rotate([0, 90, 0])
                cylinder(h=wall_thickness + 0.2, d=sw1_hole_dia, $fn=24);

        // SW2 access hole (left wall)
        sw2_y = board_offset[1] + sw2_pos[1];
        sw2_z = board_offset[2] + board_thickness + sw2_z_height;
        translate([-0.1, sw2_y, sw2_z])
            rotate([0, 90, 0])
                cylinder(h=wall_thickness + 0.2, d=sw2_hole_dia, $fn=24);

        // Corner screw holes for lid attachment
        for (pos = corner_positions) {
            translate([pos[0], pos[1], ext_height_base - corner_boss_height - 0.1])
                cylinder(h=corner_boss_height + 0.2, d=corner_screw_dia, $fn=24);
        }

        // Corner relief cuts - square off inner corners so PCB and lid can fit
        // The corner bosses (7mm dia at corner) intrude ~2.5mm into the cavity
        // We need to cut this intrusion flat, starting above the PCB mounting bosses
        relief_size = corner_boss_dia/2 + 1;  // 4.5mm - covers the 2.5mm intrusion + clearance
        relief_z_start = wall_thickness + standoff_height + 0.5;  // Just above PCB mounting bosses
        relief_height = ext_height_base - relief_z_start + 0.1;

        // Cut at each inner corner of cavity
        // Bottom-left corner
        translate([wall_thickness - 0.1, wall_thickness - 0.1, relief_z_start])
            cube([relief_size + 0.1, relief_size + 0.1, relief_height]);
        // Bottom-right corner
        translate([ext_width - wall_thickness - relief_size, wall_thickness - 0.1, relief_z_start])
            cube([relief_size + 0.1, relief_size + 0.1, relief_height]);
        // Top-left corner
        translate([wall_thickness - 0.1, ext_depth - wall_thickness - relief_size, relief_z_start])
            cube([relief_size + 0.1, relief_size + 0.1, relief_height]);
        // Top-right corner
        translate([ext_width - wall_thickness - relief_size, ext_depth - wall_thickness - relief_size, relief_z_start])
            cube([relief_size + 0.1, relief_size + 0.1, relief_height]);
    }
}

// ----------------------------------------------------------------------------
// LID
// ----------------------------------------------------------------------------

module lid() {
    lid_inner_width = cavity_width - 0.3;  // Slight clearance for fit
    lid_inner_depth = cavity_depth - 0.3;

    // Corner positions (matching base)
    corner_boss_offset = wall_thickness / 2;
    corner_positions = [
        [corner_boss_offset, corner_boss_offset],
        [ext_width - corner_boss_offset, corner_boss_offset],
        [corner_boss_offset, ext_depth - corner_boss_offset],
        [ext_width - corner_boss_offset, ext_depth - corner_boss_offset]
    ];

    difference() {
        union() {
            // Top plate
            cube([ext_width, ext_depth, lid_thickness]);

            // Corner extensions (matching base corner bosses)
            for (pos = corner_positions) {
                translate([pos[0], pos[1], 0])
                    cylinder(h=lid_thickness, d=corner_boss_dia, $fn=32);
            }

            // Inner walls that fit inside base
            translate([wall_thickness + 0.15, wall_thickness + 0.15, -lid_overlap + lid_thickness])
                difference() {
                    cube([lid_inner_width, lid_inner_depth, lid_overlap]);
                    // Hollow out
                    translate([wall_thickness, wall_thickness, -0.1])
                        cube([lid_inner_width - wall_thickness*2, lid_inner_depth - wall_thickness*2, lid_overlap + 0.2]);
                }
        }

        // Countersunk screw holes at corners
        for (pos = corner_positions) {
            // Through hole
            translate([pos[0], pos[1], -0.1])
                cylinder(h=lid_thickness + 0.2, d=corner_clearance_dia, $fn=24);
            // Countersink
            translate([pos[0], pos[1], lid_thickness - 1.8])
                cylinder(h=2.0, d1=corner_clearance_dia, d2=corner_countersink_dia, $fn=24);
        }

        // Corner notches in inner walls to clear base corner bosses
        // Only cut the inner wall portion below the top plate
        for (pos = corner_positions) {
            translate([pos[0], pos[1], -lid_overlap + lid_thickness - 0.1])
                cylinder(h=lid_overlap - lid_thickness + 0.2, d=corner_boss_dia + 0.6, $fn=32);
        }
    }
}

// ----------------------------------------------------------------------------
// PCB MOCKUP (for visualization)
// ----------------------------------------------------------------------------

module pcb_mockup() {
    color("green", 0.5)
    translate(board_offset)
        cube([board_width, board_depth, board_thickness]);

    // Mounting holes
    color("gold", 0.5)
    for (hole = mount_holes) {
        translate([board_offset[0] + hole[0], board_offset[1] + hole[1], board_offset[2] - 0.1])
            cylinder(h=board_thickness + 0.2, d=mounting_hole_dia, $fn=24);
    }
}

// ----------------------------------------------------------------------------
// BATTERY MOCKUP (for visualization)
// ----------------------------------------------------------------------------

module battery_mockup() {
    // Small battery: 29mm x 36mm x 4.75mm
    bat_width = 29;
    bat_depth = 36;
    bat_height = 4.75;

    color("blue", 0.3)
    translate([
        board_offset[0] + (board_width - bat_width) / 2,
        board_offset[1] + (board_depth - bat_depth) / 2,
        wall_thickness + 1  // 1mm clearance from floor
    ])
        cube([bat_width, bat_depth, bat_height]);
}

// ----------------------------------------------------------------------------
// RENDER OPTIONS
// ----------------------------------------------------------------------------

// Uncomment the section you want to render:

// === OPTION 1: Show assembled view ===
// base();
// translate([0, 0, ext_height_base]) lid();
// pcb_mockup();
// battery_mockup();

// === OPTION 2: Show exploded view ===
//base();
//translate([0, 0, ext_height_base + 15]) lid();
//pcb_mockup();
//battery_mockup();

// === OPTION 3: Base only (for STL export) ===
// base();

// === OPTION 4: Lid only (for STL export) ===
// translate([0, 0, lid_thickness]) rotate([180, 0, 0]) lid();

// === OPTION 5: Print layout (both parts) ===
base();
translate([ext_width + 10, 0, lid_thickness]) rotate([180, 0, 0]) lid();

// === OPTION 6: Right wall 2D projection (for paper print - USB, LED, SW1) ===
// Export as SVG, print at 100% scale to verify cutout positions
// projection(cut=true)
//     translate([0, 0, ext_width - wall_thickness/2])
//         rotate([0, 90, 0])
//             base();
// translate([-15, -5, 0]) square([10, 10]);  // 10mm reference square (outside main geometry)

// === OPTION 7: Left wall 2D projection (for paper print - terminal block, SW2) ===
// Export as SVG, print at 100% scale to verify cutout positions
// projection(cut=true)
//     translate([0, 0, -wall_thickness/2])
//         rotate([0, -90, 0])
//             base();
// translate([-15, -5, 0]) square([10, 10]);  // 10mm reference square (outside main geometry)
