#!/usr/bin/env python3
"""
Office Reconstruction Generator for Gazebo Harmonic & Nav2
Produces:
  1. models/office_environment/model.sdf
  2. models/office_environment/model.config
  3. worlds/robot_world.sdf
  4. maps/office_map.pgm & maps/office_map.yaml
"""

import os
import math
import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
MODELS_DIR = os.path.join(PROJECT_DIR, "models", "office_environment")
WORLDS_DIR = os.path.join(PROJECT_DIR, "worlds")
MAPS_DIR = os.path.join(PROJECT_DIR, "maps")

os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(WORLDS_DIR, exist_ok=True)
os.makedirs(MAPS_DIR, exist_ok=True)

# -------------------------------------------------------------
# Color Definitions (RGBA)
# -------------------------------------------------------------
C_OUTER_WALL  = "0.85 0.85 0.88 1.0"
C_INNER_WALL  = "0.88 0.88 0.85 1.0"
C_GLASS       = "0.40 0.70 0.90 0.45"
C_FLOOR_LOBBY = "0.88 0.85 0.80 1.0"  # Polished marble
C_FLOOR_CORR  = "0.84 0.82 0.78 1.0"
C_FLOOR_CARP  = "0.45 0.45 0.47 1.0"  # Charcoal carpet
C_FLOOR_WOOD  = "0.68 0.52 0.38 1.0"  # Hardwood / parquet
C_MARBLE_DESK = "0.94 0.94 0.95 1.0"
C_OAK_WOOD    = "0.74 0.58 0.42 1.0"
C_BIRCH_WOOD  = "0.82 0.72 0.56 1.0"
C_WALNUT_WOOD = "0.42 0.28 0.18 1.0"
C_CHAIR_MESH  = "0.22 0.22 0.24 1.0"
C_CHAIR_FRAME = "0.78 0.78 0.80 1.0"
C_TV_FRAME    = "0.08 0.08 0.08 1.0"
C_TV_SCREEN   = "0.15 0.35 0.65 1.0"  # Display graphic glow
C_HOSPITAL_BD = "0.92 0.92 0.90 1.0"
C_HOSP_LINEN  = "0.38 0.58 0.78 1.0"
C_SOFA_FABRIC = "0.92 0.90 0.85 1.0"
C_METAL_TRIM  = "0.75 0.75 0.76 1.0"
C_PLANT_GREEN = "0.18 0.58 0.22 1.0"
C_POT_WHITE   = "0.92 0.92 0.92 1.0"

class SDFBuilder:
    def __init__(self):
        self.elements = []
        self.wall_boxes = []  # For map generation: (cx, cy, sx, sy)
        self.furniture_boxes = []  # (cx, cy, sx, sy)

    def add_box(self, name, cx, cy, cz, sx, sy, sz, color, collision=True, opacity=1.0, is_wall=False, is_furn=False):
        """Adds a standard box geometry link."""
        col_str = ""
        if collision:
            col_str = f"""
      <collision name="{name}_col">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
      </collision>"""
            if is_wall:
                self.wall_boxes.append((cx, cy, sx, sy))
            elif is_furn:
                self.furniture_boxes.append((cx, cy, sx, sy))

        vis_str = f"""
      <visual name="{name}_vis">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
        </material>
      </visual>"""

        link_str = f"""
    <link name="{name}">
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>{col_str}{vis_str}
    </link>"""
        self.elements.append(link_str)

    def add_cylinder(self, name, cx, cy, cz, radius, length, color, collision=True, is_furn=False):
        col_str = ""
        if collision:
            col_str = f"""
      <collision name="{name}_col">
        <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
      </collision>"""
            self.furniture_boxes.append((cx, cy, radius*2, radius*2))

        vis_str = f"""
      <visual name="{name}_vis">
        <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>"""

        link_str = f"""
    <link name="{name}">
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>{col_str}{vis_str}
    </link>"""
        self.elements.append(link_str)

    def to_sdf(self, model_name="office_environment"):
        body = "\n".join(self.elements)
        return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{model_name}">
    <static>true</static>
{body}
  </model>
</sdf>
"""

builder = SDFBuilder()
H_WALL = 2.60  # Ceiling height 2.6m
Z_WALL = H_WALL / 2.0  # 1.30m
T_WALL = 0.18  # Wall thickness

print("Building Office Geometry...")

# -------------------------------------------------------------
# 1. Floors
# -------------------------------------------------------------
# Main Lobby & Corridor Floor
builder.add_box("floor_lobby", 0.0, 2.5, -0.01, 8.0, 10.0, 0.02, C_FLOOR_LOBBY, collision=False)
builder.add_box("floor_corridor_west", -25.0, 7.25, -0.01, 50.0, 3.5, 0.02, C_FLOOR_CORR, collision=False)
builder.add_box("floor_corridor_east", 16.0, 7.25, -0.01, 30.0, 3.5, 0.02, C_FLOOR_CORR, collision=False)

# Board Room Carpet
builder.add_box("floor_board_room", -26.68, 10.88, -0.01, 9.4, 4.8, 0.02, C_FLOOR_CARP, collision=False)

# Accounts Workstation Carpet
builder.add_box("floor_accounts_ws", -48.25, 0.0, -0.01, 5.0, 6.5, 0.02, C_FLOOR_CARP, collision=False)

# Experience Centres Floors (Wood / Parquet)
builder.add_box("floor_living_demo", 1.10, 10.88, -0.01, 7.8, 4.8, 0.02, C_FLOOR_WOOD, collision=False)
builder.add_box("floor_hotel_exp", -8.41, 10.88, -0.01, 7.8, 4.8, 0.02, C_FLOOR_WOOD, collision=False)
builder.add_box("floor_hospital_exp", -8.04, 3.29, -0.01, 4.4, 7.4, 0.02, C_FLOOR_LOBBY, collision=False)

# Project Workstations Floor
builder.add_box("floor_project_ws", -38.74, 0.0, -0.01, 10.5, 12.5, 0.02, C_FLOOR_CARP, collision=False)

# Customer Success Floor
builder.add_box("floor_customer_success", 15.72, -0.73, -0.01, 8.0, 7.5, 0.02, C_FLOOR_CARP, collision=False)

# -------------------------------------------------------------
# 2. Exterior Perimeter Walls
# -------------------------------------------------------------
# North exterior wall: Y = 13.25, spans X from -52.0 to +30.0 (length 82.0m)
builder.add_box("wall_north_exterior", -11.0, 13.25, Z_WALL, 82.0, T_WALL, H_WALL, C_OUTER_WALL, is_wall=True)

# South exterior wall: Y = -14.50, spans X from -52.0 to +30.0
builder.add_box("wall_south_exterior", -11.0, -14.50, Z_WALL, 82.0, T_WALL, H_WALL, C_OUTER_WALL, is_wall=True)

# West exterior wall: X = -52.0, spans Y from -14.5 to +13.25 (length 27.75m)
builder.add_box("wall_west_exterior", -52.0, -0.62, Z_WALL, T_WALL, 27.75, H_WALL, C_OUTER_WALL, is_wall=True)

# East exterior wall: X = +30.0, spans Y from -14.5 to +13.25
builder.add_box("wall_east_exterior", 30.0, -0.62, Z_WALL, T_WALL, 27.75, H_WALL, C_OUTER_WALL, is_wall=True)

# -------------------------------------------------------------
# 3. Main East-West Corridor Walls & Doorways
# -------------------------------------------------------------
# The main circulation corridor runs along Y in [6.0, 8.5] (corridor width 2.5m).

# North Corridor Wall (separates North cabins/Board Room/Exp Centres from Corridor)
# Runs along Y = 8.50 with openings for doors:
# Doorway to Board Room: at X in [-24.5, -21.5] (opening 3.0m — widened for Nav2 clearance)
# Doorway to Hotel Exp: at X in [-11.7, -8.7] (opening 3.0m — widened for Nav2 clearance)
# Opening to Living Demo Room: at X in [-3.5, -1.5] (opening 2.0m)
# Doorway to Rahul Cabin: at X in [-36.6, -35.0] (opening 1.6m)
# Doorway to Muskan Cabin: at X in [21.0, 22.2] (opening 1.2m)

# Segment 1 (Far West to Rahul Cabin door): X from -52.0 to -36.6 (length 15.4m)
builder.add_box("wall_corr_n_1", -44.30, 8.50, Z_WALL, 15.4, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment 2 (Rahul Cabin to Board Room door): X from -35.0 to -24.5 (length 10.5m)
builder.add_box("wall_corr_n_2", -29.75, 8.50, Z_WALL, 10.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment 3 (Board Room door to Hotel Exp door): X from -21.5 to -11.7 (length 9.8m)
builder.add_box("wall_corr_n_3", -16.60, 8.50, Z_WALL, 9.8, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment 4 (Hotel Exp door to Living Demo opening): X from -8.7 to -3.5 (length 5.2m)
builder.add_box("wall_corr_n_4", -6.10, 8.50, Z_WALL, 5.2, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment 5 (Living Demo opening to Muskan Cabin door): X from -1.5 to 21.0 (length 22.5m)
builder.add_box("wall_corr_n_5", 9.75, 8.50, Z_WALL, 22.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment 6 (Muskan Cabin door to East wall): X from 22.2 to 30.0 (length 7.8m)
builder.add_box("wall_corr_n_6", 26.10, 8.50, Z_WALL, 7.8, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# South Corridor Wall (separates Corridor from Reception, Hospital Room, Workstations)
# Runs along Y = 6.00 with openings:
# Reception entrance: X in [-2.86, +2.86] (wide 5.72m open circulation between Corridor and Reception)
# Hospital Room door: X in [-9.0, -7.2] (opening 1.8m)
# Project / Accounts corridor access: X in [-35.0, -32.0] (wide 3.0m passageway)
# Customer Success hallway access: X in [6.5, 8.5] (opening 2.0m)

# Segment A: X from -52.0 to -35.0 (length 17.0m)
builder.add_box("wall_corr_s_1", -43.50, 6.00, Z_WALL, 17.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment B: X from -32.0 to -9.5 (length 22.5m) — Hospital door at X in [-9.5, -6.5] (3.0m opening)
builder.add_box("wall_corr_s_2", -20.75, 6.00, Z_WALL, 22.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment C: X from -6.5 to -2.86 (length 3.64m) — Hospital door ends at X=-6.5
builder.add_box("wall_corr_s_3", -4.68, 6.00, Z_WALL, 3.64, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Note: X in [-2.86, +2.86] is fully OPEN connecting Lobby & Corridor
# Segment D: X from +2.86 to +6.5 (length 3.64m)
builder.add_box("wall_corr_s_4", 4.68, 6.00, Z_WALL, 3.64, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Segment E: X from +8.5 to +30.0 (length 21.5m)
builder.add_box("wall_corr_s_5", 19.25, 6.00, Z_WALL, 21.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# -------------------------------------------------------------
# 4. Lobby & Reception Boundary Walls
# -------------------------------------------------------------
# Reception Area: X in [-2.86, +2.86], Y in [-1.38, +6.00]
# West Wall of Reception: X = -2.86, Y from -1.38 to 6.0
builder.add_box("wall_reception_west", -2.86, 2.31, Z_WALL, T_WALL, 7.38, H_WALL, C_INNER_WALL, is_wall=True)
# East Wall of Reception: X = +2.86, Y from -1.38 to 6.0
builder.add_box("wall_reception_east", 2.86, 2.31, Z_WALL, T_WALL, 7.38, H_WALL, C_INNER_WALL, is_wall=True)
# South Wall (Main Entrance facade - enclosed perimeter): Y = -1.38
builder.add_box("wall_entrance_facade", 0.0, -1.38, Z_WALL, 5.72, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Glass panels for sliding entrance doors (visual aesthetic)
builder.add_box("glass_door_l", -0.6, -1.36, 1.1, 1.2, 0.02, 2.2, C_GLASS, collision=False)
builder.add_box("glass_door_r",  0.6, -1.36, 1.1, 1.2, 0.02, 2.2, C_GLASS, collision=False)

# -------------------------------------------------------------
# 5. Zone 1: Detailed 3D Models in Lobby / Reception
# -------------------------------------------------------------
print("Adding detailed 3D assets to Lobby / Reception...")
# Marble Reception Desk: at X=0, Y=1.2, size 2.0m x 0.80m x 1.05m (X in [-1.0, +1.0])
builder.add_box("reception_desk_main", 0.0, 1.2, 0.525, 2.0, 0.80, 1.05, C_MARBLE_DESK, is_furn=True)
builder.add_box("reception_desk_plinth", 0.0, 1.2, 0.05, 1.9, 0.75, 0.10, "0.15 0.15 0.15 1", collision=False)
builder.add_box("reception_desk_top", 0.0, 1.2, 1.06, 2.05, 0.85, 0.04, "0.96 0.96 0.96 1", collision=False)

# Receptionist Task Chair behind desk: at X=0, Y=1.85
builder.add_box("reception_chair_seat", 0.0, 1.85, 0.45, 0.45, 0.45, 0.08, C_CHAIR_MESH, is_furn=True)
builder.add_box("reception_chair_back", 0.0, 2.05, 0.85, 0.45, 0.06, 0.72, C_CHAIR_MESH, collision=False)
builder.add_cylinder("reception_chair_base", 0.0, 1.85, 0.22, 0.25, 0.44, C_CHAIR_FRAME, collision=False)

# Security Turnstiles (Access Gates flanking desk):
# Left Turnstile Gate ("IN"): Pedestal on wall at X = -2.75, Y = 1.2. Clear passage: X in [-2.65, -1.00] (1.65m wide)
builder.add_box("turnstile_in_pedestal", -2.75, 1.2, 0.5, 0.18, 0.8, 1.0, C_METAL_TRIM, is_furn=True)
builder.add_box("turnstile_in_gate", -2.65, 1.55, 0.65, 0.04, 0.70, 0.7, C_GLASS, collision=False)

# Right Turnstile Gate ("OUT"): Pedestal on wall at X = +2.75, Y = 1.2. Clear passage: X in [1.00, 2.65] (1.65m wide)
builder.add_box("turnstile_out_pedestal", 2.75, 1.2, 0.5, 0.18, 0.8, 1.0, C_METAL_TRIM, is_furn=True)
builder.add_box("turnstile_out_gate", 2.65, 1.55, 0.65, 0.04, 0.70, 0.7, C_GLASS, collision=False)

# Elevator Demo Feature & Kiosk: Island partition at X=0, Y=3.6, size 1.6m x 0.2m (X in [-0.8, +0.8])
# Both sides have 2.06m wide clear walkways
builder.add_box("elevator_wall", 0.0, 3.6, Z_WALL, 1.6, 0.20, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("elevator_door_l", -0.35, 3.71, 1.1, 0.65, 0.02, 2.2, C_METAL_TRIM, collision=False)
builder.add_box("elevator_door_r",  0.35, 3.71, 1.1, 0.65, 0.02, 2.2, C_METAL_TRIM, collision=False)
builder.add_box("elevator_kiosk_pedestal", 0.75, 3.6, 0.5, 0.18, 0.18, 1.0, "0.1 0.1 0.1 1", is_furn=True)
builder.add_box("elevator_kiosk_screen", 0.75, 3.6, 1.1, 0.22, 0.05, 0.22, C_TV_SCREEN, collision=False)

# Waiting Lounge: Coffee table + 2 Armchairs behind elevator island at X = 0.0, Y = 4.8
builder.add_cylinder("lobby_coffee_table", 0.0, 4.8, 0.22, 0.45, 0.44, C_WALNUT_WOOD, is_furn=True)
builder.add_box("lobby_armchair_1", -0.8, 4.8, 0.42, 0.60, 0.60, 0.65, C_SOFA_FABRIC, is_furn=True)
builder.add_box("lobby_armchair_2",  0.8, 4.8, 0.42, 0.60, 0.60, 0.65, C_SOFA_FABRIC, is_furn=True)

# Potted Planters at entrance corners:
builder.add_cylinder("pot_lobby_l", -2.5, -0.9, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_lobby_l", -2.5, -0.9, 0.75, 0.3, 0.5, C_PLANT_GREEN, collision=False)
builder.add_cylinder("pot_lobby_r",  2.5, -0.9, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_lobby_r",  2.5, -0.9, 0.75, 0.3, 0.5, C_PLANT_GREEN, collision=False)

# -------------------------------------------------------------
# 6. Zone 2: Detailed 3D Models in Board Room
# -------------------------------------------------------------
print("Adding detailed 3D assets to Board Room...")
# Board Room bounds: X in [-31.28, -22.08], Y in [8.50, 13.25]
# Dividing Walls:
# West wall (shared with Rahul Washroom/Cabin): X = -31.28, Y in [8.50, 13.25]
builder.add_box("wall_board_west", -31.28, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# East wall (shared with Hotel Room Phone Booth): X = -22.08, Y in [8.50, 13.25]
builder.add_box("wall_board_east", -22.08, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)

# Boardroom Conference Table: centered at X = -26.68, Y = 10.88, size 5.8m x 1.4m x 0.76m
builder.add_box("board_table_top", -26.68, 10.88, 0.74, 5.8, 1.4, 0.05, C_OAK_WOOD, is_furn=True)
builder.add_box("board_table_base_l", -28.2, 10.88, 0.35, 0.4, 0.9, 0.70, "0.15 0.15 0.15 1", collision=False)
builder.add_box("board_table_base_r", -25.1, 10.88, 0.35, 0.4, 0.9, 0.70, "0.15 0.15 0.15 1", collision=False)
# Table central cable access box
builder.add_box("board_cable_box", -26.68, 10.88, 0.765, 0.8, 0.25, 0.01, C_METAL_TRIM, collision=False)

# 12 Side Chairs (6 north, 6 south) + 2 Head Chairs (1 west, 1 east)
for idx, x_chair in enumerate(np.linspace(-29.0, -24.4, 6)):
    # North side chairs (facing south)
    builder.add_box(f"board_chair_n_{idx}", x_chair, 11.85, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"board_chair_n_back_{idx}", x_chair, 12.05, 0.82, 0.46, 0.06, 0.65, C_CHAIR_MESH, collision=False)
    # South side chairs (facing north)
    builder.add_box(f"board_chair_s_{idx}", x_chair, 9.91, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"board_chair_s_back_{idx}", x_chair, 9.71, 0.82, 0.46, 0.06, 0.65, C_CHAIR_MESH, collision=False)

# Head chairs
builder.add_box("board_chair_head_w", -29.85, 10.88, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
builder.add_box("board_chair_head_e", -23.50, 10.88, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)

# Presentation Wall (West Wall, X = -31.15):
# 85" Screen with presentation display
builder.add_box("board_screen_frame", -31.15, 10.88, 1.65, 0.04, 2.0, 1.15, C_TV_FRAME, is_furn=True)
builder.add_box("board_screen_display", -31.12, 10.88, 1.65, 0.02, 1.9, 1.05, C_TV_SCREEN, collision=False)
# AV Credenza cabinet below screen
builder.add_box("board_credenza", -31.0, 10.88, 0.35, 0.45, 2.4, 0.70, C_WALNUT_WOOD, is_furn=True)

# Decorative Potted Ficus in Board Room corner:
builder.add_cylinder("pot_board", -30.5, 12.6, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_board", -30.5, 12.6, 0.8, 0.35, 0.6, C_PLANT_GREEN, collision=False)

# -------------------------------------------------------------
# 7. Zone 3: Detailed 3D Models in Experience Centres
# -------------------------------------------------------------
print("Adding detailed 3D assets to Experience Centres...")
# A. Hotel Room Experience: X in [-12.17, -4.65], Y in [8.50, 13.25]
# Dividing Wall with Living Demo Room: X = -4.65, Y in [8.50, 13.25]
builder.add_box("wall_hotel_living_div", -4.65, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)

# Hotel King Bed: centered at X = -10.2, Y = 11.6, size 2.1m x 2.0m
builder.add_box("hotel_bed_mattress", -10.2, 11.6, 0.35, 2.0, 2.0, 0.50, "0.95 0.95 0.95 1", is_furn=True)
builder.add_box("hotel_bed_headboard", -11.22, 11.6, 0.75, 0.12, 2.2, 1.10, C_WALNUT_WOOD, collision=False)
builder.add_box("hotel_bed_runner", -9.5, 11.6, 0.61, 0.5, 2.02, 0.02, "0.4 0.3 0.2 1", collision=False)
# Bedside nightstands + lamps
builder.add_box("hotel_nightstand_l", -10.2, 12.8, 0.25, 0.5, 0.5, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_cylinder("hotel_lamp_l", -10.2, 12.8, 0.65, 0.12, 0.30, "0.9 0.85 0.7 1", collision=False)
builder.add_box("hotel_nightstand_r", -10.2, 10.4, 0.25, 0.5, 0.5, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_cylinder("hotel_lamp_r", -10.2, 10.4, 0.65, 0.12, 0.30, "0.9 0.85 0.7 1", collision=False)

# Hotel Automated Bar Unit & Server Rack: at X = -5.2, Y = 12.5
builder.add_box("hotel_bar_cabinet", -5.2, 12.5, 0.65, 0.6, 1.2, 1.30, C_WALNUT_WOOD, is_furn=True)
builder.add_box("hotel_bar_glass", -5.2, 12.5, 1.5, 0.55, 1.15, 0.40, C_GLASS, collision=False)

# Hotel Room TV & Partition:
builder.add_box("hotel_tv_frame", -8.2, 8.65, 1.5, 1.4, 0.05, 0.85, C_TV_FRAME, is_furn=True)
builder.add_box("hotel_tv_screen", -8.2, 8.68, 1.5, 1.3, 0.02, 0.75, C_TV_SCREEN, collision=False)

# Round Lounge Table + 2 Chairs: at X = -6.8, Y = 10.5
builder.add_cylinder("hotel_lounge_table", -6.8, 10.5, 0.35, 0.45, 0.70, C_OAK_WOOD, is_furn=True)
builder.add_box("hotel_lounge_chair_1", -6.8, 11.2, 0.42, 0.55, 0.55, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_box("hotel_lounge_chair_2", -6.8, 9.8, 0.42, 0.55, 0.55, 0.70, C_SOFA_FABRIC, is_furn=True)

# B. Living Demo Room: X in [-4.65, +4.91], Y in [8.50, 13.25]
# East Wall (separates Living Demo from Muskan Meeting): X = 4.91, Y in [8.50, 13.25]
builder.add_box("wall_living_east", 4.91, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)

# Curved Sectional Sofa (modeled as 3 segmented blocks forming gentle arc):
builder.add_box("sofa_mid", 2.0, 12.0, 0.42, 2.2, 0.9, 0.75, C_SOFA_FABRIC, is_furn=True)
builder.add_box("sofa_left", 0.7, 11.7, 0.42, 0.9, 1.2, 0.75, C_SOFA_FABRIC, is_furn=True)
builder.add_box("sofa_right", 3.3, 11.7, 0.42, 0.9, 1.2, 0.75, C_SOFA_FABRIC, is_furn=True)

# Round Coffee Table Cluster:
builder.add_cylinder("living_coffee_table_1", 2.0, 10.6, 0.22, 0.50, 0.44, "0.15 0.15 0.15 1", is_furn=True)
builder.add_cylinder("living_coffee_table_2", 2.7, 10.3, 0.18, 0.35, 0.36, C_OAK_WOOD, is_furn=True)

# TV Wall Unit & Floating Media Console with Fireplace Effect (along East half of south wall):
builder.add_box("living_tv_wall_panel", 2.0, 8.65, 1.5, 2.8, 0.08, 2.0, C_OAK_WOOD, is_furn=True)
builder.add_box("living_tv_screen", 2.0, 8.70, 1.6, 1.8, 0.04, 1.05, C_TV_SCREEN, collision=False)
builder.add_box("living_credenza", 2.0, 8.85, 0.35, 3.0, 0.45, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_box("living_fireplace_strip", 2.0, 8.90, 0.35, 1.4, 0.02, 0.15, "0.95 0.55 0.15 1", collision=False)

# Demonstration Dining/Meeting Table (against East wall):
builder.add_box("living_dining_table", 4.1, 10.0, 0.38, 1.2, 1.8, 0.76, C_OAK_WOOD, is_furn=True)
builder.add_box("living_dining_chair_1", 4.1, 11.2, 0.42, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("living_dining_chair_2", 4.1, 8.8, 0.42, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)

# C. Hospital Room Experience: X in [-10.12, -5.96], Y in [-0.32, +6.00]
# Dividing Wall with Storage Room to West: X = -10.12, Y in [-0.32, 6.00]
builder.add_box("wall_hospital_west", -10.12, 2.84, Z_WALL, T_WALL, 6.32, H_WALL, C_INNER_WALL, is_wall=True)
# South Wall of Hospital Room: Y = -0.32, X in [-10.12, -2.86]
builder.add_box("wall_hospital_south", -6.49, -0.32, Z_WALL, 7.26, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# Modern Hospital Bed: centered at X = -8.0, Y = 2.2, size 2.2m x 1.1m x 0.85m
builder.add_box("hosp_bed_frame", -8.0, 2.2, 0.30, 2.2, 1.1, 0.40, C_HOSPITAL_BD, is_furn=True)
builder.add_box("hosp_mattress", -8.0, 2.2, 0.55, 2.0, 1.0, 0.20, C_HOSP_LINEN, collision=False)
builder.add_box("hosp_headboard", -9.05, 2.2, 0.75, 0.1, 1.1, 0.80, C_HOSPITAL_BD, collision=False)
builder.add_box("hosp_footboard", -6.95, 2.2, 0.65, 0.1, 1.1, 0.60, C_HOSPITAL_BD, collision=False)
builder.add_box("hosp_rail_left", -8.0, 2.72, 0.65, 1.2, 0.05, 0.35, C_METAL_TRIM, collision=False)
builder.add_box("hosp_rail_right", -8.0, 1.68, 0.65, 1.2, 0.05, 0.35, C_METAL_TRIM, collision=False)

# Overbed Mobile Tray Table: at X = -7.5, Y = 2.2
builder.add_box("hosp_overbed_table", -7.5, 2.2, 0.85, 0.45, 0.9, 0.04, C_BIRCH_WOOD, collision=False)

# Medical Equipment Console & Monitor on wall behind bed:
builder.add_box("hosp_med_console", -9.10, 2.2, 1.35, 0.08, 1.8, 0.40, "0.95 0.95 0.95 1", is_furn=True)
builder.add_box("hosp_patient_monitor", -9.05, 1.7, 1.50, 0.10, 0.4, 0.30, C_TV_SCREEN, collision=False)

# Bedside Medical Nightstand + Lamp:
builder.add_box("hosp_nightstand", -8.9, 3.1, 0.35, 0.5, 0.5, 0.70, C_HOSPITAL_BD, is_furn=True)

# Hospital Visitor Lounge: 2 Armchairs + Small Table at X = -7.0, Y = 4.5
builder.add_box("hosp_visitor_chair_1", -7.0, 4.2, 0.42, 0.6, 0.6, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_box("hosp_visitor_chair_2", -7.0, 5.0, 0.42, 0.6, 0.6, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_cylinder("hosp_visitor_table", -7.8, 4.6, 0.25, 0.35, 0.50, C_OAK_WOOD, is_furn=True)

# Hospital Room 65" TV: on east wall facing bed
builder.add_box("hosp_tv_screen", -6.05, 2.2, 1.5, 0.04, 1.4, 0.8, C_TV_SCREEN, is_furn=True)

# -------------------------------------------------------------
# 8. Zone 4: Detailed 3D Models in Accounts Workstation
# -------------------------------------------------------------
print("Adding detailed 3D assets to Accounts Workstation...")
# Accounts Workstation bounds: X in [-52.0, -45.98], Y in [-3.06, +3.06]
# North partition wall (separates from Accounts Meeting Room): Y = 3.06, X in [-52.0, -45.98]
builder.add_box("wall_accounts_north", -49.0, 3.06, Z_WALL, 6.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# South partition wall: Y = -3.06, X in [-52.0, -45.98]
builder.add_box("wall_accounts_south", -49.0, -3.06, Z_WALL, 6.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# Clear Glass Partition Wall to East (separating from Project Workstations):
# Along X = -45.98 with doorway at Y in [1.2, 2.5] (opening width 1.3m for robot access)
builder.add_box("glass_accounts_top", -45.98, 2.8, Z_WALL, 0.06, 0.5, H_WALL, C_GLASS, is_wall=True)
builder.add_box("glass_accounts_bot", -45.98, -1.0, Z_WALL, 0.06, 4.1, H_WALL, C_GLASS, is_wall=True)

# Two Director Desks (AD1, AD2): at Y = 2.1
builder.add_box("desk_ad1", -50.2, 2.1, 0.375, 1.5, 0.8, 0.75, C_BIRCH_WOOD, is_furn=True)
builder.add_box("chair_ad1", -50.2, 2.65, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)
builder.add_box("desk_ad2", -47.8, 2.1, 0.375, 1.5, 0.8, 0.75, C_BIRCH_WOOD, is_furn=True)
builder.add_box("chair_ad2", -47.8, 2.65, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)

# Modular Workstation Pod (8 desks AW1 to AW8, back-to-back):
# Cluster centered at X = -49.0, Y = -0.6
# Desk dimensions: 4 desks facing east, 4 desks facing west
builder.add_box("pod_desks_surface", -49.0, -0.6, 0.375, 2.6, 3.2, 0.75, C_BIRCH_WOOD, is_furn=True)
# Central acoustic privacy screens (dividers):
builder.add_box("pod_divider_long", -49.0, -0.6, 0.95, 0.05, 3.2, 0.40, "0.35 0.35 0.40 1", collision=False)
builder.add_box("pod_divider_cross_1", -49.0, -1.4, 0.95, 2.6, 0.05, 0.40, "0.35 0.35 0.40 1", collision=False)
builder.add_box("pod_divider_cross_2", -49.0, 0.2, 0.95, 2.6, 0.05, 0.40, "0.35 0.35 0.40 1", collision=False)

# 8 Ergonomic Mesh Task Chairs around pod:
y_chair_positions = [-1.8, -1.0, -0.2, 0.6]
for idx, y_pos in enumerate(y_chair_positions):
    # West side chairs (facing east)
    builder.add_box(f"chair_aw_w_{idx}", -50.8, y_pos, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    # East side chairs (facing west)
    builder.add_box(f"chair_aw_e_{idx}", -47.2, y_pos, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Desktop PC monitors on desks:
for y_pos in y_chair_positions:
    builder.add_box(f"monitor_w_{y_pos}", -49.3, y_pos, 0.95, 0.12, 0.5, 0.35, C_TV_FRAME, collision=False)
    builder.add_box(f"monitor_e_{y_pos}", -48.7, y_pos, 0.95, 0.12, 0.5, 0.35, C_TV_FRAME, collision=False)

# Office Printer & Storage Unit: at X = -51.2, Y = -2.2
builder.add_box("accounts_printer_stand", -51.2, -2.2, 0.35, 0.8, 1.0, 0.70, C_WALNUT_WOOD, is_furn=True)
builder.add_box("accounts_printer", -51.2, -2.2, 0.85, 0.6, 0.6, 0.35, "0.95 0.95 0.95 1", collision=False)

# -------------------------------------------------------------
# 9. Simplified Zones (Lightweight Walls & Furniture)
# -------------------------------------------------------------
print("Adding simplified room boundaries and major obstacles...")
# Rahul's Cabin ($23' x 15' 1" = 7.0m x 4.6m): X in [-38.3, -31.3], Y in [8.5, 13.25]
builder.add_box("wall_rahul_west", -38.3, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("rahul_exec_desk", -35.0, 11.5, 0.38, 2.2, 1.0, 0.76, C_WALNUT_WOOD, is_furn=True)

# Project Workstations Hall ($32' 5" x 40' 4" = 9.9m x 12.3m): X in [-45.0, -32.5], Y in [-6.0, 6.0]
# Divider wall with central meeting rooms: X = -32.5, Y from -6.0 to 6.0 (with corridor opening at top)
builder.add_box("wall_project_ws_east", -32.5, -0.5, Z_WALL, T_WALL, 11.0, H_WALL, C_INNER_WALL, is_wall=True)
# Simplified workstation clusters in Project Workstations:
builder.add_box("project_ws_pod_1", -41.0, 2.0, 0.38, 4.5, 2.2, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("project_ws_pod_2", -41.0, -2.5, 0.38, 4.5, 2.2, 0.76, C_BIRCH_WOOD, is_furn=True)

# Central Connector Room & Storage ($X in [-32.5, -10.1], Y in [0.0, 6.0]$):
builder.add_box("wall_connector_div", -21.0, 3.0, Z_WALL, T_WALL, 6.0, H_WALL, C_INNER_WALL, is_wall=True)

# Customer Success ($25' 4" x 23' = 7.7m x 7.0m): X in [8.5, 22.0], Y in [-5.0, 6.0]
builder.add_box("wall_cs_east", 22.0, 0.5, Z_WALL, T_WALL, 11.0, H_WALL, C_INNER_WALL, is_wall=True)
# Workstation benches in Customer Success:
builder.add_box("cs_bench_1", 15.0, 3.0, 0.38, 5.0, 1.4, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("cs_bench_2", 15.0, -0.5, 0.38, 5.0, 1.4, 0.76, C_BIRCH_WOOD, is_furn=True)

# Muskan's Cabin & Rachana's Cabin (East Wing, X in [22.0, 30.0]):
builder.add_box("wall_east_cabins_div", 26.0, 5.0, Z_WALL, 8.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# -------------------------------------------------------------
# 10. Visual Zone Labels (Floating overhead banners)
# -------------------------------------------------------------
# To make all zones instantly identifiable in Gazebo GUI & top-down view
labels = [
    ("LOBBY & RECEPTION", 0.0, 2.2, 2.8),
    ("BOARD ROOM", -26.68, 10.88, 2.8),
    ("LIVING DEMO ROOM", 1.10, 10.88, 2.8),
    ("HOTEL ROOM EXP", -8.41, 10.88, 2.8),
    ("HOSPITAL ROOM EXP", -8.04, 3.29, 2.8),
    ("ACCOUNTS WORKSTATION", -48.25, 0.0, 2.8),
    ("PROJECT WORKSTATIONS", -38.74, 0.0, 2.8),
    ("CUSTOMER SUCCESS", 15.72, -0.73, 2.8),
]
for idx, (txt, lx, ly, lz) in enumerate(labels):
    builder.add_box(f"label_banner_{idx}", lx, ly, lz, 2.8, 0.6, 0.05, "0.15 0.35 0.65 1.0", collision=False)

# Write model.sdf
model_sdf_path = os.path.join(MODELS_DIR, "model.sdf")
with open(model_sdf_path, "w") as f:
    f.write(builder.to_sdf("office_environment"))
print(f"Generated {model_sdf_path} with {len(builder.elements)} links.")

# Write model.config
model_config_path = os.path.join(MODELS_DIR, "model.config")
with open(model_config_path, "w") as f:
    f.write("""<?xml version="1.0"?>
<model>
  <name>office_environment</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>DigiValet / Antigravity</name>
  </author>
  <description>
    Complete persistent 3D office reconstruction with high-detail Experience Centres, Lobby, Board Room, and Accounts Workstation.
  </description>
</model>
""")
print(f"Generated {model_config_path}")

# -------------------------------------------------------------
# 11. Generate worlds/robot_world.sdf
# -------------------------------------------------------------
robot_world_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="robot_world">

    <!-- Physics settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential System Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Ambient / Global Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>-10 0 25 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>0.2 0.2 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>150 150</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>150 150</size></plane></geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Office Architecture & Detailed Interior Zones -->
    <include>
      <uri>package://models/office_environment</uri>
      <name>office_environment</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- Four-Wheel Robot Model starting in Lobby IN lane -->
    <include>
      <uri>package://models/four_wheel_robot</uri>
      <name>four_wheel_robot</name>
      <pose>-1.8 0.0 0.15 0 0 1.5708</pose>
    </include>

  </world>
</sdf>
"""

robot_world_path = os.path.join(WORLDS_DIR, "robot_world.sdf")
with open(robot_world_path, "w") as f:
    f.write(robot_world_content)
print(f"Generated {robot_world_path}")

# -------------------------------------------------------------
# 12. Generate Matching 2D Occupancy Grid Map for Nav2
# -------------------------------------------------------------
print("Generating matching 2D occupancy grid map...")
RESOLUTION = 0.05  # 5cm per pixel

# World extents: X in [-54.0, +32.0], Y in [-16.0, +15.0]
MAP_MIN_X = -54.0
MAP_MAX_X = 32.0
MAP_MIN_Y = -16.0
MAP_MAX_Y = 15.0

WIDTH_M = MAP_MAX_X - MAP_MIN_X    # 86.0m
HEIGHT_M = MAP_MAX_Y - MAP_MIN_Y  # 31.0m

PIX_W = int(math.ceil(WIDTH_M / RESOLUTION))   # 1720 px
PIX_H = int(math.ceil(HEIGHT_M / RESOLUTION))  # 620 px

print(f"Occupancy Grid Size: {PIX_W} x {PIX_H} pixels at {RESOLUTION}m/px")

# Initialize map: 254 = free space (white in PGM standard, ROS uses 254 as free, 0 as occupied)
# In standard ROS 2 map_server:
# Image pixel 254 (white) -> free (0% occupied)
# Image pixel 0 (black) -> occupied (100% occupied)
# Image pixel 205 (gray) -> unknown
grid = np.full((PIX_H, PIX_W), 254, dtype=np.uint8)

def world_to_map_px(wx, wy):
    px = int((wx - MAP_MIN_X) / RESOLUTION)
    # Image row 0 is top (max_y)
    py = int((MAP_MAX_Y - wy) / RESOLUTION)
    return px, py

def draw_rect(cx, cy, sx, sy, val=0):
    min_x, max_x = cx - sx/2.0, cx + sx/2.0
    min_y, max_y = cy - sy/2.0, cy + sy/2.0
    px0, py1 = world_to_map_px(min_x, min_y)
    px1, py0 = world_to_map_px(max_x, max_y)
    px0 = max(0, min(PIX_W - 1, px0))
    px1 = max(0, min(PIX_W - 1, px1))
    py0 = max(0, min(PIX_H - 1, py0))
    py1 = max(0, min(PIX_H - 1, py1))
    grid[py0:py1+1, px0:px1+1] = val

# Mark walls in map (black = 0)
for (cx, cy, sx, sy) in builder.wall_boxes:
    draw_rect(cx, cy, sx, sy, val=0)

# Mark major solid furniture obstacles (black = 0)
for (cx, cy, sx, sy) in builder.furniture_boxes:
    draw_rect(cx, cy, sx, sy, val=0)

# Save PGM
pgm_path = os.path.join(MAPS_DIR, "office_map.pgm")
map_img = Image.fromarray(grid)
map_img.save(pgm_path)
print(f"Generated {pgm_path}")

# Save YAML (ROS 2 nav2_map_server format)
# Note: origin is [min_x, min_y, 0.0] in meters
yaml_path = os.path.join(MAPS_DIR, "office_map.yaml")
with open(yaml_path, "w") as f:
    f.write(f"""image: office_map.pgm
resolution: {RESOLUTION}
origin: [{MAP_MIN_X:.2f}, {MAP_MIN_Y:.2f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
""")
print(f"Generated {yaml_path}")
print("Office reconstruction generation completed successfully!")
