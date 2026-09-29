#!/usr/bin/env python3
"""
Office Reconstruction Generator for Gazebo Harmonic & Nav2
Based on:
  - 2D Architectural Source of Truth: resources/CASADIGI FURNITURE FLOOR PLAN.pdf
  - 3D Interior Visual Source of Truth: resources/Office_3D_VIEWS.pdf

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
# Color Definitions (RGBA) Matching 3D Views & Materials
# -------------------------------------------------------------
C_OUTER_WALL     = "0.86 0.86 0.88 1.0"  # Off-white exterior architectural envelope
C_INNER_WALL     = "0.90 0.89 0.86 1.0"  # Warm alabaster interior partition walls
C_GLASS          = "0.40 0.70 0.90 0.45"  # Frameless tempered acoustic glass
C_GLASS_FRAME    = "0.15 0.15 0.15 1.0"   # Matte black aluminum mullions
C_FLOOR_LOBBY    = "0.90 0.88 0.83 1.0"   # Polished Italian marble / travertine
C_FLOOR_CORR     = "0.85 0.83 0.80 1.0"   # Terrazzo main circulation corridors
C_FLOOR_CARP     = "0.35 0.36 0.38 1.0"   # Charcoal acoustic carpet tile (cabins & bullpens)
C_FLOOR_WOOD     = "0.68 0.52 0.38 1.0"   # Natural herringbone parquet (Experience Centres)
C_FLOOR_CAFE     = "0.78 0.75 0.70 1.0"   # Light porcelain cafe & dining floor tiles
C_MARBLE_DESK    = "0.95 0.95 0.96 1.0"   # Statuario white marble reception desk
C_ACCENT_SLATS   = "0.38 0.26 0.16 1.0"   # Walnut vertical acoustic wall slats
C_OAK_WOOD       = "0.74 0.58 0.42 1.0"   # Natural white oak (conference & dining)
C_BIRCH_WOOD     = "0.82 0.72 0.56 1.0"   # Light birch workstation surfaces
C_WALNUT_WOOD    = "0.40 0.26 0.16 1.0"   # American walnut executive cabinetry & desks
C_CHAIR_MESH     = "0.22 0.22 0.24 1.0"   # Ergonomic black mesh task chairs
C_LEATHER_DARK   = "0.18 0.15 0.12 1.0"   # Dark brown executive leather seating
C_CHAIR_FRAME    = "0.78 0.78 0.80 1.0"   # Polished chrome / aluminum chair bases
C_DIVIDER_FABRIC = "0.32 0.42 0.52 1.0"   # Slate blue acoustic privacy divider screens
C_TV_FRAME       = "0.08 0.08 0.08 1.0"   # Ultra-slim display bezel
C_TV_SCREEN      = "0.12 0.32 0.62 1.0"   # Vibrant digital presentation glow
C_HOSPITAL_BD    = "0.92 0.92 0.90 1.0"   # Healthcare enamel white
C_HOSP_LINEN     = "0.35 0.55 0.75 1.0"   # Medical pastel blue bed linens
C_SOFA_FABRIC    = "0.92 0.90 0.85 1.0"   # Oatmeal textured fabric lounge seating
C_METAL_TRIM     = "0.75 0.75 0.76 1.0"   # Brushed stainless steel turnstiles & elevator doors
C_PLANT_GREEN    = "0.18 0.58 0.22 1.0"   # Lush interior ficus & monstera leaves
C_POT_WHITE      = "0.92 0.92 0.92 1.0"   # Matte white cylindrical planters
C_BANNER_BG      = "0.12 0.28 0.50 1.0"   # High-contrast navy blue zone banners


class SDFBuilder:
    def __init__(self):
        self.elements = []
        self.wall_boxes = []       # For Nav2 map generation: (cx, cy, sx, sy)
        self.furniture_boxes = []  # For Nav2 map generation: (cx, cy, sx, sy)

    def add_box(self, name, cx, cy, cz, sx, sy, sz, color, collision=True, opacity=1.0, is_wall=False, is_furn=False):
        """Adds a box link with visual and optional collision geometry."""
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
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>"""

        link_str = f"""
    <link name="{name}">
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>{col_str}{vis_str}
    </link>"""
        self.elements.append(link_str)

    def add_cylinder(self, name, cx, cy, cz, radius, length, color, collision=True, is_furn=False):
        """Adds a cylindrical link."""
        col_str = ""
        if collision:
            col_str = f"""
      <collision name="{name}_col">
        <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
      </collision>"""
            if is_furn:
                self.furniture_boxes.append((cx, cy, radius * 2.0, radius * 2.0))

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
H_WALL = 2.60          # Architectural ceiling height 2.60m
Z_WALL = H_WALL / 2.0  # 1.30m vertical center
T_WALL = 0.18          # Standard partition wall thickness (18cm)

print("Constructing Full Architectural Geometry...")

# -------------------------------------------------------------
# 1. Floors & Finishes across All Zones
# -------------------------------------------------------------
# Lobby & Corridors
builder.add_box("floor_lobby", 0.0, 2.5, -0.01, 8.0, 10.0, 0.02, C_FLOOR_LOBBY, collision=False)
builder.add_box("floor_corridor_west", -25.0, 7.25, -0.01, 54.0, 3.5, 0.02, C_FLOOR_CORR, collision=False)
builder.add_box("floor_corridor_east", 16.0, 7.25, -0.01, 30.0, 3.5, 0.02, C_FLOOR_CORR, collision=False)

# Board Room Carpet
builder.add_box("floor_board_room", -26.68, 10.88, -0.01, 9.4, 4.8, 0.02, C_FLOOR_CARP, collision=False)

# Experience Centres (Wood Parquet & Healthcare Vinyl)
builder.add_box("floor_living_demo", 1.10, 10.88, -0.01, 9.6, 4.8, 0.02, C_FLOOR_WOOD, collision=False)
builder.add_box("floor_hotel_exp", -8.41, 10.88, -0.01, 7.8, 4.8, 0.02, C_FLOOR_WOOD, collision=False)
builder.add_box("floor_hospital_exp", -8.04, 3.29, -0.01, 4.4, 7.4, 0.02, C_FLOOR_LOBBY, collision=False)

# Accounts & Project Bullpens Carpet
builder.add_box("floor_accounts_ws", -48.25, 0.0, -0.01, 7.5, 6.5, 0.02, C_FLOOR_CARP, collision=False)
builder.add_box("floor_project_ws", -38.74, -1.0, -0.01, 12.5, 14.5, 0.02, C_FLOOR_CARP, collision=False)

# Executive Cabins (Rahul, Muskan, Rachana, Suhani, Vivek)
builder.add_box("floor_rahul_suite", -45.15, 10.88, -0.01, 13.7, 4.8, 0.02, C_FLOOR_CARP, collision=False)
builder.add_box("floor_muskan_suite", 11.45, 10.88, -0.01, 13.1, 4.8, 0.02, C_FLOOR_CARP, collision=False)
builder.add_box("floor_east_cabins", 24.00, 10.88, -0.01, 12.0, 4.8, 0.02, C_FLOOR_CARP, collision=False)
builder.add_box("floor_suhani_vivek", 5.68, 3.00, -0.01, 5.64, 6.0, 0.02, C_FLOOR_CARP, collision=False)

# Customer Success & Cafe / Dining
builder.add_box("floor_customer_success", 15.25, 0.50, -0.01, 13.5, 11.0, 0.02, C_FLOOR_CARP, collision=False)
builder.add_box("floor_digi_cafe", 15.25, -9.75, -0.01, 13.5, 9.5, 0.02, C_FLOOR_CAFE, collision=False)
builder.add_box("floor_exec_dining", 26.00, -4.25, -0.01, 8.0, 20.5, 0.02, C_FLOOR_CARP, collision=False)

# -------------------------------------------------------------
# 2. Exterior Perimeter Walls (Envelope: 82.0m x 27.75m)
# -------------------------------------------------------------
# North exterior wall: Y = 13.25, spans X from -52.0 to +30.0 (82.0m)
builder.add_box("wall_north_exterior", -11.0, 13.25, Z_WALL, 82.0, T_WALL, H_WALL, C_OUTER_WALL, is_wall=True)

# South exterior wall: Y = -14.50, spans X from -52.0 to +30.0
builder.add_box("wall_south_exterior", -11.0, -14.50, Z_WALL, 82.0, T_WALL, H_WALL, C_OUTER_WALL, is_wall=True)

# West exterior wall: X = -52.0, spans Y from -14.5 to +13.25 (27.75m)
builder.add_box("wall_west_exterior", -52.0, -0.62, Z_WALL, T_WALL, 27.75, H_WALL, C_OUTER_WALL, is_wall=True)

# East exterior wall: X = +30.0, spans Y from -14.5 to +13.25
builder.add_box("wall_east_exterior", 30.0, -0.62, Z_WALL, T_WALL, 27.75, H_WALL, C_OUTER_WALL, is_wall=True)

# -------------------------------------------------------------
# 3. Main East-West Corridor Walls & Doorways (Y in [6.00, 8.50])
# -------------------------------------------------------------
# North Corridor Wall (Y = 8.50) with doorways into North zones:
# 1. Rahul Sleeping & Accounts Meeting: X from -52.0 to -36.6 (15.4m)
builder.add_box("wall_corr_n_1", -44.30, 8.50, Z_WALL, 15.4, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Rahul Cabin: X in [-36.6, -35.0] (1.6m opening)
# 2. Between Rahul Cabin & Board Room: X from -35.0 to -24.5 (10.5m)
builder.add_box("wall_corr_n_2", -29.75, 8.50, Z_WALL, 10.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Board Room: X in [-24.5, -21.5] (3.0m opening for Nav2)
# 3. Between Board Room & Hotel Exp (fronting Phone Booths & Server): X from -21.5 to -11.7 (9.8m)
builder.add_box("wall_corr_n_3", -16.60, 8.50, Z_WALL, 9.8, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Hotel Exp: X in [-11.7, -8.7] (3.0m opening for Nav2)
# 4. Between Hotel Exp & Living Demo: X from -8.7 to -3.5 (5.2m)
builder.add_box("wall_corr_n_4", -6.10, 8.50, Z_WALL, 5.2, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Opening to Living Demo: X in [-3.5, -1.5] (2.0m open threshold)
# 5. East of Living Demo to Muskan Meeting door: X from -1.5 to 8.5 (10.0m)
builder.add_box("wall_corr_n_5a", 3.50, 8.50, Z_WALL, 10.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Muskan Meeting: X in [8.5, 10.0] (1.5m opening)
# 6. Muskan Meeting to Muskan Cabin door: X from 10.0 to 11.5 (1.5m)
builder.add_box("wall_corr_n_5b", 10.75, 8.50, Z_WALL, 1.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Muskan Cabin: X in [11.5, 13.0] (1.5m opening)
# 7. Muskan Cabin to Rachana Cabin door: X from 13.0 to 22.0 (9.0m)
builder.add_box("wall_corr_n_5c", 17.50, 8.50, Z_WALL, 9.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Rachana Cabin: X in [22.0, 23.5] (1.5m opening)
# 8. Between Rachana Cabin & Creche door: X from 23.5 to 24.5 (1.0m)
builder.add_box("wall_corr_n_6a", 24.00, 8.50, Z_WALL, 1.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Creche: X in [24.5, 26.0] (1.5m opening)
# 9. Creche to East Wall: X from 26.0 to 30.0 (4.0m)
builder.add_box("wall_corr_n_6b", 28.00, 8.50, Z_WALL, 4.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# South Corridor Wall (Y = 6.00) with openings into South zones:
# 1. Accounts WS & Project Bullpen North: X from -52.0 to -35.0 (17.0m)
builder.add_box("wall_corr_s_1", -43.50, 6.00, Z_WALL, 17.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Passageway into Project & Accounts aisles: X in [-35.0, -32.0] (3.0m wide)
# 2. Central Core & Storage North: X from -32.0 to -9.5 (22.5m)
builder.add_box("wall_corr_s_2", -20.75, 6.00, Z_WALL, 22.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Doorway to Hospital Exp: X in [-9.5, -6.5] (3.0m opening for Nav2)
# 3. Hospital Exp to Reception: X from -6.5 to -2.86 (3.64m)
builder.add_box("wall_corr_s_3", -4.68, 6.00, Z_WALL, 3.64, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Wide open threshold connecting Reception & Corridor: X in [-2.86, +2.86] (5.72m)
# 4. Reception to Suhani/Vivek Cabin: X from +2.86 to +6.5 (3.64m)
builder.add_box("wall_corr_s_4", 4.68, 6.00, Z_WALL, 3.64, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Hallway access to Customer Success & Digi Cafe: X in [6.5, 8.5] (2.0m opening)
# 5. CS & East Dining North: X from +8.5 to +30.0 (21.5m)
builder.add_box("wall_corr_s_5", 19.25, 6.00, Z_WALL, 21.5, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# -------------------------------------------------------------
# 4. North Zones Dividing Partitions
# -------------------------------------------------------------
# Partition between Rahul Sleeping Space and Accounts Meeting: X = -44.0, Y in [8.5, 13.25]
builder.add_box("wall_rahul_sleep_div", -44.0, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Accounts Meeting and Rahul Cabin: X = -38.3, Y in [8.5, 13.25]
builder.add_box("wall_rahul_west", -38.3, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Rahul Cabin and Board Room: X = -31.28, Y in [8.5, 13.25]
builder.add_box("wall_board_west", -31.28, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Board Room and Phone Booths: X = -22.08, Y in [8.5, 13.25]
builder.add_box("wall_board_east", -22.08, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Phone Booths and Server Room: X = -17.0, Y in [8.5, 13.25]
builder.add_box("wall_server_div", -17.0, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Server Room and Hotel Exp: X = -12.17, Y in [8.5, 13.25]
builder.add_box("wall_hotel_west", -12.17, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Hotel Exp and Living Demo: X = -4.65, Y in [8.5, 13.25]
builder.add_box("wall_hotel_living_div", -4.65, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Living Demo and Muskan Meeting: X = 4.91, Y in [8.5, 13.25]
builder.add_box("wall_living_east", 4.91, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Muskan Meeting and Muskan Cabin: X = 10.5, Y in [8.5, 13.25]
builder.add_box("wall_muskan_meeting_div", 10.5, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Muskan Cabin and Rachana Cabin: X = 18.0, Y in [8.5, 13.25]
builder.add_box("wall_muskan_rachana_div", 18.0, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)
# Partition between Rachana Cabin and Creche: X = 24.0, Y in [8.5, 13.25]
builder.add_box("wall_rachana_creche_div", 24.0, 10.88, Z_WALL, T_WALL, 4.75, H_WALL, C_INNER_WALL, is_wall=True)

# -------------------------------------------------------------
# 5. South Zones Dividing Partitions
# -------------------------------------------------------------
# Accounts Workstation Enclosure (X in [-52.0, -45.0], Y in [-3.06, 3.06])
builder.add_box("wall_accounts_north", -48.5, 3.06, Z_WALL, 7.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_accounts_south", -48.5, -3.06, Z_WALL, 7.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Glass partition facing Project Bullpen at X = -45.98 (door at Y in [1.2, 2.5])
builder.add_box("glass_accounts_top", -45.98, 2.80, Z_WALL, 0.06, 0.5, H_WALL, C_GLASS, is_wall=True)
builder.add_box("glass_accounts_bot", -45.98, -1.00, Z_WALL, 0.06, 4.1, H_WALL, C_GLASS, is_wall=True)

# Project Bullpen dividing wall with Central Core: X = -32.5, Y in [-10.0, 6.0]
builder.add_box("wall_project_ws_east", -32.5, -2.0, Z_WALL, T_WALL, 16.0, H_WALL, C_INNER_WALL, is_wall=True)

# Central Core & Storage divider: X = -21.0, Y in [0.0, 6.0]
builder.add_box("wall_connector_div", -21.0, 3.0, Z_WALL, T_WALL, 6.0, H_WALL, C_INNER_WALL, is_wall=True)
# Core southern service wall: Y = -6.0, X in [-32.5, -10.12]
builder.add_box("wall_core_south", -21.31, -6.0, Z_WALL, 22.38, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# Hospital Exp Room:
# West wall: X = -10.12, Y in [-0.32, 6.00]
builder.add_box("wall_hospital_west", -10.12, 2.84, Z_WALL, T_WALL, 6.32, H_WALL, C_INNER_WALL, is_wall=True)
# South wall: Y = -0.32, X in [-10.12, -2.86]
builder.add_box("wall_hospital_south", -6.49, -0.32, Z_WALL, 7.26, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# Reception & Lobby Area (X in [-2.86, +2.86], Y in [-1.38, 6.00])
builder.add_box("wall_reception_west", -2.86, 2.31, Z_WALL, T_WALL, 7.38, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_reception_east", 2.86, 2.31, Z_WALL, T_WALL, 7.38, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_entrance_facade", 0.0, -1.38, Z_WALL, 5.72, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
# Glass sliding entrance doors (visual aesthetics)
builder.add_box("glass_door_l", -0.6, -1.36, 1.1, 1.2, 0.02, 2.2, C_GLASS, collision=False)
builder.add_box("glass_door_r",  0.6, -1.36, 1.1, 1.2, 0.02, 2.2, C_GLASS, collision=False)

# Suhani & Vivek Cabins (X in [2.86, 8.5], Y in [0.0, 6.0])
builder.add_box("wall_vivek_suhani_div", 5.68, 3.0, Z_WALL, T_WALL, 6.0, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_suhani_south", 5.68, 0.0, Z_WALL, 5.64, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# Customer Success Bullpen & Digi Cafe (X in [8.5, 22.0])
builder.add_box("wall_cs_west", 8.5, 0.5, Z_WALL, T_WALL, 11.0, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_cs_east", 22.0, 0.5, Z_WALL, T_WALL, 11.0, H_WALL, C_INNER_WALL, is_wall=True)
# Half-height acoustic partition separating CS Bullpen from Digi Cafe (Y = -5.0, openings for dining circulation)
builder.add_box("wall_cs_cafe_div_1", 9.75, -5.0, 0.65, 2.5, T_WALL, 1.30, C_INNER_WALL, is_wall=True)
builder.add_box("wall_cs_cafe_div_2", 15.25, -5.0, 0.65, 3.5, T_WALL, 1.30, C_INNER_WALL, is_wall=True)
builder.add_box("wall_cs_cafe_div_3", 21.0, -5.0, 0.65, 2.0, T_WALL, 1.30, C_INNER_WALL, is_wall=True)

# Executive Dining & Meeting Rooms East Wing (X in [22.0, 30.0], Y in [-14.5, 6.0])
builder.add_box("wall_exec_dining_div_1", 26.0, -6.0, Z_WALL, 8.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("wall_exec_dining_div_2", 26.0, 0.0, Z_WALL, 8.0, T_WALL, H_WALL, C_INNER_WALL, is_wall=True)

# -------------------------------------------------------------
# 6. Detailed 3D Models: Lobby & Reception (3D Views pp. 3-8)
# -------------------------------------------------------------
print("Adding Lobby & Reception 3D models...")
# Monolithic Statuario Marble Reception Desk
builder.add_box("reception_desk_main", 0.0, 1.2, 0.525, 2.0, 0.80, 1.05, C_MARBLE_DESK, is_furn=True)
builder.add_box("reception_desk_plinth", 0.0, 1.2, 0.05, 1.9, 0.75, 0.10, "0.15 0.15 0.15 1", collision=False)
builder.add_box("reception_desk_top", 0.0, 1.2, 1.06, 2.05, 0.85, 0.04, "0.96 0.96 0.96 1", collision=False)
# Walnut slat acoustic feature paneling on side walls
builder.add_box("slats_reception_w", -2.82, 1.2, Z_WALL, 0.04, 3.0, H_WALL, C_ACCENT_SLATS, collision=False)
builder.add_box("slats_reception_e",  2.82, 1.2, Z_WALL, 0.04, 3.0, H_WALL, C_ACCENT_SLATS, collision=False)

# Receptionist Task Chair
builder.add_box("reception_chair_seat", 0.0, 1.85, 0.45, 0.45, 0.45, 0.08, C_CHAIR_MESH, is_furn=True)
builder.add_box("reception_chair_back", 0.0, 2.05, 0.85, 0.45, 0.06, 0.72, C_CHAIR_MESH, collision=False)
builder.add_cylinder("reception_chair_base", 0.0, 1.85, 0.22, 0.25, 0.44, C_CHAIR_FRAME, collision=False)

# Security Turnstiles (Access Gates flanking desk)
builder.add_box("turnstile_in_pedestal", -2.75, 1.2, 0.5, 0.18, 0.8, 1.0, C_METAL_TRIM, is_furn=True)
builder.add_box("turnstile_in_gate", -2.65, 1.55, 0.65, 0.04, 0.70, 0.7, C_GLASS, collision=False)
builder.add_box("turnstile_out_pedestal", 2.75, 1.2, 0.5, 0.18, 0.8, 1.0, C_METAL_TRIM, is_furn=True)
builder.add_box("turnstile_out_gate", 2.65, 1.55, 0.65, 0.04, 0.70, 0.7, C_GLASS, collision=False)

# Elevator Demo Feature & Kiosk
builder.add_box("elevator_wall", 0.0, 3.6, Z_WALL, 1.6, 0.20, H_WALL, C_INNER_WALL, is_wall=True)
builder.add_box("elevator_door_l", -0.35, 3.71, 1.1, 0.65, 0.02, 2.2, C_METAL_TRIM, collision=False)
builder.add_box("elevator_door_r",  0.35, 3.71, 1.1, 0.65, 0.02, 2.2, C_METAL_TRIM, collision=False)
builder.add_box("elevator_kiosk_pedestal", 0.75, 3.6, 0.5, 0.18, 0.18, 1.0, "0.1 0.1 0.1 1", is_furn=True)
builder.add_box("elevator_kiosk_screen", 0.75, 3.6, 1.1, 0.22, 0.05, 0.22, C_TV_SCREEN, collision=False)

# Waiting Lounge: Coffee table + 2 Armchairs behind elevator island
builder.add_cylinder("lobby_coffee_table", 0.0, 4.8, 0.22, 0.45, 0.44, C_WALNUT_WOOD, is_furn=True)
builder.add_box("lobby_armchair_1", -0.8, 4.8, 0.42, 0.60, 0.60, 0.65, C_SOFA_FABRIC, is_furn=True)
builder.add_box("lobby_armchair_2",  0.8, 4.8, 0.42, 0.60, 0.60, 0.65, C_SOFA_FABRIC, is_furn=True)

# Potted Planters at entrance corners
builder.add_cylinder("pot_lobby_l", -2.5, -0.9, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_lobby_l", -2.5, -0.9, 0.75, 0.3, 0.5, C_PLANT_GREEN, collision=False)
builder.add_cylinder("pot_lobby_r",  2.5, -0.9, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_lobby_r",  2.5, -0.9, 0.75, 0.3, 0.5, C_PLANT_GREEN, collision=False)

# -------------------------------------------------------------
# 7. Detailed 3D Models: Board Room (3D Views pp. 25-27)
# -------------------------------------------------------------
print("Adding Board Room 3D models...")
# Boardroom Conference Table: centered at X = -26.68, Y = 10.88, 5.8m x 1.4m x 0.76m
builder.add_box("board_table_top", -26.68, 10.88, 0.74, 5.8, 1.4, 0.05, C_OAK_WOOD, is_furn=True)
builder.add_box("board_table_base_l", -28.2, 10.88, 0.35, 0.4, 0.9, 0.70, "0.15 0.15 0.15 1", collision=False)
builder.add_box("board_table_base_r", -25.1, 10.88, 0.35, 0.4, 0.9, 0.70, "0.15 0.15 0.15 1", collision=False)
builder.add_box("board_cable_box", -26.68, 10.88, 0.765, 0.8, 0.25, 0.01, C_METAL_TRIM, collision=False)

# 12 Side Chairs (6 north, 6 south) + 2 Head Chairs
for idx, x_chair in enumerate(np.linspace(-29.0, -24.4, 6)):
    # North side chairs
    builder.add_box(f"board_chair_n_{idx}", x_chair, 11.85, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"board_chair_n_back_{idx}", x_chair, 12.05, 0.82, 0.46, 0.06, 0.65, C_CHAIR_MESH, collision=False)
    # South side chairs
    builder.add_box(f"board_chair_s_{idx}", x_chair, 9.91, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"board_chair_s_back_{idx}", x_chair, 9.71, 0.82, 0.46, 0.06, 0.65, C_CHAIR_MESH, collision=False)

# Head chairs
builder.add_box("board_chair_head_w", -29.85, 10.88, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)
builder.add_box("board_chair_head_e", -23.50, 10.88, 0.45, 0.5, 0.5, 0.08, C_CHAIR_MESH, is_furn=True)

# Presentation Display Wall (West Wall, X = -31.15)
builder.add_box("board_screen_frame", -31.15, 10.88, 1.65, 0.04, 2.0, 1.15, C_TV_FRAME, is_furn=True)
builder.add_box("board_screen_display", -31.12, 10.88, 1.65, 0.02, 1.9, 1.05, C_TV_SCREEN, collision=False)
builder.add_box("board_credenza", -31.0, 10.88, 0.35, 0.45, 2.4, 0.70, C_WALNUT_WOOD, is_furn=True)
# Acoustic fabric wall panels on North wall
builder.add_box("board_acoustic_panel", -26.68, 13.15, 1.5, 6.0, 0.04, 1.8, "0.30 0.32 0.35 1", collision=False)

# Corner Potted Planter
builder.add_cylinder("pot_board", -30.5, 12.6, 0.3, 0.25, 0.6, C_POT_WHITE, is_furn=True)
builder.add_cylinder("plant_board", -30.5, 12.6, 0.8, 0.35, 0.6, C_PLANT_GREEN, collision=False)

# -------------------------------------------------------------
# 8. Detailed 3D Models: Hotel Experience Centre
# -------------------------------------------------------------
print("Adding Hotel Experience Centre 3D models...")
# King Hospitality Bed: centered at X = -10.2, Y = 11.6, 2.1m x 2.0m
builder.add_box("hotel_bed_mattress", -10.2, 11.6, 0.35, 2.0, 2.0, 0.50, "0.95 0.95 0.95 1", is_furn=True)
builder.add_box("hotel_bed_headboard", -11.22, 11.6, 0.75, 0.12, 2.2, 1.10, C_WALNUT_WOOD, collision=False)
builder.add_box("hotel_bed_runner", -9.5, 11.6, 0.61, 0.5, 2.02, 0.02, "0.4 0.3 0.2 1", collision=False)
# Bedside nightstands + lamps
builder.add_box("hotel_nightstand_l", -10.2, 12.8, 0.25, 0.5, 0.5, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_cylinder("hotel_lamp_l", -10.2, 12.8, 0.65, 0.12, 0.30, "0.9 0.85 0.7 1", collision=False)
builder.add_box("hotel_nightstand_r", -10.2, 10.4, 0.25, 0.5, 0.5, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_cylinder("hotel_lamp_r", -10.2, 10.4, 0.65, 0.12, 0.30, "0.9 0.85 0.7 1", collision=False)

# Automated Mini-Bar Cabinet: at X = -5.2, Y = 12.5
builder.add_box("hotel_bar_cabinet", -5.2, 12.5, 0.65, 0.6, 1.2, 1.30, C_WALNUT_WOOD, is_furn=True)
builder.add_box("hotel_bar_glass", -5.2, 12.5, 1.5, 0.55, 1.15, 0.40, C_GLASS, collision=False)

# Hotel 65" TV
builder.add_box("hotel_tv_frame", -8.2, 8.65, 1.5, 1.4, 0.05, 0.85, C_TV_FRAME, is_furn=True)
builder.add_box("hotel_tv_screen", -8.2, 8.68, 1.5, 1.3, 0.02, 0.75, C_TV_SCREEN, collision=False)

# Round Lounge Table + 2 Chairs
builder.add_cylinder("hotel_lounge_table", -6.8, 10.5, 0.35, 0.45, 0.70, C_OAK_WOOD, is_furn=True)
builder.add_box("hotel_lounge_chair_1", -6.8, 11.2, 0.42, 0.55, 0.55, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_box("hotel_lounge_chair_2", -6.8, 9.8, 0.42, 0.55, 0.55, 0.70, C_SOFA_FABRIC, is_furn=True)

# -------------------------------------------------------------
# 9. Detailed 3D Models: Living Demo Room (3D Views pp. 28-34)
# -------------------------------------------------------------
print("Adding Living Demo Room 3D models...")
# Curved Sectional Sofa (3 segmented blocks)
builder.add_box("sofa_mid", 2.0, 12.0, 0.42, 2.2, 0.9, 0.75, C_SOFA_FABRIC, is_furn=True)
builder.add_box("sofa_left", 0.7, 11.7, 0.42, 0.9, 1.2, 0.75, C_SOFA_FABRIC, is_furn=True)
builder.add_box("sofa_right", 3.3, 11.7, 0.42, 0.9, 1.2, 0.75, C_SOFA_FABRIC, is_furn=True)

# Round Coffee Table Cluster
builder.add_cylinder("living_coffee_table_1", 2.0, 10.6, 0.22, 0.50, 0.44, "0.15 0.15 0.15 1", is_furn=True)
builder.add_cylinder("living_coffee_table_2", 2.7, 10.3, 0.18, 0.35, 0.36, C_OAK_WOOD, is_furn=True)

# Media Feature Wall: 65" TV, Floating Console, Wood Paneling & Fireplace
builder.add_box("living_tv_wall_panel", 2.0, 8.65, 1.5, 2.8, 0.08, 2.0, C_OAK_WOOD, is_furn=True)
builder.add_box("living_tv_screen", 2.0, 8.70, 1.6, 1.8, 0.04, 1.05, C_TV_SCREEN, collision=False)
builder.add_box("living_credenza", 2.0, 8.85, 0.35, 3.0, 0.45, 0.50, C_WALNUT_WOOD, is_furn=True)
builder.add_box("living_fireplace_strip", 2.0, 8.90, 0.35, 1.4, 0.02, 0.15, "0.95 0.55 0.15 1", collision=False)

# Demonstration Smart Dining Table & Chairs
builder.add_box("living_dining_table", 4.1, 10.0, 0.38, 1.2, 1.8, 0.76, C_OAK_WOOD, is_furn=True)
builder.add_box("living_dining_chair_1", 4.1, 11.2, 0.42, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("living_dining_chair_2", 4.1, 8.8, 0.42, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)

# -------------------------------------------------------------
# 10. Detailed 3D Models: Hospital Experience Centre (3D Views pp. 35-37)
# -------------------------------------------------------------
print("Adding Hospital Experience Centre 3D models...")
# Motorized Hospital Bed: centered at X = -8.0, Y = 2.2, 2.2m x 1.1m
builder.add_box("hosp_bed_frame", -8.0, 2.2, 0.30, 2.2, 1.1, 0.40, C_HOSPITAL_BD, is_furn=True)
builder.add_box("hosp_mattress", -8.0, 2.2, 0.55, 2.0, 1.0, 0.20, C_HOSP_LINEN, collision=False)
builder.add_box("hosp_headboard", -9.05, 2.2, 0.75, 0.1, 1.1, 0.80, C_HOSPITAL_BD, collision=False)
builder.add_box("hosp_footboard", -6.95, 2.2, 0.65, 0.1, 1.1, 0.60, C_HOSPITAL_BD, collision=False)
builder.add_box("hosp_rail_left", -8.0, 2.72, 0.65, 1.2, 0.05, 0.35, C_METAL_TRIM, collision=False)
builder.add_box("hosp_rail_right", -8.0, 1.68, 0.65, 1.2, 0.05, 0.35, C_METAL_TRIM, collision=False)

# Overbed Mobile Tray Table
builder.add_box("hosp_overbed_table", -7.5, 2.2, 0.85, 0.45, 0.9, 0.04, C_BIRCH_WOOD, collision=False)

# Medical Equipment Console & Patient Monitor behind bed
builder.add_box("hosp_med_console", -9.10, 2.2, 1.35, 0.08, 1.8, 0.40, "0.95 0.95 0.95 1", is_furn=True)
builder.add_box("hosp_patient_monitor", -9.05, 1.7, 1.50, 0.10, 0.4, 0.30, C_TV_SCREEN, collision=False)

# Bedside Medical Nightstand
builder.add_box("hosp_nightstand", -8.9, 3.1, 0.35, 0.5, 0.5, 0.70, C_HOSPITAL_BD, is_furn=True)

# Hospital Visitor Lounge: 2 Armchairs + Small Table
builder.add_box("hosp_visitor_chair_1", -7.0, 4.2, 0.42, 0.6, 0.6, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_box("hosp_visitor_chair_2", -7.0, 5.0, 0.42, 0.6, 0.6, 0.70, C_SOFA_FABRIC, is_furn=True)
builder.add_cylinder("hosp_visitor_table", -7.8, 4.6, 0.25, 0.35, 0.50, C_OAK_WOOD, is_furn=True)

# Hospital Room 65" TV
builder.add_box("hosp_tv_screen", -6.05, 2.2, 1.5, 0.04, 1.4, 0.8, C_TV_SCREEN, is_furn=True)

# -------------------------------------------------------------
# 11. Detailed 3D Models: Accounts Workstations (3D Views pp. 46-48)
# -------------------------------------------------------------
print("Adding Accounts Workstations 3D models...")
# Two Director Desks (AD1, AD2) along North wall
builder.add_box("desk_ad1", -50.2, 2.1, 0.375, 1.5, 0.8, 0.75, C_BIRCH_WOOD, is_furn=True)
builder.add_box("chair_ad1", -50.2, 2.65, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)
builder.add_box("desk_ad2", -47.8, 2.1, 0.375, 1.5, 0.8, 0.75, C_BIRCH_WOOD, is_furn=True)
builder.add_box("chair_ad2", -47.8, 2.65, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)

# Modular Workstation Pod (8 desks AW1 to AW8, back-to-back): centered at X = -49.0, Y = -0.6
builder.add_box("pod_desks_surface", -49.0, -0.6, 0.375, 2.6, 3.2, 0.75, C_BIRCH_WOOD, is_furn=True)
builder.add_box("pod_divider_long", -49.0, -0.6, 0.95, 0.05, 3.2, 0.40, C_DIVIDER_FABRIC, collision=False)
builder.add_box("pod_divider_cross_1", -49.0, -1.4, 0.95, 2.6, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
builder.add_box("pod_divider_cross_2", -49.0, 0.2, 0.95, 2.6, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)

# 8 Task Chairs around pod
y_chair_positions = [-1.8, -1.0, -0.2, 0.6]
for idx, y_pos in enumerate(y_chair_positions):
    builder.add_box(f"chair_aw_w_{idx}", -50.8, y_pos, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"chair_aw_e_{idx}", -47.2, y_pos, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    # Desktop PC monitors
    builder.add_box(f"monitor_w_{idx}", -49.3, y_pos, 0.95, 0.12, 0.5, 0.35, C_TV_FRAME, collision=False)
    builder.add_box(f"monitor_e_{idx}", -48.7, y_pos, 0.95, 0.12, 0.5, 0.35, C_TV_FRAME, collision=False)

# Multifunction Printer Stand & Copier
builder.add_box("accounts_printer_stand", -51.2, -2.2, 0.35, 0.8, 1.0, 0.70, C_WALNUT_WOOD, is_furn=True)
builder.add_box("accounts_printer", -51.2, -2.2, 0.85, 0.6, 0.6, 0.35, "0.95 0.95 0.95 1", collision=False)

# -------------------------------------------------------------
# 12. Detailed 3D Models: Rahul Cabin & Suite (3D Views pp. 9-14)
# -------------------------------------------------------------
print("Adding Rahul Cabin & Suite 3D models...")
# Executive L-Desk in rich American Walnut
builder.add_box("rahul_exec_desk_main", -34.8, 11.5, 0.38, 2.2, 1.0, 0.76, C_WALNUT_WOOD, is_furn=True)
builder.add_box("rahul_exec_desk_return", -33.9, 12.3, 0.38, 0.8, 1.2, 0.76, C_WALNUT_WOOD, is_furn=True)
# High-back executive leather chair
builder.add_box("rahul_exec_chair_seat", -34.8, 12.3, 0.45, 0.55, 0.55, 0.10, C_LEATHER_DARK, is_furn=True)
builder.add_box("rahul_exec_chair_back", -34.8, 12.55, 0.95, 0.55, 0.08, 0.90, C_LEATHER_DARK, collision=False)
# 2 Guest Armchairs
builder.add_box("rahul_guest_chair_1", -35.6, 10.6, 0.42, 0.55, 0.55, 0.70, C_LEATHER_DARK, is_furn=True)
builder.add_box("rahul_guest_chair_2", -34.0, 10.6, 0.42, 0.55, 0.55, 0.70, C_LEATHER_DARK, is_furn=True)
# 4-pax Circular Meeting Table
builder.add_cylinder("rahul_meet_table", -36.6, 11.2, 0.38, 0.55, 0.76, C_OAK_WOOD, is_furn=True)
builder.add_box("rahul_meet_chair_1", -36.6, 12.0, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("rahul_meet_chair_2", -36.6, 10.4, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
# Bookshelf & Credenza along North wall
builder.add_box("rahul_credenza", -34.8, 13.0, 0.85, 2.6, 0.45, 1.70, C_WALNUT_WOOD, is_furn=True)

# Rahul Sleeping / Rest Space (X in [-44.0, -38.3])
builder.add_box("rahul_sleep_daybed", -41.0, 11.5, 0.35, 1.4, 2.0, 0.50, C_SOFA_FABRIC, is_furn=True)
builder.add_box("rahul_sleep_wardrobe", -39.0, 12.5, 1.0, 0.8, 1.2, 2.0, C_WALNUT_WOOD, is_furn=True)

# Accounts Meeting Room (X in [-52.0, -44.0])
builder.add_box("acc_meet_table", -48.0, 11.0, 0.38, 2.8, 1.2, 0.76, C_OAK_WOOD, is_furn=True)
for mx in [-49.0, -48.0, -47.0]:
    builder.add_box(f"acc_meet_ch_n_{mx}", mx, 11.8, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"acc_meet_ch_s_{mx}", mx, 10.2, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("acc_meet_tv", -51.8, 11.0, 1.5, 0.04, 1.6, 0.9, C_TV_SCREEN, is_furn=True)

# -------------------------------------------------------------
# 13. Phone Booths & Server/UPS Room (X in [-22.08, -12.17])
# -------------------------------------------------------------
print("Adding Phone Booths & Server/UPS Room 3D models...")
# Acoustic Phone Booth 1
builder.add_box("phone_booth_1_desk", -20.5, 11.0, 0.38, 0.6, 1.0, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("phone_booth_1_stool", -19.8, 11.0, 0.45, 0.4, 0.4, 0.5, C_CHAIR_MESH, is_furn=True)
# Acoustic Phone Booth 2
builder.add_box("phone_booth_2_desk", -18.5, 11.0, 0.38, 0.6, 1.0, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("phone_booth_2_stool", -17.8, 11.0, 0.45, 0.4, 0.4, 0.5, C_CHAIR_MESH, is_furn=True)

# Server & UPS Equipment Racks
for idx, sx in enumerate([-15.5, -14.2, -12.9]):
    builder.add_box(f"server_rack_{idx}", sx, 12.0, 1.0, 0.8, 0.9, 2.0, "0.10 0.10 0.12 1", is_furn=True)
    builder.add_box(f"server_leds_{idx}", sx, 11.52, 1.2, 0.7, 0.02, 1.2, "0.2 0.8 0.4 1", collision=False)

# -------------------------------------------------------------
# 14. Project & HR Workstations Bullpen (X in [-45.0, -32.5])
# -------------------------------------------------------------
print("Adding Project & HR Workstations Bullpen...")
# Pod 1: North-West (4 desks)
builder.add_box("proj_pod_1_desks", -41.0, 3.5, 0.38, 2.8, 1.6, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("proj_pod_1_div", -41.0, 3.5, 0.95, 2.8, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for px in [-41.8, -40.2]:
    builder.add_box(f"proj_p1_ch_n_{px}", px, 4.4, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"proj_p1_ch_s_{px}", px, 2.6, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Pod 2: North-East (4 desks)
builder.add_box("proj_pod_2_desks", -36.5, 3.5, 0.38, 2.8, 1.6, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("proj_pod_2_div", -36.5, 3.5, 0.95, 2.8, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for px in [-37.3, -35.7]:
    builder.add_box(f"proj_p2_ch_n_{px}", px, 4.4, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"proj_p2_ch_s_{px}", px, 2.6, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Pod 3: South-West (6 desks)
builder.add_box("proj_pod_3_desks", -41.0, -1.5, 0.38, 4.2, 1.6, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("proj_pod_3_div", -41.0, -1.5, 0.95, 4.2, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for px in [-42.2, -41.0, -39.8]:
    builder.add_box(f"proj_p3_ch_n_{px}", px, -0.6, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"proj_p3_ch_s_{px}", px, -2.4, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Pod 4: South-East (6 desks)
builder.add_box("proj_pod_4_desks", -36.5, -1.5, 0.38, 4.2, 1.6, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("proj_pod_4_div", -36.5, -1.5, 0.95, 4.2, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for px in [-37.7, -36.5, -35.3]:
    builder.add_box(f"proj_p4_ch_n_{px}", px, -0.6, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"proj_p4_ch_s_{px}", px, -2.4, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Collaboration Whiteboard on West Wall
builder.add_box("proj_whiteboard", -44.9, 1.0, 1.5, 0.04, 3.0, 1.2, "0.95 0.95 0.95 1", collision=False)

# -------------------------------------------------------------
# 15. Muskan Meeting, Muskan Cabin, Rachana Cabin & Creche
# -------------------------------------------------------------
print("Adding Muskan & East Wing Cabins...")
# Muskan Meeting (X in [4.91, 10.5])
builder.add_box("muskan_meet_table", 7.7, 10.88, 0.38, 2.4, 1.1, 0.76, C_OAK_WOOD, is_furn=True)
for mx in [7.0, 8.4]:
    builder.add_box(f"muskan_m_ch_n_{mx}", mx, 11.6, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"muskan_m_ch_s_{mx}", mx, 10.1, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("muskan_meet_tv", 5.0, 10.88, 1.5, 0.04, 1.4, 0.8, C_TV_SCREEN, is_furn=True)

# Muskan Cabin (X in [10.5, 18.0]) (3D Views pp. 21-24)
builder.add_box("muskan_exec_desk", 14.2, 11.5, 0.38, 2.0, 0.9, 0.76, C_WALNUT_WOOD, is_furn=True)
builder.add_box("muskan_exec_chair", 14.2, 12.2, 0.45, 0.5, 0.5, 0.85, C_LEATHER_DARK, is_furn=True)
builder.add_box("muskan_visitor_ch_1", 13.5, 10.6, 0.42, 0.5, 0.5, 0.70, C_LEATHER_DARK, is_furn=True)
builder.add_box("muskan_visitor_ch_2", 14.9, 10.6, 0.42, 0.5, 0.5, 0.70, C_LEATHER_DARK, is_furn=True)
builder.add_box("muskan_lounge_sofa", 16.5, 10.8, 0.42, 0.8, 2.2, 0.75, C_SOFA_FABRIC, is_furn=True)
builder.add_box("muskan_credenza", 14.2, 13.0, 0.6, 2.4, 0.45, 1.2, C_WALNUT_WOOD, is_furn=True)

# Rachana Cabin (X in [18.0, 24.0])
builder.add_box("rachana_exec_desk", 21.0, 11.5, 0.38, 2.0, 0.9, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("rachana_exec_chair", 21.0, 12.2, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)
builder.add_box("rachana_guest_ch", 21.0, 10.6, 0.42, 0.5, 0.5, 0.70, C_CHAIR_MESH, is_furn=True)
builder.add_box("rachana_storage", 18.5, 11.5, 0.75, 0.5, 1.8, 1.5, C_BIRCH_WOOD, is_furn=True)

# Creche / Mother's Room (X in [24.0, 30.0])
builder.add_box("creche_daybed", 27.0, 11.5, 0.35, 1.2, 1.8, 0.50, C_SOFA_FABRIC, is_furn=True)
builder.add_box("creche_play_cubes", 25.5, 10.5, 0.3, 0.8, 0.8, 0.6, "0.9 0.4 0.4 1", is_furn=True)

# -------------------------------------------------------------
# 16. Vivek Cabin & Suhani Cabin (3D Views pp. 15-20)
# -------------------------------------------------------------
print("Adding Vivek & Suhani Cabins 3D models...")
# Vivek Cabin (X in [2.86, 5.68], Y in [0.0, 6.0])
builder.add_box("vivek_exec_desk", 4.3, 2.5, 0.38, 1.8, 0.9, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("vivek_exec_chair", 4.3, 1.8, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)
builder.add_box("vivek_guest_chair", 4.3, 3.4, 0.42, 0.5, 0.5, 0.70, C_CHAIR_MESH, is_furn=True)

# Suhani Cabin (X in [5.68, 8.5], Y in [0.0, 6.0])
builder.add_box("suhani_exec_desk", 7.1, 2.5, 0.38, 1.8, 0.9, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("suhani_exec_chair", 7.1, 1.8, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)
builder.add_box("suhani_guest_chair", 7.1, 3.4, 0.42, 0.5, 0.5, 0.70, C_CHAIR_MESH, is_furn=True)

# -------------------------------------------------------------
# 17. Customer Success Bullpen (X in [8.5, 22.0], Y in [-5.0, 6.0])
# -------------------------------------------------------------
print("Adding Customer Success Bullpen 3D models...")
# Modular Pod 1: 8 desks centered at X = 13.0, Y = 2.0
builder.add_box("cs_pod_1_desks", 13.0, 2.0, 0.38, 4.8, 1.8, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("cs_pod_1_div", 13.0, 2.0, 0.95, 4.8, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for cx in [11.2, 12.4, 13.6, 14.8]:
    builder.add_box(f"cs_p1_ch_n_{cx}", cx, 3.0, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"cs_p1_ch_s_{cx}", cx, 1.0, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# Modular Pod 2: 8 desks centered at X = 18.0, Y = 2.0
builder.add_box("cs_pod_2_desks", 18.0, 2.0, 0.38, 4.8, 1.8, 0.76, C_BIRCH_WOOD, is_furn=True)
builder.add_box("cs_pod_2_div", 18.0, 2.0, 0.95, 4.8, 0.05, 0.40, C_DIVIDER_FABRIC, collision=False)
for cx in [16.2, 17.4, 18.6, 19.8]:
    builder.add_box(f"cs_p2_ch_n_{cx}", cx, 3.0, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"cs_p2_ch_s_{cx}", cx, 1.0, 0.45, 0.48, 0.48, 0.8, C_CHAIR_MESH, is_furn=True)

# CS Manager Desk
builder.add_box("cs_lead_desk", 11.0, -1.8, 0.38, 1.8, 0.9, 0.76, C_WALNUT_WOOD, is_furn=True)
builder.add_box("cs_lead_chair", 11.0, -2.5, 0.45, 0.5, 0.5, 0.85, C_CHAIR_MESH, is_furn=True)

# CS Real-Time Metric Display Boards
builder.add_box("cs_dashboard_display", 15.5, 5.85, 1.6, 2.8, 0.04, 1.2, C_TV_SCREEN, is_furn=True)

# -------------------------------------------------------------
# 18. Digi Cafe & Dining (3D Views pp. 54-55)
# -------------------------------------------------------------
print("Adding Digi Cafe & Dining 3D models...")
# Coffee / Espresso Bar & Serving Counter along south wall
builder.add_box("cafe_counter", 13.5, -13.6, 0.5, 7.0, 0.9, 1.0, C_MARBLE_DESK, is_furn=True)
builder.add_box("cafe_espresso_machine", 12.0, -13.6, 1.15, 0.8, 0.5, 0.45, "0.2 0.2 0.2 1", collision=False)

# 3 High Poseur Cafe Tables + Stools
for idx, tx in enumerate([11.5, 15.0, 18.5]):
    builder.add_cylinder(f"cafe_high_table_{idx}", tx, -10.5, 0.525, 0.45, 1.05, C_OAK_WOOD, is_furn=True)
    builder.add_cylinder(f"cafe_stool_a_{idx}", tx - 0.55, -10.5, 0.38, 0.2, 0.75, "0.15 0.15 0.15 1", is_furn=True)
    builder.add_cylinder(f"cafe_stool_b_{idx}", tx + 0.55, -10.5, 0.38, 0.2, 0.75, "0.15 0.15 0.15 1", is_furn=True)

# 2 Dining Tables with 6 chairs each
for idx, tx in enumerate([13.0, 18.0]):
    builder.add_box(f"dining_table_{idx}", tx, -7.5, 0.38, 2.2, 1.1, 0.76, C_OAK_WOOD, is_furn=True)
    for cx in [-0.7, 0.0, 0.7]:
        builder.add_box(f"dining_ch_n_{idx}_{cx}", tx + cx, -6.8, 0.42, 0.45, 0.45, 0.75, C_SOFA_FABRIC, is_furn=True)
        builder.add_box(f"dining_ch_s_{idx}_{cx}", tx + cx, -8.2, 0.42, 0.45, 0.45, 0.75, C_SOFA_FABRIC, is_furn=True)

# -------------------------------------------------------------
# 19. Executive Dining & East Wing Meeting Suite
# -------------------------------------------------------------
print("Adding Executive Dining & East Suite 3D models...")
# Executive Dining Room (X in [22.0, 30.0], Y in [-14.5, -6.0])
builder.add_box("exec_dining_table", 26.0, -10.0, 0.38, 4.2, 1.3, 0.76, C_WALNUT_WOOD, is_furn=True)
for dx in [-1.4, -0.5, 0.5, 1.4]:
    builder.add_box(f"exec_din_ch_n_{dx}", 26.0 + dx, -9.2, 0.45, 0.5, 0.5, 0.85, C_LEATHER_DARK, is_furn=True)
    builder.add_box(f"exec_din_ch_s_{dx}", 26.0 + dx, -10.8, 0.45, 0.5, 0.5, 0.85, C_LEATHER_DARK, is_furn=True)
builder.add_box("exec_dining_buffet", 26.0, -13.8, 0.45, 3.6, 0.55, 0.90, C_WALNUT_WOOD, is_furn=True)

# East Meeting Room (X in [22.0, 30.0], Y in [-6.0, 0.0])
builder.add_box("east_meet_table", 26.0, -3.0, 0.38, 2.6, 1.2, 0.76, C_OAK_WOOD, is_furn=True)
for mx in [-0.8, 0.0, 0.8]:
    builder.add_box(f"east_m_ch_n_{mx}", 26.0 + mx, -2.2, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
    builder.add_box(f"east_m_ch_s_{mx}", 26.0 + mx, -3.8, 0.45, 0.45, 0.45, 0.75, C_CHAIR_MESH, is_furn=True)
builder.add_box("east_meet_tv", 22.1, -3.0, 1.5, 0.04, 1.6, 0.9, C_TV_SCREEN, is_furn=True)

# -------------------------------------------------------------
# 20. Visual Zone Labels (Floating overhead at Z = 2.80m)
# -------------------------------------------------------------
print("Adding floating overhead zone labels...")
labels = [
    ("LOBBY & RECEPTION", 0.0, 2.2, 2.80),
    ("BOARD ROOM", -26.68, 10.88, 2.80),
    ("HOTEL ROOM EXP", -8.41, 10.88, 2.80),
    ("LIVING DEMO ROOM", 1.10, 10.88, 2.80),
    ("HOSPITAL ROOM EXP", -8.04, 3.29, 2.80),
    ("ACCOUNTS WORKSTATIONS", -48.25, 0.0, 2.80),
    ("PROJECT BULLPEN", -38.74, 1.0, 2.80),
    ("RAHUL CABIN & SUITE", -35.0, 10.88, 2.80),
    ("MUSKAN CABIN & MEETING", 11.45, 10.88, 2.80),
    ("CUSTOMER SUCCESS", 15.25, 1.5, 2.80),
    ("DIGI CAFE & DINING", 15.25, -9.5, 2.80),
    ("EXECUTIVE DINING", 26.0, -10.0, 2.80),
]
for idx, (txt, lx, ly, lz) in enumerate(labels):
    builder.add_box(f"label_banner_{idx}", lx, ly, lz, 3.4, 0.6, 0.05, C_BANNER_BG, collision=False)

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
    Complete architectural office reconstruction matching CASADIGI floor plan and 3D views.
  </description>
</model>
""")
print(f"Generated {model_config_path}")

# -------------------------------------------------------------
# 21. Generate worlds/robot_world.sdf
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

    <!-- Directional Sun & Ambient Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>-10 0 25 0 0 0</pose>
      <diffuse>0.88 0.88 0.88 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
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

    <!-- Complete Office Architecture & Interior Reconstruction -->
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
# 22. Generate Matching 2D Occupancy Grid Map for Nav2
# -------------------------------------------------------------
print("Generating matching 2D occupancy grid map...")
RESOLUTION = 0.05  # 5cm per pixel

MAP_MIN_X = -54.0
MAP_MAX_X = 32.0
MAP_MIN_Y = -16.0
MAP_MAX_Y = 15.0

WIDTH_M = MAP_MAX_X - MAP_MIN_X    # 86.0m
HEIGHT_M = MAP_MAX_Y - MAP_MIN_Y  # 31.0m

PIX_W = int(math.ceil(WIDTH_M / RESOLUTION))   # 1720 px
PIX_H = int(math.ceil(HEIGHT_M / RESOLUTION))  # 620 px

print(f"Occupancy Grid Size: {PIX_W} x {PIX_H} pixels at {RESOLUTION}m/px")

# Initialize map: 254 = free space
grid = np.full((PIX_H, PIX_W), 254, dtype=np.uint8)

def world_to_map_px(wx, wy):
    px = int((wx - MAP_MIN_X) / RESOLUTION)
    py = int((MAP_MAX_Y - wy) / RESOLUTION)  # Image row 0 is top (max_y)
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

# Mark solid furniture obstacles (black = 0)
for (cx, cy, sx, sy) in builder.furniture_boxes:
    draw_rect(cx, cy, sx, sy, val=0)

# Save PGM
pgm_path = os.path.join(MAPS_DIR, "office_map.pgm")
map_img = Image.fromarray(grid)
map_img.save(pgm_path)
print(f"Generated {pgm_path}")

# Save YAML (ROS 2 nav2_map_server format)
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
