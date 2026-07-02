"""
geometry_builder.py — Fusion 360 API geometry construction for SpaghettiStructureGen.
Handles: sketch creation, member cylinder bodies, joint spheres, color-coding.

All dimensions arrive in millimetres; Fusion API uses centimetres internally
(1 cm internal = 10 mm input), so values are divided by 10 before passing to API.
"""

from __future__ import annotations
import math
import traceback
from typing import Dict, List, Optional, Tuple

import adsk.core
import adsk.fusion

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SSG_COMP_NAME = 'SSG_TrussStructure'
SSG_SKETCH_NAME = 'SSG_TrussSketch'

# Appearance names — we look these up from Fusion's material library
_APPEARANCE_TENSION_NAME = 'SSG_Tension'
_APPEARANCE_COMPRESSION_NAME = 'SSG_Compression'
_APPEARANCE_ZERO_NAME = 'SSG_Zero'
_APPEARANCE_JOINT_NAME = 'SSG_Joint'

# RGB tuples for force colour coding
_COLOR_TENSION = adsk.core.Color.create(30, 80, 220, 255)       # Blue
_COLOR_COMPRESSION = adsk.core.Color.create(220, 50, 30, 255)   # Red
_COLOR_ZERO = adsk.core.Color.create(180, 180, 180, 255)        # Grey
_COLOR_JOINT = adsk.core.Color.create(255, 200, 0, 255)         # Amber


# ---------------------------------------------------------------------------
# ── Utility helpers ──────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def _mm(v: float) -> float:
    """Convert mm to Fusion internal units (cm)."""
    return v / 10.0


def _get_or_create_component(
    root: adsk.fusion.Component,
    name: str,
) -> adsk.fusion.Component:
    """Return existing component occurrence by name, or create a new one."""
    for occ in root.occurrences:
        if occ.component.name == name:
            return occ.component
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def _get_or_create_appearance(
    design: adsk.fusion.Design,
    name: str,
    color: adsk.core.Color,
) -> Optional[adsk.fusion.Appearance]:
    """Return or create a flat-color appearance in the design."""
    try:
        # Check if it already exists in design appearances
        existing = design.appearances.itemByName(name)
        if existing:
            return existing

        # Find a base appearance to copy from Fusion's library
        app = adsk.core.Application.get()
        lib = app.materialLibraries.itemByName('Fusion 360 Appearance Library')
        if not lib:
            # Try first available library
            for l in app.materialLibraries:
                if 'Appearance' in l.name:
                    lib = l
                    break
        if not lib:
            return None

        # Look for a simple plastic appearance to copy
        base_app = None
        for candidate in ['Paint - Enamel Glossy (Red)', 'Plastic - Matte', 'Generic']:
            base_app = lib.appearances.itemByName(candidate)
            if base_app:
                break
        # Fallback: use first appearance
        if not base_app and lib.appearances.count > 0:
            base_app = lib.appearances.item(0)
        if not base_app:
            return None

        new_app = design.appearances.addByCopy(base_app, name)

        # Override color property
        for prop in new_app.appearanceProperties:
            if prop.objectType == 'adsk::fusion::ColorProperty':
                prop.value = color
                break

        return new_app
    except Exception:
        return None


# ---------------------------------------------------------------------------
# ── Clear previous SSG geometry ──────────────────────────────────────────────
# ---------------------------------------------------------------------------

def clear_ssg_geometry(design: adsk.fusion.Design) -> None:
    """Delete all occurrences named SSG_* from the root component."""
    root = design.rootComponent
    to_delete = []
    for occ in root.occurrences:
        if occ.component.name.startswith('SSG_'):
            to_delete.append(occ)
    for occ in to_delete:
        try:
            occ.deleteMe()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# ── Core geometry builder helpers ────────────────────────────────────────────
# ---------------------------------------------------------------------------

def _build_cylinder_body(
    comp: adsk.fusion.Component,
    p1_mm: Tuple[float, float, float],
    p2_mm: Tuple[float, float, float],
    dia_mm: float,
) -> Optional[adsk.fusion.BRepBody]:
    """
    Create a cylinder from p1 to p2 with given diameter, inside comp.
    Strategy: create a circle sketch perpendicular to the member axis at p1,
    then extrude to p2 length.
    """
    try:
        sketches = comp.sketches
        planes = comp.constructionPlanes

        x1, y1, z1 = [_mm(v) for v in p1_mm]
        x2, y2, z2 = [_mm(v) for v in p2_mm]

        # Member axis vector
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-9:
            return None

        # -- Build a construction plane at p1, normal to member axis --
        plane_input = planes.createInput()

        origin = adsk.core.Point3D.create(x1, y1, z1)
        axis_vec = adsk.core.Vector3D.create(dx / length, dy / length, dz / length)

        # Find a perpendicular direction to construct the plane
        ref_vec = adsk.core.Vector3D.create(1, 0, 0)
        if abs(axis_vec.dotProduct(ref_vec)) > 0.9:
            ref_vec = adsk.core.Vector3D.create(0, 1, 0)
        x_dir = axis_vec.crossProduct(ref_vec)
        x_dir.normalize()
        y_dir = axis_vec.crossProduct(x_dir)
        y_dir.normalize()

        plane_input.setByThreePoints(
            origin,
            adsk.core.Point3D.create(
                x1 + x_dir.x, y1 + x_dir.y, z1 + x_dir.z
            ),
            adsk.core.Point3D.create(
                x1 + y_dir.x, y1 + y_dir.y, z1 + y_dir.z
            ),
        )
        plane = planes.add(plane_input)

        # -- Sketch a circle on that plane --
        sk = sketches.add(plane)
        center2d = sk.modelToSketchSpace(origin)
        r_cm = _mm(dia_mm) / 2.0
        sk.sketchCurves.sketchCircles.addByCenterRadius(center2d, r_cm)

        # -- Extrude along member axis --
        prof = sk.profiles.item(0)
        extrudes = comp.features.extrudeFeatures
        ext_input = extrudes.createInput(
            prof,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        dist_input = adsk.fusion.DistanceExtentDefinition.create(
            adsk.core.ValueInput.createByReal(length)
        )
        ext_input.setOneSideExtent(dist_input, adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ext_input.creationOccurrence = comp.parentDesign.rootComponent.occurrences.item(0) if False else None

        # Direction: the extrude goes in sketch's Z (which is our axis_vec)
        body = extrudes.add(ext_input).bodies.item(0)
        return body
    except Exception:
        return None


def _build_sphere_body(
    comp: adsk.fusion.Component,
    center_mm: Tuple[float, float, float],
    radius_mm: float,
) -> Optional[adsk.fusion.BRepBody]:
    """Create a sphere body at center_mm with given radius."""
    try:
        cx, cy, cz = [_mm(v) for v in center_mm]
        r_cm = _mm(radius_mm)

        # Revolve a semicircle on a convenient plane
        # Use XZ plane (Y=0) passing through center
        sketches = comp.sketches
        xy_plane = comp.xZConstructionPlane

        sk = sketches.add(xy_plane)
        sk.isComputeDeferred = True

        # Semicircle points in local sketch coordinates
        # Sketch is on XZ plane; center at (cx, cz) in sketch space
        # We'll draw in absolute sketch space, translated
        center_sk = adsk.core.Point3D.create(cx, cz, 0)

        # Full circle, then revolve 360 → sphere
        circle = sk.sketchCurves.sketchCircles.addByCenterRadius(center_sk, r_cm)

        # Draw axis line for revolve (vertical line through center)
        line_start = adsk.core.Point3D.create(cx, cz - r_cm * 2, 0)
        line_end   = adsk.core.Point3D.create(cx, cz + r_cm * 2, 0)
        axis_line  = sk.sketchCurves.sketchLines.addByTwoPoints(line_start, line_end)

        sk.isComputeDeferred = False

        if sk.profiles.count == 0:
            return None
        prof = sk.profiles.item(0)

        revolves = comp.features.revolveFeatures
        rev_input = revolves.createInput(
            prof,
            axis_line,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        rev_input.setAngleExtent(
            False,
            adsk.core.ValueInput.createByString('360 deg'),
        )
        return revolves.add(rev_input).bodies.item(0)
    except Exception:
        return None


def _apply_appearance(
    body: adsk.fusion.BRepBody,
    appearance: Optional[adsk.fusion.Appearance],
) -> None:
    if appearance and body:
        try:
            body.appearance = appearance
        except Exception:
            pass


# ---------------------------------------------------------------------------
# ── Main build functions ──────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def build_bridge(
    design: adsk.fusion.Design,
    nodes: list,
    members: list,
    forces: Dict[int, float],
    bundle_dia_mm: float,
    deck_width_mm: float,
    joint_radius_mm: float = 3.0,
) -> None:
    """
    Build the full bridge geometry:
    - Two parallel trusses mirrored at deck_width_mm
    - Deck cross-members at each panel point
    - Joint spheres at each node
    """
    root = design.rootComponent

    # Prepare appearances
    app_tension    = _get_or_create_appearance(design, _APPEARANCE_TENSION_NAME,    _COLOR_TENSION)
    app_compress   = _get_or_create_appearance(design, _APPEARANCE_COMPRESSION_NAME, _COLOR_COMPRESSION)
    app_zero       = _get_or_create_appearance(design, _APPEARANCE_ZERO_NAME,       _COLOR_ZERO)
    app_joint      = _get_or_create_appearance(design, _APPEARANCE_JOINT_NAME,      _COLOR_JOINT)

    def pick_appearance(force_n: float):
        if force_n > 0.5:
            return app_tension
        elif force_n < -0.5:
            return app_compress
        return app_zero

    # -- Truss A (y=0) --
    comp_a = _get_or_create_component(root, 'SSG_TrussA')
    _build_truss_members(comp_a, nodes, members, forces, bundle_dia_mm, pick_appearance)
    _build_joints(comp_a, nodes, joint_radius_mm, app_joint)

    # -- Truss B (y = deck_width_mm) — shift all nodes in Y --
    nodes_b = [(x, y + deck_width_mm, z) for (x, y, z) in nodes]
    comp_b = _get_or_create_component(root, 'SSG_TrussB')
    _build_truss_members(comp_b, nodes_b, members, forces, bundle_dia_mm, pick_appearance)
    _build_joints(comp_b, nodes_b, joint_radius_mm, app_joint)

    # -- Deck cross-members (connecting bottom chords of truss A and B) --
    comp_deck = _get_or_create_component(root, 'SSG_Deck')
    bottom_nodes = [i for i, (x, y, z) in enumerate(nodes) if z == 0.0]
    for idx in bottom_nodes:
        p1 = nodes[idx]
        p2 = nodes_b[idx]
        body = _build_cylinder_body(comp_deck, p1, p2, bundle_dia_mm * 0.8)
        _apply_appearance(body, app_zero)


def build_tower(
    design: adsk.fusion.Design,
    nodes: list,
    members: list,
    forces: Dict[int, float],
    bundle_dia_mm: float,
    joint_radius_mm: float = 3.0,
) -> None:
    """Build the full tower geometry (3D)."""
    root = design.rootComponent

    app_tension    = _get_or_create_appearance(design, _APPEARANCE_TENSION_NAME,    _COLOR_TENSION)
    app_compress   = _get_or_create_appearance(design, _APPEARANCE_COMPRESSION_NAME, _COLOR_COMPRESSION)
    app_zero       = _get_or_create_appearance(design, _APPEARANCE_ZERO_NAME,       _COLOR_ZERO)
    app_joint      = _get_or_create_appearance(design, _APPEARANCE_JOINT_NAME,      _COLOR_JOINT)

    def pick_appearance(force_n: float):
        if force_n > 0.5:
            return app_tension
        elif force_n < -0.5:
            return app_compress
        return app_zero

    comp = _get_or_create_component(root, 'SSG_Tower')
    _build_truss_members(comp, nodes, members, forces, bundle_dia_mm, pick_appearance)
    _build_joints(comp, nodes, joint_radius_mm, app_joint)


def _build_truss_members(
    comp: adsk.fusion.Component,
    nodes: list,
    members: list,
    forces: Dict[int, float],
    bundle_dia_mm: float,
    pick_appearance,
) -> None:
    """Build all member cylinder bodies inside comp."""
    for mid, (i, j, mtype) in enumerate(members):
        body = _build_cylinder_body(comp, nodes[i], nodes[j], bundle_dia_mm)
        if body:
            f = forces.get(mid, 0.0)
            _apply_appearance(body, pick_appearance(f))


def _build_joints(
    comp: adsk.fusion.Component,
    nodes: list,
    radius_mm: float,
    appearance: Optional[adsk.fusion.Appearance],
) -> None:
    """Build small sphere bodies at each node."""
    for node in nodes:
        body = _build_sphere_body(comp, node, radius_mm)
        _apply_appearance(body, appearance)


# ---------------------------------------------------------------------------
# ── 2D Sketch overview (lightweight wire-frame for InputChanged preview) ─────
# ---------------------------------------------------------------------------

def build_sketch_preview(
    design: adsk.fusion.Design,
    nodes: list,
    members: list,
    mode: str,
) -> None:
    """
    Draw a lightweight sketch wire-frame for fast InputChanged preview.
    This is faster than regenerating full solid geometry each keystroke.
    Members are drawn as sketch lines on the appropriate plane.
    """
    root = design.rootComponent
    sketch_name = SSG_SKETCH_NAME + '_Preview'

    # Remove old preview sketch
    for sk in root.sketches:
        if sk.name == sketch_name:
            sk.deleteMe()
            break

    if mode == 'Bridge':
        plane = root.xZConstructionPlane
    else:
        plane = root.xYConstructionPlane

    sk = root.sketches.add(plane)
    sk.name = sketch_name
    sk.isComputeDeferred = True

    lines = sk.sketchCurves.sketchLines

    for i, j, _ in members:
        nx, ny, nz = nodes[i]
        mx, my, mz = nodes[j]
        if mode == 'Bridge':
            p1 = adsk.core.Point3D.create(_mm(nx), _mm(nz), 0)
            p2 = adsk.core.Point3D.create(_mm(mx), _mm(mz), 0)
        else:
            p1 = adsk.core.Point3D.create(_mm(nx), _mm(ny), 0)
            p2 = adsk.core.Point3D.create(_mm(mx), _mm(my), 0)
        try:
            lines.addByTwoPoints(p1, p2)
        except Exception:
            pass

    sk.isComputeDeferred = False
