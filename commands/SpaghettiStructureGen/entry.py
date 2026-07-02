"""
entry.py — SpaghettiStructureGen command: dialog, event handlers.

Handler flow:
  command_created   → builds all dialog inputs, registers sub-handlers
  command_input_changed → recomputes truss + BOM, redraws sketch preview
  command_preview   → marks result valid (geometry built in inputChanged)
  command_execute   → builds full solid geometry, optionally exports CSV
  command_destroy   → clears local_handlers

All event handlers are wrapped in try/except per Fusion add-in best practice.
"""

from __future__ import annotations
import math
import os
import traceback

import adsk.core
import adsk.fusion

from ...lib import fusionAddInUtils as futil
from ... import config

# Pure-Python math modules (no Fusion API)
from ...truss_math import (
    warren_nodes, pratt_nodes, howe_nodes, k_truss_nodes,
    tower_nodes,
    solve_2d, solve_tower_approximate,
    compute_bom, bom_summary, export_csv,
    bundle_strand_count,
)
from ...geometry_builder import (
    clear_ssg_geometry,
    build_bridge, build_tower,
    build_sketch_preview,
)

# ---------------------------------------------------------------------------
# Command identity
# ---------------------------------------------------------------------------
CMD_ID          = f'{config.COMPANY_NAME}_SpaghettiStructureGen_cmd'
CMD_NAME        = 'Spaghetti Structure Gen'
CMD_Description = ('Parametrically generates truss geometry for spaghetti '
                   'bridge/tower engineering competitions.')
IS_PROMOTED     = True

WORKSPACE_ID        = 'FusionSolidEnvironment'
PANEL_ID            = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID   = 'ScriptsManagerCommand'

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

# ---------------------------------------------------------------------------
# Competition defaults (edit to match your competition rules)
# ---------------------------------------------------------------------------
COMP_SPAN_MIN_MM        = 100.0
COMP_SPAN_MAX_MM        = 600.0
COMP_HEIGHT_MIN_MM      = 30.0
COMP_HEIGHT_MAX_MM      = 200.0
COMP_TOWER_HEIGHT_MAX_MM= 500.0
COMP_WEIGHT_LIMIT_G     = 20.0   # grams
COMP_LOAD_N             = 150.0  # Newtons (≈ 15 kg)

local_handlers: list = []

# ---------------------------------------------------------------------------
# Module-level state shared between handlers
# ---------------------------------------------------------------------------
_current_nodes:   list = []
_current_members: list = []
_current_forces:  dict = {}
_current_bom:     list = []


# ---------------------------------------------------------------------------
# Add-in lifecycle
# ---------------------------------------------------------------------------

def start() -> None:
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER
        )
        futil.add_handler(cmd_def.commandCreated, command_created)

        workspace = ui.workspaces.itemById(WORKSPACE_ID)
        panel     = workspace.toolbarPanels.itemById(PANEL_ID)
        control   = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
        control.isPromoted = IS_PROMOTED

        futil.log(f'{CMD_NAME} started.')
    except Exception:
        ui.messageBox(f'SpaghettiStructureGen start error:\n{traceback.format_exc()}')


def stop() -> None:
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        workspace  = ui.workspaces.itemById(WORKSPACE_ID)
        panel      = workspace.toolbarPanels.itemById(PANEL_ID)
        ctrl       = panel.controls.itemById(CMD_ID)
        cmd_def    = ui.commandDefinitions.itemById(CMD_ID)
        if ctrl:    ctrl.deleteMe()
        if cmd_def: cmd_def.deleteMe()
    except Exception:
        ui.messageBox(f'SpaghettiStructureGen stop error:\n{traceback.format_exc()}')


# ---------------------------------------------------------------------------
# CommandCreated — build dialog
# ---------------------------------------------------------------------------

def command_created(args: adsk.core.CommandCreatedEventArgs) -> None:
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        futil.log(f'{CMD_NAME} CommandCreated')

        cmd    = args.command
        inputs = cmd.commandInputs

        # ── MODE ────────────────────────────────────────────────────────────
        mode_dd = inputs.addDropDownCommandInput(
            'mode', 'Structure Mode',
            adsk.core.DropDownStyles.TextListDropDownStyle,
        )
        mode_dd.listItems.add('Bridge', True)
        mode_dd.listItems.add('Tower',  False)

        # ── BRIDGE GROUP ────────────────────────────────────────────────────
        bg = inputs.addGroupCommandInput('bridge_group', 'Bridge Parameters')
        bi = bg.children

        bi.addValueInput(
            'span', 'Span Length (mm)', 'mm',
            adsk.core.ValueInput.createByReal(40.0),   # 400mm → 40cm
        )
        bi.addValueInput(
            'truss_height', 'Truss Height (mm)', 'mm',
            adsk.core.ValueInput.createByReal(8.0),    # 80mm
        )
        pattern_dd = bi.addDropDownCommandInput(
            'truss_pattern', 'Truss Pattern',
            adsk.core.DropDownStyles.TextListDropDownStyle,
        )
        pattern_dd.listItems.add('Warren', True)
        pattern_dd.listItems.add('Pratt',  False)
        pattern_dd.listItems.add('Howe',   False)
        pattern_dd.listItems.add('K-Truss', False)

        panels_si = bi.addIntegerSliderCommandInput(
            'num_panels', 'Number of Panels', 2, 16
        )
        panels_si.valueOne = 6

        bi.addValueInput(
            'deck_width', 'Deck Width (mm)', 'mm',
            adsk.core.ValueInput.createByReal(6.0),    # 60mm
        )

        # ── TOWER GROUP ──────────────────────────────────────────────────────
        tg = inputs.addGroupCommandInput('tower_group', 'Tower Parameters')
        ti = tg.children

        ti.addValueInput(
            'base_width', 'Base Width (mm)', 'mm',
            adsk.core.ValueInput.createByReal(10.0),   # 100mm
        )
        ti.addValueInput(
            'top_width', 'Top Width (mm)', 'mm',
            adsk.core.ValueInput.createByReal(4.0),    # 40mm
        )
        ti.addValueInput(
            'tower_height', 'Tower Height (mm)', 'mm',
            adsk.core.ValueInput.createByReal(30.0),   # 300mm
        )
        segs_si = ti.addIntegerSliderCommandInput(
            'num_segments', 'Vertical Segments', 3, 12
        )
        segs_si.valueOne = 5

        xsec_dd = ti.addDropDownCommandInput(
            'cross_section', 'Cross-Section',
            adsk.core.DropDownStyles.TextListDropDownStyle,
        )
        xsec_dd.listItems.add('Square',     True)
        xsec_dd.listItems.add('Triangular', False)

        brace_dd = ti.addDropDownCommandInput(
            'bracing', 'Bracing Pattern',
            adsk.core.DropDownStyles.TextListDropDownStyle,
        )
        brace_dd.listItems.add('X-Brace',  True)
        brace_dd.listItems.add('K-Brace',  False)
        brace_dd.listItems.add('Diagonal', False)

        # ── MEMBER PROPERTIES (shared) ──────────────────────────────────────
        mg = inputs.addGroupCommandInput('member_group', 'Member Properties')
        mi = mg.children

        mi.addValueInput(
            'bundle_dia', 'Bundle Diameter (mm)', 'mm',
            adsk.core.ValueInput.createByReal(0.5),    # 5mm
        )
        mi.addValueInput(
            'strand_dia', 'Strand Diameter (mm)', 'mm',
            adsk.core.ValueInput.createByReal(0.17),   # 1.7mm
        )
        mi.addValueInput(
            'strand_len', 'Max Strand Length (mm)', 'mm',
            adsk.core.ValueInput.createByReal(25.0),   # 250mm
        )

        # ── LOAD & ANALYSIS ─────────────────────────────────────────────────
        ag = inputs.addGroupCommandInput('analysis_group', 'Load & Analysis')
        ai = ag.children

        ai.addValueInput(
            'load_magnitude', 'Applied Load (N)', '',
            adsk.core.ValueInput.createByReal(COMP_LOAD_N),
        )
        load_node_si = ai.addIntegerSliderCommandInput(
            'load_node', 'Load Node Index', 0, 20
        )
        load_node_si.valueOne = 0   # will be updated dynamically

        # ── COMPETITION LIMITS ───────────────────────────────────────────────
        cg = inputs.addGroupCommandInput('comp_group', 'Competition Limits')
        ci = cg.children

        ci.addValueInput(
            'weight_limit', 'Weight Limit (g)', '',
            adsk.core.ValueInput.createByReal(COMP_WEIGHT_LIMIT_G),
        )

        # ── BOM SUMMARY (read-only) ──────────────────────────────────────────
        inputs.addTextBoxCommandInput(
            'bom_summary', 'BOM Summary',
            '<i>Configure inputs to see BOM estimate.</i>',
            6, True,   # 6 rows, read-only
        )

        # ── EXPORT ──────────────────────────────────────────────────────────
        inputs.addBoolValueInput(
            'export_csv', 'Export CSV Cut List', True, '', False
        )

        # Initial mode visibility
        _update_visibility(inputs, 'Bridge')

        # Register sub-handlers
        futil.add_handler(cmd.execute,       command_execute,       local_handlers=local_handlers)
        futil.add_handler(cmd.inputChanged,  command_input_changed, local_handlers=local_handlers)
        futil.add_handler(cmd.executePreview,command_preview,       local_handlers=local_handlers)
        futil.add_handler(cmd.validateInputs,command_validate,      local_handlers=local_handlers)
        futil.add_handler(cmd.destroy,       command_destroy,       local_handlers=local_handlers)

    except Exception:
        ui.messageBox(f'{CMD_NAME} CommandCreated error:\n{traceback.format_exc()}')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _update_visibility(inputs: adsk.core.CommandInputs, mode: str) -> None:
    """Show/hide Bridge or Tower parameter groups."""
    bg = inputs.itemById('bridge_group')
    tg = inputs.itemById('tower_group')
    if bg: bg.isVisible = (mode == 'Bridge')
    if tg: tg.isVisible = (mode == 'Tower')


def _get_str(inputs: adsk.core.CommandInputs, id_: str) -> str:
    item = inputs.itemById(id_)
    if item and hasattr(item, 'selectedItem'):
        return item.selectedItem.name
    return ''


def _get_val_cm(inputs: adsk.core.CommandInputs, id_: str) -> float:
    """Return value in cm (Fusion internal) from a ValueCommandInput."""
    item = inputs.itemById(id_)
    if item:
        return item.value   # already in cm
    return 0.0


def _get_val_mm(inputs: adsk.core.CommandInputs, id_: str) -> float:
    """Return value converted from cm to mm."""
    return _get_val_cm(inputs, id_) * 10.0


def _get_slider(inputs: adsk.core.CommandInputs, id_: str) -> int:
    item = inputs.itemById(id_)
    if item:
        return int(item.valueOne)
    return 0


def _recompute(inputs: adsk.core.CommandInputs) -> tuple:
    """
    Recompute nodes, members, forces and BOM from current dialog values.
    Returns (nodes, members, forces, bom, summary, mode).
    """
    global _current_nodes, _current_members, _current_forces, _current_bom

    mode = _get_str(inputs, 'mode')

    bundle_dia = _get_val_mm(inputs, 'bundle_dia')
    strand_dia = _get_val_mm(inputs, 'strand_dia')
    strand_len = _get_val_mm(inputs, 'strand_len')
    load_mag   = _get_val_cm(inputs, 'load_magnitude')  # N (dimensionless in Fusion)
    load_node_idx = _get_slider(inputs, 'load_node')

    if mode == 'Bridge':
        span          = _get_val_mm(inputs, 'span')
        truss_h       = _get_val_mm(inputs, 'truss_height')
        n_panels      = _get_slider(inputs, 'num_panels')
        pattern       = _get_str(inputs, 'truss_pattern')

        pattern_map = {
            'Warren':  warren_nodes,
            'Pratt':   pratt_nodes,
            'Howe':    howe_nodes,
            'K-Truss': k_truss_nodes,
        }
        gen_fn = pattern_map.get(pattern, warren_nodes)
        nodes, members = gen_fn(
            max(span, 10.0),
            max(truss_h, 5.0),
            max(n_panels, 2),
        )

        # Clamp load node
        load_node_idx = min(load_node_idx, len(nodes) - 1)

        # Supports: pin at node 0, roller at node n_panels (bottom chord)
        n_bot = n_panels + 1
        supports = [(0, 'pin'), (n_panels, 'roller')]
        loads    = [(load_node_idx, 0.0, -abs(load_mag))]
        forces   = solve_2d(nodes, members, supports, loads)

    else:  # Tower
        base_w  = _get_val_mm(inputs, 'base_width')
        top_w   = _get_val_mm(inputs, 'top_width')
        t_height= _get_val_mm(inputs, 'tower_height')
        n_segs  = _get_slider(inputs, 'num_segments')
        xsec    = _get_str(inputs, 'cross_section').lower()
        bracing = _get_str(inputs, 'bracing').replace('-', '_').lower()

        xsec_map    = {'square': 'square', 'triangular': 'triangular'}
        bracing_map = {'x_brace': 'x_brace', 'k_brace': 'k_brace', 'diagonal': 'diagonal'}

        nodes, members = tower_nodes(
            max(base_w, 20.0),
            max(top_w, 10.0),
            max(t_height, 50.0),
            max(n_segs, 2),
            xsec_map.get(xsec, 'square'),
            bracing_map.get(bracing, 'x_brace'),
        )

        load_node_idx = min(load_node_idx, len(nodes) - 1)
        n_corners = 4 if 'square' in xsec else 3
        base_nodes = list(range(n_corners))
        forces = solve_tower_approximate(
            nodes, members, load_node_idx, -abs(load_mag), base_nodes
        )

    # BOM
    bom  = compute_bom(nodes, members, bundle_dia, strand_dia, strand_len)
    summ = bom_summary(bom)

    _current_nodes   = nodes
    _current_members = members
    _current_forces  = forces
    _current_bom     = bom

    return nodes, members, forces, bom, summ, mode


def _format_bom_html(summ: dict, weight_limit_g: float) -> str:
    """Format BOM summary as HTML for the text box."""
    total_strands = summ['total_strands']
    total_mass    = summ['total_mass_g']
    over_limit    = total_mass > weight_limit_g

    color = '#cc0000' if over_limit else '#007a00'
    status = f'<b style="color:{color};">{"⚠ OVER LIMIT" if over_limit else "✓ WITHIN LIMIT"}</b>'

    return (
        f'<b>Total Strands:</b> {total_strands}<br>'
        f'<b>Total Mass:</b> {total_mass:.2f} g<br>'
        f'<b>Weight Limit:</b> {weight_limit_g:.1f} g<br>'
        f'Status: {status}'
    )


# ---------------------------------------------------------------------------
# InputChanged
# ---------------------------------------------------------------------------

def command_input_changed(args: adsk.core.InputChangedEventArgs) -> None:
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        inputs       = args.inputs
        changed_id   = args.input.id
        futil.log(f'{CMD_NAME} InputChanged: {changed_id}')

        # Update mode visibility
        mode = _get_str(inputs, 'mode')
        _update_visibility(inputs, mode)

        # Recompute truss
        nodes, members, forces, bom, summ, mode = _recompute(inputs)

        # Update load_node slider max
        load_node_si = inputs.itemById('load_node')
        if load_node_si and len(nodes) > 1:
            load_node_si.maximumValue = len(nodes) - 1

        # Update BOM text box
        weight_limit = _get_val_cm(inputs, 'weight_limit')   # unitless N → treat as grams
        bom_tb = inputs.itemById('bom_summary')
        if bom_tb:
            bom_tb.formattedText = _format_bom_html(summ, weight_limit)

        # Draw sketch preview (fast wire-frame)
        design = app.activeProduct
        if isinstance(design, adsk.fusion.Design):
            build_sketch_preview(design, nodes, members, mode)

    except Exception:
        app.userInterface.messageBox(
            f'{CMD_NAME} InputChanged error:\n{traceback.format_exc()}'
        )


# ---------------------------------------------------------------------------
# ExecutePreview
# ---------------------------------------------------------------------------

def command_preview(args: adsk.core.CommandEventArgs) -> None:
    app = adsk.core.Application.get()
    try:
        futil.log(f'{CMD_NAME} ExecutePreview')
        # Sketch preview already drawn in inputChanged; mark valid.
        args.isValidResult = True
    except Exception:
        app.userInterface.messageBox(
            f'{CMD_NAME} Preview error:\n{traceback.format_exc()}'
        )


# ---------------------------------------------------------------------------
# Execute (OK button)
# ---------------------------------------------------------------------------

def command_execute(args: adsk.core.CommandEventArgs) -> None:
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        futil.log(f'{CMD_NAME} Execute')

        inputs = args.command.commandInputs
        nodes, members, forces, bom, summ, mode = _recompute(inputs)

        bundle_dia = _get_val_mm(inputs, 'bundle_dia')

        design = app.activeProduct
        if not isinstance(design, adsk.fusion.Design):
            ui.messageBox('Please open a Fusion 360 design before running this command.')
            return

        # Clear old geometry
        clear_ssg_geometry(design)

        # Remove old preview sketch
        root = design.rootComponent
        for sk in root.sketches:
            if 'SSG_TrussSketch' in sk.name:
                sk.deleteMe()

        # Build solid geometry
        if mode == 'Bridge':
            deck_width = _get_val_mm(inputs, 'deck_width')
            build_bridge(design, nodes, members, forces, bundle_dia, deck_width)
        else:
            build_tower(design, nodes, members, forces, bundle_dia)

        # Export CSV if requested
        export_flag: adsk.core.BoolValueCommandInput = inputs.itemById('export_csv')
        if export_flag and export_flag.value:
            downloads = os.path.join(os.path.expanduser('~'), 'Downloads')
            csv_path  = os.path.join(downloads, 'SpaghettiStructureGen_CutList.csv')
            export_csv(bom, forces, csv_path)
            ui.messageBox(f'Cut list exported to:\n{csv_path}')

        ui.messageBox(
            f'✓ {mode} geometry generated!\n\n'
            f'Members: {len(members)}\n'
            f'Total strands: {summ["total_strands"]}\n'
            f'Estimated mass: {summ["total_mass_g"]:.2f} g'
        )

    except Exception:
        app.userInterface.messageBox(
            f'{CMD_NAME} Execute error:\n{traceback.format_exc()}'
        )


# ---------------------------------------------------------------------------
# ValidateInputs
# ---------------------------------------------------------------------------

def command_validate(args: adsk.core.ValidateInputsEventArgs) -> None:
    app = adsk.core.Application.get()
    try:
        inputs = args.inputs
        mode   = _get_str(inputs, 'mode')
        ok     = True

        if mode == 'Bridge':
            span = _get_val_mm(inputs, 'span')
            h    = _get_val_mm(inputs, 'truss_height')
            if span < 10 or h < 5:
                ok = False
        else:
            bw = _get_val_mm(inputs, 'base_width')
            th = _get_val_mm(inputs, 'tower_height')
            if bw < 10 or th < 20:
                ok = False

        bd = _get_val_mm(inputs, 'bundle_dia')
        if bd <= 0:
            ok = False

        args.areInputsValid = ok
    except Exception:
        app.userInterface.messageBox(
            f'{CMD_NAME} Validate error:\n{traceback.format_exc()}'
        )


# ---------------------------------------------------------------------------
# Destroy
# ---------------------------------------------------------------------------

def command_destroy(args: adsk.core.CommandEventArgs) -> None:
    try:
        futil.log(f'{CMD_NAME} Destroy')
        global local_handlers
        local_handlers = []
    except Exception:
        adsk.core.Application.get().userInterface.messageBox(
            f'{CMD_NAME} Destroy error:\n{traceback.format_exc()}'
        )
