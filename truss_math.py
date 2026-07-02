"""
truss_math.py — Pure Python truss mathematics for SpaghettiStructureGen.
No Fusion API imports — fully unit-testable outside Fusion 360.

Coordinate convention (matches Fusion XZ plane bridge layout):
  Bridge: nodes are (x, 0, z)  — x along span, z vertical
  Tower:  nodes are (x, y, z)  — x/y plan, z vertical
All dimensions in millimetres unless noted.

Member force sign convention:
  Positive (+) = tension
  Negative (−) = compression
"""

from __future__ import annotations
import math
import os
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------
Node2D = Tuple[float, float, float]   # (x, 0, z)
Node3D = Tuple[float, float, float]   # (x, y, z)
Member = Tuple[int, int, str]          # (i, j, type_label)
TrussData = Tuple[List[Node3D], List[Member]]

# ---------------------------------------------------------------------------
# ── BRIDGE NODE GENERATORS ──────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def _bridge_base(span: float, height: float, n: int) -> Tuple[List[Node3D], List[Node3D]]:
    """Return (bottom_nodes, top_nodes) for a flat-chord parallel truss."""
    dx = span / n
    bot = [(i * dx, 0.0, 0.0) for i in range(n + 1)]
    top = [(i * dx, 0.0, height) for i in range(n + 1)]
    return bot, top


def warren_nodes(span: float, height: float, n_panels: int) -> TrussData:
    """Warren truss — alternating diagonals, no verticals (classic zigzag)."""
    n = n_panels
    bot, top = _bridge_base(span, height, n)
    nodes: List[Node3D] = bot + top
    # Index mapping: bot[i]=i, top[i]=n+1+i
    nb = n + 1
    members: List[Member] = []
    # Bottom chord
    for i in range(n):
        members.append((i, i + 1, 'bottom_chord'))
    # Top chord
    for i in range(n):
        members.append((nb + i, nb + i + 1, 'top_chord'))
    # Diagonals — alternating
    for i in range(n):
        if i % 2 == 0:
            members.append((i, nb + i + 1, 'diagonal'))      # bottom-left to top-right
        else:
            members.append((nb + i, i + 1, 'diagonal'))      # top-left to bottom-right
    return nodes, members


def pratt_nodes(span: float, height: float, n_panels: int) -> TrussData:
    """Pratt truss — verticals + diagonals inclined toward centre (tension diagonals)."""
    n = n_panels
    bot, top = _bridge_base(span, height, n)
    nodes: List[Node3D] = bot + top
    nb = n + 1
    members: List[Member] = []
    for i in range(n):
        members.append((i, i + 1, 'bottom_chord'))
    for i in range(n):
        members.append((nb + i, nb + i + 1, 'top_chord'))
    # Verticals at every internal node
    for i in range(1, n):
        members.append((i, nb + i, 'vertical'))
    # End verticals
    members.append((0, nb, 'vertical'))
    members.append((n, nb + n, 'vertical'))
    # Diagonals — lean toward centre (Pratt: diagonals in tension under gravity)
    mid = n // 2
    for i in range(n):
        if i < mid:
            members.append((i, nb + i + 1, 'diagonal'))   # left half: bot-left→top-right
        else:
            members.append((nb + i, i + 1, 'diagonal'))   # right half: top-left→bot-right
    return nodes, members


def howe_nodes(span: float, height: float, n_panels: int) -> TrussData:
    """Howe truss — verticals + diagonals inclined outward from centre (compression diagonals)."""
    n = n_panels
    bot, top = _bridge_base(span, height, n)
    nodes: List[Node3D] = bot + top
    nb = n + 1
    members: List[Member] = []
    for i in range(n):
        members.append((i, i + 1, 'bottom_chord'))
    for i in range(n):
        members.append((nb + i, nb + i + 1, 'top_chord'))
    for i in range(1, n):
        members.append((i, nb + i, 'vertical'))
    members.append((0, nb, 'vertical'))
    members.append((n, nb + n, 'vertical'))
    # Diagonals opposite to Pratt
    mid = n // 2
    for i in range(n):
        if i < mid:
            members.append((nb + i, i + 1, 'diagonal'))   # top-left→bot-right
        else:
            members.append((i, nb + i + 1, 'diagonal'))   # bot-left→top-right
    return nodes, members


def k_truss_nodes(span: float, height: float, n_panels: int) -> TrussData:
    """K-truss — each panel has a mid-height node with two diagonals forming a K."""
    n = n_panels
    bot, top = _bridge_base(span, height, n)
    nodes: List[Node3D] = bot + top
    # Add mid-height nodes at each internal vertical
    mid_start = len(nodes)
    dx = span / n
    for i in range(1, n):
        nodes.append((i * dx, 0.0, height / 2))
    nb = n + 1
    members: List[Member] = []
    for i in range(n):
        members.append((i, i + 1, 'bottom_chord'))
    for i in range(n):
        members.append((nb + i, nb + i + 1, 'top_chord'))
    # End verticals
    members.append((0, nb, 'vertical'))
    members.append((n, nb + n, 'vertical'))
    # K-bracing per panel (except outer panels)
    for i in range(1, n):
        m = mid_start + (i - 1)   # mid node index for vertical i
        # Vertical from bottom to mid
        members.append((i, m, 'vertical'))
        # Vertical from mid to top
        members.append((m, nb + i, 'vertical'))
        # Diagonals from mid toward adjacent panel
        if i > 0:
            members.append((i - 1, m, 'diagonal'))
            members.append((nb + i - 1, m, 'diagonal'))
    return nodes, members


# ---------------------------------------------------------------------------
# ── TOWER NODE GENERATOR ────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def _interpolate(v0: float, v1: float, t: float) -> float:
    return v0 + (v1 - v0) * t


def tower_nodes(
    base_width: float,
    top_width: float,
    height: float,
    n_segs: int,
    cross_section: str = 'square',   # 'square' | 'triangular'
    bracing: str = 'x_brace',        # 'x_brace' | 'k_brace' | 'diagonal'
) -> TrussData:
    """
    Generate a 3D tower node grid.

    cross_section='square'    → 4-corner column with square cross-section
    cross_section='triangular'→ 3-corner column (equilateral triangle plan)
    bracing applies to every face between adjacent levels.
    """
    nodes: List[Node3D] = []
    members: List[Member] = []

    # --- Plan-view corners per level ---
    if cross_section == 'triangular':
        def level_corners(w: float) -> List[Tuple[float, float]]:
            r = w / math.sqrt(3)
            return [
                (r * math.cos(math.radians(90 + 120 * k)),
                 r * math.sin(math.radians(90 + 120 * k)))
                for k in range(3)
            ]
        n_corners = 3
    else:  # square
        def level_corners(w: float) -> List[Tuple[float, float]]:
            h = w / 2
            return [(-h, -h), (h, -h), (h, h), (-h, h)]
        n_corners = 4

    # Node index helper: level l, corner c → index
    def ni(l: int, c: int) -> int:
        return l * n_corners + c

    # Build node grid
    for l in range(n_segs + 1):
        t = l / n_segs
        z = _interpolate(0.0, height, t)
        w = _interpolate(base_width, top_width, t)
        for (px, py) in level_corners(w):
            nodes.append((px, py, z))

    # Uprights (vertical columns)
    for l in range(n_segs):
        for c in range(n_corners):
            members.append((ni(l, c), ni(l + 1, c), 'upright'))

    # Horizontal rings at each level
    for l in range(n_segs + 1):
        for c in range(n_corners):
            members.append((ni(l, c), ni(l, (c + 1) % n_corners), 'horizontal'))

    # Face diagonals/bracing
    for l in range(n_segs):
        for c in range(n_corners):
            c_next = (c + 1) % n_corners
            a = ni(l, c)
            b = ni(l, c_next)
            a2 = ni(l + 1, c)
            b2 = ni(l + 1, c_next)

            if bracing == 'x_brace':
                members.append((a, b2, 'diagonal'))
                members.append((b, a2, 'diagonal'))
            elif bracing == 'k_brace':
                # Midpoint of upright at c, add extra node
                mid_z = (nodes[a][2] + nodes[a2][2]) / 2
                mid_x = (nodes[a][0] + nodes[a2][0]) / 2
                mid_y = (nodes[a][1] + nodes[a2][1]) / 2
                mid_idx = len(nodes)
                nodes.append((mid_x, mid_y, mid_z))
                members.append((a, mid_idx, 'k_vertical'))
                members.append((mid_idx, a2, 'k_vertical'))
                members.append((b, mid_idx, 'diagonal'))
                members.append((b2, mid_idx, 'diagonal'))
            else:  # diagonal
                members.append((a, b2, 'diagonal'))

    return nodes, members


# ---------------------------------------------------------------------------
# ── METHOD OF JOINTS — 2D PLANAR SOLVER ─────────────────────────────────────
# ---------------------------------------------------------------------------

def _dist(a: Node3D, b: Node3D) -> float:
    return math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))


def _unit(a: Node3D, b: Node3D) -> Tuple[float, float]:
    """Unit vector (x, z) from a to b (bridge XZ-plane solver)."""
    dx = b[0] - a[0]
    dz = b[2] - a[2]
    L = math.hypot(dx, dz)
    return (dx / L, dz / L) if L > 1e-12 else (0.0, 0.0)


def solve_2d(
    nodes: List[Node3D],
    members: List[Member],
    supports: List[Tuple[int, str]],   # [(node_idx, 'pin'|'roller'), ...]
    loads: List[Tuple[int, float, float]],  # [(node_idx, Fx, Fz), ...]
) -> Dict[int, float]:
    """
    Method-of-joints static solver for a 2D planar truss (nodes on XZ plane).

    Returns dict {member_index: axial_force_N}
    Positive = tension, Negative = compression.

    Supports:
      'pin'    → fixes x and z DOF
      'roller' → fixes z DOF only

    Uses Gaussian elimination on the system of 2-DOF joint equilibrium equations.
    """
    n_nodes = len(nodes)
    n_members = len(members)

    # Count DOF / reactions
    n_reactions = 0
    for _, stype in supports:
        n_reactions += 2 if stype == 'pin' else 1

    n_unknowns = n_members + n_reactions
    n_equations = 2 * n_nodes

    # Build (n_eq x n_unknowns) matrix + RHS
    A = [[0.0] * n_unknowns for _ in range(n_equations)]
    b = [0.0] * n_equations

    # Map reaction unknowns: after member forces
    reaction_col: Dict[int, List[int]] = {}  # node_idx → [col_x, col_z?]
    col = n_members
    for node_idx, stype in supports:
        cols = []
        if stype == 'pin':
            cols.append(col);   col += 1
            cols.append(col);   col += 1
        else:  # roller
            cols.append(col);   col += 1
        reaction_col[node_idx] = cols

    # Fill equilibrium equations per node
    for ni_idx, (nx, ny, nz) in enumerate(nodes):
        row_x = 2 * ni_idx
        row_z = 2 * ni_idx + 1

        # Member contributions
        for m_idx, (i, j, _) in enumerate(members):
            if i == ni_idx:
                ux, uz = _unit(nodes[i], nodes[j])
                A[row_x][m_idx] += ux
                A[row_z][m_idx] += uz
            elif j == ni_idx:
                ux, uz = _unit(nodes[j], nodes[i])
                A[row_x][m_idx] += ux
                A[row_z][m_idx] += uz

        # Reaction contributions
        if ni_idx in reaction_col:
            rcols = reaction_col[ni_idx]
            if len(rcols) == 2:   # pin: Rx, Rz
                A[row_x][rcols[0]] += 1.0
                A[row_z][rcols[1]] += 1.0
            else:                 # roller: Rz only
                A[row_z][rcols[0]] += 1.0

        # External loads (RHS) — move to right side
        for load_node, Fx, Fz in loads:
            if load_node == ni_idx:
                b[row_x] -= Fx
                b[row_z] -= Fz

    # Gaussian elimination with partial pivoting
    forces = _gauss_solve(A, b, n_equations, n_unknowns)

    if forces is None:
        # Singular — return zeros (structure may be improperly constrained)
        return {i: 0.0 for i in range(n_members)}

    return {i: forces[i] for i in range(n_members)}


def _gauss_solve(
    A: List[List[float]],
    b: List[float],
    n_eq: int,
    n_unk: int,
) -> Optional[List[float]]:
    """Gaussian elimination — least-squares via normal equations if over-determined."""
    import copy
    # Use normal equations: A^T A x = A^T b  (works for over-determined systems)
    # Build n_unk x n_unk normal system
    ATA = [[0.0] * n_unk for _ in range(n_unk)]
    ATb = [0.0] * n_unk
    for i in range(n_eq):
        for j in range(n_unk):
            ATb[j] += A[i][j] * b[i]
            for k in range(n_unk):
                ATA[j][k] += A[i][j] * A[i][k]

    # Augmented matrix
    M = [ATA[r][:] + [ATb[r]] for r in range(n_unk)]

    for col in range(n_unk):
        # Find pivot
        pivot = -1
        best = 0.0
        for row in range(col, n_unk):
            if abs(M[row][col]) > best:
                best = abs(M[row][col])
                pivot = row
        if pivot == -1 or best < 1e-14:
            continue
        M[col], M[pivot] = M[pivot], M[col]
        inv = 1.0 / M[col][col]
        for k in range(col, n_unk + 1):
            M[col][k] *= inv
        for row in range(n_unk):
            if row != col and abs(M[row][col]) > 1e-14:
                factor = M[row][col]
                for k in range(col, n_unk + 1):
                    M[row][k] -= factor * M[col][k]

    return [M[r][n_unk] for r in range(n_unk)]


# ---------------------------------------------------------------------------
# ── QUICK 3D TOWER FORCE ESTIMATE ───────────────────────────────────────────
# ---------------------------------------------------------------------------

def solve_tower_approximate(
    nodes: List[Node3D],
    members: List[Member],
    load_node: int,
    load_fz: float,   # downward force (negative z)
    base_node_indices: List[int],
) -> Dict[int, float]:
    """
    Approximate tower force distribution using level-by-level vertical load path.
    Each upright carries an equal share of the cumulative vertical load above it.
    Diagonals carry shear via trigonometry.
    Returns member forces dict (same sign convention as solve_2d).
    """
    forces: Dict[int, float] = {}
    n_uprights_per_level = 0
    for idx, (i, j, t) in enumerate(members):
        forces[idx] = 0.0

    # Count corners from upright count at level 0
    upright_indices = [idx for idx, (i, j, t) in enumerate(members) if t == 'upright']
    if not upright_indices:
        return forces

    # Determine n_corners from horizontal member pattern
    horiz = [(i, j) for (i, j, t) in members if t == 'horizontal']
    # n_corners = number of horizontals at level 0
    n_corners = sum(1 for i, j in horiz if i < 4 and j < 4)
    if n_corners < 2:
        n_corners = 4  # fallback

    # Total load on each upright
    total_vertical = abs(load_fz)
    per_upright = total_vertical / max(n_corners, 1)

    for idx, (i, j, t) in enumerate(members):
        if t == 'upright':
            # Compression from gravity
            forces[idx] = -per_upright
        elif t in ('diagonal', 'k_vertical'):
            # Shear distributed across diagonals per face
            Li = _dist(nodes[i], nodes[j])
            Lvert = abs(nodes[j][2] - nodes[i][2])
            if Li > 1e-9:
                sin_angle = Lvert / Li
                shear = per_upright * 0.5
                forces[idx] = shear / max(sin_angle, 0.1) * (1 if t == 'diagonal' else -1)
        elif t in ('horizontal', 'chord'):
            forces[idx] = -per_upright * 0.25  # compression in rings

    return forces


# ---------------------------------------------------------------------------
# ── BOM / STRAND BUDGET ──────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def bundle_strand_count(bundle_dia_mm: float, strand_dia_mm: float) -> int:
    """
    Estimate number of strands that fit in a circular bundle of diameter bundle_dia_mm
    using hexagonal close packing (packing fraction ≈ π/(2√3) ≈ 0.9069).
    """
    if strand_dia_mm <= 0:
        return 1
    bundle_r = bundle_dia_mm / 2.0
    strand_r = strand_dia_mm / 2.0
    packing = math.pi / (2.0 * math.sqrt(3))
    area_bundle = math.pi * bundle_r ** 2
    area_strand = math.pi * strand_r ** 2
    n = packing * area_bundle / area_strand
    return max(1, int(math.ceil(n)))


def compute_bom(
    nodes: List[Node3D],
    members: List[Member],
    bundle_dia_mm: float,
    strand_dia_mm: float = 1.7,
    strand_len_mm: float = 250.0,
    pasta_density_g_cm3: float = 1.13,
) -> List[dict]:
    """
    Compute per-member BOM.

    Returns list of dicts:
      member_id, i, j, type, length_mm, strands_in_bundle,
      strand_lengths_needed, total_strands, mass_g
    """
    rows = []
    n_bundle = bundle_strand_count(bundle_dia_mm, strand_dia_mm)
    strand_r_cm = strand_dia_mm / 20.0   # mm → cm
    strand_area_cm2 = math.pi * strand_r_cm ** 2

    for mid, (i, j, mtype) in enumerate(members):
        L_mm = _dist(nodes[i], nodes[j])
        # Strand lengths (sections) needed per strand to span member
        strand_sections = max(1, int(math.ceil(L_mm / strand_len_mm)))
        # Total raw strands purchased = strands_in_bundle × sections
        total_strands = n_bundle * strand_sections
        # Volume of the bundle cylinder (cm^3)
        L_cm = L_mm / 10.0
        vol_cm3 = strand_area_cm2 * n_bundle * L_cm
        mass_g = vol_cm3 * pasta_density_g_cm3
        rows.append({
            'member_id': mid,
            'i': i,
            'j': j,
            'type': mtype,
            'length_mm': round(L_mm, 2),
            'strands_in_bundle': n_bundle,
            'strand_lengths_needed': strand_sections,
            'total_strands': total_strands,
            'mass_g': round(mass_g, 4),
        })
    return rows


def bom_summary(bom_rows: List[dict]) -> dict:
    """Aggregate BOM: total strands, total mass (g)."""
    total_strands = sum(r['total_strands'] for r in bom_rows)
    total_mass_g = sum(r['mass_g'] for r in bom_rows)
    return {'total_strands': total_strands, 'total_mass_g': round(total_mass_g, 3)}


# ---------------------------------------------------------------------------
# ── CSV EXPORT ───────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def export_csv(
    bom_rows: List[dict],
    forces: Dict[int, float],
    filepath: str,
) -> None:
    """Write cut-list CSV to filepath."""
    header = (
        'member_id,i,j,type,length_mm,strands_in_bundle,'
        'strand_lengths_needed,total_strands,mass_g,'
        'force_type,force_N\n'
    )
    lines = [header]
    for r in bom_rows:
        mid = r['member_id']
        f = forces.get(mid, 0.0)
        force_type = 'tension' if f > 0.01 else ('compression' if f < -0.01 else 'zero')
        lines.append(
            f"{mid},{r['i']},{r['j']},{r['type']},{r['length_mm']},"
            f"{r['strands_in_bundle']},{r['strand_lengths_needed']},"
            f"{r['total_strands']},{r['mass_g']},"
            f"{force_type},{round(f, 2)}\n"
        )
    with open(filepath, 'w', newline='') as fh:
        fh.writelines(lines)


# ---------------------------------------------------------------------------
# ── SELF-TEST (run as standalone script) ─────────────────────────────────────
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    print('=== truss_math self-test ===')

    # --- Warren bridge ---
    nodes, members = warren_nodes(400.0, 80.0, 6)
    print(f'Warren 6-panel: {len(nodes)} nodes, {len(members)} members')
    assert len(nodes) == 14, f'Expected 14 nodes, got {len(nodes)}'
    assert len(members) == 18, f'Expected 18 members, got {len(members)}'  # 6+6+6

    # --- Pratt bridge ---
    nodes, members = pratt_nodes(400.0, 80.0, 6)
    print(f'Pratt 6-panel: {len(nodes)} nodes, {len(members)} members')

    # --- Howe bridge ---
    nodes, members = howe_nodes(400.0, 80.0, 6)
    print(f'Howe 6-panel: {len(nodes)} nodes, {len(members)} members')

    # --- K-truss ---
    nodes, members = k_truss_nodes(400.0, 80.0, 4)
    print(f'K-truss 4-panel: {len(nodes)} nodes, {len(members)} members')

    # --- Square tower ---
    nodes, members = tower_nodes(100.0, 40.0, 300.0, 5, 'square', 'x_brace')
    print(f'Square tower 5-seg: {len(nodes)} nodes, {len(members)} members')
    assert len(nodes) == 24, f'Expected 24 nodes, got {len(nodes)}'  # 6 levels × 4

    # --- Triangular tower ---
    nodes, members = tower_nodes(100.0, 40.0, 300.0, 4, 'triangular', 'diagonal')
    print(f'Triangular tower 4-seg: {len(nodes)} nodes, {len(members)} members')
    assert len(nodes) == 15, f'Expected 15 nodes, got {len(nodes)}'  # 5 levels × 3

    # --- 2D Solver: simple pin-pin beam ---
    nodes_s, members_s = warren_nodes(400.0, 80.0, 4)
    supports_s = [(0, 'pin'), (4, 'roller')]    # node 0 pin, node 4 roller
    loads_s = [(2, 0.0, -100.0)]               # 100N down at mid node (bot node 2)
    forces_s = solve_2d(nodes_s, members_s, supports_s, loads_s)
    print(f'Warren 4-panel forces: {len(forces_s)} members solved')
    print(f'  Member 0 force: {forces_s[0]:.2f} N')

    # --- BOM test ---
    nodes_b, members_b = pratt_nodes(400.0, 80.0, 6)
    bom = compute_bom(nodes_b, members_b, bundle_dia_mm=5.0, strand_dia_mm=1.7)
    summary = bom_summary(bom)
    print(f'BOM: {len(bom)} members, total strands={summary["total_strands"]}, '
          f'mass={summary["total_mass_g"]}g')
    assert summary['total_mass_g'] > 0
    assert summary['total_strands'] > 0

    # --- CSV export test ---
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), 'ssg_test.csv')
    export_csv(bom, forces_s, tmp)
    print(f'CSV exported to {tmp}')
    assert os.path.exists(tmp)

    print('=== All tests passed ===')
