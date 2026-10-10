"""Quadric-error mesh simplification for the prepared props (Garland and Heckbert, 1997).

Half-edge collapses move a vertex onto a neighbour, so every kept vertex keeps its own normal and
texture coordinates. The topology is that of the positions: the vertices at one place are its
copies on either side of a UV or normal seam, and a collapse moves each copy onto the neighbour's
copy on the same side, or is rejected if one has none, so texture islands do not crack. Vertices
on open edges stay fixed. A collapse is also rejected when it would flip a triangle or join two
sheets (the link condition). Only NumPy and the standard library are used.
"""
import heapq

import numpy as np


def _plane_quadrics(positions, triangles):
    """Area-weighted plane quadric of each triangle as 10 coefficients of the symmetric 4x4 matrix."""
    a, b, c = (positions[triangles[:, k]] for k in range(3))
    normal = np.cross(b - a, c - a)
    area = np.linalg.norm(normal, axis=1)
    unit = normal / np.maximum(area, 1e-20)[:, None]
    d = -np.einsum('ij,ij->i', unit, a)
    p = np.column_stack([unit, d])
    rows, cols = np.triu_indices(4)
    return (p[:, rows] * p[:, cols]) * (0.5 * area)[:, None]


def _cost(q, p):
    x, y, z = p
    return (q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x + q[4] * y * y + 2 * q[5] * y * z
            + 2 * q[6] * y + q[7] * z * z + 2 * q[8] * z + q[9])


def simplify(positions, triangles, target):
    """Collapse edges until at most target triangles remain (or no valid collapse is left).

    positions: (n, 3) floats; triangles: (m, 3) indices. Returns (kept vertex indices, triangles
    indexing the kept vertices); the caller gathers every vertex attribute with the kept indices.
    """
    positions = np.asarray(positions, float)
    faces = np.asarray(triangles, np.int64)
    # Welded positions: the topology and the quadrics are those of the surface; the vertices at one
    # place are its copies on either side of a seam.
    scale = max(float(np.ptp(positions, axis=0).max()), 1e-9)
    _, weld, copies = np.unique(np.round(positions / scale * 2**20).astype(np.int64), axis=0,
                                return_inverse=True, return_counts=True)
    weld = weld.ravel()
    welded_faces = weld[faces]
    count = len(copies)
    face_quadrics = _plane_quadrics(positions, faces)
    quadrics = np.zeros((count, 10))
    for k in range(3):
        np.add.at(quadrics, welded_faces[:, k], face_quadrics)
    quadrics = quadrics.tolist()
    place = np.zeros((count, 3))
    place[weld] = positions
    place = place.tolist()
    # Vertices on open edges of the surface stay fixed.
    edges = np.sort(np.vstack([welded_faces[:, [0, 1]], welded_faces[:, [1, 2]], welded_faces[:, [2, 0]]]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    locked = np.zeros(count, bool)
    locked[unique[counts == 1].ravel()] = True
    locked = locked.tolist()
    weld = weld.tolist()
    face_list = faces.tolist()
    vertex_faces = [set() for _ in range(count)]
    for f, face in enumerate(face_list):
        for x in face:
            vertex_faces[weld[x]].add(f)
    alive = [True] * len(face_list)
    live_faces = len(face_list)
    stamp = [0] * count

    def neighbours(v):
        result = set()
        for f in vertex_faces[v]:
            result.update(weld[x] for x in face_list[f])
        result.discard(v)
        return result

    def normal(face, moved=None, to=None):
        a, b, c = (to if weld[x] == moved else place[weld[x]] for x in face)
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        w = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        return (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])

    heap = []

    def push(u, v):
        if not locked[u]:
            q = [a + b for a, b in zip(quadrics[u], quadrics[v])]
            heapq.heappush(heap, (_cost(q, place[v]), u, v, stamp[u], stamp[v]))

    for u in range(count):
        for v in neighbours(u):
            push(u, v)

    while live_faces > target and heap:
        _, u, v, su, sv = heapq.heappop(heap)
        if su != stamp[u] or sv != stamp[v]:
            continue
        shared = vertex_faces[u] & vertex_faces[v]
        if not shared or len(neighbours(u) & neighbours(v)) != len(shared):
            continue  # link condition: only the vertices opposite the edge may be common
        # Each copy of u must move onto the copy of v on its side of any seam; a copy of u with no
        # edge to v would tear its texture island open.
        onto = {}
        for f in shared:
            a = next(x for x in face_list[f] if weld[x] == u)
            b = next(x for x in face_list[f] if weld[x] == v)
            if onto.setdefault(a, b) != b:
                break
        else:
            moved = vertex_faces[u] - shared
            valid = True
            for f in moved:
                a = next(x for x in face_list[f] if weld[x] == u)
                before, after = normal(face_list[f]), normal(face_list[f], u, place[v])
                dot = before[0] * after[0] + before[1] * after[1] + before[2] * after[2]
                size = (sum(x * x for x in before) * sum(x * x for x in after)) ** 0.5
                if a not in onto or size <= 0 or dot < 0.3 * size:
                    valid = False
                    break
            if not valid:
                continue
            for f in shared:
                alive[f] = False
                live_faces -= 1
                for x in face_list[f]:
                    vertex_faces[weld[x]].discard(f)
            for f in moved:
                face_list[f] = [onto[x] if weld[x] == u else x for x in face_list[f]]
                vertex_faces[v].add(f)
            vertex_faces[u] = set()
            quadrics[v] = [a + b for a, b in zip(quadrics[u], quadrics[v])]
            # Only the edges at v change cost; the others are re-checked when popped.
            stamp[u] += 1
            stamp[v] += 1
            for x in neighbours(v):
                push(v, x)
                push(x, v)

    remaining = np.array([face for face, live in zip(face_list, alive) if live], np.int64).reshape(-1, 3)
    kept = np.unique(remaining)
    remap = np.full(len(positions), -1, np.int64)
    remap[kept] = np.arange(len(kept))
    return kept, remap[remaining]


def simplify_part(part, ratio):
    """A flatten_scene() part reduced to about ratio of its triangles."""
    target = max(16, int(len(part['indices']) * ratio))
    kept, triangles = simplify(part['POSITION'], part['indices'], target)
    result = {key: part[key][kept] for key in ('POSITION', 'NORMAL', 'TEXCOORD_0') if key in part}
    result['material'] = part['material']
    result['indices'] = triangles.astype(np.uint32)
    return result
