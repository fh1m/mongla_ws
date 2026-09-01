#!/usr/bin/env python3

"""Added mass for Dubomini, by boundary-element solve on the real hull.

WHY THIS MATTERS MORE THAN IT SOUNDS
------------------------------------
Added mass is the water a vehicle must accelerate along with itself, and
underwater it is not a correction -- it is comparable to the vehicle's own mass
and dominates how it starts, stops and turns. Duburi's heave term is 18.7 kg
against a 13.5 kg hull. Get it wrong and the vehicle feels wrong in a way no
amount of drag tuning fixes.

WHAT IS SOLVED, AND ON WHAT
---------------------------
Capytaine solves linear potential flow on the hull and returns the full 6x6
added-mass matrix, evaluated at INFINITE frequency with the free surface
removed -- the standard treatment for a submerged ROV, where the vehicle is deep
enough that wave effects do not reach it and added mass is effectively constant.

It runs on the CONVEX HULL, not the raw assembly, and that is forced rather than
chosen: `hullv3.stl` is 3,276 disconnected components and NOT water-tight, and a
BEM panel method needs a closed surface with consistent normals. The hull is
2,472 faces and 31.8 L against the assembly's 2.15 M faces.

That substitution is a real approximation and it runs in a KNOWN DIRECTION: a
convex envelope displaces more water than an open frame, so these coefficients
are an UPPER BOUND. The vehicle will feel slightly more sluggish to accelerate
than the real one. Said here rather than discovered later.

SIGN CONVENTION -- the trap
---------------------------
SDF's //inertial/fluid_added_mass takes POSITIVE magnitudes, while the
Hydrodynamics plugin's own <xDotU> family is negative. `generate_model.py`
rejects negatives for exactly this reason. Capytaine returns positive added
mass, so it feeds the SDF form directly -- but anything copied from a paper
usually does not.
"""

import sys

import numpy as np


def solve(hull_path: str, rho: float = 1000.0):
    import capytaine as cpt
    mesh = cpt.load_mesh(hull_path, file_format='stl')
    body = cpt.FloatingBody(mesh=mesh, name='dubomini')
    body.add_all_rigid_body_dofs()
    body.center_of_mass = np.zeros(3)
    # Immersed: no free surface, infinite depth. `water_depth=inf` with
    # `free_surface=inf` is Capytaine's way of saying "deeply submerged".
    problems = [cpt.RadiationProblem(body=body, radiating_dof=d,
                                     omega=np.inf, free_surface=np.inf,
                                     water_depth=np.inf, rho=rho)
                for d in body.dofs]
    solver = cpt.BEMSolver()
    results = solver.solve_all(problems, progress_bar=False)
    # READ THE RESULTS DIRECTLY, do not assemble a dataset.
    #
    # `cpt.assemble_dataset` SILENTLY DROPS every case without a free surface
    # ("Cases without a free surface (free_surface=inf) are ignored" --
    # capytaine issue #88), which is precisely the deeply-submerged case an ROV
    # is. The solve succeeds and the dataset comes back with no `added_mass`
    # variable at all, which reads like the solver failed when it did not.
    dofs = list(body.dofs)
    A = np.zeros((len(dofs), len(dofs)))
    for r in results:
        i = dofs.index(r.radiating_dof)
        for j, d in enumerate(dofs):
            A[j, i] = r.added_mass[d]
    return A, dofs


def main(argv=None):
    a = argv if argv is not None else sys.argv[1:]
    if not a:
        print('usage: added_mass.py HULL.stl', file=sys.stderr)
        return 2
    try:
        A, dofs = solve(a[0])
    except Exception as exc:                                   # noqa: BLE001
        print(f'BEM solve FAILED: {type(exc).__name__}: {exc}', file=sys.stderr)
        print('fall back to the analytic ellipsoid values and SAY SO',
              file=sys.stderr)
        return 1
    np.set_printoptions(precision=3, suppress=True, linewidth=140)
    print('dofs:', dofs)
    print(A)
    print()
    diag = np.diag(A)
    names = ['xDotU (surge)', 'yDotV (sway)', 'zDotW (heave)',
             'kDotP (roll)', 'mDotQ (pitch)', 'nDotR (yaw)']
    print('SDF //inertial/fluid_added_mass (positive magnitudes):')
    for n, v in zip(names, diag):
        print(f'  {n:16s} {v:10.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
