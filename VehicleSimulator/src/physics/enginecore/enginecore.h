#pragma once
/*===========================================================================
 *  enginecore.h  --  C interface of EngineSim's compiled cylinder core.
 *
 *  enginecore.c, fields.h and gas_tables.inc in this directory are VENDORED
 *  from ../EngineSim by tools/sync_enginecore.py and must not be edited here
 *  (see that script). This header is the only hand-written file in the
 *  directory: enginecore.c has no header of its own, because EngineSim loads
 *  it through ctypes.
 *
 *  The two functions and their array layouts are documented where they are
 *  defined - enginecore.c and ../EngineSim/engine_sim/native_core.py - and
 *  used in exactly one place, physics/ResolvedEngine.cpp.
 *=========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* Load the eight gas-property tables (8 x 251 doubles, see gas_tables.inc)
 * and derive the orifice constants. Call once before es_advance(). */
void es_init_tables(const double *tables);

/* Run the substep loop of one engine step: every cylinder, the exhaust
 * plenum (and turbine stage), the boost plenum, the throttle and the intake
 * manifold. K = engine constants, B = per-step inputs and outputs, cs = the
 * cylinders' state (ncs doubles each); indices are the K_*, B_* and F_*
 * macros of fields.h. */
void es_advance(const double *K, const double *vol, const double *ain, const double *aex,
                double *cs, int ncs, double *trace, double *B, double *loc,
                const double *begin_k, const double *escale, int n_es);

#ifdef __cplusplus
}
#endif
